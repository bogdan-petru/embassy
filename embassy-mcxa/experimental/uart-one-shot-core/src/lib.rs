//! L4 finite UART TX protocol. No PAC, HAL, timer, allocation or unsafe code.
//! Completion means the transmitter sent the last bit, not a peer acknowledgement.
#![no_std]
#![forbid(unsafe_code)]

use core::future::poll_fn;
use core::task::Poll;

pub use uart_one_shot_vocabulary::{Backend, DmaPoll, Error};

#[must_use]
pub struct Idle<B: Backend>(B);
#[must_use]
pub struct Streaming<B: Backend>(Option<B::Streaming>);
/// The source is no longer used by DMA. UART may still be shifting bytes out.
#[must_use]
pub struct DmaQuiescent<B: Backend>(B);
#[must_use]
pub struct WireComplete<B: Backend>(B);

impl<B: Backend> Idle<B> {
    pub fn new(backend: B) -> Self {
        Self(backend)
    }

    pub fn transmit(self, bytes: &'static [u8]) -> Result<Streaming<B>, Error> {
        validate_length(bytes.len())?;
        self.0.arm(bytes).map(|lease| Streaming(Some(lease)))
    }
}

impl<B: Backend> Streaming<B> {
    pub async fn drain(mut self) -> Result<DmaQuiescent<B>, Error> {
        poll_fn(|cx| {
            let lease = self.0.take().expect("pending restores the same live DMA lease");
            match B::poll_dma(lease, cx) {
                DmaPoll::Pending(lease) => {
                    self.0 = Some(lease);
                    Poll::Pending
                }
                DmaPoll::Complete(backend) => Poll::Ready(Ok(DmaQuiescent(backend))),
                DmaPoll::Failed(error) => Poll::Ready(Err(error)),
            }
        })
        .await
    }
}

impl<B: Backend> DmaQuiescent<B> {
    pub async fn complete(mut self) -> Result<WireComplete<B>, Error> {
        poll_fn(|cx| self.0.poll_wire(cx)).await?;
        Ok(WireComplete(self.0))
    }
}

impl<B: Backend> WireComplete<B> {
    pub fn finish(self) {
        drop(self.0);
    }
}

pub fn validate_length(length: usize) -> Result<(), Error> {
    if (1..=32).contains(&length) {
        Ok(())
    } else {
        Err(Error::InvalidInput)
    }
}

/// The same sequence is executed by the HAL and the host conformance tests.
pub async fn run_write<B: Backend>(backend: B, bytes: &'static [u8]) -> Result<(), Error> {
    let drained = Idle::new(backend).transmit(bytes)?.drain().await?;
    drained.complete().await?.finish();
    Ok(())
}
