use core::future::poll_fn;
use core::task::Poll;

use crate::{Backend, DmaPoll, Error, ReceivePoll, Status, WakeOn};

/// Already-owned hardware, before START. Construction belongs to the HAL.
#[must_use]
pub struct Idle<B: Backend>(B);
/// START was queued, not necessarily acknowledged or executed on the wire.
#[must_use]
pub struct StartQueued<B: Backend>(B);
/// The DMA lease owns the pair; no CPU command authority is retained here.
#[must_use]
pub struct Streaming<B: Backend> {
    // Taken only inside a synchronous poll. Pending always restores the lease.
    // Option is checked poll plumbing, not an independently forgeable receipt.
    lease: Option<B::Streaming>,
}
/// The backend reported successful DMA completion, not cancellation/shutdown.
/// For the audited Device-backed implementation this means all bytes reached
/// the FIFO, DONE had no DMA error, requests are off and the channel is quiescent.
/// A safe mock Backend can misreport protocol facts; no memory-safety decision
/// may rely on this generic protocol state instead of the lower owned capability.
#[must_use]
pub struct TxDmaQuiescent<B: Backend>(B);
/// Repeated START and one RECEIVE command were queued; DMA owns the RX buffer.
#[must_use]
pub struct Receiving<B: Backend> {
    lease: Option<B::Receiving>,
}
/// Receive DMA completed and released exclusive memory after quiescence.
/// Keep it with the bus owner until STOP is observed, not just DMA DONE.
#[must_use]
pub struct RxDmaQuiescent<B: Backend> {
    backend: B,
    bytes: &'static mut [u8],
}
/// STOP has been queued. The owned hardware still owes cleanup if abandoned.
#[must_use]
pub struct Stopping<B: Backend, P = ()>(B, P);

impl<B: Backend> Idle<B> {
    pub fn new(backend: B) -> Self {
        Self(backend)
    }

    pub fn start(self, address: u8) -> Result<StartQueued<B>, Error> {
        if !valid_address(address) {
            return Err(Error::InvalidInput);
        }
        self.0.start(address).map(StartQueued)
    }
}

impl<B: Backend> StartQueued<B> {
    pub fn transmit(self, bytes: &'static [u8]) -> Result<Streaming<B>, Error> {
        if !valid_length(bytes.len()) {
            return Err(Error::InvalidInput);
        }
        self.0.arm(bytes).map(|lease| Streaming { lease: Some(lease) })
    }
}

impl<B: Backend> Streaming<B> {
    pub async fn drain(mut self) -> Result<TxDmaQuiescent<B>, Error> {
        poll_fn(|cx| {
            let lease = self.lease.as_mut().expect("live lease until terminal poll");
            checked(B::observe_streaming(lease, cx, WakeOn::Completion), false)?;
            let lease = self.lease.take().expect("same lease consumed by this poll");
            match B::poll_dma(lease, cx) {
                DmaPoll::Pending(lease) => {
                    self.lease = Some(lease);
                    Poll::Pending
                }
                DmaPoll::Complete(backend) => Poll::Ready(Ok(TxDmaQuiescent(backend))),
                DmaPoll::Failed(error) => Poll::Ready(Err(error)),
            }
        })
        .await
    }
}

impl<B: Backend> TxDmaQuiescent<B> {
    pub async fn stop(mut self) -> Result<Stopping<B>, Error> {
        poll_fn(|cx| {
            checked(self.0.observe(cx, WakeOn::TxSpace), false)?;
            self.0.try_stop()
        })
        .await?;
        Ok(Stopping(self.0, ()))
    }

    /// Only a short write-to-read repeated START: no STOP, chunks or retries.
    pub async fn receive(mut self, address: u8, bytes: &'static mut [u8]) -> Result<Receiving<B>, Error> {
        validate_transfer(address, bytes.len())?;
        let length = bytes.len() as u8;
        poll_fn(|cx| {
            checked(self.0.observe(cx, WakeOn::TxSpace), false)?;
            self.0.read_ready(address, length)
        })
        .await?;
        self.0
            .arm_receive(address, bytes)
            .map(|lease| Receiving { lease: Some(lease) })
    }
}

impl<B: Backend> Receiving<B> {
    pub async fn drain(mut self) -> Result<RxDmaQuiescent<B>, Error> {
        poll_fn(|cx| {
            let lease = self.lease.as_mut().expect("live RX lease until terminal poll");
            checked(B::observe_receive(lease, cx, WakeOn::Completion), false)?;
            let lease = self.lease.take().expect("same RX lease consumed by this poll");
            match B::poll_receive(lease, cx) {
                ReceivePoll::Pending(lease) => {
                    self.lease = Some(lease);
                    Poll::Pending
                }
                ReceivePoll::Complete((backend, bytes)) => Poll::Ready(Ok(RxDmaQuiescent { backend, bytes })),
                ReceivePoll::Failed(error) => Poll::Ready(Err(error)),
            }
        })
        .await
    }
}

impl<B: Backend> RxDmaQuiescent<B> {
    pub async fn stop(mut self) -> Result<Stopping<B, &'static mut [u8]>, Error> {
        poll_fn(|cx| {
            checked(self.backend.observe(cx, WakeOn::TxSpace), false)?;
            self.backend.try_stop()
        })
        .await?;
        Ok(Stopping(self.backend, self.bytes))
    }
}

impl<B: Backend, P> Stopping<B, P> {
    /// Observe this owner's STOP and an idle controller without a fault, then
    /// release the backend before returning the payload. The physical fact is
    /// the audited Device's promise, not a proof about an arbitrary safe mock.
    /// The addressed device may still be processing the completed transaction.
    pub async fn complete(mut self) -> Result<P, Error> {
        poll_fn(|cx| {
            let status = checked(self.0.observe(cx, WakeOn::Completion), true)?;
            if status.stop_seen && status.controller_idle {
                Poll::Ready(Ok(()))
            } else {
                Poll::Pending
            }
        })
        .await?;
        let Self(backend, payload) = self;
        drop(backend);
        Ok(payload)
    }
}

/// Pure protocol decision, also called directly by the bounded Kani harnesses.
/// It decodes no MMIO and grants no hardware or DMA authority.
pub fn checked(status: Status, want_stop: bool) -> Result<Status, Error> {
    let error = if status.pin_low_timeout {
        Some(Error::PinLowTimeout)
    } else if status.nack {
        Some(Error::Nack)
    } else if status.arbitration_loss {
        Some(Error::ArbitrationLoss)
    } else if status.fifo_error {
        Some(Error::Fifo)
    } else if status.stop_seen && !want_stop {
        Some(Error::UnexpectedStop)
    } else {
        None
    };
    match error {
        Some(error) => Err(error),
        None => Ok(status),
    }
}

fn valid_address(address: u8) -> bool {
    (0x08..=0x77).contains(&address)
}

fn valid_length(length: usize) -> bool {
    (1..=32).contains(&length)
}

/// Preflight shared by the public HAL and pure host/proof checks.
pub fn validate_transfer(address: u8, length: usize) -> Result<(), Error> {
    if valid_address(address) && valid_length(length) {
        Ok(())
    } else {
        Err(Error::InvalidInput)
    }
}
