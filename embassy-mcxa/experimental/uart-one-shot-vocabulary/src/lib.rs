//! L3 UART vocabulary, with no PAC, MMIO or timer dependency.
#![no_std]
#![forbid(unsafe_code)]

use core::task::{Context, Poll};

pub use uart_one_shot_interface::Error;
use uart_one_shot_interface::{Completion, Cpu, Device, InFlight};

/// Safe protocol facts. A mock can lie, but cannot unwrap real DMA resources.
pub trait Backend: Sized {
    type Streaming;
    fn arm(self, bytes: &'static [u8]) -> Result<Self::Streaming, Error>;
    fn poll_dma(lease: Self::Streaming, cx: &mut Context<'_>) -> DmaPoll<Self>;
    fn poll_wire(&mut self, cx: &mut Context<'_>) -> Poll<Result<(), Error>>;
}

pub enum DmaPoll<B: Backend> {
    Pending(B::Streaming),
    Complete(B),
    Failed(Error),
}

pub struct Hardware<D: Device>(Cpu<D>);
pub struct Streaming<D: Device>(InFlight<D>);

impl<D: Device> Hardware<D> {
    pub fn new(cpu: Cpu<D>) -> Self {
        Self(cpu)
    }
}

impl<D: Device> Backend for Hardware<D> {
    type Streaming = Streaming<D>;

    fn arm(self, bytes: &'static [u8]) -> Result<Self::Streaming, Error> {
        self.0.arm(bytes).map(Streaming)
    }

    fn poll_dma(lease: Self::Streaming, cx: &mut Context<'_>) -> DmaPoll<Self> {
        match lease.0.poll(cx) {
            Completion::Pending(lease) => DmaPoll::Pending(Streaming(lease)),
            Completion::Complete(cpu) => DmaPoll::Complete(Self(cpu)),
            Completion::Failed(error) => DmaPoll::Failed(error),
        }
    }

    fn poll_wire(&mut self, cx: &mut Context<'_>) -> Poll<Result<(), Error>> {
        self.0.poll_wire(cx)
    }
}
