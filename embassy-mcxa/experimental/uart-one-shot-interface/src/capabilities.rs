use core::task::{Context, Poll};

use crate::{Device, DmaResult, Error};

#[must_use]
pub struct Cpu<D: Device>(D);

#[must_use]
pub struct InFlight<D: Device>(D::InFlight);

pub enum Completion<D: Device> {
    Pending(InFlight<D>),
    Complete(Cpu<D>),
    Failed(Error),
}

impl<D: Device> Cpu<D> {
    pub fn new(device: D) -> Self {
        Self(device)
    }

    pub fn arm(self, bytes: &'static [u8]) -> Result<InFlight<D>, Error> {
        self.0.arm(bytes).map(InFlight)
    }

    pub fn poll_wire(&mut self, cx: &mut Context<'_>) -> Poll<Result<(), Error>> {
        self.0.poll_wire(cx)
    }
}

impl<D: Device> InFlight<D> {
    pub fn poll(self, cx: &mut Context<'_>) -> Completion<D> {
        match D::poll_dma(self.0, cx) {
            DmaResult::Pending(lease) => Completion::Pending(Self(lease)),
            DmaResult::Complete(device) => Completion::Complete(Cpu::new(device)),
            DmaResult::Failed(error) => Completion::Failed(error),
        }
    }
}
