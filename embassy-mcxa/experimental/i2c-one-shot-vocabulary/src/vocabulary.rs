use core::task::{Context, Poll};

use i2c_one_shot_interface::{Command, Cpu, Device, InFlight, Receiving};

use crate::{Backend, DmaPoll, Error, ReceivePoll, Status, WakeOn};

/// Named operations. There is no PAC, raw device, address or DMA-channel access.
pub struct Hardware<D: Device> {
    cpu: Cpu<D>,
}

impl<D: Device> Hardware<D> {
    pub fn new(cpu: Cpu<D>) -> Self {
        Self { cpu }
    }
}

impl<D: Device> Backend for Hardware<D> {
    type Streaming = InFlight<D>;
    type Receiving = Receiving<D>;

    fn start(mut self, address: u8) -> Result<Self, Error> {
        let status = self.cpu.snapshot();
        if !status.bus_idle || !status.controller_idle {
            return Err(Error::Busy);
        }
        self.cpu.command(Command::Start(address));
        Ok(self)
    }

    fn arm(self, bytes: &'static [u8]) -> Result<Self::Streaming, Error> {
        self.cpu.arm(bytes)
    }

    fn observe(&mut self, cx: &mut Context<'_>, wake: WakeOn) -> Status {
        self.cpu.observe(cx, wake)
    }

    fn observe_streaming(lease: &mut Self::Streaming, cx: &mut Context<'_>, wake: WakeOn) -> Status {
        lease.observe(cx, wake)
    }

    fn poll_dma(lease: Self::Streaming, cx: &mut Context<'_>) -> DmaPoll<Self> {
        lease.poll(cx).map(core::convert::identity, Self::new)
    }

    fn try_stop(&mut self) -> Poll<Result<(), Error>> {
        // Current FIFO state, checked at the command's point of use. No await
        // or detached "space available" receipt separates check from write.
        if !self.cpu.fifo_empty() {
            return Poll::Pending;
        }
        self.cpu.command(Command::Stop);
        Poll::Ready(Ok(()))
    }

    fn read_ready(&mut self, address: u8, length: u8) -> Poll<Result<(), Error>> {
        if !(0x08..=0x77).contains(&address) || !(1..=32).contains(&length) {
            return Poll::Ready(Err(Error::InvalidInput));
        }
        // This only waits. The consuming lower operation rechecks FIFO space,
        // arms RX DMA, and then queues START-read/RECEIVE in that order.
        // Returning readiness must not start traffic before DMA is armed.
        if !self.cpu.fifo_empty() {
            return Poll::Pending;
        }
        Poll::Ready(Ok(()))
    }

    fn arm_receive(self, address: u8, bytes: &'static mut [u8]) -> Result<Self::Receiving, Error> {
        self.cpu.arm_receive(address, bytes)
    }

    fn observe_receive(lease: &mut Self::Receiving, cx: &mut Context<'_>, wake: WakeOn) -> Status {
        lease.observe(cx, wake)
    }

    fn poll_receive(lease: Self::Receiving, cx: &mut Context<'_>) -> ReceivePoll<Self> {
        lease
            .poll(cx)
            .map(core::convert::identity, |(cpu, bytes)| (Self::new(cpu), bytes))
    }
}
