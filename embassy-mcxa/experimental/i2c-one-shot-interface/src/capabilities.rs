//! Only the trusted binding deals in raw Device/InFlight values. These owned
//! capabilities deliberately have no into_inner, address or channel accessor.
use core::task::Context;

use crate::{Command, Device, Error, Status, Step, WakeOn};

/// CPU authority over one already-acquired controller/channel pair.
#[must_use]
pub struct Cpu<D: Device> {
    device: D,
}

/// The same pair and static source in flight. No CPU command/reset interface.
#[must_use]
pub struct InFlight<D: Device> {
    lease: D::InFlight,
}

/// Pending retains ownership; only completed quiescence restores CPU authority.
pub type Completion<D> = Step<InFlight<D>, Cpu<D>>;

/// Owns the controller/channel and an exclusive static receive buffer.
/// No buffer, raw lease, CPU command or reset accessor exists while in flight.
#[must_use]
pub struct Receiving<D: Device> {
    lease: D::Receiving,
}

pub type ReceiveCompletion<D> = Step<Receiving<D>, (Cpu<D>, &'static mut [u8])>;

impl<D: Device> Cpu<D> {
    /// Called by the physical binding with resources it already owns.
    pub fn new(device: D) -> Self {
        Self { device }
    }

    pub fn snapshot(&self) -> Status {
        self.device.snapshot()
    }

    pub fn command(&mut self, command: Command) {
        self.device.command(command);
    }

    pub fn fifo_empty(&self) -> bool {
        self.device.fifo_empty()
    }

    pub fn observe(&mut self, cx: &mut Context<'_>, wake: WakeOn) -> Status {
        self.device.observe(cx, wake)
    }

    /// Consume all CPU authority. No partially prepared channel escapes.
    pub fn arm(self, bytes: &'static [u8]) -> Result<InFlight<D>, Error> {
        self.device.arm(bytes).map(|lease| InFlight { lease })
    }

    /// Transfer both CPU authority and exclusive storage to the lower lease.
    pub fn arm_receive(self, address: u8, bytes: &'static mut [u8]) -> Result<Receiving<D>, Error> {
        self.device.arm_receive(address, bytes).map(|lease| Receiving { lease })
    }
}

impl<D: Device> InFlight<D> {
    pub fn observe(&mut self, cx: &mut Context<'_>, wake: WakeOn) -> Status {
        D::observe_dma(&mut self.lease, cx, wake)
    }

    /// Request-off, quiescence and fencing belong to the lower binding, never
    /// to the caller. Dropping Pending invokes that binding's own cleanup.
    pub fn poll(self, cx: &mut Context<'_>) -> Completion<D> {
        D::poll_dma(self.lease, cx).map(|lease| Self { lease }, Cpu::new)
    }
}

impl<D: Device> Receiving<D> {
    pub fn observe(&mut self, cx: &mut Context<'_>, wake: WakeOn) -> Status {
        D::observe_receive(&mut self.lease, cx, wake)
    }

    pub fn poll(self, cx: &mut Context<'_>) -> ReceiveCompletion<D> {
        D::poll_receive(self.lease, cx).map(|lease| Self { lease }, |(device, bytes)| (Cpu::new(device), bytes))
    }
}
