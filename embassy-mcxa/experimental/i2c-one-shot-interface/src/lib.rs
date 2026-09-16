//! Layer 2: safe, opaque CPU/DMA capabilities over an audited device binding.
//! Acquisition, interrupt binding, DMA arming and quiescence stay below this
//! interface. Upper layers cannot extract a raw device or in-flight lease.
#![no_std]
#![deny(unsafe_code)]

#[forbid(unsafe_code)]
mod capabilities;
#[forbid(unsafe_code)]
mod types;

pub use capabilities::{Completion, Cpu, InFlight, ReceiveCompletion, Receiving};
pub use types::{Command, DmaResult, Error, ReceiveResult, Status, Step, WakeOn};

use core::task::Context;

/// Trusted physical binding, not the protocol vocabulary's implementation.
/// This is an open unsafe contract, not compiler sealing or hardware proof.
/// The HAL constructs it from owned resources, then wraps it in `Cpu` before
/// passing anything to the unsafe-free vocabulary. No default bodies belong here.
///
/// # Safety
///
/// Implementors own the clocked controller, its inseparable destination/request
/// pair, the bound DMA channel and interrupts. EVERY safe call order must be
/// memory-safe, even when upper-layer protocol logic is wrong. No assumption
/// that START was sent, STOP will follow, or Drop will run can justify memory
/// safety. Static immutable sources remain valid if an owner is forgotten.
///
/// `arm` consumes CPU authority and retains the same controller, channel and
/// source in `InFlight`. It prepares the TCD and paired request BEFORE enabling
/// DMA IRQ/request and peripheral requests. No independently callable prepare,
/// select-request, enable-request or resource-release primitive is exposed.
/// Errors and ordinary Drop disable requests and quiesce before reset/release.
///
/// `poll_dma` returns the SAME live lease on Pending. Complete returns CPU
/// authority only after DONE without error, every requested byte delivered to
/// the TX FIFO, requests off, and quiescence complete. Cancellation may quiesce
/// but never manufactures Complete. Failed consumes and cleans the lease.
///
/// `arm_receive` additionally consumes EXCLUSIVE static application storage.
/// After validating address/length and fresh FIFO space, it prepares and arms
/// the RX channel and peripheral requests BEFORE issuing START-read/RECEIVE.
/// This compound operation exposes no command authority on its live lease.
/// The receive lease retains it without CPU reads/writes while DMA may write.
/// `poll_receive` may return that storage only after requests-off, quiescence
/// and an acquire fence. This applies on every safe call order, not just a
/// correct I2C sequence. Drop quiesces before abandoning the reference;
/// forgetting a lease keeps its static storage inaccessible indefinitely.
/// Failure consumes the one-shot buffer; no alias or recovery handle escapes.
///
/// Snapshots decode one status read without choosing protocol fault priority.
/// Observe registers a real waker before arming and reading latched status;
/// DMA Pending retains a wake path. Command writes only exist on CPU authority.
///
/// These promises do not prove peer cooperation, clock timing, that Drop runs
/// for leaked futures, or eventual completion of an active DMA transaction.
#[allow(unsafe_code)]
pub unsafe trait Device: Sized {
    type InFlight;
    type Receiving;
    fn snapshot(&self) -> Status;
    fn command(&mut self, command: Command);
    fn fifo_empty(&self) -> bool;
    fn arm(self, bytes: &'static [u8]) -> Result<Self::InFlight, Error>;
    fn observe(&mut self, cx: &mut Context<'_>, wake: WakeOn) -> Status;
    fn observe_dma(lease: &mut Self::InFlight, cx: &mut Context<'_>, wake: WakeOn) -> Status;
    fn poll_dma(lease: Self::InFlight, cx: &mut Context<'_>) -> DmaResult<Self>;
    fn arm_receive(self, address: u8, bytes: &'static mut [u8]) -> Result<Self::Receiving, Error>;
    fn observe_receive(lease: &mut Self::Receiving, cx: &mut Context<'_>, wake: WakeOn) -> Status;
    fn poll_receive(lease: Self::Receiving, cx: &mut Context<'_>) -> ReceiveResult<Self>;
}
