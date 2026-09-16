//! L2 trust boundary. CPU and in-flight owners have no raw-resource accessor.
#![no_std]
#![deny(unsafe_code)]

#[forbid(unsafe_code)]
mod capabilities;
pub use capabilities::{Completion, Cpu, InFlight};

use core::task::{Context, Poll};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    InvalidInput,
    Clock,
    Dma,
    Timeout,
    Reset,
}

pub enum DmaResult<D: Device> {
    Pending(D::InFlight),
    Complete(D),
    Failed(Error),
}

/// Audited physical implementation, not a generic protocol-completion receipt.
///
/// # Safety
/// Own the clocked UART, pin, matched DMA destination/request, channel and IRQ
/// bindings for every pending operation and its cleanup. EVERY safe call order
/// must be memory-safe, even if upper logic is wrong. `arm` consumes CPU access,
/// retains the immutable static source, prepares the DMA descriptor/request pair
/// before enabling requests, and exposes no intermediate configuration handle.
/// `Pending` returns that same live owner. `Complete` requires DONE without a DMA
/// error, requests disabled and channel quiescence; cancellation cannot mint it.
/// Error/Drop must stop DMA before reset/resource release. Forgetting an owner
/// may retain resources forever but cannot invalidate its static source.
/// `poll_wire` arms its wake path and freshly observes UART transmitter-complete:
/// successful DMA alone does not establish the final stop bit left the pin.
/// None of these promises establishes that a peer received/accepted the bytes.
#[allow(unsafe_code)]
pub unsafe trait Device: Sized {
    type InFlight;
    fn arm(self, bytes: &'static [u8]) -> Result<Self::InFlight, Error>;
    fn poll_dma(lease: Self::InFlight, cx: &mut Context<'_>) -> DmaResult<Self>;
    fn poll_wire(&mut self, cx: &mut Context<'_>) -> Poll<Result<(), Error>>;
}
