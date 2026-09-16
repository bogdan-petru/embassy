//! Private effect seam BELOW the adapter, not a replacement for its algorithm.
//! Tests record these HAL calls, not the internals of eDMA or the NVIC hardware.

use core::task::{Context, Poll};

use maitake_sync::WaitCell;

use super::Error;
use super::endpoint::{ErasedRxEndpoint, ErasedTxEndpoint};

// This module has no public re-export. Only the audited hardware binding and
// its child test fixture implement the seam; application code cannot supply it.
pub(in crate::i2c::one_shot) trait Effects {
    fn i2c_wait(&self) -> &WaitCell;
    fn dma_wait(&self) -> &WaitCell;
    fn irq_disable(&self);
    fn irq_unpend(&self);
    fn irq_enable(&self);
    fn now_us(&self) -> u64;
    fn prepare(&mut self, bytes: &'static [u8], endpoint: &ErasedTxEndpoint) -> Result<(), Error>;
    fn select_request(&self, endpoint: &ErasedTxEndpoint);
    /// Configure the exact RX channel/path while the caller still retains the
    /// exclusive static destination in its private pending lease.
    fn prepare_receive(&mut self, bytes: &mut [u8], endpoint: &ErasedRxEndpoint) -> Result<(), Error>;
    fn select_receive_request(&self, endpoint: &ErasedRxEndpoint);
    fn dma_interrupt_enable(&self);
    fn dma_request_enable(&self);
    fn dma_error(&self) -> bool;
    fn dma_done(&self) -> bool;
    fn quiesce(&self);
    fn release_pins(&mut self);

    // Observation only: registration itself is shared software below, not an
    // overridable test implementation that could hide a production wake bug.
    fn i2c_wait_observed(&self, _ready: bool) {}
    fn dma_wait_observed(&self, _ready: bool) {}
}

pub(super) fn register_wait(wait: &WaitCell, cx: &mut Context<'_>, observed: impl FnOnce(bool)) -> Poll<()> {
    let result = wait.poll_wait(cx).map(|_| ());
    observed(result.is_ready());
    result
}

// Both production and injected host interrupts execute this very body.
pub(super) fn wake_interrupt(disable: impl FnOnce(), wait: &WaitCell) {
    disable();
    wait.wake();
}
