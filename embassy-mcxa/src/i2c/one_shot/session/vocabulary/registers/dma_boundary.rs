//! Reviewed, concrete companion to the generated register policy.
//!
//! No generated Streaming -> Enabled transition exists: writing MDER=0 (TDDE
//! or RDDE) alone does not return CPU authority while an eDMA minor loop may
//! still be active.
//! The only restoration here consumes BOTH the register owner and its effects
//! (which own the channel), quiesces that channel, and returns the same pair.

use core::marker::PhantomData;
use core::sync::atomic::{Ordering, fence};

use super::super::effects::Effects;
use super::{Enabled, Lpi2cRegisters, MDER_OFFSET, Mder, Off, State, Streaming};

// A fixed request-off operation, not an arbitrary MDER write. Kept private so
// Streaming cannot obtain a CPU state merely by performing this write.
fn requests_off<S: State>(registers: &mut Lpi2cRegisters<S>) {
    // SAFETY: the generated owner retains this live mapping; MDER is a separate
    // writable field, disjoint from the retained pure-read MSR/MFSR handles.
    // Only the current paired owner calls this synchronous fixed-width access.
    unsafe { registers.field_mut::<safe_mmio::fields::WriteOnly<u32>>(MDER_OFFSET) }.write(Mder(0).0);
}

impl Lpi2cRegisters<Off> {
    pub(in super::super) fn disable_requests(&mut self) {
        requests_off(self);
    }
}

impl Lpi2cRegisters<Enabled> {
    pub(in super::super) fn disable_requests(&mut self) {
        requests_off(self);
    }
}

impl Lpi2cRegisters<Streaming> {
    pub(in super::super) fn quiesce_with<E: Effects>(mut self, effects: E) -> (Lpi2cRegisters<Enabled>, E) {
        requests_off(&mut self);
        effects.quiesce();
        fence(Ordering::Acquire);

        // Deliberately concrete: preserve the exact owner's mapping and retained
        // handles, never reacquire uniqueness or mint a detached quiescence token.
        // Adding a new retained field makes this construction fail to compile
        // until its move is reviewed here. There is no arbitrary-state cast.
        let enabled = Lpi2cRegisters {
            base: self.base,
            __read_pure_msr: self.__read_pure_msr,
            __read_pure_mfsr: self.__read_pure_mfsr,
            state: PhantomData,
            _not_send_sync: PhantomData,
        };
        (enabled, effects)
    }
}
