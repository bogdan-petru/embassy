//! Confirmed local state, not a write-only typestate assertion.
//!
//! The production reset algorithm runs over these individual safe-mmio effects.
//! Only a non-streaming owner can enter it; the caller must first quiesce DMA.
//! Reset is abrupt and bounded, never a peer-dependent graceful MEN-off wait.
#![forbid(unsafe_code)]

use core::marker::PhantomData;

use crate::i2c::reset_sequence::{self, ResetError, Role};

use super::{ControlIo, Enabled, Lpi2cRegisters, Off, State, Unconfirmed};

// Private helper; no public generic state cast or Streaming reset shortcut.
fn reset<S: State>(mut registers: Lpi2cRegisters<S>, now: impl Fn() -> u64) -> Result<Lpi2cRegisters<Off>, ResetError> {
    let mut io = ControlIo {
        registers: &mut registers,
        now,
    };
    // The borrow ends only after reset assertion AND release were observed.
    drop(reset_sequence::reset(&mut io, Role::Controller, false)?);
    Ok(Lpi2cRegisters {
        base: registers.base,
        __read_pure_msr: registers.__read_pure_msr,
        __read_pure_mfsr: registers.__read_pure_mfsr,
        state: PhantomData,
        _not_send_sync: PhantomData,
    })
}

impl Lpi2cRegisters<Unconfirmed> {
    pub(in super::super) fn reset_confirmed(self, now: impl Fn() -> u64) -> Result<Lpi2cRegisters<Off>, ResetError> {
        reset(self, now)
    }
}

impl Lpi2cRegisters<Enabled> {
    pub(in super::super) fn reset_confirmed(self, now: impl Fn() -> u64) -> Result<Lpi2cRegisters<Off>, ResetError> {
        reset(self, now)
    }
}

impl Lpi2cRegisters<Off> {
    pub(in super::super) fn enable_confirmed(
        mut self,
        now: impl Fn() -> u64,
    ) -> Result<Lpi2cRegisters<Enabled>, ResetError> {
        let mut io = ControlIo {
            registers: &mut self,
            now,
        };
        reset_sequence::confirm_disabled(&mut io, Role::Controller)?.enable()?;
        Ok(Lpi2cRegisters {
            base: self.base,
            __read_pure_msr: self.__read_pure_msr,
            __read_pure_mfsr: self.__read_pure_mfsr,
            state: PhantomData,
            _not_send_sync: PhantomData,
        })
    }
}
