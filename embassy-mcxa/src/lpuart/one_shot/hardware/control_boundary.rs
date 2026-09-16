//! Reviewed state transitions around generated accessors. A state is returned
//! after its hardware handshake, never merely because the write was attempted.
#![forbid(unsafe_code)]

use core::marker::PhantomData;

use uart_one_shot_interface::Error;

use super::super::reset_sequence::{self, BaudConfig, Register, RegisterIo};
use super::{ControlIo, Enabled, Off, State, Streaming, UartRegisters, Unconfirmed};
use crate::pac::lpuart::{Fifo, Water};

fn reset<S: State>(mut registers: UartRegisters<S>, now: impl Fn() -> u64) -> Result<UartRegisters<Off>, Error> {
    reset_sequence::reset(&mut ControlIo {
        registers: &mut registers,
        now,
    })?;
    Ok(UartRegisters {
        base: registers.base,
        state: PhantomData,
        _not_send_sync: PhantomData,
    })
}

impl UartRegisters<Unconfirmed> {
    pub(in super::super) fn reset_confirmed(self, now: impl Fn() -> u64) -> Result<UartRegisters<Off>, Error> {
        reset(self, now)
    }
}

impl UartRegisters<Off> {
    pub(in super::super) fn configure_confirmed(
        mut self,
        timing: BaudConfig,
        now: impl Fn() -> u64,
    ) -> Result<UartRegisters<Enabled>, Error> {
        let mut fifo = Fifo(0);
        fifo.set_txfe(true);
        self.write_fifo(fifo.0);
        let mut water = Water(0);
        water.set_txwater(0);
        self.write_water(water.0);
        reset_sequence::enable(
            &mut ControlIo {
                registers: &mut self,
                now,
            },
            timing,
        )?;
        Ok(UartRegisters {
            base: self.base,
            state: PhantomData,
            _not_send_sync: PhantomData,
        })
    }
}

impl UartRegisters<Enabled> {
    pub(in super::super) fn reset_confirmed(self, now: impl Fn() -> u64) -> Result<UartRegisters<Off>, Error> {
        reset(self, now)
    }

    pub(in super::super) fn into_arm_dma(mut self, timing: BaudConfig) -> UartRegisters<Streaming> {
        ControlIo {
            registers: &mut self,
            now: || 0,
        }
        .write(Register::Baud, timing.word(true));
        UartRegisters {
            base: self.base,
            state: PhantomData,
            _not_send_sync: PhantomData,
        }
    }

    pub(in super::super) fn wire_complete(&mut self, enable_irq: impl FnOnce()) -> bool {
        reset_sequence::wire_complete(
            &mut ControlIo {
                registers: self,
                now: || 0,
            },
            enable_irq,
        )
    }

    pub(in super::super) fn disable_wire_interrupt(&mut self) {
        ControlIo {
            registers: self,
            now: || 0,
        }
        .write(Register::Ctrl, reset_sequence::control(true, false));
    }
}

impl UartRegisters<Streaming> {
    /// Only this paired exit can restore CPU authority: it runs peripheral
    /// request-off/confirmation and the actual owned channel's quiesce first.
    /// Kept inside the trusted L2 boundary, never offered as a public receipt.
    pub(in super::super) fn disable_dma_and_restore(
        mut self,
        timing: BaudConfig,
        quiesce: impl FnOnce() -> bool,
        now: impl Fn() -> u64,
    ) -> Result<UartRegisters<Enabled>, Error> {
        reset_sequence::stop_dma(
            &mut ControlIo {
                registers: &mut self,
                now,
            },
            timing,
            quiesce,
        )?;
        Ok(UartRegisters {
            base: self.base,
            state: PhantomData,
            _not_send_sync: PhantomData,
        })
    }
}
