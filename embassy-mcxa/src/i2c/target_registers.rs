//! Safe target-side operations over the Tock-style LPI2C register map.
//!
//! Same closed-vocabulary design as [`super::controller_registers`], for
//! the target's transfer hot paths. The flag-priority decisions that the
//! two-board hardware tests caught being made wrongly at call sites live
//! here, in exactly one place each:
//!
//! - [`TargetRegisters::rx_event`]: pending RX **data drains before**
//!   STOP / repeated-START are honored. Firmware entering late (delayed
//!   ISR on a busy system) must not drop bytes that arrived before the
//!   controller terminated the transfer.
//! - [`TargetRegisters::tx_step`]: **termination wins over** TX-space —
//!   pushing a byte into a transfer that already ended is never right.
//!
//! Call sites consume typed events and cannot reorder these checks.

use tock_registers::interfaces::{ReadWriteable, Readable, Writeable};

use super::lpi2c_regs::{self, LpI2cRegisters, SCR, SIER, SRDR, SSR, STDR};
use crate::pac;

/// One step of target receive progress.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[must_use]
pub(super) enum TargetRxEvent {
    /// A byte the controller wrote, popped from the RX FIFO.
    Byte(u8),
    /// The controller issued a STOP (no data left pending).
    Stopped,
    /// The controller issued a repeated START (no data left pending).
    Restarted,
}

/// One step of target transmit progress.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[must_use]
pub(super) enum TargetTxStep {
    /// The transmit register has room for the next byte.
    Room,
    /// The transfer ended (STOP or repeated START); push nothing more.
    Ended,
}

/// Safe target-specific operations over the LPI2C register block.
pub(super) struct TargetRegisters {
    regs: &'static LpI2cRegisters,
}

impl TargetRegisters {
    pub(super) fn new(regs: pac::lpi2c::Lpi2c) -> Self {
        Self {
            regs: lpi2c_regs::from_pac(regs),
        }
    }

    /// Disable the target interrupt mask if any source is enabled.
    ///
    /// Returns whether the driver should wake its waiter.
    pub(super) fn disable_interrupts_if_enabled(&self) -> bool {
        if self.regs.sier.get() == 0 {
            return false;
        }

        self.regs.sier.set(0);
        true
    }

    /// Interrupts relevant while receiving data (respond_to_write).
    pub(super) fn enable_receive_interrupts(&self) {
        self.regs
            .sier
            .write(SIER::FEIE::SET + SIER::BEIE::SET + SIER::SDIE::SET + SIER::RSIE::SET + SIER::RDIE::SET);
    }

    /// Interrupts relevant while transmitting data (respond_to_read).
    pub(super) fn enable_transmit_interrupts(&self) {
        self.regs
            .sier
            .write(SIER::FEIE::SET + SIER::BEIE::SET + SIER::SDIE::SET + SIER::RSIE::SET + SIER::TDIE::SET);
    }

    pub(super) fn reset_fifos(&self) {
        critical_section::with(|_| {
            self.regs.scr.modify(SCR::RTF::SET + SCR::RRF::SET);
        });
    }

    /// One step of receive progress, in the only correct order: pending
    /// data first, then termination flags. Returns `None` to keep
    /// waiting. SDF/RSF are not consumed here; the respond flow's status
    /// lifecycle owns their clearing, exactly as before.
    pub(super) fn rx_event(&self) -> Option<TargetRxEvent> {
        let ssr = self.regs.ssr.extract();
        if ssr.is_set(SSR::RDF) {
            return Some(TargetRxEvent::Byte(self.regs.srdr.read(SRDR::DATA) as u8));
        }
        if ssr.is_set(SSR::SDF) {
            return Some(TargetRxEvent::Stopped);
        }
        if ssr.is_set(SSR::RSF) {
            return Some(TargetRxEvent::Restarted);
        }
        None
    }

    /// Non-consuming readiness check for [`Self::rx_event`], for use in
    /// wake conditions.
    pub(super) fn rx_ready(&self) -> bool {
        let ssr = self.regs.ssr.extract();
        ssr.is_set(SSR::RDF) || ssr.is_set(SSR::SDF) || ssr.is_set(SSR::RSF)
    }

    /// One step of transmit progress: termination first, then room.
    pub(super) fn tx_step(&self) -> Option<TargetTxStep> {
        let ssr = self.regs.ssr.extract();
        if ssr.is_set(SSR::SDF) || ssr.is_set(SSR::RSF) {
            return Some(TargetTxStep::Ended);
        }
        if ssr.is_set(SSR::TDF) {
            return Some(TargetTxStep::Room);
        }
        None
    }

    /// Non-consuming readiness check for [`Self::tx_step`], for use in
    /// wake conditions.
    pub(super) fn tx_ready(&self) -> bool {
        let ssr = self.regs.ssr.extract();
        ssr.is_set(SSR::TDF) || ssr.is_set(SSR::SDF) || ssr.is_set(SSR::RSF)
    }

    /// Push one byte into the target transmit register.
    pub(super) fn push_tx(&self, byte: u8) {
        self.regs.stdr.write(STDR::DATA.val(byte as u32));
    }
}
