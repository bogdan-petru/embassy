//! Safe target-side operations over the LPI2C register block.
//!
//! Same two-layer split as [`super::controller_registers`]: Tock cells
//! decide access direction, the PAC's value types define every field.
//! And the same closed-vocabulary design — the flag-priority decisions
//! that the two-board hardware tests caught being made wrongly at call
//! sites live here, in exactly one place each:
//!
//! - [`TargetRegisters::rx_event`]: pending RX **data drains before**
//!   faults and before STOP / repeated-START are honored. Firmware
//!   entering late (delayed ISR on a busy system) must not drop bytes
//!   that arrived before the controller terminated the transfer.
//! - [`TargetRegisters::tx_step`]: **faults and termination win over**
//!   TX-space — pushing a byte into a transfer that already ended, or
//!   that has faulted, is never right.
//! - [`TargetRegisters::listen_ready`]: faults are part of the wake
//!   condition, because their interrupts are armed.
//!
//! Call sites consume typed events and cannot reorder these checks; the
//! wrapper exposes no raw status accessor to reorder them with.

use tock_registers::interfaces::{Readable, Writeable};

use super::lpi2c_regs::{self, LpI2cRegisters};
use crate::pac;
use crate::pac::lpi2c::{Scr, ScrRrf, ScrRtf, Sier, Srdr, Ssr, Stdr};

/// Hardware faults the target status register can report mid-transfer.
///
/// The interrupt masks enable BEIE/FEIE, so these MUST be part of every
/// readiness/event check: a fault that wakes the waiter without
/// surfacing as an event would re-arm and wake forever.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum TargetFault {
    /// Bit error: the target saw a bus level conflicting with what it
    /// was driving.
    Bit,
    /// FIFO error (receive overflow / transmit underflow).
    Fifo,
}

/// One step of target receive progress.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[must_use]
pub(super) enum TargetRxEvent {
    /// A byte the controller wrote, popped from the RX FIFO.
    Byte(u8),
    /// A fault; bytes already drained are valid, nothing further is.
    Fault(TargetFault),
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
    /// A fault; the transfer is compromised, push nothing more.
    Fault(TargetFault),
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

    fn ssr(&self) -> Ssr {
        Ssr(self.regs.ssr.get())
    }

    fn write_sier(&self, f: impl FnOnce(&mut Sier)) {
        let mut v = Sier(0);
        f(&mut v);
        self.regs.sier.set(v.0);
    }

    fn modify_scr(&self, f: impl FnOnce(&mut Scr)) {
        let mut v = Scr(self.regs.scr.get());
        f(&mut v);
        self.regs.scr.set(v.0);
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
        self.write_sier(|w| {
            w.set_feie(true);
            w.set_beie(true);
            w.set_sdie(true);
            w.set_rsie(true);
            w.set_rdie(true);
        });
    }

    /// Interrupts relevant while transmitting data (respond_to_read).
    pub(super) fn enable_transmit_interrupts(&self) {
        self.write_sier(|w| {
            w.set_feie(true);
            w.set_beie(true);
            w.set_sdie(true);
            w.set_rsie(true);
            w.set_tdie(true);
        });
    }

    pub(super) fn reset_fifos(&self) {
        critical_section::with(|_| {
            self.modify_scr(|w| {
                w.set_rtf(ScrRtf::NowEmpty);
                w.set_rrf(ScrRrf::NowEmpty);
            });
        });
    }

    /// One step of receive progress, in the only correct order: pending
    /// data drains first (bytes received before a fault or STOP are
    /// valid), then faults, then termination flags. Returns `None` to
    /// keep waiting. SDF/RSF/BEF/FEF are not consumed here; the respond
    /// flow's status lifecycle owns their clearing.
    pub(super) fn rx_event(&self) -> Option<TargetRxEvent> {
        let ssr = self.ssr();
        if ssr.rdf() {
            return Some(TargetRxEvent::Byte(Srdr(self.regs.srdr.get()).data()));
        }
        if ssr.bef() {
            return Some(TargetRxEvent::Fault(TargetFault::Bit));
        }
        if ssr.fef() {
            return Some(TargetRxEvent::Fault(TargetFault::Fifo));
        }
        if ssr.sdf() {
            return Some(TargetRxEvent::Stopped);
        }
        if ssr.rsf() {
            return Some(TargetRxEvent::Restarted);
        }
        None
    }

    /// Non-consuming readiness check for [`Self::rx_event`], for use in
    /// wake conditions.
    pub(super) fn rx_ready(&self) -> bool {
        let ssr = self.ssr();
        ssr.rdf() || ssr.sdf() || ssr.rsf() || ssr.bef() || ssr.fef()
    }

    /// One step of transmit progress: faults first (the transfer is
    /// compromised), then termination, then room.
    pub(super) fn tx_step(&self) -> Option<TargetTxStep> {
        let ssr = self.ssr();
        if ssr.bef() {
            return Some(TargetTxStep::Fault(TargetFault::Bit));
        }
        if ssr.fef() {
            return Some(TargetTxStep::Fault(TargetFault::Fifo));
        }
        if ssr.sdf() || ssr.rsf() {
            return Some(TargetTxStep::Ended);
        }
        if ssr.tdf() {
            return Some(TargetTxStep::Room);
        }
        None
    }

    /// Non-consuming readiness check for [`Self::tx_step`], for use in
    /// wake conditions.
    pub(super) fn tx_ready(&self) -> bool {
        let ssr = self.ssr();
        ssr.tdf() || ssr.sdf() || ssr.rsf() || ssr.bef() || ssr.fef()
    }

    /// Readiness for `listen`: any event that ends the wait for a new
    /// transaction — an address match (or its general-call / SMBus-alert
    /// classifications), a STOP, **or a fault**.
    ///
    /// BEF/FEF belong here because `enable_listen_ints` arms BEIE/FEIE:
    /// a fault that wakes the ISR but is not part of the wake condition
    /// leaves the waiter re-arming a still-latched, level-triggered
    /// source forever. The caller's status read classifies and clears
    /// them.
    pub(super) fn listen_ready(&self) -> bool {
        let ssr = self.ssr();
        ssr.avf() || ssr.sarf() || ssr.gcf() || ssr.sdf() || ssr.bef() || ssr.fef()
    }

    /// Push one byte into the target transmit register.
    pub(super) fn push_tx(&self, byte: u8) {
        let mut v = Stdr(0);
        v.set_data(byte);
        self.regs.stdr.set(v.0);
    }

    // DMA-transfer vocabulary. A DMA-driven transfer moves data
    // through the engine, so RDF/TDF are not firmware events — but
    // every OTHER decision (what wakes the waiter, how a finished
    // chunk is classified, in which priority co-latched flags are
    // honored) is the same decision the interrupt paths make, and it
    // is made HERE, once, so the two modes cannot drift. The
    // marooned-residue and FEF-before-BEF defects both lived in a DMA
    // path that re-made these decisions at the call site.

    /// Interrupts for a DMA-driven transfer: fault and termination
    /// sources only (RDF/TDF service the DMA engine, not firmware).
    /// The armed set equals [`Self::dma_transfer_event`]'s wake set.
    pub(super) fn enable_dma_transfer_interrupts(&self) {
        self.write_sier(|w| {
            w.set_feie(true);
            w.set_beie(true);
            w.set_sdie(true);
            w.set_rsie(true);
        });
    }

    /// Wake condition paired with
    /// [`Self::enable_dma_transfer_interrupts`]: any latched fault or
    /// termination. (DMA completion wakes through the channel's own
    /// cell and is checked separately.)
    pub(super) fn dma_transfer_event(&self) -> bool {
        let ssr = self.ssr();
        ssr.fef() || ssr.bef() || ssr.sdf() || ssr.rsf()
    }

    /// Drain RX residue into `data[from..]`, returning the new count.
    ///
    /// Routes through [`Self::rx_event`], so the drain inherits the
    /// vocabulary's order (data first) and its popping discipline. The
    /// loop ends at buffer capacity or on the first non-data event —
    /// which is NOT consumed, so the caller's classification still
    /// sees it.
    pub(super) fn drain_rx(&self, data: &mut [u8], mut from: usize) -> usize {
        while from < data.len() {
            match self.rx_event() {
                Some(TargetRxEvent::Byte(b)) => {
                    data[from] = b;
                    from += 1;
                }
                _ => break,
            }
        }
        from
    }

    /// Whether RX data is still pending — the full-chunk residue
    /// decision after a drain hit buffer capacity.
    pub(super) fn rx_pending(&self) -> bool {
        self.ssr().rdf()
    }

    /// Terminal classification of a DMA chunk, in the vocabulary's
    /// single flag-priority order: bit error before FIFO error
    /// (matching [`Self::rx_event`]/[`Self::tx_step`]), faults before
    /// termination. Consumes nothing.
    pub(super) fn chunk_end(&self) -> ChunkEnd {
        let ssr = self.ssr();
        if ssr.bef() {
            ChunkEnd::Fault(TargetFault::Bit)
        } else if ssr.fef() {
            ChunkEnd::Fault(TargetFault::Fifo)
        } else if ssr.sdf() {
            ChunkEnd::Stopped
        } else if ssr.rsf() {
            ChunkEnd::Restarted
        } else {
            ChunkEnd::Continue
        }
    }
}

/// Terminal classification of a DMA chunk transfer — see
/// [`TargetRegisters::chunk_end`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[must_use]
pub(super) enum ChunkEnd {
    /// A fault; the transfer is compromised.
    Fault(TargetFault),
    /// The controller issued a STOP.
    Stopped,
    /// The controller issued a repeated START.
    Restarted,
    /// No fault or termination latched.
    Continue,
}
