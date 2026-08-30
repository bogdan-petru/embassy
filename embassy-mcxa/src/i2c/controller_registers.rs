//! Safe controller-side operations over the LPI2C register block.
//!
//! Two layers meet here. [`super::lpi2c_regs`] supplies `tock-registers`
//! MMIO cells, so access direction is enforced by type: `MRDR` is
//! read-only and popping, `MTDR` is write-only. The PAC supplies every
//! field *meaning* — each raw word is converted through the PAC's own
//! value type (`Msr`, `Mier`, `Cmd`, …), so bit positions and
//! enumerated values are defined exactly once, in the generated code.
//!
//! The API is a deliberately *closed vocabulary*: there is no bare
//! "FIFO empty" predicate to wait on. Every wait primitive couples data
//! or space readiness with the error flags, so a driver loop that spins
//! forever on a halted transfer — the bug class the two-board hardware
//! tests found in all three read paths — cannot be expressed against
//! this interface.

use tock_registers::interfaces::{Readable, Writeable};

use super::lpi2c_regs::{self, LpI2cRegisters};
use crate::pac;
pub(super) use crate::pac::lpi2c::Cmd as ControllerCommand;
use crate::pac::lpi2c::{
    Alf, Dmf, Epf, Mbf, Mcfgr1, Mcr, McrRrf, McrRtf, Mder, Mfsr, Mier, Mrdr, Msr, MsrFef, MsrSdf, Mtdr, Ndf, Param,
    Pltf, Stf,
};

/// A typed snapshot of the controller error flags relevant to transfers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct ControllerStatus {
    error: Option<ControllerStatusError>,
}

impl ControllerStatus {
    fn from_snapshot(msr: &Msr) -> Self {
        // Priority mirrors the hardware relevance order: an address NACK
        // explains everything after it, arbitration loss next, FIFO
        // error last.
        let error = if msr.ndf() == Ndf::IntYes {
            Some(ControllerStatusError::AddressNack)
        } else if msr.alf() == Alf::IntYes {
            Some(ControllerStatusError::ArbitrationLoss)
        } else if msr.fef() == MsrFef::IntYes {
            Some(ControllerStatusError::Fifo)
        } else if msr.pltf() == Pltf::IntYes {
            // Fires only when MCFGR3[PINLOW] is configured, but the
            // interrupt masks enable PLTIE, so it must be part of the
            // fault set: an unsurfaced wake-without-error would loop
            // the waiter forever.
            Some(ControllerStatusError::PinLowTimeout)
        } else {
            None
        };

        Self { error }
    }

    pub(super) fn error(self) -> Option<ControllerStatusError> {
        self.error
    }
}

/// Controller errors represented by the hardware status register.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum ControllerStatusError {
    AddressNack,
    ArbitrationLoss,
    Fifo,
    PinLowTimeout,
}

/// One step of receive progress. Data drains before faults surface:
/// bytes that arrived before an error are valid and must not be lost.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[must_use]
pub(super) enum RxStep {
    Byte(u8),
    Fault(ControllerStatusError),
    /// The transfer terminated with no data pending and no fault
    /// flagged. Observed on FRDM-MCXA577 during chained multi-command
    /// reads under interrupt-latency stress: the transfer ends mid-read
    /// with the remaining queued commands discarded — the same silicon
    /// quirk family as the spurious arbitration loss. Without this
    /// variant the reader waits forever for bytes that never arrive.
    Ended,
}

/// One step of transmit-space progress. Faults surface before space:
/// pushing more commands into a halted transfer would never complete.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[must_use]
pub(super) enum TxStep {
    Room,
    Fault(ControllerStatusError),
}

/// Safe controller-specific operations over the LPI2C register block.
pub(super) struct ControllerRegisters {
    regs: &'static LpI2cRegisters,
}

impl ControllerRegisters {
    pub(super) fn new(regs: pac::lpi2c::Lpi2c) -> Self {
        Self {
            regs: lpi2c_regs::from_pac(regs),
        }
    }

    // Raw word <-> PAC value type. Field meanings live in the PAC; the
    // Tock cells only decide who may read and who may write.

    fn msr(&self) -> Msr {
        Msr(self.regs.msr.get())
    }

    fn mfsr(&self) -> Mfsr {
        Mfsr(self.regs.mfsr.get())
    }

    fn write_msr(&self, v: Msr) {
        self.regs.msr.set(v.0);
    }

    fn write_mier(&self, f: impl FnOnce(&mut Mier)) {
        let mut v = Mier(0);
        f(&mut v);
        self.regs.mier.set(v.0);
    }

    fn modify_mcr(&self, f: impl FnOnce(&mut Mcr)) {
        let mut v = Mcr(self.regs.mcr.get());
        f(&mut v);
        self.regs.mcr.set(v.0);
    }

    fn modify_mder(&self, f: impl FnOnce(&mut Mder)) {
        let mut v = Mder(self.regs.mder.get());
        f(&mut v);
        self.regs.mder.set(v.0);
    }

    /// Disable the controller interrupt mask if any source is enabled.
    ///
    /// Returns whether the driver should wake its waiter.
    pub(super) fn disable_interrupts_if_enabled(&self) -> bool {
        if self.regs.mier.get() == 0 {
            return false;
        }

        self.regs.mier.set(0);
        true
    }

    /// Enable only the error interrupt sources (NACK, arbitration loss,
    /// FIFO error, pin-low timeout). Used while DMA moves the data, where
    /// TDF/RDF service the DMA engine but an error still needs to wake
    /// the waiting task.
    pub(super) fn enable_error_interrupts(&self) {
        // Note: deliberately no EPIE/SDIE. Both are level-latched and
        // polluted by the silicon's spurious-flag quirks (false STOP
        // detection; EPF from repeated STARTs), so arming them storms
        // the waker. Silent early termination is handled by bounded
        // waits in the driver instead.
        self.write_mier(|w| {
            w.set_ndie(true);
            w.set_alie(true);
            w.set_feie(true);
            w.set_pltie(true);
        });
    }

    /// Arm the transmit-path interrupt set and evaluate its wake
    /// condition, as ONE operation: the armed set and the predicate
    /// are defined together so they cannot drift apart (an armed
    /// source outside the wake set re-arms and interrupts forever —
    /// the listen-side RSIE mismatch was exactly this class).
    pub(super) fn tx_settle_wake(&self) -> bool {
        self.enable_transmit_interrupts();
        self.tx_settled()
    }

    /// Arm the receive-path interrupt set and evaluate its wake
    /// condition, as one operation — see [`Self::tx_settle_wake`].
    pub(super) fn rx_wake(&self) -> bool {
        self.enable_receive_interrupts();
        self.rx_ready()
    }

    /// Arm the error interrupt set (for DMA-driven transfers, where
    /// TDF/RDF service the engine) and report any latched error, as
    /// one operation — see [`Self::tx_settle_wake`].
    pub(super) fn error_wake(&self) -> Option<ControllerStatusError> {
        self.enable_error_interrupts();
        self.read_status().error()
    }

    pub(super) fn enable_receive_interrupts(&self) {
        // No EPIE/SDIE — see `enable_error_interrupts`.
        self.write_mier(|w| {
            w.set_rdie(true);
            w.set_ndie(true);
            w.set_alie(true);
            w.set_feie(true);
            w.set_pltie(true);
        });
    }

    pub(super) fn enable_transmit_interrupts(&self) {
        self.write_mier(|w| {
            w.set_tdie(true);
            w.set_ndie(true);
            w.set_alie(true);
            w.set_feie(true);
            w.set_pltie(true);
        });
    }

    pub(super) fn reset_fifos(&self) {
        critical_section::with(|_| {
            self.modify_mcr(|w| {
                w.set_rtf(McrRtf::Reset);
                w.set_rrf(McrRrf::Reset);
            });
        });
    }

    pub(super) fn clear_all_status(&self) {
        let mut v = Msr(0);
        v.set_epf(Epf::IntYes);
        v.set_sdf(MsrSdf::IntYes);
        v.set_ndf(Ndf::IntYes);
        v.set_alf(Alf::IntYes);
        v.set_fef(MsrFef::IntYes);
        v.set_pltf(Pltf::IntYes);
        v.set_dmf(Dmf::IntYes);
        v.set_stf(Stf::IntYes);
        self.write_msr(v);
    }

    /// Read and clear one coherent status snapshot.
    ///
    /// MSR flags are write-one-to-clear. Writing the sampled snapshot
    /// back clears only flags observed by this read, avoiding a
    /// read/clear race with a flag that arrives after the snapshot.
    pub(super) fn take_status(&self) -> ControllerStatus {
        let msr = self.msr();
        self.write_msr(msr);
        ControllerStatus::from_snapshot(&msr)
    }

    pub(super) fn read_status(&self) -> ControllerStatus {
        ControllerStatus::from_snapshot(&self.msr())
    }

    pub(super) fn clear_current_status(&self) {
        let msr = self.msr();
        self.write_msr(msr);
    }

    /// Reference Manual 40.7.1.5: after an address NACK, a STOP must be
    /// sent by software when automatic STOP generation is disabled and
    /// nothing else is queued that would terminate the transfer.
    pub(super) fn needs_manual_stop_after_nack(&self) -> bool {
        !Mcfgr1(self.regs.mcfgr1.get()).autostop() && self.mfsr().txcount() == 0
    }

    /// One step of receive progress: a received byte (popped from the RX
    /// FIFO), a fault that means no more data will arrive, an early
    /// termination, or `None` to keep waiting. There is deliberately no
    /// data-only variant of this wait — see the module docs.
    pub(super) fn rx_step(&self) -> Option<RxStep> {
        if self.mfsr().rxcount() != 0 {
            return Some(RxStep::Byte(Mrdr(self.regs.mrdr.get()).data()));
        }
        if let Some(e) = self.read_status().error() {
            return Some(RxStep::Fault(e));
        }
        if self.transfer_ended() {
            return Some(RxStep::Ended);
        }
        None
    }

    /// Non-consuming readiness check for [`Self::rx_step`], for use in
    /// wake conditions. True when a byte, a fault, or an early transfer
    /// termination is pending.
    pub(super) fn rx_ready(&self) -> bool {
        self.mfsr().rxcount() != 0 || self.read_status().error().is_some() || self.transfer_ended()
    }

    /// True when this controller has ended the packet (EPF) and gone
    /// idle (MBF clear) — i.e. the transfer terminated for real.
    ///
    /// Deliberately NOT based on SDF: the same silicon quirk that raises
    /// spurious ArbitrationLoss is a false STOP *detection*, so SDF can
    /// latch mid-transfer while the read continues perfectly well
    /// (observed with MSB-rich data). EPF is set only by this master's
    /// own end-of-packet, and the MBF guard rejects any transient state
    /// while a transfer is still active. Flags from a *previous*
    /// transaction are cleared by the `take_status` inside the START's
    /// own status check.
    pub(super) fn transfer_ended(&self) -> bool {
        let msr = self.msr();
        msr.epf() == Epf::IntYes && msr.mbf() == Mbf::Idle
    }

    /// Capacity of the shared command/transmit FIFO, in entries.
    pub(super) fn tx_fifo_capacity(&self) -> usize {
        1usize << Param(self.regs.param.get()).mtxfifo()
    }

    /// True when the command FIFO has fully drained *or* a fault is
    /// pending — i.e. the wait for a queued command is over, one way or
    /// the other. Callers classify with `take_status`/`read_status`.
    pub(super) fn tx_settled(&self) -> bool {
        self.mfsr().txcount() == 0 || self.read_status().error().is_some()
    }

    /// One step of transmit-space progress: room in the command FIFO, a
    /// fault that means the transfer is dead, or `None` to keep waiting.
    pub(super) fn tx_room_step(&self) -> Option<TxStep> {
        if let Some(e) = self.read_status().error() {
            return Some(TxStep::Fault(e));
        }
        if (self.mfsr().txcount() as usize) < self.tx_fifo_capacity() {
            return Some(TxStep::Room);
        }
        None
    }

    /// Push a typed controller command into the transmit FIFO.
    pub(super) fn write_command(&self, command: ControllerCommand, data: u8) {
        #[cfg(feature = "defmt")]
        defmt::trace!(
            "Sending cmd '{}' ({}) with data '{:02x}' MSR: {:08x}",
            command,
            command as u8,
            data,
            self.regs.msr.get()
        );

        let mut v = Mtdr(0);
        v.set_data(data);
        v.set_cmd(command);
        self.regs.mtdr.set(v.0);
    }

    // DMA plumbing: request enables and the FIFO data addresses the DMA
    // engine reads/writes directly.

    pub(super) fn set_rx_dma(&self, enable: bool) {
        self.modify_mder(|w| w.set_rdde(enable));
    }

    pub(super) fn set_tx_dma(&self, enable: bool) {
        self.modify_mder(|w| w.set_tdde(enable));
    }

    pub(super) fn rx_data_ptr(&self) -> *const u8 {
        &self.regs.mrdr as *const _ as *const u8
    }

    pub(super) fn tx_data_ptr(&self) -> *mut u8 {
        &self.regs.mtdr as *const _ as *mut u8
    }
}
