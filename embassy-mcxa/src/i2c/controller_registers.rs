//! Safe controller-side operations over the Tock-style LPI2C register map.
//!
//! Built on [`super::lpi2c_regs`] (`tock-registers`), so register access
//! typing is enforced by construction: `MRDR` is read-only and popping,
//! `MTDR` is write-only, W1C status handling goes through
//! [`LocalRegisterCopy`] snapshots.
//!
//! The API is a deliberately *closed vocabulary*: there is no bare
//! "FIFO empty" predicate to wait on. Every wait primitive couples data
//! or space readiness with the error flags, so a driver loop that spins
//! forever on a halted transfer — the bug class the two-board hardware
//! tests found in all three read paths — cannot be expressed against
//! this interface.

use tock_registers::LocalRegisterCopy;
use tock_registers::interfaces::{ReadWriteable, Readable, Writeable};

use super::lpi2c_regs::{self, LpI2cRegisters, MCFGR1, MCR, MDER, MFSR, MIER, MRDR, MSR, MTDR, PARAM};
use crate::pac;

/// Commands for the controller transmit FIFO (MTDR\[CMD\]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u32)]
pub(super) enum ControllerCommand {
    Transmit = 0,
    Receive = 1,
    Stop = 2,
    Start = 4,
    StartHs = 6,
}

/// A typed snapshot of the controller error flags relevant to transfers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct ControllerStatus {
    error: Option<ControllerStatusError>,
}

impl ControllerStatus {
    fn from_snapshot(msr: &LocalRegisterCopy<u32, MSR::Register>) -> Self {
        // Priority mirrors the hardware relevance order: an address NACK
        // explains everything after it, arbitration loss next, FIFO
        // error last.
        let error = if msr.is_set(MSR::NDF) {
            Some(ControllerStatusError::AddressNack)
        } else if msr.is_set(MSR::ALF) {
            Some(ControllerStatusError::ArbitrationLoss)
        } else if msr.is_set(MSR::FEF) {
            Some(ControllerStatusError::Fifo)
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
}

/// One step of receive progress. Data drains before faults surface:
/// bytes that arrived before an error are valid and must not be lost.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[must_use]
pub(super) enum RxStep {
    Byte(u8),
    Fault(ControllerStatusError),
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
        self.regs
            .mier
            .write(MIER::NDIE::SET + MIER::ALIE::SET + MIER::FEIE::SET + MIER::PLTIE::SET);
    }

    pub(super) fn enable_receive_interrupts(&self) {
        self.regs
            .mier
            .write(MIER::RDIE::SET + MIER::NDIE::SET + MIER::ALIE::SET + MIER::FEIE::SET + MIER::PLTIE::SET);
    }

    pub(super) fn enable_transmit_interrupts(&self) {
        self.regs
            .mier
            .write(MIER::TDIE::SET + MIER::NDIE::SET + MIER::ALIE::SET + MIER::FEIE::SET + MIER::PLTIE::SET);
    }

    pub(super) fn reset_fifos(&self) {
        critical_section::with(|_| {
            self.regs.mcr.modify(MCR::RTF::SET + MCR::RRF::SET);
        });
    }

    pub(super) fn clear_all_status(&self) {
        self.regs.msr.write(
            MSR::EPF::SET
                + MSR::SDF::SET
                + MSR::NDF::SET
                + MSR::ALF::SET
                + MSR::FEF::SET
                + MSR::PLTF::SET
                + MSR::DMF::SET
                + MSR::STF::SET,
        );
    }

    /// Read and clear one coherent status snapshot.
    ///
    /// MSR flags are write-one-to-clear. Writing the sampled snapshot
    /// back clears only flags observed by this read, avoiding a
    /// read/clear race with a flag that arrives after the snapshot.
    pub(super) fn take_status(&self) -> ControllerStatus {
        let msr = self.regs.msr.extract();
        self.regs.msr.set(msr.get());
        ControllerStatus::from_snapshot(&msr)
    }

    pub(super) fn read_status(&self) -> ControllerStatus {
        ControllerStatus::from_snapshot(&self.regs.msr.extract())
    }

    pub(super) fn clear_current_status(&self) {
        let msr = self.regs.msr.extract();
        self.regs.msr.set(msr.get());
    }

    /// Reference Manual 40.7.1.5: after an address NACK, a STOP must be
    /// sent by software when automatic STOP generation is disabled and
    /// nothing else is queued that would terminate the transfer.
    pub(super) fn needs_manual_stop_after_nack(&self) -> bool {
        self.regs.mcfgr1.read(MCFGR1::AUTOSTOP) == 0 && self.regs.mfsr.read(MFSR::TXCOUNT) == 0
    }

    /// One step of receive progress: a received byte (popped from the RX
    /// FIFO), a fault that means no more data will arrive, or `None` to
    /// keep waiting. There is deliberately no data-only variant of this
    /// wait — see the module docs.
    pub(super) fn rx_step(&self) -> Option<RxStep> {
        if self.regs.mfsr.read(MFSR::RXCOUNT) != 0 {
            return Some(RxStep::Byte(self.regs.mrdr.read(MRDR::DATA) as u8));
        }
        if let Some(e) = self.read_status().error() {
            return Some(RxStep::Fault(e));
        }
        None
    }

    /// Non-consuming readiness check for [`Self::rx_step`], for use in
    /// wake conditions. True when a byte or a fault is pending.
    pub(super) fn rx_ready(&self) -> bool {
        self.regs.mfsr.read(MFSR::RXCOUNT) != 0 || self.read_status().error().is_some()
    }

    /// One step of transmit-space progress: room in the command FIFO, a
    /// fault that means the transfer is dead, or `None` to keep waiting.
    pub(super) fn tx_room_step(&self) -> Option<TxStep> {
        if let Some(e) = self.read_status().error() {
            return Some(TxStep::Fault(e));
        }
        let size = 1u32 << self.regs.param.read(PARAM::MTXFIFO);
        if self.regs.mfsr.read(MFSR::TXCOUNT) < size {
            return Some(TxStep::Room);
        }
        None
    }

    /// True when the command FIFO has fully drained *or* a fault is
    /// pending — i.e. the wait for a queued command is over, one way or
    /// the other. Callers classify with `take_status`/`read_status`.
    pub(super) fn tx_settled(&self) -> bool {
        self.regs.mfsr.read(MFSR::TXCOUNT) == 0 || self.read_status().error().is_some()
    }

    /// Push a typed controller command into the transmit FIFO.
    pub(super) fn write_command(&self, command: ControllerCommand, data: u8) {
        #[cfg(feature = "defmt")]
        defmt::trace!(
            "Sending cmd {} with data '{:02x}' MSR: {:08x}",
            command,
            data,
            self.regs.msr.get()
        );

        self.regs
            .mtdr
            .write(MTDR::DATA.val(data as u32) + MTDR::CMD.val(command as u32));
    }

    // DMA plumbing: request enables and the FIFO data addresses the DMA
    // engine reads/writes directly.

    pub(super) fn set_rx_dma(&self, enable: bool) {
        self.regs.mder.modify(MDER::RDDE.val(enable as u32));
    }

    pub(super) fn set_tx_dma(&self, enable: bool) {
        self.regs.mder.modify(MDER::TDDE.val(enable as u32));
    }

    pub(super) fn rx_data_ptr(&self) -> *const u8 {
        &self.regs.mrdr as *const _ as *const u8
    }

    pub(super) fn tx_data_ptr(&self) -> *mut u8 {
        &self.regs.mtdr as *const _ as *mut u8
    }
}
