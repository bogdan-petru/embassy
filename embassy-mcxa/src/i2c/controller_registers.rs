//! PAC-backed register operations used by the LPI2C controller driver.
//!
//! This module intentionally does not define another register map. It keeps
//! `nxp-pac` as the MMIO implementation and gives the controller a small set of
//! typed, safe operations instead of exposing individual PAC reads and writes.

use crate::pac;
pub(super) use crate::pac::lpi2c::Cmd as ControllerCommand;
use crate::pac::lpi2c::{Alf, Dmf, Epf, McrRrf, McrRtf, Msr, MsrFef, MsrSdf, Ndf, Pltf, Stf};

/// A typed snapshot of the controller error flags relevant to transfers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct ControllerStatus {
    error: Option<ControllerStatusError>,
}

impl ControllerStatus {
    fn from_register(msr: &Msr) -> Self {
        Self::from_flags(
            msr.ndf() == Ndf::IntYes,
            msr.alf() == Alf::IntYes,
            msr.fef() == MsrFef::IntYes,
        )
    }

    fn from_flags(address_nack: bool, arbitration_loss: bool, fifo_error: bool) -> Self {
        let error = if address_nack {
            Some(ControllerStatusError::AddressNack)
        } else if arbitration_loss {
            Some(ControllerStatusError::ArbitrationLoss)
        } else if fifo_error {
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

/// Safe controller-specific operations over the PAC LPI2C register block.
pub(super) struct ControllerRegisters {
    regs: pac::lpi2c::Lpi2c,
}

impl ControllerRegisters {
    pub(super) fn new(regs: pac::lpi2c::Lpi2c) -> Self {
        Self { regs }
    }

    /// Disable the controller interrupt mask if any source is enabled.
    ///
    /// Returns whether the driver should wake its waiter.
    pub(super) fn disable_interrupts_if_enabled(&self) -> bool {
        if self.regs.mier().read().0 == 0 {
            return false;
        }

        self.regs.mier().write(|w| {
            w.set_tdie(false);
            w.set_rdie(false);
            w.set_epie(false);
            w.set_sdie(false);
            w.set_ndie(false);
            w.set_alie(false);
            w.set_feie(false);
            w.set_pltie(false);
            w.set_dmie(false);
            w.set_stie(false);
        });

        true
    }

    /// Enable only the error interrupt sources (NACK, arbitration loss,
    /// FIFO error, pin-low timeout). Used while DMA moves the data, where
    /// TDF/RDF service the DMA engine but an error still needs to wake
    /// the waiting task.
    pub(super) fn enable_error_interrupts(&self) {
        self.regs.mier().write(|w| {
            w.set_ndie(true);
            w.set_alie(true);
            w.set_feie(true);
            w.set_pltie(true);
        });
    }

    pub(super) fn enable_receive_interrupts(&self) {
        self.regs.mier().write(|w| {
            w.set_rdie(true);
            w.set_ndie(true);
            w.set_alie(true);
            w.set_feie(true);
            w.set_pltie(true);
        });
    }

    pub(super) fn enable_transmit_interrupts(&self) {
        self.regs.mier().write(|w| {
            w.set_tdie(true);
            w.set_ndie(true);
            w.set_alie(true);
            w.set_feie(true);
            w.set_pltie(true);
        });
    }

    pub(super) fn reset_fifos(&self) {
        critical_section::with(|_| {
            self.regs.mcr().modify(|w| {
                w.set_rtf(McrRtf::Reset);
                w.set_rrf(McrRrf::Reset);
            });
        });
    }

    pub(super) fn clear_all_status(&self) {
        self.regs.msr().write(|w| {
            w.set_epf(Epf::IntYes);
            w.set_sdf(MsrSdf::IntYes);
            w.set_ndf(Ndf::IntYes);
            w.set_alf(Alf::IntYes);
            w.set_fef(MsrFef::IntYes);
            w.set_pltf(Pltf::IntYes);
            w.set_dmf(Dmf::IntYes);
            w.set_stf(Stf::IntYes);
        });
    }

    /// Read and clear one coherent status snapshot.
    ///
    /// MSR flags are write-one-to-clear. Writing the sampled value back clears
    /// only flags observed by this read, avoiding a read/clear race with a flag
    /// that arrives after the snapshot.
    pub(super) fn take_status(&self) -> ControllerStatus {
        let msr = self.regs.msr().read();
        self.regs.msr().write(|w| *w = msr);
        ControllerStatus::from_register(&msr)
    }

    pub(super) fn read_status(&self) -> ControllerStatus {
        ControllerStatus::from_register(&self.regs.msr().read())
    }

    pub(super) fn clear_current_status(&self) {
        let msr = self.regs.msr().read();
        self.regs.msr().write(|w| *w = msr);
    }

    pub(super) fn automatic_stop_enabled(&self) -> bool {
        self.regs.mcfgr1().read().autostop()
    }

    pub(super) fn tx_fifo_full(&self) -> bool {
        let txfifo_size = 1 << self.regs.param().read().mtxfifo();
        self.regs.mfsr().read().txcount() == txfifo_size
    }

    pub(super) fn tx_fifo_empty(&self) -> bool {
        self.regs.mfsr().read().txcount() == 0
    }

    pub(super) fn rx_fifo_empty(&self) -> bool {
        self.regs.mfsr().read().rxcount() == 0
    }

    pub(super) fn read_data(&self) -> u8 {
        self.regs.mrdr().read().data()
    }

    /// Push a typed controller command into the transmit FIFO.
    pub(super) fn write_command(&self, command: ControllerCommand, data: u8) {
        #[cfg(feature = "defmt")]
        defmt::trace!(
            "Sending cmd '{}' ({}) with data '{:08x}' MSR: {:08x}",
            command,
            command as u8,
            data,
            self.regs.msr().read()
        );

        self.regs.mtdr().write(|w| {
            w.set_data(data);
            w.set_cmd(command);
        });
    }
}
