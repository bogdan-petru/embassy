//! # LPI2C Controller Driver
//!
//! This module provides a driver for the Low-Power Inter-Integrated
//! Circuit (LPI2C) controller, supporting blocking,
//! interrupt-only async, and DMA async modes of operation.
//!
//! The driver support all transfer speeds except for Fast Mode+.
//!
//! ## Features
//!
//! - **Blocking and Asynchronous Modes**: Supports both blocking and
//! async APIs for flexibility in different runtime environments.
//! - **DMA Support**: Enables high-performance data transfers using
//! DMA.
//! - **Configurable Bus Speeds**: Supports standard (100 kHz), fast
//! (400 kHz), and fast-plus (1 MHz) modes. Ultra-fast (3.4 MHz) mode
//! is not yet implemented.
//! - **Error Handling**: Comprehensive error reporting, including
//! FIFO errors, arbitration loss, and address NACK conditions.
//! - **Embedded HAL Compatibility**: Implements traits from
//! `embedded-hal` and `embedded-hal-async` for interoperability with
//! other libraries.
//!
//! ### Error Types
//!
//! - `SetupError`: Errors related to hardware initialization, such as
//! clock configuration issues.
//! - `IOError`: Errors during I2C operations, including FIFO errors,
//! arbitration loss, and invalid buffer lengths.
//!
//! ## Example
//!
//! ```rust,no_run
//! #![no_std]
//! #![no_main]
//!
//! # extern crate panic_halt;
//! # extern crate embassy_mcxa;
//! # extern crate embassy_executor;
//! # use panic_halt as _;
//! use embassy_executor::Spawner;
//! use embassy_mcxa::clocks::config::Div8;
//! use embassy_mcxa::config::Config;
//! use embassy_mcxa::i2c::controller::{self, I2c, Speed};
//!
//! #[embassy_executor::main]
//! async fn main(_spawner: Spawner) {
//!     let mut config = Config::default();
//!     config.clock_cfg.sirc.fro_lf_div = Div8::from_divisor(1);
//!
//!     let p = embassy_mcxa::init(config);
//!
//!     let mut i2c = I2c::new_blocking(p.LPI2C2, p.P1_9, p.P1_8, Default::default()).unwrap();
//!
//!     // Write data
//!     i2c.blocking_write(0x50, &[0x01, 0x02, 0x03]).unwrap();
//!
//!     // Read data
//!     let mut buffer = [0u8; 3];
//!     i2c.blocking_read(0x50, &mut buffer).unwrap();
//! }
//! ```

use core::future::Future;
use core::marker::PhantomData;

use embassy_hal_internal::Peri;
use embassy_hal_internal::drop::OnDrop;

use super::controller_registers::{
    ControllerCommand, ControllerRegisters, ControllerStatus, ControllerStatusError, RxStep, TxStep,
};
use super::{Async, AsyncMode, Blocking, Dma, Info, Instance, Mode, SclPin, SdaPin};
use crate::clocks::periph_helpers::{Div4, Lpi2cClockSel, Lpi2cConfig};
use crate::clocks::{ClockError, PoweredClock, WakeGuard, enable_and_reset};
use crate::dma::{Channel, DMA_MAX_TRANSFER_SIZE, DmaChannel, TransferOptions};
use crate::gpio::{AnyPin, SealedPin};
use crate::interrupt;
use crate::interrupt::typelevel::Interrupt;
use crate::pac::lpi2c::{Dozen, Prescale};

/// Errors exclusive to HW initialization
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum SetupError {
    /// Clock configuration error.
    ClockSetup(ClockError),
    /// Other internal errors or unexpected state.
    Other,
}

/// I/O Errors
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum IOError {
    /// FIFO Error, the command in the FIFO queue expected the controller to be in a STARTed state, but it was not.
    ///
    /// Even though a START could have been issued earlier, the controller might now be in a different state.
    /// For example, a NAK condition was detected and the controller automatically issued a STOP.
    FifoError,
    /// Reading for I2C failed.
    ReadFail,
    /// Writing to I2C failed.
    WriteFail,
    /// I2C address NAK condition.
    AddressNack,
    /// Bus level arbitration loss.
    ArbitrationLoss,
    /// SCL or SDA held low longer than the configured pin-low timeout
    /// (MCFGR3\[PINLOW\]).
    PinLowTimeout,
    /// The transfer terminated early: a STOP/end-of-packet latched with
    /// received bytes still owed and no fault flagged. Observed on
    /// FRDM-MCXA577 under interrupt-latency stress during chained
    /// multi-command reads; same spurious-flag silicon family as the
    /// ArbitrationLoss quirk.
    ///
    /// The transfer is already broken on the wire. Whether re-reading is
    /// safe depends on the device (a cursor has already advanced;
    /// destructive reads have already consumed data), so the driver does
    /// not retry on its own — see [`Config::allow_chunked_reads`].
    UnexpectedStop,
    /// The bus made no forward progress within
    /// [`Config::transfer_timeout`] — e.g. a target clock-stretching
    /// longer than the configured budget, or a transfer that died with
    /// no status flag at all.
    Timeout,
    /// The requested read cannot be performed as a single atomic bus
    /// transaction, and non-atomic chunking is not enabled.
    ///
    /// Raised by the DMA path for reads longer than the command FIFO can
    /// hold in RECEIVE commands (nothing refills the FIFO while the CPU
    /// sleeps on the DMA completion). Use a shorter read, an
    /// interrupt-driven `I2c<Async>`, or opt in via
    /// [`Config::allow_chunked_reads`].
    ChunkingRequired,
    /// Address out of range.
    AddressOutOfRange(u8),
    /// Invalid write buffer length.
    InvalidWriteBufferLength,
    /// Invalid read buffer length.
    InvalidReadBufferLength,
    /// Other internal errors or unexpected state.
    Other,
}

impl From<crate::dma::InvalidParameters> for IOError {
    fn from(_value: crate::dma::InvalidParameters) -> Self {
        IOError::Other
    }
}

impl From<ControllerStatusError> for IOError {
    fn from(value: ControllerStatusError) -> Self {
        match value {
            ControllerStatusError::AddressNack => IOError::AddressNack,
            ControllerStatusError::ArbitrationLoss => IOError::ArbitrationLoss,
            ControllerStatusError::Fifo => IOError::FifoError,
            ControllerStatusError::PinLowTimeout => IOError::PinLowTimeout,
        }
    }
}

/// I2C interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        T::PERF_INT_INCR();
        let registers = ControllerRegisters::new(T::info().regs());
        if registers.disable_interrupts_if_enabled() {
            T::PERF_INT_WAKE_INCR();
            T::info().wait_cell().wake();
        }
    }
}

/// Bus speed (nominal SCL, no clock stretching)
#[derive(Clone, Copy, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Speed {
    #[default]
    /// 100 kbit/sec
    Standard,
    /// 400 kbit/sec
    Fast,
    /// 1 Mbit/sec
    FastPlus,
    /// 3.4 Mbit/sec
    UltraFast,
}

impl From<Speed> for u32 {
    fn from(val: Speed) -> Self {
        match val {
            Speed::Standard => 100_000,
            Speed::Fast => 400_000,
            Speed::FastPlus => 1_000_000,
            Speed::UltraFast => 3_400_000,
        }
    }
}

/// Compute LPI2C controller MCFGR1.PRESCALE + MCCR0 fields from peripheral
/// input frequency and target SCL frequency.
///
/// Mirrors the NXP SDK `LPI2C_MasterSetBaudRate` algorithm
/// (see `fsl_lpi2c.c`). For each prescaler 0..=7, computes the period
/// in periph cycles using round-to-nearest division, picks the one with
/// the smallest absolute error to the target. Then derives:
///   - CLKHI = (clkCycle - SCL_LATENCY) / 2, clamped down so that
///     tBUF >= 0.52/baud.
///   - CLKLO = clkCycle - CLKHI.
///   - SETHOLD = clk_bdr/divider/2 - 1   (~half SCL period).
///   - DATAVD  = clk_bdr/divider/4 - 1   (~quarter SCL period).
///
/// Where SCL_LATENCY = (2 + FILTSCL) / 2^prescale and we assume FILTSCL=0
/// (we do not program MCFGR2 in this driver).
fn compute_baud_params(src_hz: u32, baud_hz: u32) -> (Prescale, u8, u8, u8, u8) {
    let filt_scl: u32 = 0;

    let prescalers = [
        Prescale::DivideBy1,
        Prescale::DivideBy2,
        Prescale::DivideBy4,
        Prescale::DivideBy8,
        Prescale::DivideBy16,
        Prescale::DivideBy32,
        Prescale::DivideBy64,
        Prescale::DivideBy128,
    ];

    let (best_prescale, best_div, best_clk_cycle, _) = prescalers.iter().fold(
        (Prescale::DivideBy1, 1u32, 0u32, u32::MAX),
        |best @ (_, _, _, best_err), &prescale| {
            let divider: u32 = 1u32 << (prescale as u8);
            let scl_lat = (2 + filt_scl) / divider;

            // a = round(src / divider / baud)
            let a = (10 * src_hz / divider / baud_hz + 5) / 10;
            let b = scl_lat + 2;
            if a <= b {
                return best;
            }
            let clk_cycle = a - b;
            if clk_cycle > 120u32.saturating_sub(scl_lat) {
                return best;
            }

            let computed = (src_hz / divider) / (clk_cycle + 2 + scl_lat);
            let abs_err = computed.abs_diff(baud_hz);
            if abs_err < best_err {
                (prescale, divider, clk_cycle, abs_err)
            } else {
                best
            }
        },
    );

    let scl_lat = (2 + filt_scl) / best_div;
    let mut tmp_high = best_clk_cycle.saturating_sub(scl_lat) / 2;

    // Clamp tmp_high so tBUF >= 0.52 * SCL period:
    //   CLKHI <= clkCycle - 0.52*src/baud/divider + 1
    let a_tbuf = 13 * src_hz / baud_hz / best_div / 25;
    let max_high = best_clk_cycle.saturating_sub(a_tbuf).saturating_add(1);
    if tmp_high > max_high {
        tmp_high = max_high;
    }

    let clk_bdr = src_hz / baud_hz;
    let tmp_hold = (clk_bdr / best_div / 2).saturating_sub(1);
    let tmp_datavd = (clk_bdr / best_div / 4).saturating_sub(1);

    let clkhi = (tmp_high & 0x3F) as u8;
    let clklo = ((best_clk_cycle - tmp_high) & 0x3F) as u8;
    let sethold = (tmp_hold & 0x3F) as u8;
    let datavd = (tmp_datavd & 0xFF) as u8;

    (best_prescale, clklo, clkhi, sethold, datavd)
}

/// Proof that a START was issued and its transaction is open on the
/// wire — a transaction token. Produced only by `start`/`async_start`;
/// consumed either by `stop`/`async_stop` or by the NEXT start (a
/// repeated START surrenders the prior transaction's token to mint
/// its successor). The engines are split into `*_txn` operations,
/// which leave the transaction open and hand the token back, and
/// `*_close` operations, which consume it with a trailing STOP and
/// return nothing.
///
/// Being precise about what each tier enforces:
///
/// - **Compile-enforced**: a driver-initiated trailing stop cannot be
///   issued without a token (`remediation()`'s recovery STOP is
///   outside the token discipline by design — it is cleanup, not
///   protocol); no operation both ends a transaction and yields a
///   token (a path that promises a STOP has nothing to leak); a token
///   cannot be used twice (no `Copy`/`Clone` — moves are checked).
/// - **Runtime-enforced**: the token carries its controller's
///   register-block address, and every consumption asserts it — a
///   token from a different instance fails deterministically instead
///   of silently continuing the wrong bus's transaction.
/// - **Advisory**: the token is not lifetime-branded to the controller
///   (branding would forbid holding it across the `&mut self`
///   operations a transaction chain is made of), so abandoning one —
///   an explicit drop, or passing `None` where a live continuation
///   exists — remains expressible. `#[must_use]` makes that a
///   visible, deliberate act (an error under this repo's
///   `-Dwarnings` builds) rather than an accident.
///
/// No `Drop` impl, deliberately: cleanup on abandonment
/// (drop-cancellation, error unwind) stays with the hardware-
/// validated OnDrop guards and the self-recovering start/stop arms —
/// the token encodes ORDER and IDENTITY, not cleanup.
#[must_use]
struct Started {
    /// The owning controller's register-block address.
    block: usize,
}

/// I2C controller configuration
#[derive(Clone, Copy)]
#[non_exhaustive]
pub struct Config {
    /// Bus speed
    pub speed: Speed,

    /// Clock configuration
    pub clock_config: ClockConfig,

    /// Allow a read to be split into several bus transactions.
    ///
    /// **Off by default, and unsafe for many devices.** Reads longer
    /// than one RECEIVE command (256 bytes) are normally issued as a
    /// single addressed transaction with chained commands. Two
    /// situations cannot be served that way:
    ///
    /// * the DMA path cannot chain more commands than the transmit FIFO
    ///   holds, because nothing refills it while the CPU sleeps on the
    ///   DMA completion;
    /// * the silicon can terminate a chained read early and silently
    ///   (see [`IOError::UnexpectedStop`]).
    ///
    /// With this disabled the driver reports [`IOError::ChunkingRequired`]
    /// or the underlying error and leaves the decision to the caller.
    /// With it enabled the driver falls back to re-addressed,
    /// STOP-separated 256-byte chunks, which:
    ///
    /// * releases the bus between chunks, so another controller may
    ///   interleave and disturb the device;
    /// * re-reads from the caller's buffer start after a partial
    ///   transfer, so a device with an auto-incrementing pointer returns
    ///   **shifted data**, and a destructive read (FIFO pop,
    ///   clear-on-read) has already lost the bytes consumed by the
    ///   partial attempt.
    ///
    /// Only enable it for stateless-read devices on a single-controller
    /// bus.
    pub allow_chunked_reads: bool,

    /// Budget for forward progress within a transfer: the wait for one
    /// received byte, for the command FIFO to drain, or for a DMA
    /// transfer to complete (scaled by length).
    ///
    /// Exceeding it yields [`IOError::Timeout`] rather than hanging.
    /// I2C places no upper bound on clock stretching, so raise this for
    /// devices that stretch heavily (EEPROM write cycles, slow ADC
    /// conversions).
    pub transfer_timeout: embassy_time::Duration,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            speed: Speed::default(),
            clock_config: ClockConfig::default(),
            // Atomic transactions by default; opting out is a deliberate
            // per-device decision.
            allow_chunked_reads: false,
            transfer_timeout: embassy_time::Duration::from_millis(100),
        }
    }
}

/// I2C controller clock configuration
#[derive(Clone, Copy)]
#[non_exhaustive]
pub struct ClockConfig {
    /// Powered clock configuration
    pub power: PoweredClock,
    /// LPI2C clock source
    pub source: Lpi2cClockSel,
    /// LPI2C pre-divider
    pub div: Div4,
}

impl Default for ClockConfig {
    fn default() -> Self {
        Self {
            power: PoweredClock::NormalEnabledDeepSleepDisabled,
            source: Lpi2cClockSel::FroLfDiv,
            div: const { Div4::no_div() },
        }
    }
}

/// I2C Controller Driver.
pub struct I2c<'d, M: Mode> {
    info: &'static Info,
    _scl: Peri<'d, AnyPin>,
    _sda: Peri<'d, AnyPin>,
    mode: M,
    is_hs: bool,
    /// See [`Config::allow_chunked_reads`].
    allow_chunked_reads: bool,
    /// See [`Config::transfer_timeout`].
    timeout: embassy_time::Duration,
    /// Peripheral input clock frequency in Hz, captured at construction.
    /// Used to compute MCCR0 timing parameters (e.g. when [`set_config`]
    /// changes the bus speed).
    freq: u32,
    _wg: Option<WakeGuard>,
}

impl<'d> I2c<'d, Blocking> {
    /// Creates a new blocking instance of the I2C Controller bus driver.
    ///
    /// This method initializes the I2C controller in blocking mode, allowing
    /// synchronous read and write operations.  The I2C bus is configured based
    /// on the provided `Config` structure, which specifies parameters such as
    /// bus speed and clock settings.
    ///
    /// # Arguments
    ///
    /// - `peri`: The peripheral instance representing the I2C controller hardware.
    /// - `scl`: The pin to be used for the I2C clock line (SCL).
    /// - `sda`: The pin to be used for the I2C data line (SDA).
    /// - `config`: A `Config` structure specifying the desired I2C configuration, including bus speed and clock settings.
    ///
    /// # Returns
    ///
    /// - `Ok(Self)`: A new instance of the I2C driver in blocking mode if initialization is successful.
    /// - `Err(SetupError)`: An error if the initialization fails, such as due to invalid clock configuration.
    ///
    /// # Behavior
    ///
    /// - The I2C controller is configured and enabled based on the provided `Config`.
    /// - Any external pins used for SCL and SDA will be placed into a disabled state when the driver instance is dropped.
    ///
    /// # Errors
    ///
    /// - `SetupError::ClockSetup`: If there is an issue with the clock configuration.
    /// - `SetupError::Other`: For other unexpected initialization errors.
    pub fn new_blocking<T: Instance>(
        peri: Peri<'d, T>,
        scl: Peri<'d, impl SclPin<T>>,
        sda: Peri<'d, impl SdaPin<T>>,
        config: Config,
    ) -> Result<Self, SetupError> {
        Self::new_inner(peri, scl, sda, config, Blocking)
    }
}

impl<'d, M: Mode> I2c<'d, M> {
    #[inline(always)]
    fn registers(&self) -> ControllerRegisters {
        ControllerRegisters::new(self.info.regs())
    }

    fn new_inner<T: Instance>(
        _peri: Peri<'d, T>,
        scl: Peri<'d, impl SclPin<T>>,
        sda: Peri<'d, impl SdaPin<T>>,
        config: Config,
        mode: M,
    ) -> Result<Self, SetupError> {
        let ClockConfig { power, source, div } = config.clock_config;

        // Enable clocks
        let conf = Lpi2cConfig {
            power,
            source,
            div,
            instance: T::CLOCK_INSTANCE,
        };

        let parts = unsafe { enable_and_reset::<T>(&conf).map_err(SetupError::ClockSetup)? };

        scl.mux();
        sda.mux();

        let _scl = scl.into();
        let _sda = sda.into();

        let inst = Self {
            info: T::info(),
            _scl,
            _sda,
            mode,
            is_hs: config.speed == Speed::UltraFast,
            allow_chunked_reads: config.allow_chunked_reads,
            timeout: config.transfer_timeout,
            freq: parts.freq,
            _wg: parts.wake_guard,
        };

        inst.set_configuration(&config);

        Ok(inst)
    }

    fn set_configuration(&self, config: &Config) {
        // One-time cross-check of the Tock register map against the
        // PAC's generated accessors (catches layout drift in either).
        super::lpi2c_regs::check_layout(self.info.regs());

        // Disable the controller.
        critical_section::with(|_| self.info.regs().mcr().modify(|w| w.set_men(false)));

        // Soft-reset the controller, read and write FIFOs.
        self.reset_fifos();
        critical_section::with(|_| {
            self.info.regs().mcr().modify(|w| w.set_rst(true));
            // According to Reference Manual section 40.7.1.4, "There
            // is no minimum delay required before clearing the
            // software reset", therefore we clear it immediately.
            self.info.regs().mcr().modify(|w| w.set_rst(false));

            self.info.regs().mcr().modify(|w| {
                w.set_dozen(Dozen::Enabled);
                w.set_dbgen(false);
            });
        });

        let target_hz: u32 = config.speed.into();
        // UltraFast (HS) mode requires programming MCCR1 and special start
        // commands beyond what this driver currently supports. Leave it
        // explicitly unimplemented until the HS path is wired up end-to-end.
        if config.speed == Speed::UltraFast {
            todo!("LPI2C UltraFast (HS) mode is not yet supported");
        }
        let (prescale, clklo, clkhi, sethold, datavd) = compute_baud_params(self.freq, target_hz);

        critical_section::with(|_| {
            self.info.regs().mcfgr1().modify(|w| w.set_prescale(prescale));
            self.info.regs().mccr0().modify(|w| {
                w.set_clklo(clklo);
                w.set_clkhi(clkhi);
                w.set_sethold(sethold);
                w.set_datavd(datavd);
            });

            // Enable the controller.
            self.info.regs().mcr().modify(|w| w.set_men(true));
        });

        // Clear all flags.
        self.registers().clear_all_status();
    }

    fn remediation(&self) {
        #[cfg(feature = "defmt")]
        defmt::trace!("Recovering controller",);

        // Recovery must not re-enter the fault-aware wait paths that call
        // it (`wait_tx_room` -> `status_and_act` -> here): with a fault
        // that keeps re-latching, that cycle recurses until the stack
        // overflows. Everything below is self-contained.
        //
        // Reset the FIFOs first: this drops whatever the aborted transfer
        // left queued and guarantees room for the recovery STOP. Then
        // clear any still-latched fault *before* queueing the STOP —
        // `tx_settled` below exits on any error flag, and a stale fault
        // would trip it immediately, after which the trailing FIFO reset
        // would discard the STOP before it ever reached the bus, leaving
        // the transaction active. (Callers that arrive via
        // `status_and_act` already cleared the flags; callers that arrive
        // via a non-clearing `read_status` fault have not.)
        self.reset_fifos();
        self.registers().clear_all_status();
        self.registers().write_command(ControllerCommand::STOP, 0);

        // Wait for the STOP to be consumed. `tx_settled` also returns on
        // a fault, but a target holding SCL low satisfies neither
        // condition, so the wait is bounded like every other: recovery
        // must not be the one path that can still hang. Either way the
        // STOP may still sit in the TX FIFO, ready to confuse the next
        // transaction, so reset again to guarantee a clean slate.
        let deadline = embassy_time::Instant::now() + self.timeout;
        while !self.registers().tx_settled() {
            if embassy_time::Instant::now() > deadline {
                #[cfg(feature = "defmt")]
                defmt::warn!("recovery STOP did not drain within the transfer timeout");
                break;
            }
        }
        self.reset_fifos();

        // Clear any residual MSR flags raised by the recovery STOP
        // (FEF in particular) so the next transaction starts clean.
        self.registers().clear_current_status();
    }

    /// The register-block address identifying this controller for
    /// transaction-token binding.
    fn block_addr(&self) -> usize {
        self.info.regs().as_ptr() as usize
    }

    /// Consume a transaction token, asserting it belongs to THIS
    /// controller. Cross-instance tokens compile (the token is not
    /// lifetime-branded — see [`Started`]) but are a severe protocol
    /// violation, so they fail deterministically here.
    fn consume_token(&self, open: Started) {
        let Started { block } = open;
        assert!(
            block == self.block_addr(),
            "i2c: transaction token from a different controller instance"
        );
    }

    /// Resets both TX and RX FIFOs dropping their contents.
    fn reset_fifos(&self) {
        self.registers().reset_fifos();
    }

    /// Recover from an I2C error by resetting FIFOs and clearing all
    /// status flags.  Without this, a NACK or FIFO error leaves the
    /// LPI2C controller in a state where every subsequent transaction
    /// fails with FifoError.
    fn recover_from_error(&self) {
        self.reset_fifos();
        self.registers().clear_all_status();
    }

    /// Blocking wait for the command FIFO to drain (or a fault to
    /// appear), bounded by [`Config::transfer_timeout`]: a target
    /// holding SCL low satisfies neither condition and would otherwise
    /// spin forever.
    fn wait_tx_settled(&self) -> Result<(), IOError> {
        let deadline = embassy_time::Instant::now() + self.timeout;
        while !self.registers().tx_settled() {
            if embassy_time::Instant::now() > deadline {
                return Err(IOError::Timeout);
            }
        }
        Ok(())
    }

    /// Blocking wait for room in the command FIFO, honoring faults: a
    /// halted transfer never frees space, so a data-only wait here would
    /// spin forever. On a fault, the standard classification/recovery
    /// path runs before the error is returned.
    fn wait_tx_room(&self) -> Result<(), IOError> {
        let deadline = embassy_time::Instant::now() + self.timeout;
        loop {
            if embassy_time::Instant::now() > deadline {
                return Err(IOError::Timeout);
            }
            match self.registers().tx_room_step() {
                Some(TxStep::Room) => return Ok(()),
                Some(TxStep::Fault(_)) => {
                    let res = self.status_and_act();
                    if let Err(e) = res {
                        if matches!(
                            e,
                            IOError::ArbitrationLoss | IOError::FifoError | IOError::PinLowTimeout
                        ) {
                            self.remediation();
                        }
                        return Err(e);
                    }
                    // The flag cleared concurrently; keep waiting.
                }
                None => {}
            }
        }
    }

    /// Parses the controller status producing an
    /// appropriate `Result<(), Error>` variant.
    fn parse_status(&self, status: ControllerStatus) -> Result<(), IOError> {
        match status.error() {
            Some(e) => Err(e.into()),
            None => Ok(()),
        }
    }

    /// Reads, parses and clears the controller status producing an
    /// appropriate `Result<(), Error>` variant.
    ///
    /// Will also send a STOP command if the tx_fifo is empty.
    fn status_and_act(&self) -> Result<(), IOError> {
        let status = self.parse_status(self.registers().take_status());

        if let Err(IOError::AddressNack) = status {
            // According to the Reference Manual, section 40.7.1.5
            // Controller Status (MSR), the controller will
            // automatically send a STOP condition if
            // `MCFGR1[AUTOSTOP]` is enabled or if the transmit FIFO
            // is *not* empty.
            //
            // If neither of those conditions is true, we will send a
            // STOP ourselves.
            if self.registers().needs_manual_stop_after_nack() {
                self.remediation();
            }
        }

        status
    }

    /// Inserts the given command into the outgoing FIFO.
    ///
    /// Caller must ensure there is space in the FIFO for the new
    /// command.
    fn send_cmd(&self, command: ControllerCommand, data: u8) {
        self.registers().write_command(command, data);
    }

    /// Prepares an appropriate Start condition on bus by issuing a
    /// `Start` command together with the device address and R/w bit.
    ///
    /// Blocks waiting for space in the FIFO to become available, then
    /// sends the command and blocks waiting for the FIFO to become
    /// empty ensuring the command was sent.
    fn start(&self, address: u8, read: bool, continues: Option<Started>) -> Result<Started, IOError> {
        // A repeated START surrenders the prior transaction's token
        // and mints the successor atomically (verified to belong to
        // this controller); a fresh START passes `None`.
        if let Some(open) = continues {
            self.consume_token(open);
        }
        if address >= 0x80 {
            return Err(IOError::AddressOutOfRange(address));
        }

        // Wait until we have space in the TxFIFO
        if let Err(e) = self.wait_tx_room() {
            // The fault classes were already remediated inside
            // `wait_tx_room`; a timeout means whatever clogged the
            // FIFO is still queued and must be dropped, or the next
            // transaction trips over it.
            if matches!(e, IOError::Timeout) {
                self.remediation();
            }
            return Err(e);
        }

        let addr_rw = address << 1 | if read { 1 } else { 0 };
        self.send_cmd(
            if self.is_hs {
                ControllerCommand::START_HS
            } else {
                ControllerCommand::START
            },
            addr_rw,
        );

        // Wait for TxFIFO to be drained. Timeout is its only failure:
        // the START is still queued behind a stretched clock — drop it.
        if let Err(e) = self.wait_tx_settled() {
            self.remediation();
            return Err(e);
        }

        // Check controller status
        let res = self.status_and_act();

        // `status_and_act` recovers a NACKed start itself, but a start
        // that failed with arbitration loss or a FIFO error leaves the
        // controller halted with the aborted commands still queued; a
        // subsequent transfer would then spin forever waiting for data
        // that never arrives. Recover here so a failed start always
        // returns with the controller in a clean state.
        if matches!(
            res,
            Err(IOError::ArbitrationLoss) | Err(IOError::FifoError) | Err(IOError::PinLowTimeout)
        ) {
            self.remediation();
        }

        res.map(|()| Started {
            block: self.block_addr(),
        })
    }

    /// Prepares a Stop condition on the bus.
    ///
    /// Analogous to `start`, this blocks waiting for space in the
    /// FIFO to become available, then sends the command and blocks
    /// waiting for the FIFO to become empty ensuring the command was
    /// sent.
    fn stop(&self, open: Started) -> Result<(), IOError> {
        self.consume_token(open);
        // Wait until we have space in the TxFIFO. Timeout recovery
        // mirrors `start`: fault classes were remediated inside
        // `wait_tx_room`, a timeout leaves the clog queued.
        if let Err(e) = self.wait_tx_room() {
            if matches!(e, IOError::Timeout) {
                self.remediation();
            }
            return Err(e);
        }

        self.send_cmd(ControllerCommand::STOP, 0);

        // Wait for TxFIFO to be drained; on timeout the STOP is still
        // queued behind a stretched clock — drop it.
        if let Err(e) = self.wait_tx_settled() {
            self.remediation();
            return Err(e);
        }

        let res = self.status_and_act();

        // Mirror `start` (and `async_stop`): the fault classes can
        // leave the aborted STOP queued, and callers run this after
        // defusing their guards on the strength of "stop recovers its
        // own failures".
        if matches!(
            res,
            Err(IOError::ArbitrationLoss) | Err(IOError::FifoError) | Err(IOError::PinLowTimeout)
        ) {
            self.remediation();
        }

        res
    }

    /// Read leaving the transaction OPEN (no trailing STOP): hands the
    /// token back to the caller, which must thread it onward.
    fn blocking_read_txn(&self, address: u8, read: &mut [u8], continues: Option<Started>) -> Result<Started, IOError> {
        if read.is_empty() {
            return Err(IOError::InvalidReadBufferLength);
        }

        // A chained read that died mid-transfer leaves the device in an
        // unknown state: its pointer has advanced by however many bytes
        // were clocked out, and destructive reads have already consumed
        // them. Re-reading from the caller's buffer start would return
        // shifted data as success, so it happens only when the caller
        // has explicitly accepted that trade.
        let mut carry = continues;
        match self.blocking_read_chained(address, read, carry.take()) {
            Ok(open) => Ok(open),
            Err(e @ (IOError::UnexpectedStop | IOError::Timeout)) if self.allow_chunked_reads => {
                #[cfg(feature = "defmt")]
                defmt::trace!("chained read failed ({}); retrying chunked (opted in)", e);
                let _ = e;
                // The failed chained attempt consumed the continuation
                // (its START went on the wire) and its recovery closed
                // the bus; the fallback starts fresh.
                self.blocking_read_seamed(address, read)
            }
            Err(e) => Err(e),
        }
    }

    /// Read ending the transaction with a trailing STOP: consumes the
    /// token, returns nothing to leak.
    fn blocking_read_close(&self, address: u8, read: &mut [u8], continues: Option<Started>) -> Result<(), IOError> {
        let open = self.blocking_read_txn(address, read, continues)?;
        self.stop(open)
    }

    /// One read as a single addressed transaction with chained RECEIVE
    /// commands. Does not send the trailing STOP.
    fn blocking_read_chained(
        &self,
        address: u8,
        read: &mut [u8],
        continues: Option<Started>,
    ) -> Result<Started, IOError> {
        // NOTE: start() is outside the recovery guard below —
        // `status_and_act` inside it already remediates a NACK, and
        // remediating twice corrupts the controller state for the
        // next transaction (see the async path's OnDrop note).
        let open = self.start(address, true, continues)?;

        let total = read.len();
        // Mirror the async path's OnDrop: an error past this point
        // aborts mid-transaction, and without recovery the bus is
        // left in a state that fails the next transfer.
        let mut drain = || -> Result<(), IOError> {
            // Chain RECEIVE commands under the single address phase: the
            // controller ACKs across a command boundary only when the next
            // command is already queued (otherwise it NACKs and terminates
            // the read early), so the command pipeline is kept ahead of the
            // data. This preserves >256-byte reads as ONE bus transaction —
            // no repeated START (unreliable after the auto-NACK on this
            // silicon) and no STOP seams (which would break embedded-hal
            // transaction atomicity and let another controller interleave).
            let mut queued = 0usize;
            let mut drained = 0usize;
            let mut deadline = embassy_time::Instant::now() + self.timeout;
            while drained < total {
                // Top up the command pipeline whenever there is room.
                while queued < total {
                    match self.registers().tx_room_step() {
                        Some(TxStep::Room) => {
                            let chunk = (total - queued).min(256);
                            self.send_cmd(ControllerCommand::RECEIVE, (chunk - 1) as u8);
                            queued += chunk;
                        }
                        Some(TxStep::Fault(e)) => return Err(e.into()),
                        // Command FIFO full: plenty is queued ahead of
                        // the data; go drain some of it.
                        None => break,
                    }
                }

                // Receive one byte, or bail out on a fault (NACK,
                // arbitration loss, FIFO error): no more data will
                // arrive, and a data-only wait would spin forever.
                match self.registers().rx_step() {
                    Some(RxStep::Byte(b)) => {
                        read[drained] = b;
                        drained += 1;
                        deadline = embassy_time::Instant::now() + self.timeout;
                    }
                    Some(RxStep::Fault(e)) => return Err(e.into()),
                    Some(RxStep::Ended) => return Err(IOError::UnexpectedStop),
                    // No progress for a full timeout window: the
                    // transfer died without a flag.
                    None if embassy_time::Instant::now() > deadline => {
                        return Err(IOError::Timeout);
                    }
                    None => {}
                }
            }
            Ok(())
        };
        if let Err(e) = drain() {
            self.remediation();
            return Err(e);
        }

        Ok(open)
    }

    /// Fallback: one read as re-addressed 256-byte chunks, each ended
    /// with a STOP. Not atomic, but immune to the chained-boundary
    /// early-termination quirk. Does not send the trailing STOP.
    fn blocking_read_seamed(&self, address: u8, read: &mut [u8]) -> Result<Started, IOError> {
        let nchunks = read.len().div_ceil(256);
        // Carries the final chunk's transaction out of the loop; every
        // non-final chunk's is consumed by its seam STOP.
        // Seamed chunks always start fresh transactions.
        let mut last_open = None;
        for (idx, chunk) in read.chunks_mut(256).enumerate() {
            let open = self.start(address, true, None)?;

            // Outside the drain guard: `wait_tx_room` remediates the
            // fault classes itself, and routing it through the guard
            // would remediate those twice. Its Timeout arm does not.
            if let Err(e) = self.wait_tx_room() {
                if matches!(e, IOError::Timeout) {
                    self.remediation();
                }
                return Err(e);
            }

            let mut drain = || -> Result<(), IOError> {
                self.send_cmd(ControllerCommand::RECEIVE, (chunk.len() - 1) as u8);

                let mut deadline = embassy_time::Instant::now() + self.timeout;
                for byte in chunk.iter_mut() {
                    *byte = loop {
                        match self.registers().rx_step() {
                            Some(RxStep::Byte(b)) => {
                                deadline = embassy_time::Instant::now() + self.timeout;
                                break b;
                            }
                            Some(RxStep::Fault(e)) => return Err(e.into()),
                            Some(RxStep::Ended) => return Err(IOError::UnexpectedStop),
                            // No progress for a full timeout window:
                            // Timeout, like the chained and async
                            // paths — UnexpectedStop is reserved for
                            // an observed termination.
                            None if embassy_time::Instant::now() > deadline => {
                                return Err(IOError::Timeout);
                            }
                            None => {}
                        }
                    };
                }

                Ok(())
            };
            if let Err(e) = drain() {
                self.remediation();
                return Err(e);
            }

            // End every non-final chunk with a STOP; a repeated START
            // right after the auto-NACK of a consumed RECEIVE command
            // is not reliably accepted on this silicon. Outside the
            // drain guard: `stop` recovers its own failures, and the
            // guard must not stack a second remediation on top.
            if idx + 1 < nchunks {
                self.stop(open)?;
            } else {
                last_open = Some(open);
            }
        }
        // Non-empty reads are guaranteed by the caller, so the loop ran.
        Ok(last_open.expect("blocking_read_seamed called with an empty buffer"))
    }

    /// Write leaving the transaction OPEN — see `blocking_read_txn`.
    fn blocking_write_txn(&self, address: u8, write: &[u8], continues: Option<Started>) -> Result<Started, IOError> {
        let open = self.start(address, false, continues)?;

        // Usually, embassy HALs error out with an empty write,
        // however empty writes are useful for writing I2C scanning
        // logic through write probing. That is, we send a start with
        // R/w bit cleared, but instead of writing any data, just send
        // the stop onto the bus. This has the effect of checking if
        // the resulting address got an ACK but causing no
        // side-effects to the device on the other end.
        //
        // Because of this, we are not going to error out in case of
        // empty writes.
        if write.is_empty() {
            #[cfg(feature = "defmt")]
            defmt::trace!("Empty write, write probing?");
            return Ok(open);
        }

        // Mirror the read paths' recovery guard: an error mid-write
        // aborts the transaction with TRANSMIT commands still queued
        // and the bus held. `wait_tx_room` remediates the fault
        // classes itself; its Timeout arm does not.
        let push = || -> Result<(), IOError> {
            for byte in write {
                // Wait until we have space in the TxFIFO
                self.wait_tx_room()?;

                self.send_cmd(ControllerCommand::TRANSMIT, *byte);
            }
            Ok(())
        };
        if let Err(e) = push() {
            if matches!(e, IOError::Timeout) {
                self.remediation();
            }
            return Err(e);
        }

        Ok(open)
    }

    /// Write ending the transaction with a trailing STOP — see
    /// `blocking_read_close`.
    fn blocking_write_close(&self, address: u8, write: &[u8], continues: Option<Started>) -> Result<(), IOError> {
        let open = self.blocking_write_txn(address, write, continues)?;
        self.stop(open)
    }

    // Public API: Blocking

    /// Reads data from the specified I2C address into the provided buffer.
    ///
    /// This method blocks the caller until the operation is complete.
    ///
    /// # Arguments
    ///
    /// - `address`: The 7-bit I2C address of the target device.
    /// - `read`: A mutable buffer to store the data read from the device.
    ///
    /// # Returns
    ///
    /// - `Ok(())` if the read operation is successful.
    /// - `Err(IOError)` if an error occurs during the operation, such as an address NACK or FIFO error.
    ///
    /// # Errors
    ///
    /// - `IOError::AddressNack`: If the device does not acknowledge the address.
    /// - `IOError::FifoError`: If there is an issue with the FIFO queue.
    /// - Other variants of `IOError` for specific I2C errors.
    ///
    /// # Notes
    ///
    /// The driver will attempt to fill the buffer with data. If the
    /// buffer length exceeds the maximum transfer size of the
    /// controller, the read operation will be performed in multiple
    /// chunks. This will be transparent to the caller.
    pub fn blocking_read(&mut self, address: u8, read: &mut [u8]) -> Result<(), IOError> {
        self.blocking_read_close(address, read, None)
    }

    /// Writes data to the specified I2C address from the provided buffer.
    ///
    /// This method blocks the caller until the operation is complete.
    ///
    /// # Arguments
    ///
    /// - `address`: The 7-bit I2C address of the target device.
    /// - `write`: A buffer containing the data to be written to the device.
    ///
    /// # Returns
    ///
    /// - `Ok(())` if the write operation is successful.
    /// - `Err(IOError)` if an error occurs during the operation, such as an address NACK or FIFO error.
    ///
    /// # Errors
    ///
    /// - `IOError::AddressNack`: If the device does not acknowledge the address.
    /// - `IOError::FifoError`: If there is an issue with the FIFO queue.
    /// - Other variants of `IOError` for specific I2C errors.
    pub fn blocking_write(&mut self, address: u8, write: &[u8]) -> Result<(), IOError> {
        self.blocking_write_close(address, write, None)
    }

    /// Performs a combined write and read operation on the specified I2C
    /// address.
    ///
    /// This method first writes data to the device, then reads data from the
    /// device into the provided buffer.  The caller is blocked until the
    /// operation is complete.
    ///
    /// # Arguments
    ///
    /// - `address`: The 7-bit I2C address of the target device.
    /// - `write`: A buffer containing the data to be written to the device.
    /// - `read`: A mutable buffer to store the data read from the device.
    ///
    /// # Returns
    ///
    /// - `Ok(())` if the write-read operation is successful.
    /// - `Err(IOError)` if an error occurs during the operation, such as an address NACK or FIFO error.
    ///
    /// # Errors
    ///
    /// - `IOError::AddressNack`: If the device does not acknowledge the address.
    /// - `IOError::FifoError`: If there is an issue with the FIFO queue.
    /// - Other variants of `IOError` for specific I2C errors.
    pub fn blocking_write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), IOError> {
        // Reject a doomed read before the no-STOP write half touches
        // the bus — see `async_write_read`.
        if read.is_empty() {
            return Err(IOError::InvalidReadBufferLength);
        }
        let open = self.blocking_write_txn(address, write, None)?;
        // The read half's repeated START consumes the write half's
        // token; its trailing STOP closes the transaction.
        self.blocking_read_close(address, read, Some(open))
    }
}

#[allow(private_bounds)]
impl<'d, M: AsyncMode> I2c<'d, M>
where
    Self: AsyncEngine,
{
    /// Schedule sending a START command and await it being pulled from the FIFO.
    ///
    /// Does not indicate that the command was responded to.
    ///
    /// The wait is bounded by [`Config::transfer_timeout`] like its
    /// blocking counterpart: a target stretching SCL indefinitely
    /// satisfies neither the drain condition nor any error flag. A
    /// start that fails is returned with the controller recovered, and
    /// the awaited wait is guarded against drop-cancellation — callers
    /// arm their cancellation guards only *after* the start (see the
    /// seamed branch), so nobody else would clean up either kind of
    /// abort.
    async fn async_start(&self, address: u8, read: bool, continues: Option<Started>) -> Result<Started, IOError> {
        // A repeated START surrenders the prior transaction's token
        // and mints the successor atomically (verified to belong to
        // this controller); a fresh START passes `None`.
        if let Some(open) = continues {
            self.consume_token(open);
        }
        if address >= 0x80 {
            return Err(IOError::AddressOutOfRange(address));
        }

        // send the start command
        let addr_rw = address << 1 | if read { 1 } else { 0 };
        self.send_cmd(
            if self.is_hs {
                ControllerCommand::START_HS
            } else {
                ControllerCommand::START
            },
            addr_rw,
        );

        // Cancellation guard for THIS await: if the caller's future is
        // dropped here (an outer select/timeout), the queued START
        // still executes autonomously on the wire and opens a
        // transaction nothing would ever close. Defused the moment the
        // wait resolves — the arms below own their remediation, so
        // recovery stays exactly-once.
        let on_drop = OnDrop::new(|| self.remediation());

        let waited = embassy_time::with_timeout(
            self.timeout,
            self.info.wait_cell().wait_for(|| self.registers().tx_settle_wake()),
        )
        .await;
        on_drop.defuse();

        match waited {
            Ok(Ok(())) => {}
            Ok(Err(_)) => return Err(IOError::Other),
            Err(_) => {
                // The START never drained: drop it, or the next
                // transaction trips over the stale queued command.
                self.remediation();
                return Err(IOError::Timeout);
            }
        }

        // Note: the START + ACK/NACK have not necessarily been finished here.
        // thus this might return Ok(()), but might at a later state result in NAK or FifoError.
        let res = self.status_and_act();

        // Mirror the blocking `start`: `status_and_act` recovers a
        // NACKed start itself, but a start that failed with arbitration
        // loss or a FIFO error leaves the controller halted with the
        // aborted commands still queued.
        if matches!(
            res,
            Err(IOError::ArbitrationLoss) | Err(IOError::FifoError) | Err(IOError::PinLowTimeout)
        ) {
            self.remediation();
        }

        res.map(|()| Started {
            block: self.block_addr(),
        })
    }

    /// Schedule a STOP command and await it being pulled from the FIFO.
    ///
    /// Bounded like [`Self::async_start`], and like it fully
    /// self-recovering: a timeout means the STOP is stuck behind a
    /// stretched clock and remediation drops it, and the fault classes
    /// (arbitration loss, FIFO error, pin-low timeout) can leave the
    /// aborted STOP queued — `tx_settled` also exits on an error flag
    /// — so they remediate too. Callers rely on this: every seam and
    /// trailing stop runs AFTER its cancellation guard was defused
    /// (letting a still-armed guard also fire would run remediation
    /// twice, the double-recovery hazard the OnDrop placement notes
    /// warn about), so nobody else would clean up a failed stop. The
    /// awaited wait is guarded against drop-cancellation for the same
    /// reason — see `async_start`.
    async fn async_stop(&self, open: Started) -> Result<(), IOError> {
        self.consume_token(open);
        // send the stop command
        self.send_cmd(ControllerCommand::STOP, 0);

        // Cancellation guard for THIS await — see `async_start`. A
        // dropped stop mostly self-heals (the queued STOP executes
        // autonomously), but one stuck behind a stretched clock would
        // otherwise stay queued for the next transaction to trip over.
        let on_drop = OnDrop::new(|| self.remediation());

        let waited = embassy_time::with_timeout(
            self.timeout,
            self.info.wait_cell().wait_for(|| self.registers().tx_settle_wake()),
        )
        .await;
        on_drop.defuse();

        match waited {
            Ok(Ok(())) => {}
            Ok(Err(_)) => return Err(IOError::Other),
            Err(_) => {
                self.remediation();
                return Err(IOError::Timeout);
            }
        }

        let res = self.status_and_act();

        if matches!(
            res,
            Err(IOError::ArbitrationLoss) | Err(IOError::FifoError) | Err(IOError::PinLowTimeout)
        ) {
            self.remediation();
        }

        res
    }

    // Public API: Async

    /// Reads data from the specified I2C address into the provided buffer asynchronously.
    ///
    /// This method performs the read operation without blocking the caller,
    /// returning a `Future` that resolves when the operation is complete.
    ///
    /// # Arguments
    ///
    /// - `address`: The 7-bit I2C address of the target device.
    /// - `read`: A mutable buffer to store the data read from the device.
    ///
    /// # Returns
    ///
    /// - A `Future` that resolves to `Ok(())` if the read operation is successful.
    /// - Resolves to `Err(IOError)` if an error occurs during the operation, such as an address NACK or FIFO error.
    ///
    /// # Errors
    ///
    /// - `IOError::AddressNack`: If the device does not acknowledge the address.
    /// - `IOError::FifoError`: If there is an issue with the FIFO queue.
    /// - Other variants of `IOError` for specific I2C errors.
    pub async fn async_read(&mut self, address: u8, read: &mut [u8]) -> Result<(), IOError> {
        self.async_read_close(address, read, None).await
    }

    /// Writes data to the specified I2C address from the provided buffer asynchronously.
    ///
    /// This method performs the write operation without blocking the caller, returning a `Future` that resolves when the operation is complete.
    ///
    /// # Arguments
    ///
    /// - `address`: The 7-bit I2C address of the target device.
    /// - `write`: A buffer containing the data to be written to the device.
    ///
    /// # Returns
    ///
    /// - A `Future` that resolves to `Ok(())` if the write operation is successful.
    /// - Resolves to `Err(IOError)` if an error occurs during the operation, such as an address NACK or FIFO error.
    ///
    /// # Errors
    ///
    /// - `IOError::AddressNack`: If the device does not acknowledge the address.
    /// - `IOError::FifoError`: If there is an issue with the FIFO queue.
    /// - Other variants of `IOError` for specific I2C errors.
    pub async fn async_write(&mut self, address: u8, write: &[u8]) -> Result<(), IOError> {
        self.async_write_close(address, write, None).await
    }

    /// Performs a combined write and read operation on the specified I2C
    /// address asynchronously.
    ///
    /// This method first writes data to the device, then reads data from the
    /// device into the provided buffer. The operation is performed without
    /// blocking the caller.
    ///
    /// # Arguments
    ///
    /// - `address`: The 7-bit I2C address of the target device.
    /// - `write`: A buffer containing the data to be written to the device.
    /// - `read`: A mutable buffer to store the data read from the device.
    ///
    /// # Returns
    ///
    /// - `Ok(())` if the write-read operation is successful.
    /// - `Err(IOError)` if an error occurs during the operation, such as an address NACK or FIFO error.
    ///
    /// # Errors
    ///
    /// - `IOError::AddressNack`: If the device does not acknowledge the address.
    /// - `IOError::FifoError`: If there is an issue with the FIFO queue.
    /// - Other variants of `IOError` for specific I2C errors.
    pub async fn async_write_read<'a>(
        &'a mut self,
        address: u8,
        write: &'a [u8],
        read: &'a mut [u8],
    ) -> Result<(), IOError> {
        // Reject a doomed read BEFORE the no-STOP write half touches
        // the bus. The read-side preflight checks (empty buffer, DMA
        // command-FIFO capacity) run before any guard is armed, so a
        // rejection after the write would return with the transaction
        // still open and the bus held — nothing would ever send the
        // STOP.
        <Self as AsyncEngine>::read_preflight(self, read)?;
        let open = <Self as AsyncEngine>::async_write_txn(self, address, write, None).await?;
        // The read half's repeated START consumes the write half's
        // token; its trailing STOP closes the transaction.
        self.async_read_close(address, read, Some(open)).await
    }

    /// Read ending the transaction with a trailing STOP: consumes the
    /// token, returns nothing to leak.
    async fn async_read_close(
        &mut self,
        address: u8,
        read: &mut [u8],
        continues: Option<Started>,
    ) -> Result<(), IOError> {
        let open = <Self as AsyncEngine>::async_read_txn(self, address, read, continues).await?;
        self.async_stop(open).await
    }

    /// Write ending the transaction with a trailing STOP — see
    /// [`Self::async_read_close`].
    async fn async_write_close(
        &mut self,
        address: u8,
        write: &[u8],
        continues: Option<Started>,
    ) -> Result<(), IOError> {
        let open = <Self as AsyncEngine>::async_write_txn(self, address, write, continues).await?;
        self.async_stop(open).await
    }
}

trait AsyncEngine {
    /// Validate a read request before ANY bus activity. Combined
    /// transactions run this before their write half; the read paths
    /// run it at entry. Must stay side-effect free.
    fn read_preflight(&self, read: &[u8]) -> Result<(), IOError>;

    /// Read leaving the transaction OPEN (no trailing STOP): hands the
    /// token back to the caller, which must thread it onward. The
    /// close variants live on the driver (`async_read_close`).
    fn async_read_txn<'a>(
        &'a mut self,
        address: u8,
        read: &'a mut [u8],
        continues: Option<Started>,
    ) -> impl Future<Output = Result<Started, IOError>> + 'a;

    /// Write leaving the transaction OPEN — see [`Self::async_read_txn`].
    fn async_write_txn<'a>(
        &'a mut self,
        address: u8,
        write: &'a [u8],
        continues: Option<Started>,
    ) -> impl Future<Output = Result<Started, IOError>> + 'a;
}

impl<'d> I2c<'d, Async> {
    /// Creates a new interrupt-only asynchronous instance of the I2C Controller
    /// bus driver.
    ///
    /// This method initializes the I2C controller in asynchronous mode,
    /// enabling non-blocking operations using futures.  The I2C bus is
    /// configured based on the provided `Config` structure, which specifies
    /// parameters such as bus speed and clock settings.
    ///
    /// # Arguments
    ///
    /// - `peri`: The peripheral instance representing the I2C controller hardware.
    /// - `scl`: The pin to be used for the I2C clock line (SCL).
    /// - `sda`: The pin to be used for the I2C data line (SDA).
    /// - `_irq`: The interrupt binding for the I2C controller, ensuring that an interrupt handler is registered.
    /// - `config`: A `Config` structure specifying the desired I2C configuration, including bus speed and clock settings.
    ///
    /// # Returns
    ///
    /// - `Ok(Self)`: A new instance of the I2C driver in asynchronous mode if initialization is successful.
    /// - `Err(SetupError)`: An error if the initialization fails, such as due to invalid clock configuration.
    ///
    /// # Behavior
    ///
    /// - The I2C controller is configured and enabled based on the provided `Config`.
    /// - The interrupt for the I2C controller is enabled to support asynchronous operations.
    /// - Any external pins used for SCL and SDA will be placed into a disabled state when the driver instance is dropped.
    ///
    /// # Errors
    ///
    /// - `SetupError::ClockSetup`: If there is an issue with the clock configuration.
    /// - `SetupError::Other`: For other unexpected initialization errors.
    pub fn new_async<T: Instance>(
        peri: Peri<'d, T>,
        scl: Peri<'d, impl SclPin<T>>,
        sda: Peri<'d, impl SdaPin<T>>,
        _irq: impl crate::interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Result<Self, SetupError> {
        T::Interrupt::unpend();

        // Safety: `_irq` ensures an Interrupt Handler exists.
        unsafe { T::Interrupt::enable() };

        Self::new_inner(peri, scl, sda, config, Async)
    }
}

impl<'d> I2c<'d, Async> {
    /// One read as a single addressed transaction with chained RECEIVE
    /// commands. Does not send the trailing STOP.
    async fn async_read_chained(
        &self,
        address: u8,
        read: &mut [u8],
        continues: Option<Started>,
    ) -> Result<Started, IOError> {
        let open = self.async_start(address, true, continues).await?;

        // perform corrective action if the future is dropped or an
        // error happens between here and the end of the read.
        //
        // NOTE: this *must* be set up *after* async_start. async_start
        // already runs `status_and_act`, which on NACK performs its
        // own remediation; if we set OnDrop earlier, the early `?`
        // return would invoke remediation a second time and corrupt
        // the controller state for the next transaction.
        let on_drop = OnDrop::new(|| self.remediation());

        let total = read.len();
        // Chain RECEIVE commands under the single address phase: the
        // controller ACKs across a command boundary only when the next
        // command is already queued (otherwise it NACKs and terminates
        // the read early), so the command pipeline is kept ahead of the
        // data. This preserves >256-byte reads as ONE bus transaction —
        // no repeated START (unreliable after the auto-NACK on this
        // silicon) and no STOP seams (which would break embedded-hal
        // transaction atomicity and let another controller interleave).
        let mut queued = 0usize;
        let mut drained = 0usize;
        while drained < total {
            // Top up the command pipeline whenever there is room.
            while queued < total {
                match self.registers().tx_room_step() {
                    Some(TxStep::Room) => {
                        let chunk = (total - queued).min(256);
                        self.send_cmd(ControllerCommand::RECEIVE, (chunk - 1) as u8);
                        queued += chunk;
                    }
                    Some(TxStep::Fault(e)) => return Err(e.into()),
                    // Command FIFO full: plenty is queued ahead of the
                    // data; go drain some of it.
                    None => break,
                }
            }

            let timed_out = match embassy_time::with_timeout(
                self.timeout,
                self.info.wait_cell().wait_for(|| self.registers().rx_wake()),
            )
            .await
            {
                Ok(Ok(())) => false,
                Ok(Err(_)) => return Err(IOError::ReadFail),
                Err(_) => true,
            };

            match self.registers().rx_step() {
                Some(RxStep::Byte(b)) => {
                    read[drained] = b;
                    drained += 1;
                }
                // Surface the fault that woke us. If the flag cleared
                // in between, loop back and wait again.
                Some(RxStep::Fault(e)) => return Err(e.into()),
                Some(RxStep::Ended) => return Err(IOError::UnexpectedStop),
                // Nothing pending after a full timeout window: the
                // transfer stalled or died without a flag.
                None if timed_out => return Err(IOError::Timeout),
                None => {}
            }
        }

        on_drop.defuse();

        Ok(open)
    }

    /// Fallback: one read as re-addressed 256-byte chunks, each ended
    /// with a STOP. Not atomic, but immune to the chained-boundary
    /// early-termination quirk. Does not send the trailing STOP.
    async fn async_read_seamed(&self, address: u8, read: &mut [u8]) -> Result<Started, IOError> {
        let nchunks = read.len().div_ceil(256);
        // Carries the final chunk's transaction out of the loop; every
        // non-final chunk's is consumed by its seam STOP. Seamed
        // chunks always start fresh transactions.
        let mut last_open = None;
        for (idx, chunk) in read.chunks_mut(256).enumerate() {
            let open = self.async_start(address, true, None).await?;

            // See async_read_chained for the OnDrop placement rationale.
            let on_drop = OnDrop::new(|| self.remediation());

            self.send_cmd(ControllerCommand::RECEIVE, (chunk.len() - 1) as u8);

            for byte in chunk.iter_mut() {
                loop {
                    let timed_out = match embassy_time::with_timeout(
                        self.timeout,
                        self.info.wait_cell().wait_for(|| self.registers().rx_wake()),
                    )
                    .await
                    {
                        Ok(Ok(())) => false,
                        Ok(Err(_)) => return Err(IOError::ReadFail),
                        Err(_) => true,
                    };

                    match self.registers().rx_step() {
                        Some(RxStep::Byte(b)) => {
                            *byte = b;
                            break;
                        }
                        Some(RxStep::Fault(e)) => return Err(e.into()),
                        Some(RxStep::Ended) => return Err(IOError::UnexpectedStop),
                        None if timed_out => return Err(IOError::Timeout),
                        None => {}
                    }
                }
            }

            // The chunk's data is drained; defuse before the seam STOP,
            // which recovers its own failures (see `async_stop`).
            on_drop.defuse();

            // End every non-final chunk with a STOP; a repeated START
            // right after the auto-NACK of a consumed RECEIVE command is
            // not reliably accepted on this silicon.
            if idx + 1 < nchunks {
                self.async_stop(open).await?;
            } else {
                last_open = Some(open);
            }
        }
        // Non-empty reads are guaranteed by the preflight, so the loop ran.
        Ok(last_open.expect("async_read_seamed called with an empty buffer"))
    }
}

impl<'d> AsyncEngine for I2c<'d, Async> {
    fn read_preflight(&self, read: &[u8]) -> Result<(), IOError> {
        if read.is_empty() {
            return Err(IOError::InvalidReadBufferLength);
        }
        Ok(())
    }

    async fn async_read_txn(
        &mut self,
        address: u8,
        read: &mut [u8],
        continues: Option<Started>,
    ) -> Result<Started, IOError> {
        self.read_preflight(read)?;

        // A chained read that died mid-transfer leaves the device in an
        // unknown state: its pointer has advanced by however many bytes
        // were clocked out, and destructive reads have already consumed
        // them. Re-reading from the caller's buffer start would return
        // shifted data as success, so it happens only when the caller
        // has explicitly accepted that trade.
        let mut carry = continues;
        match self.async_read_chained(address, read, carry.take()).await {
            Ok(open) => Ok(open),
            Err(e @ (IOError::UnexpectedStop | IOError::Timeout)) if self.allow_chunked_reads => {
                #[cfg(feature = "defmt")]
                defmt::trace!("chained read failed ({}); retrying chunked (opted in)", e);
                let _ = e;
                // The failed chained attempt consumed the continuation
                // (its START went on the wire) and its recovery closed
                // the bus; the fallback starts fresh.
                self.async_read_seamed(address, read).await
            }
            Err(e) => Err(e),
        }
    }

    async fn async_write_txn(
        &mut self,
        address: u8,
        write: &[u8],
        continues: Option<Started>,
    ) -> Result<Started, IOError> {
        let open = self.async_start(address, false, continues).await?;

        // Usually, embassy HALs error out with an empty write,
        // however empty writes are useful for writing I2C scanning
        // logic through write probing. That is, we send a start with
        // R/w bit cleared, but instead of writing any data, just send
        // the stop onto the bus. This has the effect of checking if
        // the resulting address got an ACK but causing no
        // side-effects to the device on the other end.
        //
        // Because of this, we are not going to error out in case of
        // empty writes.
        if write.is_empty() {
            #[cfg(feature = "defmt")]
            defmt::trace!("Empty write, write probing?");
            return Ok(open);
        }

        // Corrective action if the future is dropped or a write step
        // fails. Armed only AFTER the empty-write return above: this
        // guard exists to close an *aborted* transaction, and the
        // empty-write path returns with its transaction either cleanly
        // stopped or deliberately left open for a combined
        // transaction's read half — a guard firing on that success
        // would inject a STOP into it.
        let on_drop = OnDrop::new(|| self.remediation());

        for byte in write {
            // initiate transmit
            self.send_cmd(ControllerCommand::TRANSMIT, *byte);

            match embassy_time::with_timeout(
                self.timeout,
                self.info.wait_cell().wait_for(|| self.registers().tx_settle_wake()),
            )
            .await
            {
                Ok(Ok(())) => {}
                Ok(Err(_)) => return Err(IOError::WriteFail),
                // No progress within the transfer timeout (a target
                // holding SCL low): the guard above closes the
                // transaction on this return.
                Err(_) => return Err(IOError::Timeout),
            }

            // NOT `status_and_act`: its NACK arm can run remediation
            // itself (manual STOP when the FIFO drained before the
            // NACK was observed), and the still-armed guard would then
            // remediate a second time on the error return. The guard
            // is the single remediator for every error class here —
            // remediation subsumes the manual-STOP special case.
            self.parse_status(self.registers().take_status())?;
        }

        // Nothing after the data loop needs the guard; the close
        // variant owns the trailing stop, which recovers its own
        // failures without stacking a second remediation.
        on_drop.defuse();

        Ok(open)
    }
}

impl<'d> I2c<'d, Dma<'d>> {
    /// Creates a new asynchronous instance of the I2C Controller bus driver with DMA support.
    ///
    /// This method initializes the I2C controller in asynchronous mode with
    /// Direct Memory Access (DMA) support, enabling efficient non-blocking
    /// operations for large data transfers.  The I2C bus is configured based on
    /// the provided `Config` structure, which specifies parameters such as bus
    /// speed and clock settings.
    ///
    /// # Arguments
    ///
    /// - `peri`: The peripheral instance representing the I2C controller hardware.
    /// - `scl`: The pin to be used for the I2C clock line (SCL).
    /// - `sda`: The pin to be used for the I2C data line (SDA).
    /// - `tx_dma`: The DMA channel to be used for transmitting data.
    /// - `rx_dma`: The DMA channel to be used for receiving data.
    /// - `_irq`: The interrupt binding for the I2C controller, ensuring that an interrupt handler is registered.
    /// - `config`: A `Config` structure specifying the desired I2C configuration, including bus speed and clock settings.
    ///
    /// # Returns
    ///
    /// - `Ok(Self)`: A new instance of the I2C driver in asynchronous mode with DMA support if initialization is successful.
    /// - `Err(SetupError)`: An error if the initialization fails, such as due to invalid clock configuration.
    ///
    /// # Behavior
    ///
    /// - The I2C controller is configured and enabled based on the provided `Config`.
    /// - The interrupt for the I2C controller is enabled to support asynchronous operations.
    /// - The specified DMA channels are initialized and their interrupts are enabled.
    /// - Any external pins used for SCL and SDA will be placed into a disabled state when the driver instance is dropped.
    ///
    /// # Errors
    ///
    /// - `SetupError::ClockSetup`: If there is an issue with the clock configuration.
    /// - `SetupError::Other`: For other unexpected initialization errors.
    pub fn new_async_with_dma<T: Instance>(
        peri: Peri<'d, T>,
        scl: Peri<'d, impl SclPin<T>>,
        sda: Peri<'d, impl SdaPin<T>>,
        tx_dma: Peri<'d, impl Channel>,
        rx_dma: Peri<'d, impl Channel>,
        _irq: impl crate::interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Result<Self, SetupError> {
        T::Interrupt::unpend();

        // Safety: `_irq` ensures an Interrupt Handler exists.
        unsafe { T::Interrupt::enable() };

        // enable this channel's interrupt
        let tx_dma = DmaChannel::new(tx_dma);
        let rx_dma = DmaChannel::new(rx_dma);

        tx_dma.enable_interrupt();
        rx_dma.enable_interrupt();

        Self::new_inner(
            peri,
            scl,
            sda,
            config,
            Dma {
                tx_dma,
                rx_dma,
                tx_request: T::TX_DMA_REQUEST,
                rx_request: T::RX_DMA_REQUEST,
            },
        )
    }
}

impl<'d> I2c<'d, Dma<'d>> {
    /// Run one RX DMA transfer covering `buf`, waking on completion or on
    /// a bus fault (which would otherwise leave the DMA waiting forever).
    /// The caller owns command queueing and recovery (OnDrop).
    async fn dma_read_into(&self, buf: &mut [u8]) -> Result<(), IOError> {
        let peri_addr = self.registers().rx_data_ptr();

        unsafe {
            // Clean up channel state
            self.mode.rx_dma.disable_request();
            self.mode.rx_dma.clear_done();
            self.mode.rx_dma.clear_interrupt();

            // Set DMA request source from instance type (type-safe)
            self.mode.rx_dma.set_request_source(self.mode.rx_request);

            // Configure TCD for peripheral-to-memory transfer
            self.mode
                .rx_dma
                .setup_read_from_peripheral(peri_addr, buf, false, TransferOptions::COMPLETE_INTERRUPT)?;

            // Enable I2C RX DMA request
            self.registers().set_rx_dma(true);

            // Enable DMA channel request
            self.mode.rx_dma.enable_request();
        }

        // Wait for completion asynchronously — or for a bus error
        // (NACK, arbitration loss, FIFO error) that stops the
        // transfer, in which case the DMA would never complete and
        // waiting on it alone would hang forever. The whole wait is
        // bounded: the silicon can terminate a transfer silently (no
        // flag, no interrupt), which only a timeout can catch.
        let wait = core::future::poll_fn(|cx| {
            // Drain any stale WOKEN token and finish REGISTERED:
            // `poll_wait` registers the waker only when it returns
            // Pending — discarding a `Ready` (a token left by a
            // cancellation racing a completion, which `quiesce` cannot
            // remove) would park this task with no waker on the cell,
            // and the real completion would then wake nobody. Same
            // pattern as `Transfer::poll`.
            while self.mode.rx_dma.wait_cell().poll_wait(cx).is_ready() {}
            if self.mode.rx_dma.is_done() {
                return core::task::Poll::Ready(Ok(()));
            }
            while self.info.wait_cell().poll_wait(cx).is_ready() {}
            // The interrupt handler disables MIER on wake; re-arm the
            // error sources and check, as one operation.
            if let Some(e) = self.registers().error_wake() {
                return core::task::Poll::Ready(Err(IOError::from(e)));
            }
            // Early termination with the DMA incomplete: no more data
            // will arrive and the DMA would wait forever.
            if self.registers().transfer_ended() {
                return core::task::Poll::Ready(Err(IOError::UnexpectedStop));
            }
            core::task::Poll::Pending
        });
        // ~1 ms/byte of margin on top of a generous floor.
        let bound = self.timeout + embassy_time::Duration::from_millis(buf.len() as u64);
        match embassy_time::with_timeout(bound, wait).await {
            Ok(r) => r?,
            Err(_) => return Err(IOError::Timeout),
        }

        // Ensure DMA writes are visible to CPU
        cortex_m::asm::dsb();
        // Cleanup: quiesce rather than merely disabling requests, so the
        // channel is provably idle before `buf`'s borrow ends.
        self.registers().set_rx_dma(false);
        self.mode.rx_dma.quiesce();

        Ok(())
    }
}

impl<'d> I2c<'d, Dma<'d>> {
    /// One read as a single addressed transaction: every RECEIVE command
    /// queued up front (caller checks they fit the FIFO), one DMA
    /// transfer over the whole buffer. Does not send the trailing STOP.
    async fn dma_read_chained(
        &self,
        address: u8,
        read: &mut [u8],
        continues: Option<Started>,
    ) -> Result<Started, IOError> {
        let open = self.async_start(address, true, continues).await?;

        // NOTE: OnDrop *after* async_start — see the seamed branch.
        let on_drop = OnDrop::new(|| {
            // Stop the DMA request path and wait for the channel to go
            // inactive before any recovery: remediation resets the
            // FIFOs, which can reassert the peripheral request, and
            // `read` is about to be released — an in-flight minor loop
            // would write into it after this future unwound.
            // `disable_request` alone does not wait for that loop.
            self.registers().set_rx_dma(false);
            self.mode.rx_dma.quiesce();
            self.remediation();
        });

        // Queue every RECEIVE command up front (they fit).
        let total = read.len();
        let mut queued = 0usize;
        while queued < total {
            match self.registers().tx_room_step() {
                Some(TxStep::Room) => {
                    let chunk = (total - queued).min(256);
                    self.send_cmd(ControllerCommand::RECEIVE, (chunk - 1) as u8);
                    queued += chunk;
                }
                Some(TxStep::Fault(e)) => return Err(e.into()),
                // Transiently full while the START drains.
                None => {}
            }
        }

        self.dma_read_into(read).await?;

        on_drop.defuse();
        Ok(open)
    }
}

impl<'d> AsyncEngine for I2c<'d, Dma<'d>> {
    fn read_preflight(&self, read: &[u8]) -> Result<(), IOError> {
        if read.is_empty() {
            return Err(IOError::InvalidReadBufferLength);
        }
        // Longer reads than the command FIFO can chain cannot be
        // served atomically here — see `async_read_txn`.
        let ncmds = read.len().div_ceil(256);
        if ncmds > self.registers().tx_fifo_capacity() && !self.allow_chunked_reads {
            return Err(IOError::ChunkingRequired);
        }
        Ok(())
    }

    async fn async_read_txn(
        &mut self,
        address: u8,
        read: &mut [u8],
        continues: Option<Started>,
    ) -> Result<Started, IOError> {
        self.read_preflight(read)?;

        // Chain all RECEIVE commands under a single address phase when
        // they fit the command FIFO: the controller ACKs across a
        // command boundary only when the next command is already
        // queued, so a read up to capacity*256 bytes stays ONE bus
        // transaction. Longer reads cannot be served atomically here at
        // all, because nothing refills the command FIFO while the CPU
        // sleeps on the DMA completion.
        let ncmds = read.len().div_ceil(256);
        let mut seamed = ncmds > self.registers().tx_fifo_capacity();
        // The open transaction carried between branches: set by a
        // successful chained attempt or by the seamed loop's final
        // chunk.
        let mut open = None;
        let mut carry = continues;
        if !seamed {
            match self.dma_read_chained(address, read, carry.take()).await {
                Ok(o) => open = Some(o),
                // A chained read that died mid-transfer leaves the device in an
                // unknown state: its pointer has advanced by however many bytes
                // were clocked out, and destructive reads have already consumed
                // them. Re-reading from the caller's buffer start would return
                // shifted data as success, so it happens only when the caller
                // has explicitly accepted that trade.
                Err(e @ (IOError::UnexpectedStop | IOError::Timeout)) if self.allow_chunked_reads => {
                    #[cfg(feature = "defmt")]
                    defmt::trace!("chained DMA read failed ({}); retrying chunked (opted in)", e);
                    let _ = e;
                    seamed = true;
                }
                Err(e) => return Err(e),
            }
        }
        if seamed {
            // A surviving continuation is consumed by the first seam
            // chunk's START (reachable only on the straight-to-seamed
            // path; the fallback path consumed it in the failed
            // chained attempt).
            let nchunks = read.len().div_ceil(256);
            for (idx, chunk) in read.chunks_mut(256).enumerate() {
                let chunk_open = self.async_start(address, true, carry.take()).await?;

                // perform corrective action if the future is dropped or
                // an error happens between here and the end of the read.
                //
                // NOTE: this *must* be set up *after* async_start.
                // async_start already runs `status_and_act`, which on
                // NACK performs its own remediation; if we set OnDrop
                // earlier, the early `?` return would invoke remediation
                // a second time and corrupt the controller state for the
                // next transaction.
                let on_drop = OnDrop::new(|| {
                    // Request path off and channel quiesced before
                    // recovery — see `dma_read_chained`.
                    self.registers().set_rx_dma(false);
                    self.mode.rx_dma.quiesce();
                    self.remediation();
                });

                // send receive command
                self.send_cmd(ControllerCommand::RECEIVE, (chunk.len() - 1) as u8);

                self.dma_read_into(chunk).await?;

                // The chunk is drained and its channel quiesced by
                // `dma_read_into`; defuse before the seam STOP, which
                // recovers its own failures (see `async_stop`). We
                // re-arm on the next chunk if any.
                on_drop.defuse();

                // End every non-final chunk with a STOP: a repeated START
                // right after the auto-NACK of a consumed RECEIVE command
                // is not reliably accepted on this silicon.
                if idx + 1 < nchunks {
                    self.async_stop(chunk_open).await?;
                } else {
                    open = Some(chunk_open);
                }
            }
        }

        // One of the branches ran to completion, so a transaction is
        // open here.
        Ok(open.expect("read completed without an open transaction"))
    }

    async fn async_write_txn(
        &mut self,
        address: u8,
        write: &[u8],
        continues: Option<Started>,
    ) -> Result<Started, IOError> {
        let open = self.async_start(address, false, continues).await?;

        // Usually, embassy HALs error out with an empty write,
        // however empty writes are useful for writing I2C scanning
        // logic through write probing. That is, we send a start with
        // R/w bit cleared, but instead of writing any data, just send
        // the stop onto the bus. This has the effect of checking if
        // the resulting address got an ACK but causing no
        // side-effects to the device on the other end.
        //
        // Because of this, we are not going to error out in case of
        // empty writes.
        if write.is_empty() {
            #[cfg(feature = "defmt")]
            defmt::trace!("Empty write, write probing?");
            return Ok(open);
        }

        // perform corrective action if the future is dropped
        let on_drop = OnDrop::new(|| {
            // Same rationale as the DMA read path: kill the request
            // path and wait for the channel to go inactive before
            // recovery touches the FIFOs, since `write` is about to be
            // released.
            self.registers().set_tx_dma(false);
            self.mode.tx_dma.quiesce();
            self.remediation();
        });

        for chunk in write.chunks(DMA_MAX_TRANSFER_SIZE) {
            let peri_addr = self.registers().tx_data_ptr();

            unsafe {
                // Clean up channel state
                self.mode.tx_dma.disable_request();
                self.mode.tx_dma.clear_done();
                self.mode.tx_dma.clear_interrupt();

                // Set DMA request source from instance type (type-safe)
                self.mode.tx_dma.set_request_source(self.mode.tx_request);

                // Configure TCD for memory-to-peripheral transfer
                self.mode.tx_dma.setup_write_to_peripheral(
                    chunk,
                    peri_addr,
                    false,
                    TransferOptions::COMPLETE_INTERRUPT,
                )?;

                // Enable I2C TX DMA request
                self.registers().set_tx_dma(true);

                // Enable DMA channel request
                self.mode.tx_dma.enable_request();
            }

            // Wait for completion asynchronously — or for a bus error
            // (NACK, arbitration loss, FIFO error) that stops the
            // transfer, in which case the DMA would never complete and
            // waiting on it alone would hang forever. Bounded like the
            // read path: the silicon can terminate a transfer silently
            // (no flag, no interrupt), which only a timeout can catch.
            let wait = core::future::poll_fn(|cx| {
                // Drain stale tokens and finish registered — see
                // `dma_read_into`.
                while self.mode.tx_dma.wait_cell().poll_wait(cx).is_ready() {}
                if self.mode.tx_dma.is_done() {
                    return core::task::Poll::Ready(Ok(()));
                }
                while self.info.wait_cell().poll_wait(cx).is_ready() {}
                // The interrupt handler disables MIER on wake; re-arm the
                // error sources and check, as one operation.
                if let Some(e) = self.registers().error_wake() {
                    return core::task::Poll::Ready(Err(IOError::from(e)));
                }
                core::task::Poll::Pending
            });
            // ~1 ms/byte of margin on top of a generous floor.
            let bound = self.timeout + embassy_time::Duration::from_millis(chunk.len() as u64);
            match embassy_time::with_timeout(bound, wait).await {
                Ok(r) => r?,
                // The guard above is still armed: it quiesces the
                // channel and closes the transaction on this return.
                Err(_) => return Err(IOError::Timeout),
            }

            // Ensure DMA writes are visible to CPU
            cortex_m::asm::dsb();
            // Cleanup: quiesce rather than merely disabling requests, so
            // the channel is provably idle before this chunk's borrow
            // ends.
            self.registers().set_tx_dma(false);
            self.mode.tx_dma.quiesce();
        }

        // Every chunk is drained and the channel quiesced; the close
        // variant owns the trailing stop.
        on_drop.defuse();

        Ok(open)
    }
}

impl<'d, M: Mode> Drop for I2c<'d, M> {
    fn drop(&mut self) {
        self._scl.set_as_disabled();
        self._sda.set_as_disabled();
    }
}

impl<'d, M: Mode> embedded_hal_02::blocking::i2c::Read for I2c<'d, M> {
    type Error = IOError;

    fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.blocking_read(address, buffer)
    }
}

impl<'d, M: Mode> embedded_hal_02::blocking::i2c::Write for I2c<'d, M> {
    type Error = IOError;

    fn write(&mut self, address: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        self.blocking_write(address, bytes)
    }
}

impl<'d, M: Mode> embedded_hal_02::blocking::i2c::WriteRead for I2c<'d, M> {
    type Error = IOError;

    fn write_read(&mut self, address: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.blocking_write_read(address, bytes, buffer)
    }
}

impl<'d, M: Mode> embedded_hal_02::blocking::i2c::Transactional for I2c<'d, M> {
    type Error = IOError;

    fn exec(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal_02::blocking::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        // Reject a doomed read before ANY op touches the bus: the
        // read entry checks run before a guard is armed, so a
        // rejection after a no-STOP op would leave the transaction
        // open with the bus held — see `blocking_write_read`.
        for op in operations.iter() {
            if let embedded_hal_02::blocking::i2c::Operation::Read(buf) = op {
                if buf.is_empty() {
                    return Err(IOError::InvalidReadBufferLength);
                }
            }
        }

        if let Some((last, rest)) = operations.split_last_mut() {
            // Each op leaves its transaction open; the next op's
            // repeated START consumes the token, and the last op's
            // trailing STOP closes the chain.
            let mut open: Option<Started> = None;
            for op in rest {
                open = Some(match op {
                    embedded_hal_02::blocking::i2c::Operation::Read(buf) => {
                        self.blocking_read_txn(address, buf, open.take())?
                    }
                    embedded_hal_02::blocking::i2c::Operation::Write(buf) => {
                        self.blocking_write_txn(address, buf, open.take())?
                    }
                });
            }

            match last {
                embedded_hal_02::blocking::i2c::Operation::Read(buf) => {
                    self.blocking_read_close(address, buf, open.take())
                }
                embedded_hal_02::blocking::i2c::Operation::Write(buf) => {
                    self.blocking_write_close(address, buf, open.take())
                }
            }
        } else {
            Ok(())
        }
    }
}

impl embedded_hal_1::i2c::Error for IOError {
    fn kind(&self) -> embedded_hal_1::i2c::ErrorKind {
        match *self {
            Self::ArbitrationLoss => embedded_hal_1::i2c::ErrorKind::ArbitrationLoss,
            Self::AddressNack => {
                embedded_hal_1::i2c::ErrorKind::NoAcknowledge(embedded_hal_1::i2c::NoAcknowledgeSource::Address)
            }
            _ => embedded_hal_1::i2c::ErrorKind::Other,
        }
    }
}

impl<'d, M: Mode> embedded_hal_1::i2c::ErrorType for I2c<'d, M> {
    type Error = IOError;
}

impl<'d, M: Mode> embedded_hal_1::i2c::I2c for I2c<'d, M> {
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal_1::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        // Reject a doomed read before ANY op touches the bus — see
        // `exec` above; `recover_from_error` below clears flags but
        // does not close an open transaction.
        for op in operations.iter() {
            if let embedded_hal_1::i2c::Operation::Read(buf) = op {
                if buf.is_empty() {
                    return Err(IOError::InvalidReadBufferLength);
                }
            }
        }

        let result = (|| {
            if let Some((last, rest)) = operations.split_last_mut() {
                // Each op leaves its transaction open; the next op's
                // repeated START consumes the token, and the last op's
                // trailing STOP closes the chain.
                let mut open: Option<Started> = None;
                for op in rest {
                    open = Some(match op {
                        embedded_hal_1::i2c::Operation::Read(buf) => {
                            self.blocking_read_txn(address, buf, open.take())?
                        }
                        embedded_hal_1::i2c::Operation::Write(buf) => {
                            self.blocking_write_txn(address, buf, open.take())?
                        }
                    });
                }

                match last {
                    embedded_hal_1::i2c::Operation::Read(buf) => self.blocking_read_close(address, buf, open.take()),
                    embedded_hal_1::i2c::Operation::Write(buf) => self.blocking_write_close(address, buf, open.take()),
                }
            } else {
                Ok(())
            }
        })();

        if result.is_err() {
            self.recover_from_error();
        }
        result
    }
}

impl<'d, M: AsyncMode> embedded_hal_async::i2c::I2c for I2c<'d, M>
where
    I2c<'d, M>: AsyncEngine,
{
    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal_async::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        // Reject a doomed read before ANY op touches the bus. This is
        // the entry point device-driver crates actually use (the
        // trait's `write_read` is a provided method that delegates
        // here), and `read_preflight` errors — empty buffer,
        // `ChunkingRequired` on the DMA engine — fire before a
        // recovery guard is armed, so a mid-list rejection would
        // otherwise leave the transaction open with the bus held.
        for op in operations.iter() {
            if let embedded_hal_async::i2c::Operation::Read(buf) = op {
                <Self as AsyncEngine>::read_preflight(self, buf)?;
            }
        }

        let result = async {
            if let Some((last, rest)) = operations.split_last_mut() {
                // Each op leaves its transaction open; the next op's
                // repeated START consumes the token, and the last op's
                // trailing STOP closes the chain.
                let mut open: Option<Started> = None;
                for op in rest {
                    open = Some(match op {
                        embedded_hal_async::i2c::Operation::Read(buf) => {
                            <Self as AsyncEngine>::async_read_txn(self, address, buf, open.take()).await?
                        }
                        embedded_hal_async::i2c::Operation::Write(buf) => {
                            <Self as AsyncEngine>::async_write_txn(self, address, buf, open.take()).await?
                        }
                    });
                }

                match last {
                    embedded_hal_async::i2c::Operation::Read(buf) => {
                        self.async_read_close(address, buf, open.take()).await
                    }
                    embedded_hal_async::i2c::Operation::Write(buf) => {
                        self.async_write_close(address, buf, open.take()).await
                    }
                }
            } else {
                Ok(())
            }
        }
        .await;

        if result.is_err() {
            self.recover_from_error();
        }
        result
    }
}

impl<'d, M: Mode> embassy_embedded_hal::SetConfig for I2c<'d, M> {
    type Config = Config;
    type ConfigError = SetupError;

    fn set_config(&mut self, config: &Self::Config) -> Result<(), SetupError> {
        self.is_hs = config.speed == Speed::UltraFast;
        self.allow_chunked_reads = config.allow_chunked_reads;
        self.timeout = config.transfer_timeout;
        self.set_configuration(config);
        Ok(())
    }
}
