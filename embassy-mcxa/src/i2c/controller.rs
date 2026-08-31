//! # LPI2C Controller Driver
//!
//! This module provides a driver for the Low-Power Inter-Integrated
//! Circuit (LPI2C) controller, supporting blocking,
//! interrupt-only async, and DMA async modes of operation.
//!
//! The driver supports Standard, Fast, and Fast-mode Plus transfers.
//!
//! ## Features
//!
//! - **Blocking and Asynchronous Modes**: Supports both blocking and
//! async APIs for flexibility in different runtime environments.
//! - **DMA Support**: Enables high-performance data transfers using
//! DMA.
//! - **Configurable Bus Speeds**: Supports standard (100 kHz), fast
//! (400 kHz), and fast-plus (1 MHz) modes. The legacy
//! [`Speed::UltraFast`] variant denotes 3.4-Mbit/s high-speed I2C and is
//! rejected with [`SetupError::UnsupportedSpeed`] until its distinct
//! protocol setup is implemented and verified.
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

use super::{Async, AsyncMode, Blocking, Dma, Info, Instance, Mode, SclPin, SdaPin};
use crate::clocks::periph_helpers::{Div4, Lpi2cClockSel, Lpi2cConfig};
use crate::clocks::{ClockError, PoweredClock, WakeGuard, enable_and_reset};
use crate::dma::{Channel, DMA_MAX_TRANSFER_SIZE, DmaChannel, DmaRequest};
use crate::gpio::{AnyPin, SealedPin};
use crate::interrupt;
use crate::interrupt::typelevel::Interrupt;
use crate::pac::lpi2c::Prescale;
use registers::{
    CommandStep, ControllerAction, ControllerSetup, ControllerStatusError, StartAction, StartDrainStep, StopAction,
    StopStep, TransferFault,
};

// Controller protocol MMIO is private to this driver tree. The target driver
// owns a separate facade, so it cannot name controller events or operate the
// controller facade outside the session-permit protocol.
#[path = "controller_registers.rs"]
mod registers;
// Only the opaque facade name crosses the I2C sibling boundary so `Info`
// can construct it. Its operational methods remain `pub(super)` inside this
// controller tree; target code cannot operate a controller facade.
pub(in crate::i2c) use registers::ControllerRegisters;
#[path = "controller/session.rs"]
mod session;

use session::{Session, SessionRxStep, StartReservation};

/// Errors exclusive to HW initialization
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum SetupError {
    /// Clock configuration error.
    ClockSetup(ClockError),
    /// The requested bus speed needs a controller mode this driver does not
    /// yet implement.
    UnsupportedSpeed,
    /// An enabled [`PinLowTimeout`] cannot be represented by the current
    /// functional clock and derived LPI2C prescaler.
    PinLowTimeoutOutOfRange,
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
        let registers = T::info().controller_registers();
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
    /// 3.4-Mbit/s high-speed I2C.
    ///
    /// Kept for source compatibility, but this driver does not yet perform
    /// the required high-speed setup. Constructors and `SetConfig` return
    /// [`SetupError::UnsupportedSpeed`] instead of touching the peripheral.
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

/// A controller speed which is supported by this driver's complete setup and
/// command protocol.
///
/// Its constructor is private to this module. Configuration code must carry
/// this proof into the register facade, so no MMIO setup path can accidentally
/// revive the unsupported high-speed command encoding.
#[derive(Clone, Copy)]
struct SupportedSpeed(u32);

impl Speed {
    fn supported(self) -> Result<SupportedSpeed, SetupError> {
        match self {
            Speed::Standard => Ok(SupportedSpeed(100_000)),
            Speed::Fast => Ok(SupportedSpeed(400_000)),
            Speed::FastPlus => Ok(SupportedSpeed(1_000_000)),
            Speed::UltraFast => Err(SetupError::UnsupportedSpeed),
        }
    }
}

impl SupportedSpeed {
    pub(super) const fn hertz(self) -> u32 {
        self.0
    }
}

/// Hardware watchdog for SCL or SDA being held low.
///
/// This is deliberately separate from [`Config::transfer_timeout`]: the
/// latter is a software forward-progress budget, while this configures
/// `MCFGR3[PINLOW]` in hardware. It is disabled by default so existing users
/// keep their current clock-stretching semantics.
#[derive(Clone, Copy, Default)]
pub enum PinLowTimeout {
    /// Do not enable the hardware pin-low watchdog.
    #[default]
    Disabled,
    /// Fail a transfer if either bus line stays low for this long.
    ///
    /// The requested duration is rounded up to the hardware's 256-cycle
    /// quantum after the selected LPI2C prescaler. A zero or unrepresentable
    /// duration returns [`SetupError::PinLowTimeoutOutOfRange`] rather than
    /// being truncated by the PAC register setter.
    Enabled(embassy_time::Duration),
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

/// The fixed RX channel/request pair for this controller instance.
///
/// Its constructor is private to this driver: only the controller's one
/// wiring point may associate the instance's RX request with a channel.
/// The register facade accepts this distinct type rather than loose channel
/// and request arguments, so an RX arm cannot accidentally receive TX
/// plumbing at an ordinary call site.
#[must_use]
struct ControllerRxDma<'a, 'd> {
    owner: usize,
    channel: &'a DmaChannel<'d>,
    request: DmaRequest,
}

impl<'a, 'd> ControllerRxDma<'a, 'd> {
    fn new(owner: usize, channel: &'a DmaChannel<'d>, request: DmaRequest) -> Self {
        Self {
            owner,
            channel,
            request,
        }
    }

    fn owner(&self) -> usize {
        self.owner
    }

    fn channel(&self) -> &'a DmaChannel<'d> {
        self.channel
    }

    fn request(&self) -> DmaRequest {
        self.request
    }
}

/// The fixed TX channel/request pair for this controller instance. See
/// [`ControllerRxDma`] for why this is not a generic `(channel, request)`
/// tuple.
#[must_use]
struct ControllerTxDma<'a, 'd> {
    owner: usize,
    channel: &'a DmaChannel<'d>,
    request: DmaRequest,
}

impl<'a, 'd> ControllerTxDma<'a, 'd> {
    fn new(owner: usize, channel: &'a DmaChannel<'d>, request: DmaRequest) -> Self {
        Self {
            owner,
            channel,
            request,
        }
    }

    fn owner(&self) -> usize {
        self.owner
    }

    fn channel(&self) -> &'a DmaChannel<'d> {
        self.channel
    }

    fn request(&self) -> DmaRequest {
        self.request
    }
}

/// A DMA wait can end because the controller stopped without a status
/// fault, or because it produced a typed hardware fault. Keep those two
/// cases distinct until the live session has consumed any halt proof.
#[must_use]
enum DmaWaitError {
    Fault(TransferFault),
    UnexpectedStop,
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

    /// Hardware watchdog for a physically stuck-low SCL or SDA line.
    ///
    /// This is disabled by default. Enable it only after choosing a bound
    /// compatible with the devices' legitimate clock-stretching behavior;
    /// see [`PinLowTimeout`].
    pub pin_low_timeout: PinLowTimeout,

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
            pin_low_timeout: PinLowTimeout::Disabled,
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
        let setup = Self::construction_setup::<T>(&config)?;
        Self::new_inner(peri, scl, sda, config, setup, Blocking)
    }
}

impl<'d, M: Mode> I2c<'d, M> {
    #[inline(always)]
    fn registers(&self) -> ControllerRegisters {
        self.info.controller_registers()
    }

    /// Build every user-controlled controller register value before a
    /// constructor changes interrupt, DMA, pin, or LPI2C state.
    ///
    /// `Lpi2cConfig::functional_clock_hz` reads the already-initialized
    /// clock plan without programming MRCC. The resulting `ControllerSetup`
    /// is the only configuration value that crosses into the MMIO facade.
    fn construction_setup<T: Instance>(config: &Config) -> Result<ControllerSetup, SetupError> {
        let speed = config.speed.supported()?;
        let ClockConfig { power, source, div } = config.clock_config;
        let clock = Lpi2cConfig {
            power,
            source,
            div,
            instance: T::CLOCK_INSTANCE,
        };
        let input_hz = crate::clocks::with_clocks(|clocks| clock.functional_clock_hz(clocks))
            .ok_or(SetupError::ClockSetup(ClockError::NeverInitialized))?
            .map_err(SetupError::ClockSetup)?;

        ControllerSetup::new(input_hz, speed, config.pin_low_timeout)
    }

    fn new_inner<T: Instance>(
        _peri: Peri<'d, T>,
        scl: Peri<'d, impl SclPin<T>>,
        sda: Peri<'d, impl SdaPin<T>>,
        config: Config,
        setup: ControllerSetup,
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
            allow_chunked_reads: config.allow_chunked_reads,
            timeout: config.transfer_timeout,
            freq: parts.freq,
            _wg: parts.wake_guard,
        };

        inst.set_configuration(setup);

        Ok(inst)
    }

    fn set_configuration(&self, setup: ControllerSetup) {
        self.registers().configure(setup)
    }

    /// Consume a session at its defined end, asserting it belongs to
    /// THIS controller. Cross-instance sessions compile (the session
    /// is not lifetime-branded — see [`Session`]) but are a severe
    /// protocol violation, so they fail deterministically here, before
    /// the defuse.
    fn assert_session_owner(&self, open: &Session) {
        assert!(
            open.belongs_to(self.info),
            "i2c: transaction session from a different controller instance"
        );
    }

    /// Reserve the single-session slot BEFORE any wire traffic.
    ///
    /// Runtime linearity backstop: the session type is threaded
    /// linearly on every public path by construction, but nothing at
    /// compile time stops a module-internal caller from starting a
    /// second transaction while one is live — that would split
    /// recovery ownership of a single wire transaction. The
    /// reservation, not the mint, is the mutual-exclusion point: a
    /// second start must fail BEFORE its START touches the bus, not
    /// after. The returned RAII reservation releases the slot on every
    /// pre-command error/cancellation path; conversion to [`Session`]
    /// transfers that responsibility once a START enters MTDR.
    fn reserve_session(&self) -> StartReservation {
        StartReservation::acquire(self.info)
    }

    /// Drive the typed terminal sequence of a queued START in blocking
    /// mode. `StartDrained` can arise only from the facade's fault-first,
    /// empty-FIFO observation; final W1C cleanup then commits the pending
    /// session phase before this method releases its mutable borrow.
    fn settle_start_blocking(&self, open: &mut Session) -> Result<(), IOError> {
        self.assert_session_owner(open);
        let deadline = embassy_time::Instant::now() + self.timeout;
        let mut permit = open.start_status_permit();
        loop {
            match self.registers().poll_start_drain(permit) {
                StartDrainStep::Pending(next) if embassy_time::Instant::now() > deadline => {
                    drop(next);
                    return Err(IOError::Timeout);
                }
                StartDrainStep::Pending(next) => permit = next,
                StartDrainStep::Fault(fault) => return Err(open.bind_fault(fault)),
                StartDrainStep::Drained(drained) => {
                    return match self.registers().finish_start_status(drained) {
                        Ok(()) => Ok(()),
                        Err(fault) => Err(open.bind_fault(fault)),
                    };
                }
            }
        }
    }

    /// Form a semantic START action after the public address preflight.
    fn start_action(&self, address: u8, read: bool) -> StartAction {
        StartAction::new(address, read).expect("i2c: checked seven-bit address could not form a START action")
    }

    /// Attempt one active command on a live session. The facade combines
    /// fault classification, capacity validation, and MTDR emission; a
    /// caller may only choose whether a full FIFO should be waited on.
    fn try_enqueue_session(&self, action: ControllerAction, open: &mut Session) -> Result<bool, IOError> {
        self.assert_session_owner(open);
        let step = {
            let permit = open.command_permit();
            self.registers().try_enqueue_active(permit, action)
        };
        match step {
            CommandStep::Queued => Ok(true),
            CommandStep::Full => Ok(false),
            CommandStep::Fault(fault) => Err(open.bind_fault(fault)),
        }
    }

    /// Attempt a START through the only path that can atomically commit
    /// `StartPending` when the command enters MTDR.
    fn try_enqueue_start_session(&self, action: StartAction, open: &mut Session) -> Result<bool, IOError> {
        self.assert_session_owner(open);
        let step = {
            let permit = open.start_transition_permit();
            self.registers().try_enqueue_start(permit, action)
        };
        match step {
            CommandStep::Queued => Ok(true),
            CommandStep::Full => Ok(false),
            CommandStep::Fault(fault) => Err(open.bind_fault(fault)),
        }
    }

    /// Attempt a normal trailing STOP through the only gate that commits
    /// `StopPending` when MTDR accepts it. Ordinary command permits cannot
    /// use this path.
    fn try_enqueue_stop_session(&self, action: StopAction, open: &mut Session) -> Result<bool, IOError> {
        self.assert_session_owner(open);
        let step = {
            let permit = open.stop_transition_permit();
            self.registers().try_enqueue_stop(permit, action)
        };
        match step {
            CommandStep::Queued => Ok(true),
            CommandStep::Full => Ok(false),
            CommandStep::Fault(fault) => Err(open.bind_fault(fault)),
        }
    }

    /// Bounded blocking enqueue before a session exists. It owns recovery
    /// because no live session can retain a halt proof yet.
    fn enqueue_before_start(&self, action: StartAction, reservation: &mut StartReservation) -> Result<(), IOError> {
        let regs = self.registers();
        let deadline = embassy_time::Instant::now() + self.timeout;
        loop {
            match regs.try_enqueue_start(reservation.start_transition_permit(), action) {
                CommandStep::Queued => return Ok(()),
                CommandStep::Fault(fault) => {
                    return Err(session::recover_before_session_fault(&regs, self.timeout, fault));
                }
                CommandStep::Full if embassy_time::Instant::now() > deadline => {
                    session::remediate_before_session(&regs, self.timeout);
                    return Err(IOError::Timeout);
                }
                CommandStep::Full => {}
            }
        }
    }

    /// Bounded blocking enqueue after a session exists. Error returns leave
    /// the session live, so its drop remains the single recovery owner.
    fn enqueue_session_blocking(&self, action: ControllerAction, open: &mut Session) -> Result<(), IOError> {
        let deadline = embassy_time::Instant::now() + self.timeout;
        loop {
            if self.try_enqueue_session(action, open)? {
                return Ok(());
            }
            if embassy_time::Instant::now() > deadline {
                return Err(IOError::Timeout);
            }
        }
    }

    /// Blocking START enqueue for an existing session. Full FIFO waits keep
    /// the session borrowed, while the successful gate itself commits the
    /// phase transition.
    fn enqueue_start_session_blocking(&self, action: StartAction, open: &mut Session) -> Result<(), IOError> {
        let deadline = embassy_time::Instant::now() + self.timeout;
        loop {
            if self.try_enqueue_start_session(action, open)? {
                return Ok(());
            }
            if embassy_time::Instant::now() > deadline {
                return Err(IOError::Timeout);
            }
        }
    }

    /// Blocking normal-STOP gate. The session becomes `StopPending` exactly
    /// when the command enters MTDR; callers then move it into `StopWait`
    /// before any completion wait can begin.
    fn enqueue_stop_session_blocking(&self, action: StopAction, open: &mut Session) -> Result<(), IOError> {
        let deadline = embassy_time::Instant::now() + self.timeout;
        loop {
            if self.try_enqueue_stop_session(action, open)? {
                return Ok(());
            }
            if embassy_time::Instant::now() > deadline {
                return Err(IOError::Timeout);
            }
        }
    }

    /// Emit the first RECEIVE immediately after a read START. If it cannot
    /// be queued, the session remains in `ReadAddressed`, so its drop uses
    /// the only safe no-data read abort sequence.
    fn enqueue_first_receive(&self, bytes: usize, open: &mut Session) -> Result<(), IOError> {
        self.assert_session_owner(open);
        let step = {
            let permit = open.first_receive_permit();
            self.registers().try_enqueue_first_receive(permit, bytes)
        };
        match step {
            CommandStep::Queued => Ok(()),
            CommandStep::Full => Err(IOError::Other),
            CommandStep::Fault(fault) => Err(open.bind_fault(fault)),
        }
    }

    /// Enqueue a RECEIVE only after the session's first command committed
    /// it to the streaming recovery shape. There is deliberately no plain
    /// `ControllerAction::receive`: this capability is the API boundary
    /// that prevents a future edit from skipping the first-read transition.
    fn try_enqueue_read_receive(&self, bytes: usize, open: &mut Session) -> Result<bool, IOError> {
        self.assert_session_owner(open);
        let step = {
            let permit = open.read_receive_permit();
            self.registers().try_enqueue_read_receive(permit, bytes)
        };
        match step {
            CommandStep::Queued => Ok(true),
            CommandStep::Full => Ok(false),
            CommandStep::Fault(fault) => Err(open.bind_fault(fault)),
        }
    }

    /// Prepares an appropriate Start condition on bus by issuing a
    /// `Start` command together with the device address and R/w bit.
    ///
    /// Blocks waiting for space in the FIFO to become available, then
    /// sends the command and blocks waiting for the FIFO to become
    /// empty ensuring the command was sent.
    /// Open a FRESH transaction: no live session may exist for this
    /// controller (holding one and calling this drops it eventually —
    /// which now RECOVERS rather than leaks; still a caller bug, but a
    /// safe one).
    fn start_fresh(&self, address: u8, read: bool) -> Result<Session, IOError> {
        if address >= 0x80 {
            return Err(IOError::AddressOutOfRange(address));
        }
        self.start_raw(address, read)
    }

    /// Continue an open transaction with a repeated START. The existing
    /// session remains live until the START is accepted into MTDR, then is
    /// retargeted in place. This makes a full FIFO, a fault, or a cancelled
    /// async twin recover the predecessor rather than briefly leaving an
    /// open bus with no owner.
    fn start_continue(&self, address: u8, read: bool, mut open: Session) -> Result<Session, IOError> {
        if address >= 0x80 {
            return Err(IOError::AddressOutOfRange(address));
        }
        self.assert_session_owner(&open);

        self.enqueue_start_session_blocking(self.start_action(address, read), &mut open)?;
        self.settle_start_blocking(&mut open)?;
        Ok(open)
    }

    fn start_raw(&self, address: u8, read: bool) -> Result<Session, IOError> {
        // The reservation precedes the START going on the wire — see
        // `reserve_session` — so every failure below must release it.
        let mut reservation = self.reserve_session();

        // The active-command facade combines the FIFO check, fault
        // classification, and actual MTDR write. No session exists yet,
        // so this path owns recovery for a stale fault/full FIFO.
        if let Err(error) = self.enqueue_before_start(self.start_action(address, read), &mut reservation) {
            return Err(error);
        }

        // The queued START now has a live recovery owner. Its pending
        // phase remains intact through the drain/status boundary, so a
        // cancellation or late halt cannot flatten fresh-read recovery
        // into the wrong close shape.
        let mut open = reservation.into_pending_session(self.timeout);
        self.settle_start_blocking(&mut open)?;
        Ok(open)
    }

    /// Prepares a Stop condition on the bus and waits for it to
    /// PHYSICALLY complete: command FIFO empty AND the bus engine
    /// idle. "Pulled from the FIFO" is not completion — the wire
    /// condition follows later (a bit-time normally, a whole clock
    /// stretch pathologically), and a fault in that window belongs to
    /// THIS transaction.
    ///
    /// The session stays live until that completion succeeds: every
    /// error return — no FIFO room, a fault while the STOP formed, a
    /// stretched STOP that never completed within the timeout — drops
    /// the session, whose recovery closes the transaction. Nothing
    /// here recovers explicitly, so recovery stays exactly-once.
    fn stop(&self, mut open: Session) -> Result<(), IOError> {
        self.enqueue_stop_session_blocking(StopAction::new(), &mut open)?;

        // Move the queued STOP's recovery owner into the completion state
        // before polling. Every pending/completed/finalized value now owns
        // this same session, so it cannot be replayed or detached.
        let mut stop = open.into_stop_wait();

        let deadline = embassy_time::Instant::now() + self.timeout;
        let completed = loop {
            match self.registers().stop_step(stop) {
                StopStep::Completed(completed) => break completed,
                StopStep::Fault(fault) => return Err(fault.into_error()),
                StopStep::Pending(next) if embassy_time::Instant::now() > deadline => {
                    drop(next);
                    return Err(IOError::Timeout);
                }
                StopStep::Pending(next) => stop = next,
            }
        };

        // Completion is not a status read: one final snapshot both
        // classifies a fault that latched in the last poll gap as this
        // transaction's own, and clears the STOP's residual flags so
        // the next START does not misattribute them.
        let finalized = match self.registers().finish_stop(completed) {
            Ok(finalized) => finalized,
            Err(fault) => return Err(fault.into_error()),
        };
        finalized.defuse();
        Ok(())
    }

    /// Read on a FRESH transaction, leaving it OPEN (no trailing
    /// STOP): hands the session back to the caller, which must thread
    /// it onward.
    fn blocking_read_txn_fresh(&self, address: u8, read: &mut [u8]) -> Result<Session, IOError> {
        if read.is_empty() {
            return Err(IOError::InvalidReadBufferLength);
        }
        let open = self.start_fresh(address, true)?;
        self.blocking_read_resume(address, read, open)
    }

    /// Read continuing an open transaction via repeated START, leaving
    /// the successor OPEN — see [`Self::blocking_read_txn_fresh`].
    fn blocking_read_txn_continue(&self, address: u8, read: &mut [u8], open: Session) -> Result<Session, IOError> {
        if read.is_empty() {
            // The moved-in session drops on this return — recovering
            // the open transaction and releasing the bus rather than
            // silently abandoning it.
            return Err(IOError::InvalidReadBufferLength);
        }
        let open = self.start_continue(address, true, open)?;
        self.blocking_read_resume(address, read, open)
    }

    /// The read engine past the address phase: drain the open
    /// transaction with chained RECEIVE commands, falling back (only
    /// when the caller opted in) to STOP-seamed re-addressed chunks.
    fn blocking_read_resume(&self, address: u8, read: &mut [u8], open: Session) -> Result<Session, IOError> {
        // A chained read that died mid-transfer leaves the device in an
        // unknown state: its pointer has advanced by however many bytes
        // were clocked out, and destructive reads have already consumed
        // them. Re-reading from the caller's buffer start would return
        // shifted data as success, so it happens only when the caller
        // has explicitly accepted that trade.
        match self.blocking_read_chained(read, open) {
            Ok(open) => Ok(open),
            Err(e @ (IOError::UnexpectedStop | IOError::Timeout)) if self.allow_chunked_reads => {
                #[cfg(feature = "defmt")]
                defmt::trace!("chained read failed ({}); retrying chunked (opted in)", e);
                let _ = e;
                // The failed chained attempt's session dropped on its
                // error path — recovering and closing the bus — so the
                // fallback starts fresh.
                let open = self.start_fresh(address, true)?;
                self.blocking_read_seamed(address, read, open)
            }
            Err(e) => Err(e),
        }
    }

    /// One read as a single addressed transaction with chained RECEIVE
    /// commands on the already-open transaction. Does not send the
    /// trailing STOP. Every error return drops `open` — the session's
    /// drop is the single recovery for any abort past the address
    /// phase.
    fn blocking_read_chained(&self, read: &mut [u8], mut open: Session) -> Result<Session, IOError> {
        let total = read.len();
        // Chain RECEIVE commands under the single address phase: the
        // controller ACKs across a command boundary only when the next
        // command is already queued (otherwise it NACKs and terminates
        // the read early), so the command pipeline is kept ahead of the
        // data. This preserves >256-byte reads as ONE bus transaction —
        // no repeated START (unreliable after the auto-NACK on this
        // silicon) and no STOP seams (which would break embedded-hal
        // transaction atomicity and let another controller interleave).
        // The FIRST command goes in unconditionally: the start
        // transition returned only after the command FIFO drained, so
        // room for one command is guaranteed — and issuing it before
        // any fallible step upholds [`Session`]'s drop invariant
        // (read session ⇒ a data command exists, so its abort closes
        // as ReadStreaming behind that command's auto-NACK).
        let first = total.min(256);
        self.enqueue_first_receive(first, &mut open)?;
        let mut queued = first;
        let mut drained = 0usize;
        let mut deadline = embassy_time::Instant::now() + self.timeout;
        while drained < total {
            // Top up the command pipeline whenever there is room.
            while queued < total {
                match self.try_enqueue_read_receive((total - queued).min(256), &mut open)? {
                    true => {
                        let chunk = (total - queued).min(256);
                        queued += chunk;
                    }
                    // Command FIFO full: plenty is queued ahead of
                    // the data; go drain some of it.
                    false => break,
                }
            }

            // Receive one byte, or bail out on a fault (NACK,
            // arbitration loss, FIFO error): no more data will
            // arrive, and a data-only wait would spin forever.
            match open.rx_step()? {
                Some(SessionRxStep::Byte(b)) => {
                    read[drained] = b;
                    drained += 1;
                    deadline = embassy_time::Instant::now() + self.timeout;
                }
                Some(SessionRxStep::Ended) => return Err(IOError::UnexpectedStop),
                // No progress for a full timeout window: the
                // transfer died without a flag.
                None if embassy_time::Instant::now() > deadline => {
                    return Err(IOError::Timeout);
                }
                None => {}
            }
        }

        Ok(open)
    }

    /// Fallback: one read as re-addressed 256-byte chunks, each ended
    /// with a STOP. Not atomic, but immune to the chained-boundary
    /// early-termination quirk. `open` is the first chunk's
    /// already-open transaction; the final chunk's is returned OPEN.
    fn blocking_read_seamed(&self, address: u8, read: &mut [u8], open: Session) -> Result<Session, IOError> {
        // The STOP-seamed chunks up front, the final chunk (1..=256
        // bytes — the caller guarantees a non-empty read) apart, so
        // the session is threaded linearly and never conditionally
        // moved.
        let seam_len = (read.len() - 1) / 256 * 256;
        let (seams, tail) = read.split_at_mut(seam_len);
        let mut open = open;
        for chunk in seams.chunks_mut(256) {
            let drained = self.blocking_read_seam_chunk(chunk, open)?;
            // End every non-final chunk with a STOP; a repeated START
            // right after the auto-NACK of a consumed RECEIVE command
            // is not reliably accepted on this silicon. `stop`
            // recovers its own failures.
            self.stop(drained)?;
            open = self.start_fresh(address, true)?;
        }
        self.blocking_read_seam_chunk(tail, open)
    }

    /// Drain ONE seam chunk (at most 256 bytes, one RECEIVE command)
    /// on the open transaction. Error returns drop the session, whose
    /// recovery closes the transaction.
    fn blocking_read_seam_chunk(&self, chunk: &mut [u8], mut open: Session) -> Result<Session, IOError> {
        // No wait for room: the start transition returned only after
        // the command FIFO drained, so one command always fits — and
        // like the async/DMA seam chunks, the RECEIVE must be the
        // FIRST statement so the session never exists without a data
        // command behind it (the drop invariant — see [`Session`]).
        self.enqueue_first_receive(chunk.len(), &mut open)?;

        let mut deadline = embassy_time::Instant::now() + self.timeout;
        for byte in chunk.iter_mut() {
            *byte = loop {
                match open.rx_step()? {
                    Some(SessionRxStep::Byte(b)) => {
                        deadline = embassy_time::Instant::now() + self.timeout;
                        break b;
                    }
                    Some(SessionRxStep::Ended) => return Err(IOError::UnexpectedStop),
                    // No progress for a full timeout window: Timeout,
                    // like the chained and async paths —
                    // UnexpectedStop is reserved for an observed
                    // termination.
                    None if embassy_time::Instant::now() > deadline => {
                        return Err(IOError::Timeout);
                    }
                    None => {}
                }
            };
        }

        Ok(open)
    }

    /// Write on a FRESH transaction, leaving it OPEN — see
    /// [`Self::blocking_read_txn_fresh`].
    fn blocking_write_txn_fresh(&self, address: u8, write: &[u8]) -> Result<Session, IOError> {
        let open = self.start_fresh(address, false)?;
        self.blocking_write_body(write, open)
    }

    /// Write continuing an open transaction, leaving the successor
    /// OPEN — see [`Self::blocking_read_txn_continue`].
    fn blocking_write_txn_continue(&self, address: u8, write: &[u8], open: Session) -> Result<Session, IOError> {
        let open = self.start_continue(address, false, open)?;
        self.blocking_write_body(write, open)
    }

    /// The write engine past the address phase. An error mid-write
    /// aborts the transaction with TRANSMIT commands still queued and
    /// the bus held; the error returns drop `open`, whose recovery
    /// cleans both up.
    fn blocking_write_body(&self, write: &[u8], mut open: Session) -> Result<Session, IOError> {
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

        for byte in write {
            self.enqueue_session_blocking(ControllerAction::transmit(*byte), &mut open)?;
        }

        Ok(open)
    }

    /// Read ending its FRESH transaction with a trailing STOP: nothing
    /// is returned to leak.
    fn blocking_read_close_fresh(&self, address: u8, read: &mut [u8]) -> Result<(), IOError> {
        let open = self.blocking_read_txn_fresh(address, read)?;
        self.stop(open)
    }

    /// Read consuming an open transaction and ending with a STOP.
    fn blocking_read_close_continue(&self, address: u8, read: &mut [u8], open: Session) -> Result<(), IOError> {
        let open = self.blocking_read_txn_continue(address, read, open)?;
        self.stop(open)
    }

    /// Write ending its FRESH transaction with a trailing STOP — see
    /// [`Self::blocking_read_close_fresh`].
    fn blocking_write_close_fresh(&self, address: u8, write: &[u8]) -> Result<(), IOError> {
        let open = self.blocking_write_txn_fresh(address, write)?;
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
        self.blocking_read_close_fresh(address, read)
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
        self.blocking_write_close_fresh(address, write)
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
        let open = self.blocking_write_txn_fresh(address, write)?;
        // The read half's repeated START consumes the write half's
        // session; its trailing STOP closes the transaction.
        self.blocking_read_close_continue(address, read, open)
    }
}

#[allow(private_bounds)]
impl<'d, M: AsyncMode> I2c<'d, M>
where
    Self: AsyncEngine,
{
    /// Bounded async enqueue for a live session. The session remains
    /// borrowed (and therefore owns cancellation cleanup) while a full
    /// command FIFO waits for an interrupt-driven wake.
    async fn enqueue_session_async(&self, action: ControllerAction, open: &mut Session) -> Result<(), IOError> {
        loop {
            if self.try_enqueue_session(action, open)? {
                return Ok(());
            }

            match embassy_time::with_timeout(
                self.timeout,
                self.info.wait_cell().wait_for(|| self.registers().tx_room_wake()),
            )
            .await
            {
                Ok(Ok(())) => {}
                Ok(Err(_)) => return Err(IOError::Other),
                Err(_) => return Err(IOError::Timeout),
            }
        }
    }

    /// Async START enqueue for an existing session. The successful facade
    /// call owns the `StartPending` transition; awaiting FIFO room cannot
    /// leave a START action and its recovery state out of sync.
    async fn enqueue_start_session_async(&self, action: StartAction, open: &mut Session) -> Result<(), IOError> {
        loop {
            if self.try_enqueue_start_session(action, open)? {
                return Ok(());
            }

            match embassy_time::with_timeout(
                self.timeout,
                self.info.wait_cell().wait_for(|| self.registers().tx_room_wake()),
            )
            .await
            {
                Ok(Ok(())) => {}
                Ok(Err(_)) => return Err(IOError::Other),
                Err(_) => return Err(IOError::Timeout),
            }
        }
    }

    /// Async twin of [`Self::settle_start_blocking`]. The interrupt predicate
    /// is only a wake source; each wake returns to the facade's typed
    /// fault-first/empty-FIFO poll, which is the sole way to mint a drained
    /// START witness.
    async fn settle_start_async(&self, open: &mut Session) -> Result<(), IOError> {
        self.assert_session_owner(open);
        let deadline = embassy_time::Instant::now() + self.timeout;
        let mut permit = open.start_status_permit();
        loop {
            match self.registers().poll_start_drain(permit) {
                StartDrainStep::Drained(drained) => {
                    return match self.registers().finish_start_status(drained) {
                        Ok(()) => Ok(()),
                        Err(fault) => Err(open.bind_fault(fault)),
                    };
                }
                StartDrainStep::Fault(fault) => return Err(open.bind_fault(fault)),
                StartDrainStep::Pending(next) => {
                    let now = embassy_time::Instant::now();
                    if now >= deadline {
                        drop(next);
                        return Err(IOError::Timeout);
                    }
                    permit = next;
                    match embassy_time::with_timeout(
                        deadline - now,
                        self.info.wait_cell().wait_for(|| self.registers().tx_settle_wake()),
                    )
                    .await
                    {
                        Ok(Ok(())) => {}
                        Ok(Err(_)) => return Err(IOError::Other),
                        Err(_) => return Err(IOError::Timeout),
                    }
                }
            }
        }
    }

    /// Async normal-STOP gate. As with the blocking twin, only this path may
    /// change the live session to `StopPending` before it is moved into the
    /// ownership-carrying completion wait.
    async fn enqueue_stop_session_async(&self, action: StopAction, open: &mut Session) -> Result<(), IOError> {
        loop {
            if self.try_enqueue_stop_session(action, open)? {
                return Ok(());
            }

            match embassy_time::with_timeout(
                self.timeout,
                self.info.wait_cell().wait_for(|| self.registers().tx_room_wake()),
            )
            .await
            {
                Ok(Ok(())) => {}
                Ok(Err(_)) => return Err(IOError::Other),
                Err(_) => return Err(IOError::Timeout),
            }
        }
    }

    /// Schedule sending a START command and await it being pulled from the FIFO.
    ///
    /// Does not indicate that the command was responded to.
    ///
    /// The wait is bounded by [`Config::transfer_timeout`] like its
    /// blocking counterpart: a target stretching SCL indefinitely
    /// satisfies neither the drain condition nor any error flag. Once
    /// the command enters MTDR, a pending [`Session`] owns every
    /// failure and cancellation path until the START status settles.
    /// The pre-command FIFO-room wait remains separately guarded because
    /// no command, and therefore no session, exists before acceptance.
    async fn async_start_fresh(&self, address: u8, read: bool) -> Result<Session, IOError> {
        if address >= 0x80 {
            return Err(IOError::AddressOutOfRange(address));
        }
        self.async_start_raw(address, read).await
    }

    /// Continue an open transaction with a repeated START. The predecessor
    /// stays live through any FIFO-room await, then is retargeted once the
    /// command is actually queued; cancellation therefore always has one
    /// session that can close the bus.
    async fn async_start_continue(&self, address: u8, read: bool, mut open: Session) -> Result<Session, IOError> {
        if address >= 0x80 {
            return Err(IOError::AddressOutOfRange(address));
        }
        self.assert_session_owner(&open);

        self.enqueue_start_session_async(self.start_action(address, read), &mut open)
            .await?;
        self.settle_start_async(&mut open).await?;
        Ok(open)
    }

    async fn async_start_raw(&self, address: u8, read: bool) -> Result<Session, IOError> {
        // The reservation precedes the START going on the wire — see
        // `reserve_session` — so every failure below must release it.
        let mut reservation = self.reserve_session();

        // A full FIFO is the one pre-command async wait that has no
        // session yet. Its guard uses the general recovery shape because
        // this fresh START has not been emitted until the gate says so.
        {
            let queued_guard = OnDrop::new(|| {
                session::remediate_before_session(&self.registers(), self.timeout);
            });
            loop {
                match self
                    .registers()
                    .try_enqueue_start(reservation.start_transition_permit(), self.start_action(address, read))
                {
                    CommandStep::Queued => break,
                    CommandStep::Fault(fault) => {
                        queued_guard.defuse();
                        let error = session::recover_before_session_fault(&self.registers(), self.timeout, fault);
                        return Err(error);
                    }
                    CommandStep::Full => {
                        match embassy_time::with_timeout(
                            self.timeout,
                            self.info.wait_cell().wait_for(|| self.registers().tx_room_wake()),
                        )
                        .await
                        {
                            Ok(Ok(())) => {}
                            Ok(Err(_)) => return Err(IOError::Other),
                            Err(_) => return Err(IOError::Timeout),
                        }
                    }
                }
            }
            queued_guard.defuse();
        }

        // From this point the queued START can execute autonomously, so
        // create its pending session before the first await. Drop owns
        // cancellation and late-fault cleanup, including the distinction
        // between a frozen START and its requested read successor.
        let mut open = reservation.into_pending_session(self.timeout);
        self.settle_start_async(&mut open).await?;
        Ok(open)
    }

    /// Schedule a STOP command and wait for it to PHYSICALLY complete
    /// — see [`Self::stop`] for the completion condition and the
    /// session contract: the session stays live across every wait, so
    /// an error return OR a drop-cancellation recovers via the
    /// session itself, exactly once. The old internal cancellation
    /// guard is gone — the session IS the guard now.
    ///
    /// The interrupt wake fires when the STOP is pulled from the FIFO
    /// (there is deliberately no EPF/SDF wake — spurious-flag
    /// pollution, see `enable_error_interrupts`), so the tail from
    /// "pulled" to "bus idle" is a bounded poll: a bit-time normally,
    /// a clock stretch capped by the deadline pathologically.
    async fn async_stop(&self, mut open: Session) -> Result<(), IOError> {
        // TX DMA completion does not prove the controller FIFO already
        // has room for a CPU STOP. The common gate keeps this close from
        // bypassing capacity/fault classification.
        self.enqueue_stop_session_async(StopAction::new(), &mut open).await?;

        // From here every loop value owns the STOP-pending session. A
        // cancellation/drop still runs Session's recovery, while a clean
        // path can reach `defuse` only through StopFinalized.
        let mut stop = open.into_stop_wait();

        let deadline = embassy_time::Instant::now() + self.timeout;
        let waited = embassy_time::with_timeout(
            self.timeout,
            self.info.wait_cell().wait_for(|| self.registers().tx_settle_wake()),
        )
        .await;

        match waited {
            Ok(Ok(())) => {}
            Ok(Err(_)) => return Err(IOError::Other),
            Err(_) => return Err(IOError::Timeout),
        }

        let mut spins = 0u32;
        let completed = loop {
            match self.registers().stop_step(stop) {
                StopStep::Completed(completed) => break completed,
                StopStep::Fault(fault) => return Err(fault.into_error()),
                StopStep::Pending(next) if embassy_time::Instant::now() > deadline => {
                    drop(next);
                    return Err(IOError::Timeout);
                }
                // Cooperative AND cancellable: the tail from "pulled"
                // to "bus idle" has no interrupt to sleep on, and with
                // no await point the future could neither yield nor be
                // dropped (a drop landing here is safe — `StopWait`
                // drops its session and recovers). The first yields cover the normal
                // tail (a bit-time) with no added latency; past them
                // the wait is a clock stretch, so back off onto the
                // timer and let the executor actually sleep instead of
                // self-waking through the run queue.
                StopStep::Pending(next) if spins < 64 => {
                    stop = next;
                    spins += 1;
                    embassy_futures::yield_now().await;
                }
                StopStep::Pending(next) => {
                    stop = next;
                    // Clamp the back-off to the remaining budget so a
                    // short configured timeout is not overrun by a
                    // whole sleep step (the residual overrun is one
                    // timer tick, inherent to a tick-based sleep).
                    let now = embassy_time::Instant::now();
                    if now >= deadline {
                        return Err(IOError::Timeout);
                    }
                    let step = core::cmp::min(deadline - now, embassy_time::Duration::from_micros(100));
                    embassy_time::Timer::after(step).await;
                }
            }
        };

        // See `stop`: classify-and-clear as this transaction's own.
        let finalized = match self.registers().finish_stop(completed) {
            Ok(finalized) => finalized,
            Err(fault) => return Err(fault.into_error()),
        };
        finalized.defuse();
        Ok(())
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
        self.async_read_close_fresh(address, read).await
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
        self.async_write_close_fresh(address, write).await
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
        // the bus: a rejection between the halves would recover (drop)
        // the write half's open transaction — safe, but a needless
        // wire round-trip for a doomed request.
        <Self as AsyncEngine>::read_preflight(self, read)?;
        let open = <Self as AsyncEngine>::async_write_txn_fresh(self, address, write).await?;
        // The read half's repeated START consumes the write half's
        // session; its trailing STOP closes the transaction.
        self.async_read_close_continue(address, read, open).await
    }

    /// Read ending its FRESH transaction with a trailing STOP: nothing
    /// is returned to leak.
    async fn async_read_close_fresh(&mut self, address: u8, read: &mut [u8]) -> Result<(), IOError> {
        let open = <Self as AsyncEngine>::async_read_txn_fresh(self, address, read).await?;
        self.async_stop(open).await
    }

    /// Read consuming an open transaction and ending with a STOP.
    async fn async_read_close_continue(&mut self, address: u8, read: &mut [u8], open: Session) -> Result<(), IOError> {
        let open = <Self as AsyncEngine>::async_read_txn_continue(self, address, read, open).await?;
        self.async_stop(open).await
    }

    /// Write ending its FRESH transaction with a trailing STOP — see
    /// [`Self::async_read_close_fresh`].
    async fn async_write_close_fresh(&mut self, address: u8, write: &[u8]) -> Result<(), IOError> {
        let open = <Self as AsyncEngine>::async_write_txn_fresh(self, address, write).await?;
        self.async_stop(open).await
    }
}

trait AsyncEngine {
    /// Validate a read request before ANY bus activity. Combined
    /// transactions run this before their write half; the read paths
    /// run it at entry. Must stay side-effect free.
    fn read_preflight(&self, read: &[u8]) -> Result<(), IOError>;

    /// Read on a FRESH transaction, leaving it OPEN (no trailing
    /// STOP): hands the session back to the caller, which must thread
    /// it onward. The close variants live on the driver
    /// (`async_read_close_fresh`).
    fn async_read_txn_fresh<'a>(
        &'a mut self,
        address: u8,
        read: &'a mut [u8],
    ) -> impl Future<Output = Result<Session, IOError>> + 'a;

    /// Read continuing an open transaction via repeated START, leaving
    /// the successor OPEN — see [`Self::async_read_txn_fresh`].
    fn async_read_txn_continue<'a>(
        &'a mut self,
        address: u8,
        read: &'a mut [u8],
        open: Session,
    ) -> impl Future<Output = Result<Session, IOError>> + 'a;

    /// Write on a FRESH transaction, leaving it OPEN — see
    /// [`Self::async_read_txn_fresh`].
    fn async_write_txn_fresh<'a>(
        &'a mut self,
        address: u8,
        write: &'a [u8],
    ) -> impl Future<Output = Result<Session, IOError>> + 'a;

    /// Write continuing an open transaction, leaving the successor
    /// OPEN — see [`Self::async_read_txn_continue`].
    fn async_write_txn_continue<'a>(
        &'a mut self,
        address: u8,
        write: &'a [u8],
        open: Session,
    ) -> impl Future<Output = Result<Session, IOError>> + 'a;
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
        let setup = Self::construction_setup::<T>(&config)?;
        let inst = Self::new_inner(peri, scl, sda, config, setup, Async)?;

        T::Interrupt::unpend();

        // Safety: `_irq` ensures an Interrupt Handler exists.
        unsafe { T::Interrupt::enable() };

        Ok(inst)
    }
}

impl<'d> I2c<'d, Async> {
    /// The read engine past the address phase: drain the open
    /// transaction with chained RECEIVE commands, falling back (only
    /// when the caller opted in) to STOP-seamed re-addressed chunks.
    async fn async_read_resume(&self, address: u8, read: &mut [u8], open: Session) -> Result<Session, IOError> {
        // A chained read that died mid-transfer leaves the device in an
        // unknown state: its pointer has advanced by however many bytes
        // were clocked out, and destructive reads have already consumed
        // them. Re-reading from the caller's buffer start would return
        // shifted data as success, so it happens only when the caller
        // has explicitly accepted that trade.
        match self.async_read_chained(read, open).await {
            Ok(open) => Ok(open),
            Err(e @ (IOError::UnexpectedStop | IOError::Timeout)) if self.allow_chunked_reads => {
                #[cfg(feature = "defmt")]
                defmt::trace!("chained read failed ({}); retrying chunked (opted in)", e);
                let _ = e;
                // The failed chained attempt's session dropped on its
                // error path — recovering and closing the bus — so the
                // fallback starts fresh.
                let open = self.async_start_fresh(address, true).await?;
                self.async_read_seamed(address, read, open).await
            }
            Err(e) => Err(e),
        }
    }

    /// One read as a single addressed transaction with chained RECEIVE
    /// commands on the already-open transaction. Does not send the
    /// trailing STOP. Every abort past the address phase — an error
    /// return or this future dropped mid-await — drops `open`, whose
    /// recovery closes the transaction; no separate guard exists to
    /// stack a second recovery on top.
    async fn async_read_chained(&self, read: &mut [u8], mut open: Session) -> Result<Session, IOError> {
        let total = read.len();
        // First command unconditionally — see `blocking_read_chained`.
        let first = total.min(256);
        self.enqueue_first_receive(first, &mut open)?;
        // Chain RECEIVE commands under the single address phase: the
        // controller ACKs across a command boundary only when the next
        // command is already queued (otherwise it NACKs and terminates
        // the read early), so the command pipeline is kept ahead of the
        // data. This preserves >256-byte reads as ONE bus transaction —
        // no repeated START (unreliable after the auto-NACK on this
        // silicon) and no STOP seams (which would break embedded-hal
        // transaction atomicity and let another controller interleave).
        let mut queued = first;
        let mut drained = 0usize;
        while drained < total {
            // Top up the command pipeline whenever there is room.
            while queued < total {
                match self.try_enqueue_read_receive((total - queued).min(256), &mut open)? {
                    true => {
                        let chunk = (total - queued).min(256);
                        queued += chunk;
                    }
                    // Command FIFO full: plenty is queued ahead of the
                    // data; go drain some of it.
                    false => break,
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

            match open.rx_step()? {
                Some(SessionRxStep::Byte(b)) => {
                    read[drained] = b;
                    drained += 1;
                }
                Some(SessionRxStep::Ended) => return Err(IOError::UnexpectedStop),
                // Nothing pending after a full timeout window: the
                // transfer stalled or died without a flag.
                None if timed_out => return Err(IOError::Timeout),
                // `rx_step()?` already returned a bound fault. A spurious
                // wake that cleared before its snapshot simply waits again.
                None => {}
            }
        }

        Ok(open)
    }

    /// Fallback: one read as re-addressed 256-byte chunks, each ended
    /// with a STOP. Not atomic, but immune to the chained-boundary
    /// early-termination quirk. `open` is the first chunk's
    /// already-open transaction; the final chunk's is returned OPEN.
    async fn async_read_seamed(&self, address: u8, read: &mut [u8], open: Session) -> Result<Session, IOError> {
        // The STOP-seamed chunks up front, the final chunk (1..=256
        // bytes — the preflight guarantees a non-empty read) apart, so
        // the session is threaded linearly and never conditionally
        // moved.
        let seam_len = (read.len() - 1) / 256 * 256;
        let (seams, tail) = read.split_at_mut(seam_len);
        let mut open = open;
        for chunk in seams.chunks_mut(256) {
            let drained = self.async_read_seam_chunk(chunk, open).await?;
            // End every non-final chunk with a STOP; a repeated START
            // right after the auto-NACK of a consumed RECEIVE command
            // is not reliably accepted on this silicon. `async_stop`
            // recovers its own failures.
            self.async_stop(drained).await?;
            open = self.async_start_fresh(address, true).await?;
        }
        self.async_read_seam_chunk(tail, open).await
    }

    /// Drain ONE seam chunk (at most 256 bytes, one RECEIVE command)
    /// on the open transaction. Aborts — error returns and
    /// drop-cancellation alike — drop the session, whose recovery
    /// closes the transaction.
    async fn async_read_seam_chunk(&self, chunk: &mut [u8], mut open: Session) -> Result<Session, IOError> {
        self.enqueue_first_receive(chunk.len(), &mut open)?;

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

                match open.rx_step()? {
                    Some(SessionRxStep::Byte(b)) => {
                        *byte = b;
                        break;
                    }
                    Some(SessionRxStep::Ended) => return Err(IOError::UnexpectedStop),
                    None if timed_out => return Err(IOError::Timeout),
                    None => {}
                }
            }
        }

        Ok(open)
    }

    /// The write engine past the address phase. Aborts — error returns
    /// and drop-cancellation alike — drop `open`, whose recovery
    /// closes the transaction.
    async fn async_write_body(&self, write: &[u8], mut open: Session) -> Result<Session, IOError> {
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

        for byte in write {
            // initiate transmit
            self.enqueue_session_async(ControllerAction::transmit(*byte), &mut open)
                .await?;

            match embassy_time::with_timeout(
                self.timeout,
                self.info.wait_cell().wait_for(|| self.registers().tx_settle_wake()),
            )
            .await
            {
                Ok(Ok(())) => {}
                Ok(Err(_)) => return Err(IOError::WriteFail),
                // No progress within the transfer timeout (a target
                // holding SCL low): the session drop closes the
                // transaction on this return.
                Err(_) => return Err(IOError::Timeout),
            }

            // This is classification only: recovery belongs to the
            // session's drop on the error return, keeping
            // recovery exactly-once. And
            // HALT-PRESERVING: byte N can NACK after byte N+1 was
            // queued (the settle wake is "pulled from the FIFO", and
            // tx_settled also trips on the error flag with the next
            // byte still queued), so a raw clearing take would un-halt
            // the engine over that queued suffix — which, with no
            // auto-STOP fired (the FIFO was empty at the NACK
            // instant), continues the still-open transaction and
            // delivers the suffix to the target AFTER this call
            // returned failure. Preserved, the suffix stays frozen
            // until the session drop discards it.
            if let Some(fault) = self.registers().take_active_fault() {
                return Err(open.bind_fault(fault));
            }
        }

        Ok(open)
    }
}

impl<'d> AsyncEngine for I2c<'d, Async> {
    fn read_preflight(&self, read: &[u8]) -> Result<(), IOError> {
        if read.is_empty() {
            return Err(IOError::InvalidReadBufferLength);
        }
        Ok(())
    }

    async fn async_read_txn_fresh(&mut self, address: u8, read: &mut [u8]) -> Result<Session, IOError> {
        self.read_preflight(read)?;
        let open = self.async_start_fresh(address, true).await?;
        self.async_read_resume(address, read, open).await
    }

    async fn async_read_txn_continue(
        &mut self,
        address: u8,
        read: &mut [u8],
        open: Session,
    ) -> Result<Session, IOError> {
        // Preflight BEFORE the repeated START touches the bus; on a
        // rejection the moved-in session drops — recovering the open
        // transaction rather than silently abandoning it.
        self.read_preflight(read)?;
        let open = self.async_start_continue(address, true, open).await?;
        self.async_read_resume(address, read, open).await
    }

    async fn async_write_txn_fresh(&mut self, address: u8, write: &[u8]) -> Result<Session, IOError> {
        let open = self.async_start_fresh(address, false).await?;
        self.async_write_body(write, open).await
    }

    async fn async_write_txn_continue(&mut self, address: u8, write: &[u8], open: Session) -> Result<Session, IOError> {
        let open = self.async_start_continue(address, false, open).await?;
        self.async_write_body(write, open).await
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
        let setup = Self::construction_setup::<T>(&config)?;

        // enable this channel's interrupt
        let tx_dma = DmaChannel::new(tx_dma);
        let rx_dma = DmaChannel::new(rx_dma);

        tx_dma.enable_interrupt();
        rx_dma.enable_interrupt();

        let inst = Self::new_inner(
            peri,
            scl,
            sda,
            config,
            setup,
            Dma {
                tx_dma,
                rx_dma,
                tx_request: T::TX_DMA_REQUEST,
                rx_request: T::RX_DMA_REQUEST,
            },
        )?;

        T::Interrupt::unpend();

        // Safety: `_irq` ensures an Interrupt Handler exists.
        unsafe { T::Interrupt::enable() };

        Ok(inst)
    }
}

impl<'d> I2c<'d, Dma<'d>> {
    /// Brand the instance's configured RX channel/request pair for the
    /// controller facade. This is the one wiring point for RX DMA; callers
    /// cannot substitute the TX pair at the arm site.
    fn controller_rx_dma(&self) -> ControllerRxDma<'_, 'd> {
        ControllerRxDma::new(self.registers().identity(), &self.mode.rx_dma, self.mode.rx_request)
    }

    /// Brand the instance's configured TX channel/request pair. See
    /// [`Self::controller_rx_dma`] for the pairing guarantee.
    fn controller_tx_dma(&self) -> ControllerTxDma<'_, 'd> {
        ControllerTxDma::new(self.registers().identity(), &self.mode.tx_dma, self.mode.tx_request)
    }

    /// Run one RX DMA transfer covering `buf`, waking on completion or on
    /// a bus fault (which would otherwise leave the DMA waiting forever).
    /// The returned lease owns the raw FIFO endpoint, the MDER/ERQ sequence,
    /// and the cancellation cleanup; this function only waits and binds a
    /// typed fault to the session before its public `IOError` can escape.
    async fn dma_read_into(&self, buf: &mut [u8], open: &mut Session) -> Result<(), IOError> {
        // Compute this before borrowing `buf` into the DMA lease. The lease
        // keeps that borrow live until its Drop/finish quiesces the channel.
        let bound = self.timeout + embassy_time::Duration::from_millis(buf.len() as u64);
        let lease = self
            .registers()
            .arm_rx_dma(open.rx_dma_permit(), self.controller_rx_dma(), buf)?;

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
            if lease.poll_complete(cx) {
                return core::task::Poll::Ready(Ok(()));
            }
            while self.info.wait_cell().poll_wait(cx).is_ready() {}
            // The interrupt handler disables MIER on wake; re-arm the
            // error sources and check, as one operation.
            if let Some(fault) = self.registers().error_wake() {
                return core::task::Poll::Ready(Err(DmaWaitError::Fault(fault)));
            }
            // Early termination with the DMA incomplete: no more data
            // will arrive and the DMA would wait forever.
            if self.registers().transfer_ended() {
                return core::task::Poll::Ready(Err(DmaWaitError::UnexpectedStop));
            }
            core::task::Poll::Pending
        });
        let result = embassy_time::with_timeout(bound, wait).await;
        // Consume the lease before touching `open`: it both releases the
        // session borrow and preserves a first-RECEIVE execution proof from
        // the final DMA/FIFO state.
        let _ = lease.finish();

        match result {
            Ok(Ok(())) => {}
            Ok(Err(DmaWaitError::Fault(fault))) => return Err(open.bind_fault(fault)),
            Ok(Err(DmaWaitError::UnexpectedStop)) => return Err(IOError::UnexpectedStop),
            Err(_) => return Err(IOError::Timeout),
        }

        // Completion and a bus error can race: the polling fast path sees
        // the completed major loop first, so take one final typed status
        // snapshot only after the channel is quiesced.
        if let Some(fault) = self.registers().take_active_fault() {
            return Err(open.bind_fault(fault));
        }

        Ok(())
    }
}

impl<'d> I2c<'d, Dma<'d>> {
    /// The DMA read engine past the FIRST address phase: chained when
    /// the command FIFO can hold the whole pipeline, seamed otherwise
    /// (the preflight guaranteed the opt-in) or as the opted-in
    /// fallback after a failed chained attempt.
    async fn dma_read_resume(&self, address: u8, read: &mut [u8], open: Session) -> Result<Session, IOError> {
        // Chain all RECEIVE commands under a single address phase when
        // they fit the command FIFO: the controller ACKs across a
        // command boundary only when the next command is already
        // queued, so a read up to capacity*256 bytes stays ONE bus
        // transaction. Longer reads cannot be served atomically here at
        // all, because nothing refills the command FIFO while the CPU
        // sleeps on the DMA completion — they go straight to the
        // seamed path, whose first chunk runs on the already-open
        // transaction.
        let ncmds = read.len().div_ceil(256);
        if ncmds > self.registers().tx_fifo_capacity() {
            return self.dma_read_seamed(address, read, open).await;
        }
        match self.dma_read_chained(read, open).await {
            Ok(open) => Ok(open),
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
                // The failed chained attempt's session dropped on its
                // error path — recovering and closing the bus — so the
                // fallback starts fresh.
                let open = self.async_start_fresh(address, true).await?;
                self.dma_read_seamed(address, read, open).await
            }
            Err(e) => Err(e),
        }
    }

    /// One read as a single addressed transaction: every RECEIVE command
    /// queued up front (the caller checked they fit the FIFO), one DMA
    /// transfer over the whole buffer, on the already-open transaction.
    /// Does not send the trailing STOP.
    ///
    /// A live `RxDmaLease` owns the DMA handoff. On any cancellation path it
    /// drops before this session, first disabling MDER and quiescing eDMA,
    /// then allowing session recovery to reset FIFOs or release `read`.
    async fn dma_read_chained(&self, read: &mut [u8], mut open: Session) -> Result<Session, IOError> {
        // Queue every RECEIVE command up front (they fit). The first
        // goes in unconditionally — see `blocking_read_chained`.
        let total = read.len();
        let first = total.min(256);
        self.enqueue_first_receive(first, &mut open)?;
        let mut queued = first;
        let queue_deadline = embassy_time::Instant::now() + self.timeout;
        while queued < total {
            match self.try_enqueue_read_receive((total - queued).min(256), &mut open)? {
                true => {
                    let chunk = (total - queued).min(256);
                    queued += chunk;
                }
                // The CPU queues the full DMA pipeline up front. A
                // stretched START must not turn that spin into an
                // unbounded wait just because no data DMA is active yet.
                false if embassy_time::Instant::now() > queue_deadline => return Err(IOError::Timeout),
                false => {}
            }
        }

        self.dma_read_into(read, &mut open).await?;
        Ok(open)
    }

    /// Seamed path: one read as re-addressed 256-byte chunks, each
    /// ended with a STOP — see `async_read_seamed` for the trade.
    /// `open` is the first chunk's already-open transaction (on the
    /// straight-to-seamed path it continues the caller's carry); the
    /// final chunk's is returned OPEN.
    async fn dma_read_seamed(&self, address: u8, read: &mut [u8], open: Session) -> Result<Session, IOError> {
        // The STOP-seamed chunks up front, the final chunk (1..=256
        // bytes — the preflight guarantees a non-empty read) apart, so
        // the session is threaded linearly and never conditionally
        // moved.
        let seam_len = (read.len() - 1) / 256 * 256;
        let (seams, tail) = read.split_at_mut(seam_len);
        let mut open = open;
        for chunk in seams.chunks_mut(256) {
            let drained = self.dma_read_seam_chunk(chunk, open).await?;
            // End every non-final chunk with a STOP: a repeated START
            // right after the auto-NACK of a consumed RECEIVE command
            // is not reliably accepted on this silicon. `async_stop`
            // recovers its own failures.
            self.async_stop(drained).await?;
            open = self.async_start_fresh(address, true).await?;
        }
        self.dma_read_seam_chunk(tail, open).await
    }

    /// Drain ONE seam chunk (at most 256 bytes: one RECEIVE command,
    /// one DMA transfer) on the open transaction. Its scoped DMA lease
    /// quiesces before any session recovery can run.
    async fn dma_read_seam_chunk(&self, chunk: &mut [u8], mut open: Session) -> Result<Session, IOError> {
        // send receive command
        self.enqueue_first_receive(chunk.len(), &mut open)?;

        self.dma_read_into(chunk, &mut open).await?;
        Ok(open)
    }

    /// The DMA write engine past the address phase. Each scoped TX lease
    /// owns request shutdown and quiescence before recovery can touch FIFOs
    /// or the caller's write borrow can end.
    async fn dma_write_body(&self, write: &[u8], mut open: Session) -> Result<Session, IOError> {
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

        for chunk in write.chunks(DMA_MAX_TRANSFER_SIZE) {
            // Compute the deadline before the TX lease retains the source
            // borrow. It will not release that borrow until eDMA is idle.
            let bound = self.timeout + embassy_time::Duration::from_millis(chunk.len() as u64);
            let lease = self
                .registers()
                .arm_tx_dma(open.tx_dma_permit(), self.controller_tx_dma(), chunk)?;

            // Wait for completion asynchronously — or for a bus error
            // (NACK, arbitration loss, FIFO error) that stops the
            // transfer, in which case the DMA would never complete and
            // waiting on it alone would hang forever. Bounded like the
            // read path: the silicon can terminate a transfer silently
            // (no flag, no interrupt), which only a timeout can catch.
            let wait = core::future::poll_fn(|cx| {
                // Drain stale tokens and finish registered — see
                // `dma_read_into`.
                if lease.poll_complete(cx) {
                    return core::task::Poll::Ready(Ok(()));
                }
                while self.info.wait_cell().poll_wait(cx).is_ready() {}
                // The interrupt handler disables MIER on wake; re-arm the
                // error sources and check, as one operation.
                if let Some(fault) = self.registers().error_wake() {
                    return core::task::Poll::Ready(Err(DmaWaitError::Fault(fault)));
                }
                core::task::Poll::Pending
            });
            let result = embassy_time::with_timeout(bound, wait).await;
            // The lease is consumed before an error can hand a fault to the
            // session, so no DMA write remains live while recovery runs.
            let _ = lease.finish();
            match result {
                Ok(Ok(())) => {}
                Ok(Err(DmaWaitError::Fault(fault))) => return Err(open.bind_fault(fault)),
                Ok(Err(DmaWaitError::UnexpectedStop)) => unreachable!("TX DMA does not wait for receive termination"),
                // `lease.finish` already shut down the DMA request path;
                // dropping the session now owns only bus recovery.
                Err(_) => return Err(IOError::Timeout),
            }

            // As in RX, a major-loop completion can win the poll race
            // against a simultaneous controller fault. The channel is
            // quiesced before the proof is handed to the session.
            if let Some(fault) = self.registers().take_active_fault() {
                return Err(open.bind_fault(fault));
            }
        }

        Ok(open)
    }
}

impl<'d> AsyncEngine for I2c<'d, Dma<'d>> {
    fn read_preflight(&self, read: &[u8]) -> Result<(), IOError> {
        if read.is_empty() {
            return Err(IOError::InvalidReadBufferLength);
        }
        // Longer reads than the command FIFO can chain cannot be
        // served atomically here — see `dma_read_resume`.
        let ncmds = read.len().div_ceil(256);
        if ncmds > self.registers().tx_fifo_capacity() && !self.allow_chunked_reads {
            return Err(IOError::ChunkingRequired);
        }
        Ok(())
    }

    async fn async_read_txn_fresh(&mut self, address: u8, read: &mut [u8]) -> Result<Session, IOError> {
        self.read_preflight(read)?;
        let open = self.async_start_fresh(address, true).await?;
        self.dma_read_resume(address, read, open).await
    }

    async fn async_read_txn_continue(
        &mut self,
        address: u8,
        read: &mut [u8],
        open: Session,
    ) -> Result<Session, IOError> {
        // Preflight BEFORE the repeated START touches the bus; on a
        // rejection the moved-in session drops — recovering the open
        // transaction rather than silently abandoning it.
        self.read_preflight(read)?;
        let open = self.async_start_continue(address, true, open).await?;
        self.dma_read_resume(address, read, open).await
    }

    async fn async_write_txn_fresh(&mut self, address: u8, write: &[u8]) -> Result<Session, IOError> {
        let open = self.async_start_fresh(address, false).await?;
        self.dma_write_body(write, open).await
    }

    async fn async_write_txn_continue(&mut self, address: u8, write: &[u8], open: Session) -> Result<Session, IOError> {
        let open = self.async_start_continue(address, false, open).await?;
        self.dma_write_body(write, open).await
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

        if let Some((first, rest)) = operations.split_first_mut() {
            // The first op opens the chain; each further op's repeated
            // START consumes its predecessor's session; the final
            // `stop` closes the chain. The session threads linearly —
            // any error return drops it, and the drop recovers.
            let mut open = match first {
                embedded_hal_02::blocking::i2c::Operation::Read(buf) => self.blocking_read_txn_fresh(address, buf)?,
                embedded_hal_02::blocking::i2c::Operation::Write(buf) => self.blocking_write_txn_fresh(address, buf)?,
            };
            for op in rest {
                open = match op {
                    embedded_hal_02::blocking::i2c::Operation::Read(buf) => {
                        self.blocking_read_txn_continue(address, buf, open)?
                    }
                    embedded_hal_02::blocking::i2c::Operation::Write(buf) => {
                        self.blocking_write_txn_continue(address, buf, open)?
                    }
                };
            }
            self.stop(open)
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_implemented_speeds_can_cross_the_setup_boundary() {
        assert!(Speed::Standard.supported().is_ok());
        assert!(Speed::Fast.supported().is_ok());
        assert!(Speed::FastPlus.supported().is_ok());
        assert!(matches!(
            Speed::UltraFast.supported(),
            Err(SetupError::UnsupportedSpeed)
        ));
    }
}

impl<'d, M: Mode> embedded_hal_1::i2c::I2c for I2c<'d, M> {
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal_1::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        // Reject a doomed read before ANY op touches the bus — see
        // `exec` above.
        for op in operations.iter() {
            if let embedded_hal_1::i2c::Operation::Read(buf) = op {
                if buf.is_empty() {
                    return Err(IOError::InvalidReadBufferLength);
                }
            }
        }

        // No trailing cleanup: every abort recovers by construction —
        // pre-session failures inside `start`, post-defuse failures
        // inside `stop`, everything in between via the session's drop.
        if let Some((first, rest)) = operations.split_first_mut() {
            // See `exec` above for the threading.
            let mut open = match first {
                embedded_hal_1::i2c::Operation::Read(buf) => self.blocking_read_txn_fresh(address, buf)?,
                embedded_hal_1::i2c::Operation::Write(buf) => self.blocking_write_txn_fresh(address, buf)?,
            };
            for op in rest {
                open = match op {
                    embedded_hal_1::i2c::Operation::Read(buf) => self.blocking_read_txn_continue(address, buf, open)?,
                    embedded_hal_1::i2c::Operation::Write(buf) => {
                        self.blocking_write_txn_continue(address, buf, open)?
                    }
                };
            }
            self.stop(open)
        } else {
            Ok(())
        }
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
        // here); a mid-list `read_preflight` rejection — empty buffer,
        // `ChunkingRequired` on the DMA engine — would drop (and so
        // recover) the open session, which is safe but wastes the
        // whole partial transaction on a request that was doomed from
        // the start.
        for op in operations.iter() {
            if let embedded_hal_async::i2c::Operation::Read(buf) = op {
                <Self as AsyncEngine>::read_preflight(self, buf)?;
            }
        }

        // No trailing cleanup: every abort — error returns and
        // drop-cancellation alike — recovers by construction (see the
        // blocking `transaction` above).
        if let Some((first, rest)) = operations.split_first_mut() {
            // See `exec` above for the threading.
            let mut open = match first {
                embedded_hal_async::i2c::Operation::Read(buf) => {
                    <Self as AsyncEngine>::async_read_txn_fresh(self, address, buf).await?
                }
                embedded_hal_async::i2c::Operation::Write(buf) => {
                    <Self as AsyncEngine>::async_write_txn_fresh(self, address, buf).await?
                }
            };
            for op in rest {
                open = match op {
                    embedded_hal_async::i2c::Operation::Read(buf) => {
                        <Self as AsyncEngine>::async_read_txn_continue(self, address, buf, open).await?
                    }
                    embedded_hal_async::i2c::Operation::Write(buf) => {
                        <Self as AsyncEngine>::async_write_txn_continue(self, address, buf, open).await?
                    }
                };
            }
            self.async_stop(open).await
        } else {
            Ok(())
        }
    }
}

impl<'d, M: Mode> embassy_embedded_hal::SetConfig for I2c<'d, M> {
    type Config = Config;
    type ConfigError = SetupError;

    fn set_config(&mut self, config: &Self::Config) -> Result<(), SetupError> {
        let speed = config.speed.supported()?;
        let setup = ControllerSetup::new(self.freq, speed, config.pin_low_timeout)?;
        self.set_configuration(setup);
        self.allow_chunked_reads = config.allow_chunked_reads;
        self.timeout = config.transfer_timeout;
        Ok(())
    }
}
