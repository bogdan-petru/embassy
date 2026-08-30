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

/// An open bus transaction — a session whose drop path performs safe
/// recovery.
///
/// Produced only by the `start_fresh`/`start_continue` transitions;
/// consumed by `stop`/`async_stop` or by the next `start_continue`
/// (a repeated START takes the predecessor over on the wire). The
/// engines are split into `*_txn_*` operations, which leave the
/// session open and hand it back, and `*_close*` operations, which
/// consume it with a trailing STOP.
///
/// What each tier enforces:
///
/// - **Compile-enforced**: a driver-initiated trailing stop cannot be
///   issued without a session (`remediation`'s recovery STOP is
///   cleanup, outside the protocol); no operation both ends a
///   transaction and yields a session; a session cannot be used twice
///   (no `Copy`/`Clone`); and there is no fresh-start entry point that
///   accepts an optional continuation — continuing and starting fresh
///   are different functions, so "pass nothing while holding a live
///   session" is not an expressible call.
/// - **Drop-enforced**: ABANDONMENT IS RECOVERY. A session dropped on
///   any path — an error unwind, a cancelled future, plain forgetting
///   to thread it — runs the same self-contained remediation the
///   recovery arms use, closing the transaction and releasing the
///   bus. The old silent-abandonment hole is not merely linted away;
///   it now has defined, safe behavior. (Cleanup that is *channel*
///   -specific — DMA quiesce — stays in the quiesce-only guards,
///   which drop before the session by declaration order.)
/// - **Runtime-enforced**: the session carries its controller's shared
///   state and every consumption asserts identity, so a session from
///   another instance fails deterministically.
#[must_use]
struct Session {
    /// The owning controller's shared state — enough to recover
    /// without borrowing the controller (which a drop path cannot).
    info: &'static Info,
    /// The owner's transfer timeout at session start, bounding the
    /// recovery drain.
    timeout: embassy_time::Duration,
    /// Whether the transaction's most recent START addressed a READ.
    /// Recovery needs it: aborting a read must clock+NACK one byte
    /// before the STOP (see `remediate`), and the wire direction is
    /// not observable from the registers.
    read: bool,
}

impl Session {
    /// Consume without recovery: the transaction reached its defined
    /// end (a STOP was issued, or a successor START took it over).
    fn defuse(self) {
        core::mem::forget(self);
    }
}

impl Drop for Session {
    fn drop(&mut self) {
        // A session always postdates the address phase, and every
        // read engine issues its FIRST data command unconditionally,
        // before any fallible or awaitable step (the chained engines
        // and every seam chunk lead with their RECEIVE — enforced so
        // this mapping stays sound). An abandoned read is therefore
        // always aborted as ReadStreaming: its RECEIVE ends in the
        // auto-NACK that releases the target — no extra clocking
        // needed, and a command injected after that auto-NACK would
        // trip the repeated-command silicon quirk.
        let abort = if self.read {
            Abort::ReadStreaming
        } else {
            Abort::General
        };
        remediate(&ControllerRegisters::new(self.info.regs()), self.timeout, abort);
    }
}

/// What kind of wire state a recovery is unwinding — the choice of
/// closing sequence depends on it, and it is not observable from the
/// registers (hardware-diagnosed via the drop-cancellation rig test):
///
/// * a READ whose address may have ACKed with no data command behind
///   it leaves the TARGET driving SDA, where a bare STOP can never
///   form (the engine commits and wedges, unrecoverable even by an
///   engine reset — the target holds the bus);
/// * a READ with its data command issued must NOT have an extra
///   RECEIVE appended (the in-flight command's auto-NACK already
///   releases the target, and a command after an auto-NACK is the
///   documented unreliable shape on this silicon);
/// * everything else closes with a bare STOP.
#[derive(Clone, Copy, PartialEq)]
enum Abort {
    /// Writes, STARTs that never reached the wire, STOP failures,
    /// fault-halted engines: a bare STOP always forms.
    General,
    /// A read aborted between its START going out and its first data
    /// command: clock ONE byte (RECEIVE, count 0) so the auto-NACK
    /// makes the target release SDA, then STOP.
    ReadAddressed,
    /// A read aborted with data command(s) issued: bare STOP behind
    /// the in-flight command's own auto-NACK.
    ReadStreaming,
}

/// Self-contained bus recovery: reset the FIFOs, clear the latched
/// faults, push a recovery STOP, wait (bounded) for it to drain, and
/// leave a clean slate. Free-standing so [`Session`]'s drop can run it
/// without borrowing the controller; `I2c::remediation` is the same
/// code.
fn remediate(regs: &ControllerRegisters, timeout: embassy_time::Duration, abort: Abort) {
    #[cfg(feature = "defmt")]
    defmt::trace!("Recovering controller",);

    // Recovery must not re-enter the fault-aware wait/classify
    // paths that lead here (`take_status_and_recover`, the session
    // drop): with a fault that keeps re-latching, that cycle
    // recurses until the stack overflows. Everything below is
    // self-contained.
    //
    // Resetting the FIFOs drops whatever the aborted transfer left
    // queued — but a FIFO reset issued while the engine is ACTIVELY
    // RUNNING a command corrupts its transaction bookkeeping
    // (hardware-observed: the closing STOP then forms on the wire,
    // EPF/SDF latch, yet MBF/BBF stick busy forever and later
    // commands are ignored — a state not even an engine reset fully
    // unwinds). So the entry reset runs only when the engine is idle
    // or halted on a latched fault; an active abort keeps its queued
    // commands and lets the pipeline run out under the drain below,
    // which is bounded: at most a FIFO's worth of commands.
    let busy = regs.master_busy();
    let halted = regs.read_status().error().is_some();
    if !busy || halted {
        regs.reset_fifos();
    }
    regs.clear_all_status();

    // The recovery STOP is meaningful ONLY while a transfer is open
    // on the wire. Queued onto an idle controller (a NACK the
    // hardware already auto-STOPped, an abort that never reached the
    // bus) it is a protocol violation the engine refuses with FEF and
    // never consumes — the drain below would then burn its whole
    // deadline for nothing (hardware-observed on ordinary NACK
    // recoveries once the drain stopped exiting on latched faults).
    // An idle controller needs no closing: reset, clear, done.
    //
    // Re-sampled: the engine fetches queued commands autonomously, so
    // the entry sample can go stale idle→busy between it and the
    // reset above (a dropped start whose START was still queued). The
    // drain then closes the now-running transaction properly; if that
    // reset DID land on the just-started engine and wedge it, the
    // drain cannot settle and its deadline arm escalates to the
    // engine reset — a bounded ending instead of a silent skip. (The
    // opposite staleness, busy→idle, is the drain's 500 µs
    // idle-with-close-pending break below.)
    if busy || regs.master_busy() {
        let deadline = embassy_time::Instant::now() + timeout;
        // The closing sequence is shape-specific — see [`Abort`] —
        // and is queued once there is room behind whatever the abort
        // left pending (those commands run out first; a read
        // pipeline's final byte auto-NACKs, which is exactly what
        // frees the target for the STOP).
        let need = if abort == Abort::ReadAddressed { 2usize } else { 1 };
        let mut queued = false;
        let mut idle_since: Option<embassy_time::Instant> = None;
        loop {
            // The entry `busy` sample can go stale within microseconds
            // (an auto-STOP or fault-terminated transfer finishing as
            // recovery enters). The close then targets an IDLE engine:
            // a protocol violation it refuses with FEF and — with the
            // fault scrub below re-clearing it — retries forever, a
            // livelock that would burn the whole deadline (hardware-
            // observed at tens of thousands of scrubs per burn). An
            // engine that stays idle with the close still pending was
            // never going to run it: the transaction already closed
            // itself, which is all recovery wanted — drop the bogus
            // close (trailing FIFO reset) and leave. The persistence
            // window rides out the legitimate µs-scale fetch gap
            // between queueing a command and MBF asserting.
            if !regs.master_busy() && regs.tx_pending() > 0 {
                let now = embassy_time::Instant::now();
                let since = *idle_since.get_or_insert(now);
                if now - since > embassy_time::Duration::from_micros(500) {
                    break;
                }
            } else {
                idle_since = None;
            }
            // Keep the RX FIFO empty: an abandoned in-flight RECEIVE
            // stalls the engine in SCL flow control (no fault!) the
            // moment the un-popped FIFO fills, and would otherwise
            // never finish — see `discard_rx`.
            regs.discard_rx();

            if !queued && regs.tx_fifo_capacity() - regs.tx_pending() >= need {
                if abort == Abort::ReadAddressed {
                    regs.write_command(ControllerCommand::RECEIVE, 0);
                }
                regs.write_command(ControllerCommand::STOP, 0);
                queued = true;
            }

            // Settled only counts once the closing commands are in:
            // the engine idles between the aborted pipeline and the
            // close, and exiting there would leave the transaction
            // open.
            if queued && regs.recovery_settled() {
                break;
            }

            // The master HALTS on a latched fault and consumes no
            // further commands until it is cleared — so a fault
            // observed mid-drain is scrubbed (only when actually
            // latched: a tight unconditional clear loop hammering MSR
            // disturbed otherwise-clean drains on hardware) and the
            // wait continues; recovery has no caller to classify for.
            if regs.read_status().error().is_some() {
                regs.clear_all_status();
            }

            // A target holding SCL low satisfies no exit condition,
            // so the wait is bounded like every other — recovery must
            // not be the one path that can still hang — and on expiry
            // the engine is hard-reset: whatever holds it, the abort
            // must complete and release this side of the bus.
            if embassy_time::Instant::now() > deadline {
                #[cfg(feature = "defmt")]
                defmt::warn!("recovery close did not settle within the transfer timeout; resetting the engine");
                regs.reset_engine();
                break;
            }
        }
    }

    // Now provably past the active abort (or hard-reset): drop
    // whatever remains queued or received so the next transaction
    // starts from a clean slate.
    regs.reset_fifos();

    // Clear any residual MSR flags raised by the recovery close
    // (FEF in particular) so the next transaction starts clean.
    regs.clear_current_status();
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

    /// Recovery for non-read-abort contexts: START failures before the
    /// address went out, STOP failures (data phase already complete),
    /// and fault-halted engines (`take_status_and_recover`), where a
    /// bare recovery STOP always forms. Read-abort contexts carry
    /// their direction instead — see [`Session`] and the start guards.
    fn remediation(&self) {
        remediate(&self.registers(), self.timeout, Abort::General);
    }

    /// Consume a session at its defined end, asserting it belongs to
    /// THIS controller. Cross-instance sessions compile (the session
    /// is not lifetime-branded — see [`Session`]) but are a severe
    /// protocol violation, so they fail deterministically here, before
    /// the defuse.
    fn consume_session(&self, open: Session) {
        assert!(
            core::ptr::eq(open.info, self.info),
            "i2c: transaction session from a different controller instance"
        );
        open.defuse();
    }

    /// Mint the session for a transaction this controller just opened.
    fn open_session(&self, read: bool) -> Session {
        Session {
            info: self.info,
            timeout: self.timeout,
            read,
        }
    }

    /// Resets both TX and RX FIFOs dropping their contents.
    fn reset_fifos(&self) {
        self.registers().reset_fifos();
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
    /// spin forever. Classification ONLY — recovery belongs to the
    /// caller's context: `start`/`stop` recover directly (no session
    /// exists there), everywhere else the live [`Session`]'s drop is
    /// the single recoverer.
    fn wait_tx_room(&self) -> Result<(), IOError> {
        let deadline = embassy_time::Instant::now() + self.timeout;
        loop {
            if embassy_time::Instant::now() > deadline {
                return Err(IOError::Timeout);
            }
            match self.registers().tx_room_step() {
                Some(TxStep::Room) => return Ok(()),
                Some(TxStep::Fault(_)) => {
                    if let Err(e) = self.parse_status(self.registers().take_status()) {
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

    /// Take-and-classify the status, then recover for the classes a
    /// START/STOP transition must not leave behind. ONLY for
    /// `start`/`stop` (before a session exists / after it is defused)
    /// — everywhere else the live [`Session`]'s drop owns recovery.
    ///
    /// On a NACK: per RM 40.7.1.5, the controller auto-STOPs only with
    /// MCFGR1[AUTOSTOP] or a non-empty transmit FIFO; otherwise the
    /// recovery STOP is ours to send. Arbitration loss / FIFO error /
    /// pin-low timeout leave aborted commands queued for the next
    /// transaction to trip over, so they recover too.
    fn take_status_and_recover(&self) -> Result<(), IOError> {
        let status = self.parse_status(self.registers().take_status());
        match status {
            Err(IOError::AddressNack) => {
                if self.registers().needs_manual_stop_after_nack() {
                    self.remediation();
                }
            }
            Err(IOError::ArbitrationLoss) | Err(IOError::FifoError) | Err(IOError::PinLowTimeout) => {
                self.remediation();
            }
            _ => {}
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

    /// Continue an open transaction with a repeated START, consuming
    /// its session and minting the successor. On a preflight rejection
    /// the session is dropped — i.e. the open transaction is
    /// RECOVERED, not abandoned.
    fn start_continue(&self, address: u8, read: bool, open: Session) -> Result<Session, IOError> {
        if address >= 0x80 {
            return Err(IOError::AddressOutOfRange(address));
        }
        // The repeated START takes the predecessor over on the wire;
        // if the START then fails, `start_raw`'s arms recover.
        self.consume_session(open);
        self.start_raw(address, read)
    }

    fn start_raw(&self, address: u8, read: bool) -> Result<Session, IOError> {
        // Wait until we have space in the TxFIFO. No session exists
        // yet, so every failure class recovers HERE (`wait_tx_room` is
        // classification-only). The START is not on the wire yet, so
        // this is not a read abort.
        if let Err(e) = self.wait_tx_room() {
            self.remediation();
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
        // the START is still queued behind a stretched clock — drop
        // it. The address MAY complete (and, for a read, ACK) while
        // recovery runs, so the abort carries the direction.
        if let Err(e) = self.wait_tx_settled() {
            let abort = if read { Abort::ReadAddressed } else { Abort::General };
            remediate(&self.registers(), self.timeout, abort);
            return Err(e);
        }

        self.take_status_and_recover().map(|()| self.open_session(read))
    }

    /// Prepares a Stop condition on the bus.
    ///
    /// Analogous to `start`, this blocks waiting for space in the
    /// FIFO to become available, then sends the command and blocks
    /// waiting for the FIFO to become empty ensuring the command was
    /// sent.
    fn stop(&self, open: Session) -> Result<(), IOError> {
        // The session reaches its defined end here; every failure
        // below recovers directly (`stop` owns its own failures).
        self.consume_session(open);
        if let Err(e) = self.wait_tx_room() {
            self.remediation();
            return Err(e);
        }

        self.send_cmd(ControllerCommand::STOP, 0);

        // Wait for TxFIFO to be drained; on timeout the STOP is still
        // queued behind a stretched clock — drop it.
        if let Err(e) = self.wait_tx_settled() {
            self.remediation();
            return Err(e);
        }

        self.take_status_and_recover()
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
    fn blocking_read_chained(&self, read: &mut [u8], open: Session) -> Result<Session, IOError> {
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
        self.send_cmd(ControllerCommand::RECEIVE, (first - 1) as u8);
        let mut queued = first;
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
    fn blocking_read_seam_chunk(&self, chunk: &mut [u8], open: Session) -> Result<Session, IOError> {
        // No wait for room: the start transition returned only after
        // the command FIFO drained, so one command always fits — and
        // like the async/DMA seam chunks, the RECEIVE must be the
        // FIRST statement so the session never exists without a data
        // command behind it (the drop invariant — see [`Session`]).
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
    fn blocking_write_body(&self, write: &[u8], open: Session) -> Result<Session, IOError> {
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
            // Wait until we have space in the TxFIFO
            self.wait_tx_room()?;

            self.send_cmd(ControllerCommand::TRANSMIT, *byte);
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
    /// Schedule sending a START command and await it being pulled from the FIFO.
    ///
    /// Does not indicate that the command was responded to.
    ///
    /// The wait is bounded by [`Config::transfer_timeout`] like its
    /// blocking counterpart: a target stretching SCL indefinitely
    /// satisfies neither the drain condition nor any error flag. A
    /// start that fails is returned with the controller recovered and
    /// NO session minted, so recovery for a failed start runs
    /// exactly once, here. The awaited wait is guarded against
    /// drop-cancellation for the same reason: no session exists while
    /// it is pending, so nobody else would clean up that abort.
    async fn async_start_fresh(&self, address: u8, read: bool) -> Result<Session, IOError> {
        if address >= 0x80 {
            return Err(IOError::AddressOutOfRange(address));
        }
        self.async_start_raw(address, read).await
    }

    /// Continue an open transaction with a repeated START — the async
    /// twin of [`Self::start_continue`]: a preflight rejection drops
    /// (= recovers) the session; past that, the raw start's arms own
    /// every failure.
    async fn async_start_continue(&self, address: u8, read: bool, open: Session) -> Result<Session, IOError> {
        if address >= 0x80 {
            return Err(IOError::AddressOutOfRange(address));
        }
        self.consume_session(open);
        self.async_start_raw(address, read).await
    }

    async fn async_start_raw(&self, address: u8, read: bool) -> Result<Session, IOError> {
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
        // transaction nothing would ever close — for a read it may
        // ACK its address with no RECEIVE behind it, the exact shape
        // whose recovery needs the direction (see `remediate`).
        // Defused the moment the wait resolves — the arms below own
        // their remediation, so recovery stays exactly-once.
        let abort = if read { Abort::ReadAddressed } else { Abort::General };
        let on_drop = OnDrop::new(|| remediate(&self.registers(), self.timeout, abort));

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
                // transaction trips over the stale queued command —
                // and the address may still complete mid-recovery.
                remediate(&self.registers(), self.timeout, abort);
                return Err(IOError::Timeout);
            }
        }

        // Note: the START + ACK/NACK have not necessarily been finished here.
        // thus this might return Ok(()), but might at a later state result in NAK or FifoError.
        self.take_status_and_recover().map(|()| self.open_session(read))
    }

    /// Schedule a STOP command and await it being pulled from the FIFO.
    ///
    /// Bounded like [`Self::async_start_raw`], and like it fully
    /// self-recovering: a timeout means the STOP is stuck behind a
    /// stretched clock and remediation drops it, and the fault classes
    /// (arbitration loss, FIFO error, pin-low timeout) can leave the
    /// aborted STOP queued — `tx_settled` also exits on an error flag
    /// — so they remediate too. The session is consumed (defused) at
    /// entry, so recovery for a failed stop runs exactly once, here —
    /// nothing else holds a claim on the transaction. The awaited wait
    /// is guarded against drop-cancellation for the same reason — see
    /// `async_start_raw`.
    async fn async_stop(&self, open: Session) -> Result<(), IOError> {
        // The session reaches its defined end here; every failure
        // below recovers directly (`async_stop` owns its own
        // failures).
        self.consume_session(open);
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

        self.take_status_and_recover()
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
        T::Interrupt::unpend();

        // Safety: `_irq` ensures an Interrupt Handler exists.
        unsafe { T::Interrupt::enable() };

        Self::new_inner(peri, scl, sda, config, Async)
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
    async fn async_read_chained(&self, read: &mut [u8], open: Session) -> Result<Session, IOError> {
        let total = read.len();
        // First command unconditionally — see `blocking_read_chained`.
        let first = total.min(256);
        self.send_cmd(ControllerCommand::RECEIVE, (first - 1) as u8);
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
    async fn async_read_seam_chunk(&self, chunk: &mut [u8], open: Session) -> Result<Session, IOError> {
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

        Ok(open)
    }

    /// The write engine past the address phase. Aborts — error returns
    /// and drop-cancellation alike — drop `open`, whose recovery
    /// closes the transaction.
    async fn async_write_body(&self, write: &[u8], open: Session) -> Result<Session, IOError> {
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
                // holding SCL low): the session drop closes the
                // transaction on this return.
                Err(_) => return Err(IOError::Timeout),
            }

            // NOT `take_status_and_recover`: recovery here belongs to
            // the session's drop on the error return — this is
            // classification only, keeping recovery exactly-once.
            self.parse_status(self.registers().take_status())?;
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
    /// One operation owns the RX DMA handoff: DMA writes made visible,
    /// peripheral request off, channel quiesced (provably idle before
    /// the buffer borrow ends). Returns whether the major loop had
    /// completed. The register layer provides the primitives; this
    /// compound sequence is the driver's protocol, in one place.
    fn finish_rx_dma(&self) -> bool {
        cortex_m::asm::dsb();
        self.registers().set_rx_dma(false);
        self.mode.rx_dma.quiesce()
    }

    /// TX twin of [`Self::finish_rx_dma`].
    fn finish_tx_dma(&self) -> bool {
        cortex_m::asm::dsb();
        self.registers().set_tx_dma(false);
        self.mode.tx_dma.quiesce()
    }

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

        self.finish_rx_dma();

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
    /// Abort ordering on any drop path: `quiesce` is declared after
    /// `open`, so it drops FIRST — the DMA request path must be off
    /// and the channel provably inactive before the session's recovery
    /// resets the FIFOs (which can reassert the peripheral request)
    /// and before `read`'s borrow ends (an in-flight minor loop would
    /// write into it after this future unwound; `disable_request`
    /// alone does not wait for that loop).
    async fn dma_read_chained(&self, read: &mut [u8], open: Session) -> Result<Session, IOError> {
        let quiesce = OnDrop::new(|| {
            self.finish_rx_dma();
        });

        // Queue every RECEIVE command up front (they fit). The first
        // goes in unconditionally — see `blocking_read_chained`.
        let total = read.len();
        let first = total.min(256);
        self.send_cmd(ControllerCommand::RECEIVE, (first - 1) as u8);
        let mut queued = first;
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

        // `dma_read_into`'s completion path already quiesced the
        // channel; nothing is in flight past this point.
        quiesce.defuse();
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
    /// one DMA transfer) on the open transaction. Abort ordering as in
    /// [`Self::dma_read_chained`]: the quiesce guard drops before the
    /// session's recovery.
    async fn dma_read_seam_chunk(&self, chunk: &mut [u8], open: Session) -> Result<Session, IOError> {
        let quiesce = OnDrop::new(|| {
            self.finish_rx_dma();
        });

        // send receive command
        self.send_cmd(ControllerCommand::RECEIVE, (chunk.len() - 1) as u8);

        self.dma_read_into(chunk).await?;

        // The chunk is drained and its channel quiesced by
        // `dma_read_into`'s completion path.
        quiesce.defuse();
        Ok(open)
    }

    /// The DMA write engine past the address phase. Abort ordering as
    /// in [`Self::dma_read_chained`]: the quiesce guard drops before
    /// the session, so the channel is provably idle before recovery
    /// touches the FIFOs and before `write`'s borrow ends.
    async fn dma_write_body(&self, write: &[u8], open: Session) -> Result<Session, IOError> {
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

        let quiesce = OnDrop::new(|| {
            self.finish_tx_dma();
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
                // The quiesce guard and the session drop handle this
                // return, in that order.
                Err(_) => return Err(IOError::Timeout),
            }

            self.finish_tx_dma();
        }

        // Every chunk is drained and the channel quiesced; the close
        // variant owns the trailing stop.
        quiesce.defuse();

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
        self.is_hs = config.speed == Speed::UltraFast;
        self.allow_chunked_reads = config.allow_chunked_reads;
        self.timeout = config.transfer_timeout;
        self.set_configuration(config);
        Ok(())
    }
}
