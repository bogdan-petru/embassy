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
    CommandStep, ControllerAction, ControllerRegisters, ControllerStatusError, HaltSlot, HaltedFault, RecoveryClose,
    RxStep, StartAction, TransferFault,
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
/// consumed by `stop`/`async_stop` — only AFTER their STOP physically
/// completes, so a fault or stretch during the close still has a
/// recovery owner — or by the next `start_continue` (a repeated START
/// takes the predecessor over on the wire). The engines are split
/// into `*_txn_*` operations, which leave the session open and hand
/// it back, and `*_close*` operations, which consume it with a
/// trailing STOP.
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
///   session" is not an expressible call. (Full linearity is NOT
///   compile-enforced: within this module a second session could be
///   minted while one lives — that is the runtime tier's job.)
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
///   another instance fails deterministically — and `Info` carries a
///   liveness flag so minting a second session while one exists
///   (which would split recovery ownership of one wire transaction)
///   panics at the mint instead of corrupting the bus.
#[must_use]
struct Session {
    /// The owning controller's shared state — enough to recover
    /// without borrowing the controller (which a drop path cannot).
    info: &'static Info,
    /// The owner's transfer timeout at session start, bounding the
    /// recovery drain.
    timeout: embassy_time::Duration,
    /// The recovery phase currently required by the wire state. Repeated
    /// STARTs and first RECEIVEs stay explicitly pending so cancellation
    /// and a captured halted fault can select their separately proven
    /// recovery policies.
    phase: SessionPhase,
    /// A transfer-time NDF/FEF observation that must be resolved by
    /// this session's cleanup, rather than rediscovered from a later
    /// status read. It is populated immediately before the error path
    /// drops the session.
    halt: HaltSlot,
}

impl Session {
    /// Mint the only capability that may emit a first read-data command.
    /// Its constructor is private to this module, and the facade consumes it
    /// synchronously when it queues the command. A failed enqueue therefore
    /// keeps the conservative `ReadAddressed` recovery state.
    fn first_receive_permit(&mut self) -> FirstReceivePermit<'_> {
        assert!(
            self.phase == SessionPhase::Stable(Abort::ReadAddressed),
            "i2c: a first read command was requested outside the addressed-read phase"
        );
        FirstReceivePermit::new(ControllerRegisters::new(self.info.regs()).identity(), &mut self.phase)
    }

    /// Mint the opaque permit required for an ordinary CPU command. Its
    /// borrow ties the facade call to this live recovery owner; sibling I2C
    /// modules can name the type but cannot construct one.
    fn command_permit(&mut self) -> CommandPermit<'_> {
        let owner = ControllerRegisters::new(self.info.regs()).identity();
        CommandPermit::from_session(owner, self)
    }

    /// Mint a capability for a later RECEIVE only after the first command
    /// entered the FIFO. A first command still pending is valid here: the
    /// follow-on command remains ordered behind it and preserves ACKing.
    fn read_receive_permit(&self) -> ReadReceivePermit<'_> {
        assert!(
            matches!(
                self.phase,
                SessionPhase::FirstReceivePending | SessionPhase::Stable(Abort::ReadStreaming)
            ),
            "i2c: a chained read command was requested before the first RECEIVE"
        );
        ReadReceivePermit::new(ControllerRegisters::new(self.info.regs()).identity(), self)
    }

    /// A received byte proves the first queued RECEIVE executed. Later
    /// cleanup may now rely on its auto-NACK rather than inject a release
    /// command after a fault-frozen FIFO.
    fn note_read_progress(&mut self) {
        self.phase = self.phase.after_read_progress();
    }

    /// Mint the only capability that may enqueue a START for this session.
    /// The facade commits `StartPending` only if MTDR accepted the action;
    /// a Full/fault result leaves the predecessor phase untouched.
    fn start_transition_permit(&mut self) -> StartTransitionPermit<'_> {
        assert!(
            self.halt.is_empty(),
            "i2c: a session with an unresolved halt was continued"
        );
        StartTransitionPermit::new(ControllerRegisters::new(self.info.regs()).identity(), &mut self.phase)
    }

    /// Make a drained repeated START's successor phase stable. This is
    /// called only after `tx_settled` completed without a fault; error and
    /// cancellation paths instead consult the pending phase's explicit
    /// recovery policy.
    fn finish_start_transition(&mut self) {
        self.phase = match self.phase {
            SessionPhase::StartPending { after, .. } => SessionPhase::Stable(after),
            SessionPhase::Stable(_) | SessionPhase::FirstReceivePending => {
                panic!("i2c: a repeated START completed without a pending transition")
            }
        };
    }

    /// Convert a classified fault to the public error only after a
    /// halting observation has been made this session's cleanup proof.
    /// An ordinary ALF/PLTF intentionally leaves a pending command phase
    /// intact: a later NDF/FEF in cleanup still needs that predecessor
    /// policy. A halting NDF/FEF freezes its queued suffix immediately,
    /// so only that class collapses the phase to its fault recovery shape.
    fn bind_fault(&mut self, fault: TransferFault) -> IOError {
        // A fault may be observed by a command/top-up path before the RX
        // consumer gets its next turn. If the first RECEIVE has already
        // placed a byte in the hardware FIFO, it executed even though DMA
        // may not yet have moved that byte to memory. Classify this before
        // choosing the halted-fault recovery side so every CPU/DMA caller
        // shares the same proof rule.
        self.phase = self
            .phase
            .with_rx_fifo_progress(&ControllerRegisters::new(self.info.regs()));
        let error = self.halt.capture(fault);
        if !self.halt.is_empty() {
            self.phase = SessionPhase::Stable(recovery_abort_for(error, self.phase.abort_for_halted_fault()));
        }
        error.into()
    }

    /// Consume without recovery: the transaction reached its defined
    /// end (a STOP physically completed, or a successor START took it
    /// over on the wire).
    fn defuse(self) {
        assert!(
            self.halt.is_empty(),
            "i2c: a session with an unresolved halt was marked complete"
        );
        self.info
            .session_open
            .store(false, core::sync::atomic::Ordering::Relaxed);
        core::mem::forget(self);
    }
}

impl Drop for Session {
    fn drop(&mut self) {
        let regs = ControllerRegisters::new(self.info.regs());
        // Capture a fault that arrived after the caller's last event step
        // before selecting a pending-command close policy. In particular a
        // halted first RECEIVE must recover as ReadAddressed because its
        // frozen command will be discarded rather than auto-NACKing.
        if self.halt.is_empty() {
            if let Some(fault) = regs.take_active_fault() {
                let _ = self.bind_fault(fault);
            }
        }
        if let Some(halt) = self.halt.take() {
            let abort = self.phase.abort_for_cancellation();
            remediate_halted(&regs, self.timeout, abort, halt);
        } else {
            // A NDF/FEF can latch after the snapshot above. Preserve the
            // pending-command phase for the recovery loop so a late halt
            // selects the frozen-pipeline close shape, not the ordinary
            // cancellation/successor shape.
            remediate_pending(&regs, self.timeout, self.phase);
        }
        self.info
            .session_open
            .store(false, core::sync::atomic::Ordering::Relaxed);
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
#[derive(Clone, Copy, PartialEq, Eq)]
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

/// The live transaction phase, including commands that entered the FIFO but
/// whose execution cannot yet be inferred. Cancellation and a captured
/// halted fault deliberately choose different conservative recovery sides.
#[derive(Clone, Copy, PartialEq, Eq)]
enum SessionPhase {
    Stable(Abort),
    StartPending { before: Abort, after: Abort },
    FirstReceivePending,
}

impl SessionPhase {
    /// Fold a non-consuming RX FIFO observation into the first-read phase.
    /// This is safe to call repeatedly. A resident byte proves the first
    /// RECEIVE executed even if a later fault is observed by a command or
    /// recovery path before that byte reaches its normal consumer.
    fn with_rx_fifo_progress(self, regs: &ControllerRegisters) -> Self {
        if regs.rx_pending() {
            self.after_read_progress()
        } else {
            self
        }
    }

    /// Retain proof that at least one byte was received. Unlike the FIFO
    /// observation above, this is used after recovery deliberately popped
    /// the byte, so the phase cannot be reconstructed from hardware later.
    const fn after_read_progress(self) -> Self {
        match self {
            Self::FirstReceivePending => Self::Stable(Abort::ReadStreaming),
            Self::Stable(_) | Self::StartPending { .. } => self,
        }
    }

    /// Commit the first RECEIVE gate outcome. The `Some` result is limited
    /// to the addressed-read state: no later-read or unrelated transaction
    /// phase can be made streaming by this transition.
    const fn after_first_receive_enqueue(self, queued: bool) -> Option<Self> {
        match self {
            Self::Stable(Abort::ReadAddressed) => {
                if queued {
                    Some(Self::FirstReceivePending)
                } else {
                    Some(Self::Stable(Abort::ReadAddressed))
                }
            }
            Self::Stable(Abort::General)
            | Self::Stable(Abort::ReadStreaming)
            | Self::StartPending { .. }
            | Self::FirstReceivePending => None,
        }
    }

    /// Commit a START gate outcome. The predecessor is preserved on a
    /// Full/fault result; on `Queued`, both the predecessor and the
    /// START action's requested successor remain explicit until status
    /// settles or a halted fault discards the FIFO suffix.
    const fn after_start_enqueue(self, read: bool, queued: bool) -> Option<Self> {
        match self {
            Self::Stable(before) => {
                if queued {
                    Some(Self::StartPending {
                        before,
                        after: if read { Abort::ReadAddressed } else { Abort::General },
                    })
                } else {
                    Some(Self::Stable(before))
                }
            }
            Self::StartPending { .. } | Self::FirstReceivePending => None,
        }
    }

    /// On cancellation, queued commands remain ordered ahead of recovery.
    /// A pending repeated START may therefore reach the bus before close,
    /// and a pending first RECEIVE will auto-NACK before the STOP; use the
    /// successor/streaming close shape rather than sampling volatile FIFO
    /// state in an attempt to guess which side is already on the wire.
    const fn abort_for_cancellation(self) -> Abort {
        match self {
            Self::Stable(abort) => abort,
            Self::StartPending { after, .. } => after,
            Self::FirstReceivePending => Abort::ReadStreaming,
        }
    }

    /// A halting NDF/FEF freezes and later discards the queued suffix, so
    /// its pending command cannot be relied on to release the bus.
    const fn abort_for_halted_fault(self) -> Abort {
        match self {
            Self::StartPending { before, .. } => before,
            Self::FirstReceivePending => Abort::ReadAddressed,
            Self::Stable(abort) => abort,
        }
    }
}

// Compile the first-RECEIVE and pending-recovery transition tables into every
// target build. The facade consumes these exact transitions through
// `FirstReceivePermit` and the recovery owner, so a Full/fault result cannot
// silently promote an addressed-only read or collapse the wrong side of a
// pending command.
const _: () = {
    assert!(matches!(
        SessionPhase::Stable(Abort::ReadAddressed).after_first_receive_enqueue(true),
        Some(SessionPhase::FirstReceivePending)
    ));
    assert!(matches!(
        SessionPhase::Stable(Abort::ReadAddressed).after_first_receive_enqueue(false),
        Some(SessionPhase::Stable(Abort::ReadAddressed))
    ));
    assert!(matches!(
        SessionPhase::FirstReceivePending.after_first_receive_enqueue(true),
        None
    ));
    assert!(matches!(
        SessionPhase::Stable(Abort::General).after_first_receive_enqueue(true),
        None
    ));
    assert!(matches!(
        SessionPhase::Stable(Abort::ReadStreaming).after_first_receive_enqueue(true),
        None
    ));
    assert!(matches!(
        SessionPhase::StartPending {
            before: Abort::General,
            after: Abort::ReadAddressed,
        }
        .after_first_receive_enqueue(true),
        None
    ));
    assert!(matches!(
        SessionPhase::Stable(Abort::General).after_start_enqueue(true, true),
        Some(SessionPhase::StartPending {
            before: Abort::General,
            after: Abort::ReadAddressed,
        })
    ));
    assert!(matches!(
        SessionPhase::Stable(Abort::ReadAddressed).after_start_enqueue(false, false),
        Some(SessionPhase::Stable(Abort::ReadAddressed))
    ));
    assert!(matches!(
        SessionPhase::FirstReceivePending.after_start_enqueue(true, true),
        None
    ));
    assert!(matches!(
        SessionPhase::FirstReceivePending.after_read_progress(),
        SessionPhase::Stable(Abort::ReadStreaming)
    ));
    assert!(matches!(
        SessionPhase::StartPending {
            before: Abort::General,
            after: Abort::ReadAddressed,
        }
        .abort_for_cancellation(),
        Abort::ReadAddressed
    ));
    assert!(matches!(
        SessionPhase::StartPending {
            before: Abort::General,
            after: Abort::ReadAddressed,
        }
        .abort_for_halted_fault(),
        Abort::General
    ));
    assert!(matches!(
        SessionPhase::FirstReceivePending.abort_for_cancellation(),
        Abort::ReadStreaming
    ));
    assert!(matches!(
        SessionPhase::FirstReceivePending.abort_for_halted_fault(),
        Abort::ReadAddressed
    ));
    assert!(matches!(
        recovery_abort_for(ControllerStatusError::AddressNack, Abort::ReadAddressed),
        Abort::General
    ));
    assert!(matches!(
        recovery_abort_for(ControllerStatusError::Fifo, Abort::ReadAddressed),
        Abort::ReadAddressed
    ));
};

/// Authority to submit one ordinary CPU command through the facade.
///
/// This is deliberately non-constructible outside `controller.rs`. A
/// permit is minted only from a live [`Session`], then immediately consumed
/// by the MMIO facade. START has its own stronger
/// [`StartTransitionPermit`], so a future sibling-module edit cannot enqueue
/// a START, TRANSMIT, or STOP without choosing its matching ownership path.
#[must_use]
pub(in crate::i2c) struct CommandPermit<'a> {
    owner: usize,
    _owner: PhantomData<&'a mut ()>,
}

impl<'a> CommandPermit<'a> {
    fn from_session(owner: usize, _owner: &'a mut Session) -> Self {
        Self {
            owner,
            _owner: PhantomData,
        }
    }

    pub(in crate::i2c) fn owner(&self) -> usize {
        self.owner
    }
}

/// Capability consumed by a START action. It carries the mutable phase that
/// must become `StartPending` exactly when the facade accepted that START.
#[must_use]
pub(in crate::i2c) struct StartTransitionPermit<'a> {
    owner: usize,
    phase: &'a mut SessionPhase,
}

impl<'a> StartTransitionPermit<'a> {
    fn new(owner: usize, phase: &'a mut SessionPhase) -> Self {
        assert!(
            matches!(*phase, SessionPhase::Stable(_)),
            "i2c: a START was requested outside a stable transaction phase"
        );
        Self { owner, phase }
    }

    pub(in crate::i2c) fn owner(&self) -> usize {
        self.owner
    }

    pub(in crate::i2c) fn finish_enqueue(self, action: StartAction, queued: bool) {
        *self.phase = (*self.phase)
            .after_start_enqueue(action.is_read(), queued)
            .expect("i2c: a START was committed from the wrong phase");
    }
}

/// Authority to use recovery's deliberate active-fault bypass. Only the
/// controller's self-contained remediation code can mint this token, so a
/// sibling I2C module cannot turn the recovery batch into a general raw-MTDR
/// command path.
#[must_use]
pub(in crate::i2c) struct RecoveryPermit {
    owner: usize,
}

impl RecoveryPermit {
    fn new(owner: usize) -> Self {
        Self { owner }
    }

    pub(in crate::i2c) fn owner(&self) -> usize {
        self.owner
    }
}

/// Runtime reservation made before a fresh START is accepted. Its drop
/// releases the single-session slot on every pre-command error/cancellation
/// path; converting it into `Session` transfers that responsibility without
/// a manual unreserve call.
#[must_use]
struct StartReservation {
    info: &'static Info,
    armed: bool,
    phase: SessionPhase,
}

impl StartReservation {
    fn acquire(info: &'static Info) -> Self {
        assert!(
            !info.session_open.swap(true, core::sync::atomic::Ordering::Relaxed),
            "i2c: a transaction started while another session is live"
        );
        Self {
            info,
            armed: true,
            phase: SessionPhase::Stable(Abort::General),
        }
    }

    fn start_transition_permit(&mut self) -> StartTransitionPermit<'_> {
        assert!(self.armed, "i2c: a fresh START used a released reservation");
        StartTransitionPermit::new(ControllerRegisters::new(self.info.regs()).identity(), &mut self.phase)
    }

    fn into_pending_session(mut self, timeout: embassy_time::Duration) -> Session {
        assert!(self.armed, "i2c: a fresh START consumed a released reservation");
        assert!(
            matches!(self.phase, SessionPhase::StartPending { .. }),
            "i2c: a fresh START reservation became a session before its command was queued"
        );
        let phase = self.phase;
        // Only disarm after all assertions that can unwind. From here the
        // returned Session, rather than this reservation's Drop, owns the
        // liveness slot.
        self.armed = false;
        Session {
            info: self.info,
            timeout,
            phase,
            halt: HaltSlot::empty(),
        }
    }
}

impl Drop for StartReservation {
    fn drop(&mut self) {
        if self.armed {
            self.info
                .session_open
                .store(false, core::sync::atomic::Ordering::Relaxed);
        }
    }
}

/// Capability consumed by the first RECEIVE after a read START.
///
/// Its constructor is private to this controller module. The register
/// facade can inspect its controller identity and commit its phase change,
/// but cannot mint one; that prevents sibling code from treating an
/// addressed-only read as streaming without actually queueing a command.
#[must_use]
pub(in crate::i2c) struct FirstReceivePermit<'a> {
    owner: usize,
    phase: &'a mut SessionPhase,
}

impl<'a> FirstReceivePermit<'a> {
    fn new(owner: usize, phase: &'a mut SessionPhase) -> Self {
        assert!(
            *phase == SessionPhase::Stable(Abort::ReadAddressed),
            "i2c: a first read command was requested outside the addressed-read phase"
        );
        Self { owner, phase }
    }

    pub(in crate::i2c) fn owner(&self) -> usize {
        self.owner
    }

    /// Consume this permit with the single gate outcome. Both success and
    /// failure assign through the same transition table, making the
    /// addressed-state preservation on Full/fault an explicit invariant.
    pub(in crate::i2c) fn finish_enqueue(self, queued: bool) {
        *self.phase = (*self.phase)
            .after_first_receive_enqueue(queued)
            .expect("i2c: a first read command was committed from the wrong phase");
    }
}

/// Capability for follow-on RECEIVE commands after the first command
/// entered the command FIFO. It remains valid while that command is pending
/// and after a received byte proves it executed.
#[must_use]
pub(in crate::i2c) struct ReadReceivePermit<'a> {
    owner: usize,
    _session: PhantomData<&'a Session>,
}

impl<'a> ReadReceivePermit<'a> {
    fn new(owner: usize, _session: &'a Session) -> Self {
        Self {
            owner,
            _session: PhantomData,
        }
    }

    pub(in crate::i2c) fn owner(&self) -> usize {
        self.owner
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

/// Select the recovery shape for a transfer fault. An address NACK has
/// no ACKing target, so its manual close is always the general form;
/// every other class retains the caller's known wire direction.
const fn recovery_abort_for(error: ControllerStatusError, abort: Abort) -> Abort {
    match error {
        ControllerStatusError::AddressNack => Abort::General,
        ControllerStatusError::ArbitrationLoss | ControllerStatusError::Fifo | ControllerStatusError::PinLowTimeout => {
            abort
        }
    }
}

/// Resolve a classified fault before a live session exists. Once a
/// session has been minted, use [`Session::bind_fault`] instead so its
/// drop path remains the single recovery owner.
fn recover_transfer_fault(
    regs: &ControllerRegisters,
    timeout: embassy_time::Duration,
    abort: Abort,
    fault: TransferFault,
) -> IOError {
    let mut halt = HaltSlot::empty();
    let error = halt.capture(fault);
    let abort = recovery_abort_for(error, abort);
    match halt.take() {
        Some(halt) => remediate_halted(regs, timeout, abort, halt),
        None => remediate(regs, timeout, abort),
    }
    error.into()
}

fn remediate(regs: &ControllerRegisters, timeout: embassy_time::Duration, abort: Abort) {
    remediate_inner(regs, timeout, abort, None, None);
}

/// Recover a session that may still describe a command accepted into MTDR
/// but not yet known to have executed. A late NDF/FEF must select the
/// frozen-pipeline policy from that phase, rather than the ordinary
/// cancellation policy used before any halt is observed.
fn remediate_pending(regs: &ControllerRegisters, timeout: embassy_time::Duration, phase: SessionPhase) {
    remediate_inner(regs, timeout, phase.abort_for_cancellation(), None, Some(phase));
}

/// Recover using a halt proof retained by the transaction that observed
/// it. This avoids a second status observation between the API error and
/// the session's cleanup.
fn remediate_halted(regs: &ControllerRegisters, timeout: embassy_time::Duration, abort: Abort, halt: HaltedFault) {
    remediate_inner(regs, timeout, abort, Some(halt), None);
}

fn remediate_inner(
    regs: &ControllerRegisters,
    timeout: embassy_time::Duration,
    mut abort: Abort,
    known_halt: Option<HaltedFault>,
    mut late_halt_phase: Option<SessionPhase>,
) {
    #[cfg(feature = "defmt")]
    defmt::trace!("Recovering controller",);

    // Recovery must not re-enter the fault-aware transfer paths that
    // lead here (a session drop or pre-start recovery): with a fault
    // that keeps re-latching, that cycle
    // recurses until the stack overflows. Everything below is
    // self-contained.
    //
    // Resetting the FIFOs drops whatever the aborted transfer left
    // queued — but a FIFO reset issued while the engine is ACTIVELY
    // RUNNING a command corrupts its transaction bookkeeping
    // (hardware-observed: the closing STOP then forms on the wire,
    // EPF/SDF latch, yet MBF/BBF stick busy forever and later
    // commands are ignored — a state not even an engine reset fully
    // unwinds). So the entry reset runs ONLY when the engine is idle.
    // A latched fault is deliberately NOT accepted as proof of a
    // halted engine: the spurious-ALF quirk latches "arbitration
    // loss" on a transfer that is still running, and gating the reset
    // on it would land exactly the corruption above.
    //
    // The busy entry must ALSO not clear flags blindly: a latched
    // NDF/FEF is what holds a halted engine off its stale pipeline
    // (see `take_active_fault`), so the halting classes
    // are recognized FIRST — the auto-STOP is waited out and the
    // pipeline discarded, or (empty FIFO: nothing to replay, no
    // auto-STOP coming) the halt is cleared for the drain's manual
    // close. Everything else is scrubbed snapshot-honestly, which
    // cannot erase a halting fault racing in.
    let deadline = embassy_time::Instant::now() + timeout;
    // A queued first RECEIVE may have executed before recovery began,
    // leaving its byte in the hardware FIFO. Retain that proof even if a
    // halting status is only observed later in the recovery drain.
    late_halt_phase = late_halt_phase.map(|phase| phase.with_rx_fifo_progress(regs));
    let busy = regs.master_busy();
    let mut resolved = false;
    if let Some(halt) = known_halt {
        // The transaction owner observed this exact NDF/FEF before
        // returning its error. Resolve that proof directly instead of
        // relying on the drop path to rediscover a latched bit.
        resolved = regs.resolve_owned_halt(halt, timeout, deadline);
    } else if !busy {
        // Idle entry: nothing is running — dropping the stale
        // pipeline and clearing everything is unconditionally safe.
        regs.discard_idle_recovery_state();
    } else {
        if let Some(halt) = regs.observe_recovery_halt() {
            // The halt freezes the engine, so this is the authoritative
            // FIFO observation for a first RECEIVE that raced recovery's
            // entry snapshot.
            late_halt_phase = late_halt_phase.map(|phase| phase.with_rx_fifo_progress(regs));
            if let Some(phase) = late_halt_phase {
                abort = recovery_abort_for(halt.error(), phase.abort_for_halted_fault());
            }
            resolved = regs.resolve_halted_fault(halt, timeout, deadline);
        }
    }

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
    if !resolved && (busy || regs.master_busy()) {
        // The closing sequence is shape-specific — see [`Abort`] —
        // and is queued once there is room behind whatever the abort
        // left pending (those commands run out first; a read
        // pipeline's final byte auto-NACKs, which is exactly what
        // frees the target for the STOP).
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
            // The master HALTS on a latched fault and consumes no
            // further commands until it is cleared. What happens next
            // is CLASS-specific (recovery has no caller to classify
            // for, but it must still read the flags honestly — one
            // snapshot, clearing only what that snapshot's class
            // permits; see `observe_recovery_halt`):
            //
            // * ALF/PLTF may be SPURIOUS on this silicon — latched on
            //   a transfer that is still running — so the step scrubs
            //   them (only when actually latched: a tight
            //   unconditional clear loop hammering MSR disturbed
            //   otherwise-clean drains on hardware) and the run-out
            //   continues; a GENUINE arbitration loss idles the
            //   engine, which the idle-with-close-pending break above
            //   then ends.
            // * NDF/FEF are real sequencing verdicts, and the latched
            //   flag is what keeps the stale pipeline frozen. Observe
            //   them BEFORE choosing this iteration's close: once the
            //   halt is observed, the FIFO snapshot is stable proof of
            //   whether a pending first RECEIVE executed.
            if let Some(halt) = regs.observe_recovery_halt() {
                late_halt_phase = late_halt_phase.map(|phase| phase.with_rx_fifo_progress(regs));
                if let Some(phase) = late_halt_phase {
                    abort = recovery_abort_for(halt.error(), phase.abort_for_halted_fault());
                }
                if regs.resolve_halted_fault(halt, timeout, deadline) {
                    break;
                }
                // Unresolved: the no-auto-STOP path just discarded
                // the frozen pipeline — the drain's own queued
                // close included. Re-queue it, or every exit
                // condition goes dead (settled needs the close to
                // run; the idle-pending break needs something
                // pending) and the loop would burn the deadline
                // into the engine reset.
                queued = false;
                idle_since = None;
            } else {
                // Keep the RX FIFO empty: an abandoned in-flight RECEIVE
                // stalls the engine in SCL flow control (no fault!) the
                // moment the un-popped FIFO fills, and would otherwise
                // never finish — see `discard_rx`. Retain a byte that this
                // drain consumes as first-RECEIVE execution evidence for a
                // fault that may latch on a later iteration.
                if regs.discard_rx() {
                    late_halt_phase = late_halt_phase.map(SessionPhase::after_read_progress);
                }
            }

            let close = if abort == Abort::ReadAddressed {
                RecoveryClose::ReleaseAddressedRead
            } else {
                RecoveryClose::Stop
            };
            if !queued && regs.try_enqueue_recovery_close(RecoveryPermit::new(regs.identity()), close) {
                queued = true;
            }

            // Settled only counts once the closing commands are in:
            // the engine idles between the aborted pipeline and the
            // close, and exiting there would leave the transaction
            // open.
            if queued && regs.recovery_settled() {
                break;
            }

            // A target holding SCL low satisfies no exit condition,
            // so the wait is bounded like every other — recovery must
            // not be the one path that can still hang — and on expiry
            // the engine is hard-reset: whatever holds it, the abort
            // must complete and release this side of the bus.
            if embassy_time::Instant::now() > deadline {
                #[cfg(feature = "defmt")]
                defmt::warn!("recovery close did not settle within the transfer timeout; resetting the engine");
                regs.reset_after_recovery_timeout();
                break;
            }
        }
    }

    // Now provably past the active abort (or hard-reset): drop
    // whatever remains queued or received so the next transaction
    // starts from a clean slate.
    regs.finish_recovery();
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
        ControllerRegisters::check_layout(self.info.regs());

        // Disable the controller.
        critical_section::with(|_| self.info.regs().mcr().modify(|w| w.set_men(false)));

        // Soft-reset the controller, read and write FIFOs.
        self.registers().reset_while_disabled();
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
        self.registers().clear_after_init();
    }

    /// Consume a session at its defined end, asserting it belongs to
    /// THIS controller. Cross-instance sessions compile (the session
    /// is not lifetime-branded — see [`Session`]) but are a severe
    /// protocol violation, so they fail deterministically here, before
    /// the defuse.
    fn assert_session_owner(&self, open: &Session) {
        assert!(
            core::ptr::eq(open.info, self.info),
            "i2c: transaction session from a different controller instance"
        );
    }

    fn consume_session(&self, open: Session) {
        self.assert_session_owner(&open);
        open.defuse();
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

    /// Form a semantic START action after the public address preflight.
    fn start_action(&self, address: u8, read: bool) -> StartAction {
        StartAction::new(address, read, self.is_hs)
            .expect("i2c: checked seven-bit address could not form a START action")
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

    /// Bounded blocking enqueue before a session exists. It owns recovery
    /// because no live session can retain a halt proof yet.
    fn enqueue_before_start(&self, action: StartAction, reservation: &mut StartReservation) -> Result<(), IOError> {
        let regs = self.registers();
        let deadline = embassy_time::Instant::now() + self.timeout;
        loop {
            match regs.try_enqueue_start(reservation.start_transition_permit(), action) {
                CommandStep::Queued => return Ok(()),
                CommandStep::Fault(fault) => {
                    return Err(recover_transfer_fault(&regs, self.timeout, Abort::General, fault));
                }
                CommandStep::Full if embassy_time::Instant::now() > deadline => {
                    remediate(&regs, self.timeout, Abort::General);
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

        if let Err(error) = self.wait_tx_settled() {
            return Err(error);
        }
        match self.registers().take_start_status() {
            Ok(()) => {
                open.finish_start_transition();
                Ok(open)
            }
            Err(fault) => Err(open.bind_fault(fault)),
        }
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
        if let Err(e) = self.wait_tx_settled() {
            return Err(e);
        }

        match self.registers().take_start_status() {
            Ok(()) => {
                open.finish_start_transition();
                Ok(open)
            }
            Err(fault) => Err(open.bind_fault(fault)),
        }
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
        self.enqueue_session_blocking(ControllerAction::stop(), &mut open)?;

        let deadline = embassy_time::Instant::now() + self.timeout;
        let completed = loop {
            match self.registers().stop_step() {
                Some(Ok(completed)) => break completed,
                Some(Err(fault)) => return Err(open.bind_fault(fault)),
                None if embassy_time::Instant::now() > deadline => return Err(IOError::Timeout),
                None => {}
            }
        };

        // Completion is not a status read: one final snapshot both
        // classifies a fault that latched in the last poll gap as this
        // transaction's own, and clears the STOP's residual flags so
        // the next START does not misattribute them.
        if let Err(fault) = self.registers().finish_stop(completed) {
            return Err(open.bind_fault(fault));
        }
        self.consume_session(open);
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
            match self.registers().rx_step() {
                Some(RxStep::Byte(b)) => {
                    open.note_read_progress();
                    read[drained] = b;
                    drained += 1;
                    deadline = embassy_time::Instant::now() + self.timeout;
                }
                Some(RxStep::Fault(fault)) => return Err(open.bind_fault(fault)),
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
                match self.registers().rx_step() {
                    Some(RxStep::Byte(b)) => {
                        open.note_read_progress();
                        deadline = embassy_time::Instant::now() + self.timeout;
                        break b;
                    }
                    Some(RxStep::Fault(fault)) => return Err(open.bind_fault(fault)),
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

        match embassy_time::with_timeout(
            self.timeout,
            self.info.wait_cell().wait_for(|| self.registers().tx_settle_wake()),
        )
        .await
        {
            Ok(Ok(())) => {}
            Ok(Err(_)) => return Err(IOError::Other),
            Err(_) => return Err(IOError::Timeout),
        }

        match self.registers().take_start_status() {
            Ok(()) => {
                open.finish_start_transition();
                Ok(open)
            }
            Err(fault) => Err(open.bind_fault(fault)),
        }
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
                remediate(&self.registers(), self.timeout, Abort::General);
            });
            loop {
                match self
                    .registers()
                    .try_enqueue_start(reservation.start_transition_permit(), self.start_action(address, read))
                {
                    CommandStep::Queued => break,
                    CommandStep::Fault(fault) => {
                        queued_guard.defuse();
                        let error = recover_transfer_fault(&self.registers(), self.timeout, Abort::General, fault);
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

        match self.registers().take_start_status() {
            Ok(()) => {
                open.finish_start_transition();
                Ok(open)
            }
            Err(fault) => Err(open.bind_fault(fault)),
        }
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
        self.enqueue_session_async(ControllerAction::stop(), &mut open).await?;

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
            match self.registers().stop_step() {
                Some(Ok(completed)) => break completed,
                Some(Err(fault)) => return Err(open.bind_fault(fault)),
                None if embassy_time::Instant::now() > deadline => return Err(IOError::Timeout),
                // Cooperative AND cancellable: the tail from "pulled"
                // to "bus idle" has no interrupt to sleep on, and with
                // no await point the future could neither yield nor be
                // dropped (a drop landing here is safe — the live
                // session recovers). The first yields cover the normal
                // tail (a bit-time) with no added latency; past them
                // the wait is a clock stretch, so back off onto the
                // timer and let the executor actually sleep instead of
                // self-waking through the run queue.
                None if spins < 64 => {
                    spins += 1;
                    embassy_futures::yield_now().await;
                }
                None => {
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
        if let Err(fault) = self.registers().finish_stop(completed) {
            return Err(open.bind_fault(fault));
        }
        self.consume_session(open);
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

            match self.registers().rx_step() {
                Some(RxStep::Byte(b)) => {
                    open.note_read_progress();
                    read[drained] = b;
                    drained += 1;
                }
                // Surface the fault that woke us. If the flag cleared
                // in between, loop back and wait again.
                Some(RxStep::Fault(fault)) => return Err(open.bind_fault(fault)),
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

                match self.registers().rx_step() {
                    Some(RxStep::Byte(b)) => {
                        open.note_read_progress();
                        *byte = b;
                        break;
                    }
                    Some(RxStep::Fault(fault)) => return Err(open.bind_fault(fault)),
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

    /// Quiesce RX DMA and update the session phase from a post-quiesce byte
    /// count. An incomplete major loop retains CITER, while a completed one
    /// reloads it, so `quiesce`'s DONE result distinguishes the two cases.
    /// This matters on a fault: a byte in memory *or still resident in the
    /// RX FIFO* proves the first RECEIVE did execute, so recovery must not
    /// inject another release RECEIVE.
    fn finish_rx_dma_and_note(&self, total: usize, open: &mut Session) {
        let complete = self.finish_rx_dma();
        let moved = if complete {
            total
        } else {
            self.mode.rx_dma.transferred_bytes()
        };
        if moved != 0 || self.registers().rx_pending() {
            open.note_read_progress();
        }
    }

    /// TX twin of [`Self::finish_rx_dma`].
    fn finish_tx_dma(&self) -> bool {
        cortex_m::asm::dsb();
        self.registers().set_tx_dma(false);
        self.mode.tx_dma.quiesce()
    }

    /// Run one RX DMA transfer covering `buf`, waking on completion or on
    /// a bus fault (which would otherwise leave the DMA waiting forever).
    /// The caller owns command queueing; this function binds any halting
    /// fault to `open` before its public `IOError` can escape.
    async fn dma_read_into(&self, buf: &mut [u8], open: &mut Session) -> Result<(), IOError> {
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
        // ~1 ms/byte of margin on top of a generous floor.
        let bound = self.timeout + embassy_time::Duration::from_millis(buf.len() as u64);
        let total = buf.len();
        // This inner guard carries the session phase across cancellation of
        // the DMA wait itself. The outer read guards still protect their
        // wider pipelines; this one is the only layer that can distinguish
        // an incomplete DMA major loop with a partially received first byte.
        let cleanup = OnDrop::new(|| self.finish_rx_dma_and_note(total, open));
        let result = embassy_time::with_timeout(bound, wait).await;
        cleanup.defuse();
        self.finish_rx_dma_and_note(total, open);

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
    /// Abort ordering on any drop path: `quiesce` is declared after
    /// `open`, so it drops FIRST — the DMA request path must be off
    /// and the channel provably inactive before the session's recovery
    /// resets the FIFOs (which can reassert the peripheral request)
    /// and before `read`'s borrow ends (an in-flight minor loop would
    /// write into it after this future unwound; `disable_request`
    /// alone does not wait for that loop).
    async fn dma_read_chained(&self, read: &mut [u8], mut open: Session) -> Result<Session, IOError> {
        let quiesce = OnDrop::new(|| {
            self.finish_rx_dma();
        });

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
    async fn dma_read_seam_chunk(&self, chunk: &mut [u8], mut open: Session) -> Result<Session, IOError> {
        let quiesce = OnDrop::new(|| {
            self.finish_rx_dma();
        });

        // send receive command
        self.enqueue_first_receive(chunk.len(), &mut open)?;

        self.dma_read_into(chunk, &mut open).await?;

        // The chunk is drained and its channel quiesced by
        // `dma_read_into`'s completion path.
        quiesce.defuse();
        Ok(open)
    }

    /// The DMA write engine past the address phase. Abort ordering as
    /// in [`Self::dma_read_chained`]: the quiesce guard drops before
    /// the session, so the channel is provably idle before recovery
    /// touches the FIFOs and before `write`'s borrow ends.
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
                if let Some(fault) = self.registers().error_wake() {
                    return core::task::Poll::Ready(Err(DmaWaitError::Fault(fault)));
                }
                core::task::Poll::Pending
            });
            // ~1 ms/byte of margin on top of a generous floor.
            let bound = self.timeout + embassy_time::Duration::from_millis(chunk.len() as u64);
            match embassy_time::with_timeout(bound, wait).await {
                Ok(Ok(())) => {}
                Ok(Err(DmaWaitError::Fault(fault))) => return Err(open.bind_fault(fault)),
                Ok(Err(DmaWaitError::UnexpectedStop)) => unreachable!("TX DMA does not wait for receive termination"),
                // The quiesce guard and the session drop handle this
                // return, in that order.
                Err(_) => return Err(IOError::Timeout),
            }

            self.finish_tx_dma();

            // As in RX, a major-loop completion can win the poll race
            // against a simultaneous controller fault. The channel is
            // quiesced before the proof is handed to the session.
            if let Some(fault) = self.registers().take_active_fault() {
                return Err(open.bind_fault(fault));
            }
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
