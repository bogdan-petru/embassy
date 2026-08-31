//! Controller transaction ownership and phase transitions.
//!
//! This private child owns the runtime state machine, its recovery proofs,
//! and the facade-facing permits. The parent driver can orchestrate opaque
//! sessions but cannot construct or mutate their phase fields directly.

use core::marker::PhantomData;

use super::IOError;
use super::registers::{
    ControllerRegisters, ControllerStatusError, FacadeSeal, FaultSlot, HaltResolution, HaltedFault, RecoveryClose,
    RetainedFault, RxProgress, RxStep, StartAction, TransferFault,
};
use crate::i2c::Info;

/// Select the recovery shape for a transfer fault. An address NACK has
/// no ACKing target, so its manual close is always the general form;
/// every other class retains the caller's known wire direction.
const fn recovery_abort_for(error: ControllerStatusError, abort: Abort) -> Abort {
    match error {
        ControllerStatusError::AddressNack => Abort::General,
        ControllerStatusError::ArbitrationLoss | ControllerStatusError::Fifo => abort,
        // PLTF is retained as a terminal proof and never reaches this
        // selector in a well-typed call path. Retaining `abort` here keeps
        // this total `const fn` conservative if a future fault carrier is
        // refactored before its terminal proof is consumed.
        ControllerStatusError::PinLowTimeout => abort,
    }
}

/// Resolve a classified fault before a live session exists. The outer driver
/// deliberately has no access to an abort shape or recovery permit; it can
/// request only this pre-session recovery, whose known shape is `General`.
/// Once a session has been minted, use [`Session::bind_fault`] instead so its
/// drop path remains the single recovery owner.
pub(super) fn recover_before_session_fault(
    regs: &ControllerRegisters,
    timeout: embassy_time::Duration,
    fault: TransferFault,
) -> IOError {
    let mut retained = FaultSlot::empty();
    let error = retained.capture(fault);
    match retained.take() {
        Some(RetainedFault::Halted(halt)) => {
            remediate_halted(regs, timeout, recovery_abort_for(halt.error(), Abort::General), halt)
        }
        Some(RetainedFault::PinLowTimeout(timeout_fault)) => regs.terminate_pin_low_timeout(timeout_fault),
        None => remediate(regs, timeout, recovery_abort_for(error, Abort::General)),
    }
    error.into()
}

/// Recover before a session exists. This is intentionally a fixed semantic
/// operation rather than a caller-supplied close shape, so the outer driver
/// cannot emit a recovery command without this recovery owner.
pub(super) fn remediate_before_session(regs: &ControllerRegisters, timeout: embassy_time::Duration) {
    remediate(regs, timeout, Abort::General);
}

fn remediate(regs: &ControllerRegisters, timeout: embassy_time::Duration, abort: Abort) {
    remediate_inner(regs, timeout, abort, None, None);
}

/// Recover a session that may still describe a command accepted into MTDR
/// but not yet known to have executed. A late NDF/FEF must select the
/// frozen-pipeline policy from that phase, rather than the ordinary
/// cancellation policy used before any halt is observed.
fn remediate_pending(regs: &ControllerRegisters, timeout: embassy_time::Duration, pending: PendingRecovery) {
    remediate_inner(regs, timeout, pending.abort_for_cancellation(), None, Some(pending));
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
    mut late_halt_phase: Option<PendingRecovery>,
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
    late_halt_phase = retain_recovery_rx_progress(late_halt_phase, regs);
    let busy = regs.master_busy();
    let mut resolved = false;
    if let Some(halt) = known_halt {
        // The transaction owner observed this exact NDF/FEF before
        // returning its error. Resolve that proof directly instead of
        // relying on the drop path to rediscover a latched bit.
        match regs.resolve_owned_halt(halt, timeout, deadline) {
            HaltResolution::Settled => resolved = true,
            HaltResolution::NeedsManualClose => {}
            HaltResolution::PinLowTerminated => return,
        }
    } else if !busy {
        // Idle entry: nothing is running — dropping the stale
        // pipeline and clearing everything is unconditionally safe.
        regs.discard_idle_recovery_state();
    } else {
        if let Some(fault) = regs.observe_recovery_fault() {
            match fault {
                RetainedFault::Halted(halt) => {
                    // The halt freezes the engine, so this is the authoritative
                    // FIFO observation for a first RECEIVE that raced recovery's
                    // entry snapshot.
                    late_halt_phase = retain_recovery_rx_progress(late_halt_phase, regs);
                    if let Some(phase) = late_halt_phase {
                        abort = recovery_abort_for(halt.error(), phase.abort_for_halted_fault());
                    }
                    match regs.resolve_halted_fault(halt, timeout, deadline) {
                        HaltResolution::Settled => resolved = true,
                        HaltResolution::NeedsManualClose => {}
                        HaltResolution::PinLowTerminated => return,
                    }
                }
                RetainedFault::PinLowTimeout(timeout_fault) => {
                    regs.terminate_pin_low_timeout(timeout_fault);
                    return;
                }
            }
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
            // permits; see `observe_recovery_fault`):
            //
            // * ALF may be SPURIOUS on this silicon — latched on a
            //   transfer that is still running — so the step scrubs it
            //   (only when actually latched: a tight unconditional
            //   clear loop hammering MSR disturbed otherwise-clean
            //   drains on hardware) and the run-out continues. A
            //   GENUINE arbitration loss idles the engine, which the
            //   idle-with-close-pending break above then ends.
            // * PLTF is different: the command/FIFO outcome is not
            //   documented, so it immediately disables MEN and drops
            //   the controller state rather than queuing this close.
            // * NDF/FEF are real sequencing verdicts, and the latched
            //   flag is what keeps the stale pipeline frozen. Observe
            //   them BEFORE choosing this iteration's close: once the
            //   halt is observed, the FIFO snapshot is stable proof of
            //   whether a pending first RECEIVE executed.
            match regs.observe_recovery_fault() {
                Some(RetainedFault::Halted(halt)) => {
                    late_halt_phase = retain_recovery_rx_progress(late_halt_phase, regs);
                    if let Some(phase) = late_halt_phase {
                        abort = recovery_abort_for(halt.error(), phase.abort_for_halted_fault());
                    }
                    match regs.resolve_halted_fault(halt, timeout, deadline) {
                        HaltResolution::Settled => break,
                        HaltResolution::NeedsManualClose => {}
                        HaltResolution::PinLowTerminated => return,
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
                }
                Some(RetainedFault::PinLowTimeout(timeout_fault)) => {
                    regs.terminate_pin_low_timeout(timeout_fault);
                    return;
                }
                None => {
                    // Keep the RX FIFO empty: an abandoned in-flight RECEIVE
                    // stalls the engine in SCL flow control (no fault!) the
                    // moment the un-popped FIFO fills, and would otherwise
                    // never finish — see `discard_rx`. Retain a byte that this
                    // drain consumes as first-RECEIVE execution evidence for a
                    // fault that may latch on a later iteration.
                    late_halt_phase = retain_recovery_discarded_rx_progress(late_halt_phase, regs);
                }
            }

            let close = if abort == Abort::ReadAddressed {
                RecoveryClose::ReleaseAddressedRead
            } else {
                RecoveryClose::Stop
            };
            if !queued && regs.try_enqueue_recovery_close(RecoveryPermit::for_registers(regs), close) {
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
///   it now has defined, safe behavior. Channel-specific cleanup is
///   owned by scoped RX/TX DMA leases, which retain the session and
///   buffer until they disable MDER and quiesce eDMA.
/// - **Runtime-enforced**: the session carries its controller's shared
///   state and every consumption asserts identity, so a session from
///   another instance fails deterministically — and `Info` carries a
///   liveness flag so minting a second session while one exists
///   (which would split recovery ownership of one wire transaction)
///   panics at the mint instead of corrupting the bus.
#[must_use]
pub(super) struct Session {
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
    /// A transfer-time recovery-sensitive observation that must be resolved
    /// by this session's cleanup, rather than rediscovered from a later
    /// status read. It is populated immediately before the error path drops
    /// the session: NDF/FEF retain a frozen-pipeline proof, while PLTF
    /// retains terminal shutdown ownership.
    fault: FaultSlot,
}

/// Receive progress after a [`Session`] has consumed the facade's opaque
/// first-RECEIVE witness. CPU transfer loops deliberately see this simplified
/// result rather than the witness itself, so popping MRDR and updating the
/// session phase are one internal operation. A raw controller fault is bound
/// to this session before it is returned as an [`IOError`], so a loop cannot
/// accidentally match a hardware observation without retaining its recovery
/// owner.
#[must_use]
pub(super) enum SessionRxStep {
    Byte(u8),
    Ended,
}

impl Session {
    /// Mint the only capability that may emit a first read-data command.
    /// Its constructor is private to this module, and the facade consumes it
    /// synchronously when it queues the command. A failed enqueue therefore
    /// keeps the conservative `ReadAddressed` recovery state.
    pub(super) fn first_receive_permit(&mut self) -> FirstReceivePermit<'_> {
        assert!(
            self.phase == SessionPhase::Stable(Abort::ReadAddressed),
            "i2c: a first read command was requested outside the addressed-read phase"
        );
        FirstReceivePermit::new(self.info.controller_registers().identity(), &mut self.phase)
    }

    /// Mint the opaque permit required for an ordinary CPU TRANSMIT. START,
    /// first/later RECEIVE, and STOP have stronger phase-specific permits;
    /// keeping this one write-stable prevents a future internal call from
    /// appending data while a START or STOP still owns the command pipeline.
    pub(super) fn command_permit(&mut self) -> CommandPermit<'_> {
        assert!(
            self.fault.is_empty(),
            "i2c: a session with an unresolved halt was handed a transmit permit"
        );
        assert!(
            self.phase.permits_transmit(),
            "i2c: a transmit was requested outside a stable write transaction"
        );
        let owner = self.info.controller_registers().identity();
        CommandPermit::from_session(owner, self)
    }

    /// Mint a capability for a later RECEIVE only after the first command
    /// entered the FIFO. A first command still pending is valid here: the
    /// follow-on command remains ordered behind it and preserves ACKing.
    pub(super) fn read_receive_permit(&self) -> ReadReceivePermit<'_> {
        assert!(
            matches!(
                self.phase,
                SessionPhase::FirstReceivePending | SessionPhase::Stable(Abort::ReadStreaming)
            ),
            "i2c: a chained read command was requested before the first RECEIVE"
        );
        ReadReceivePermit::new(self.info.controller_registers().identity(), self)
    }

    /// Mint the capability required to arm RX DMA. Keeping the session
    /// borrow inside the resulting lease means cancellation cannot release
    /// either the session or its destination buffer before the channel has
    /// been quiesced.
    pub(super) fn rx_dma_permit(&mut self) -> RxDmaPermit<'_> {
        assert!(
            self.fault.is_empty(),
            "i2c: a session with an unresolved halt was handed to RX DMA"
        );
        let owner = self.info.controller_registers().identity();
        RxDmaPermit::new(owner, self)
    }

    /// Mint the capability required to arm TX DMA. DMA writes are valid
    /// only after a write START has settled, while the session is in its
    /// ordinary wire state.
    pub(super) fn tx_dma_permit(&mut self) -> TxDmaPermit<'_> {
        assert!(
            self.fault.is_empty(),
            "i2c: a session with an unresolved halt was handed to TX DMA"
        );
        assert!(
            self.phase == SessionPhase::Stable(Abort::General),
            "i2c: TX DMA was requested outside the write transaction phase"
        );
        TxDmaPermit::new(self.info.controller_registers().identity(), self)
    }

    /// A received byte proves the first queued RECEIVE executed. Later
    /// cleanup may now rely on its auto-NACK rather than inject a release
    /// command after a fault-frozen FIFO.
    fn note_read_progress(&mut self, progress: RxProgress) {
        let owner = self.info.controller_registers().identity();
        self.phase = self.phase.after_read_progress(progress, owner);
    }

    /// DMA can establish the same progress fact only after its register
    /// facade has disabled the request, quiesced the channel, and observed
    /// its transfer state. The private seal prevents a caller from claiming
    /// that proof without the paired MMIO cleanup.
    fn note_dma_read_progress(&mut self, _seal: FacadeSeal) {
        self.phase = self.phase.after_dma_read_progress();
    }

    /// Pop and classify one receive-side controller event. The raw facade
    /// result carries opaque evidence whenever it popped MRDR; consume that
    /// evidence here before exposing the byte, so ordinary CPU loops cannot
    /// forget the matching first-RECEIVE phase transition.
    pub(super) fn rx_step(&mut self) -> Result<Option<SessionRxStep>, IOError> {
        let Some(step) = self.info.controller_registers().rx_step() else {
            return Ok(None);
        };
        match step {
            RxStep::Byte { byte, progress } => {
                self.note_read_progress(progress);
                Ok(Some(SessionRxStep::Byte(byte)))
            }
            RxStep::Fault(fault) => Err(self.bind_fault(fault)),
            RxStep::Ended => Ok(Some(SessionRxStep::Ended)),
        }
    }

    /// Mint the only capability that may enqueue a START for this session.
    /// The facade commits `StartPending` only if MTDR accepted the action;
    /// a Full/fault result leaves the predecessor phase untouched.
    pub(super) fn start_transition_permit(&mut self) -> StartTransitionPermit<'_> {
        assert!(
            self.fault.is_empty(),
            "i2c: a session with an unresolved recovery fault was continued"
        );
        StartTransitionPermit::new(self.info.controller_registers().identity(), &mut self.phase)
    }

    /// Mint the only capability that may observe a queued START's terminal
    /// drain/status sequence. Its mutable session borrow survives until the
    /// facade either returns a fault or commits the successor phase, so a
    /// drain witness cannot be replayed against a later transaction.
    pub(super) fn start_status_permit(&mut self) -> StartStatusPermit<'_> {
        assert!(
            self.fault.is_empty(),
            "i2c: a session with an unresolved recovery fault was settled as a START"
        );
        StartStatusPermit::new(self.info.controller_registers().identity(), self)
    }

    /// Mint the only capability that may enqueue a normal trailing STOP.
    /// STOP gets its own action/phase because physical bus-idle is meaningful
    /// only after this session actually queued one.
    pub(super) fn stop_transition_permit(&mut self) -> StopTransitionPermit<'_> {
        assert!(
            self.fault.is_empty(),
            "i2c: a session with an unresolved recovery fault was closed normally"
        );
        StopTransitionPermit::new(self.info.controller_registers().identity(), &mut self.phase)
    }

    /// Turn the moved, STOP-pending session into the owner that is threaded
    /// through every terminal poll. `StopWait` owns the session rather than
    /// borrowing it, so cancellation still runs the ordinary session Drop
    /// recovery and no completion proof can be replayed against another
    /// transaction.
    pub(super) fn into_stop_wait(self) -> StopWait {
        StopWait::new(self)
    }

    /// Convert a classified fault to the public error only after a
    /// retained observation has been made this session's cleanup proof.
    /// An ordinary ALF leaves a pending command phase intact: a later
    /// NDF/FEF in cleanup still needs that predecessor policy. A halting
    /// NDF/FEF freezes its queued suffix immediately, so only that class
    /// collapses the phase to its fault recovery shape. PLTF instead stays
    /// as a terminal proof and will disable MEN on this session's drop.
    pub(super) fn bind_fault(&mut self, fault: TransferFault) -> IOError {
        // A fault may be observed by a command/top-up path before the RX
        // consumer gets its next turn. If the first RECEIVE has already
        // placed a byte in the hardware FIFO, it executed even though DMA
        // may not yet have moved that byte to memory. Classify this before
        // choosing the halted-fault recovery side so every CPU/DMA caller
        // shares the same proof rule.
        let regs = self.info.controller_registers();
        if let Some(progress) = regs.observe_rx_progress() {
            self.note_read_progress(progress);
        }
        let error = self.fault.capture(fault);
        if self.fault.has_halted_fault() {
            self.phase = SessionPhase::Stable(recovery_abort_for(error, self.phase.abort_for_halted_fault()));
        }
        error.into()
    }

    /// Consume without recovery only after a physical STOP has had its
    /// terminal status classified and cleared. The ownership-carrying
    /// [`StopFinalized`] path is the sole caller, after it moved this session
    /// through `StopPending -> StopFinalized`.
    fn defuse_after_stop(self) {
        assert!(
            self.fault.is_empty(),
            "i2c: a session with an unresolved halt was marked complete"
        );
        assert!(
            self.phase == SessionPhase::StopFinalized,
            "i2c: a session was marked complete without a finalized STOP"
        );
        self.info.release_session();
        core::mem::forget(self);
    }

    /// Keep the owner's identity opaque to the parent orchestration layer.
    /// It can reject a cross-instance session without being able to inspect
    /// or rewrite the session's phase.
    pub(super) fn belongs_to(&self, info: &'static Info) -> bool {
        core::ptr::eq(self.info, info)
    }
}

impl Drop for Session {
    fn drop(&mut self) {
        let regs = self.info.controller_registers();
        // Capture a fault that arrived after the caller's last event step
        // before selecting a pending-command close policy. In particular a
        // halted first RECEIVE must recover as ReadAddressed because its
        // frozen command will be discarded rather than auto-NACKing.
        if self.fault.is_empty() {
            if let Some(fault) = regs.take_active_fault() {
                let _ = self.bind_fault(fault);
            }
        }
        let pending = PendingRecovery(self.phase);
        match self.fault.take() {
            Some(RetainedFault::Halted(halt)) => {
                let abort = pending.abort_for_cancellation();
                remediate_halted(&regs, self.timeout, abort, halt);
            }
            Some(RetainedFault::PinLowTimeout(timeout_fault)) => {
                regs.terminate_pin_low_timeout(timeout_fault);
            }
            None => {
                // A NDF/FEF can latch after the snapshot above. Preserve the
                // pending-command phase for the recovery loop so a late halt
                // selects the frozen-pipeline close shape, not the ordinary
                // cancellation/successor shape. A late PLTF takes the terminal
                // arm of `observe_recovery_fault` before any close is queued.
                remediate_pending(&regs, self.timeout, pending);
            }
        }
        self.info.release_session();
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
    StartPending {
        before: Abort,
        after: Abort,
    },
    FirstReceivePending,
    /// A normal trailing STOP entered MTDR. The moved [`StopWait`] owns the
    /// corresponding session until terminal status is classified.
    StopPending {
        before: Abort,
    },
    /// Terminal status was cleanly classified after the physical STOP. This
    /// is deliberately a transient state: only `StopFinalized::defuse` may
    /// consume it without recovery.
    StopFinalized,
}

impl SessionPhase {
    /// Ordinary CPU transmit data is valid only after a write START fully
    /// settled. Every other phase has a more specific command owner.
    const fn permits_transmit(self) -> bool {
        matches!(self, Self::Stable(Abort::General))
    }

    /// Retain first-RECEIVE execution evidence from either an MRDR pop or a
    /// resident-FIFO observation. The opaque proof is owner-branded by the
    /// register facade, so a phase cannot be advanced with evidence from a
    /// different controller.
    fn after_read_progress(self, progress: RxProgress, owner: usize) -> Self {
        progress.consume_for(owner);
        self.after_read_progress_inner()
    }

    /// RX DMA owns a stronger, facade-sealed proof: it has disabled MDER and
    /// quiesced the channel before preserving this same phase transition.
    const fn after_dma_read_progress(self) -> Self {
        self.after_read_progress_inner()
    }

    /// Pure transition table shared by CPU/recovery proof consumption and
    /// the RX-DMA lease. Keeping the unchecked table private means all live
    /// callers must enter through one of those proof-carrying operations.
    const fn after_read_progress_inner(self) -> Self {
        match self {
            Self::FirstReceivePending => Self::Stable(Abort::ReadStreaming),
            Self::Stable(_) | Self::StartPending { .. } | Self::StopPending { .. } | Self::StopFinalized => self,
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
            | Self::FirstReceivePending
            | Self::StopPending { .. }
            | Self::StopFinalized => None,
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
            Self::StartPending { .. } | Self::FirstReceivePending | Self::StopPending { .. } | Self::StopFinalized => {
                None
            }
        }
    }

    /// Commit the terminal clean-status proof of a queued START. This is
    /// intentionally separate from enqueue: the `StartDrained` witness
    /// provides the hardware condition that makes this semantic transition
    /// legal.
    const fn after_start_settled(self) -> Option<Self> {
        match self {
            Self::StartPending { after, .. } => Some(Self::Stable(after)),
            Self::Stable(_) | Self::FirstReceivePending | Self::StopPending { .. } | Self::StopFinalized => None,
        }
    }

    /// Commit a normal trailing STOP gate outcome. Normal closes are legal
    /// only after a write START settled or after at least one read command
    /// made the stream self-releasing. A full/fault gate leaves the exact
    /// predecessor recovery shape intact.
    const fn after_stop_enqueue(self, queued: bool) -> Option<Self> {
        match self {
            Self::Stable(before @ (Abort::General | Abort::ReadStreaming)) => {
                if queued {
                    Some(Self::StopPending { before })
                } else {
                    Some(Self::Stable(before))
                }
            }
            Self::Stable(Abort::ReadAddressed)
            | Self::StartPending { .. }
            | Self::FirstReceivePending
            | Self::StopPending { .. }
            | Self::StopFinalized => None,
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
            Self::StopPending { before } => before,
            Self::StopFinalized => Abort::General,
        }
    }

    /// A halting NDF/FEF freezes and later discards the queued suffix, so
    /// its pending command cannot be relied on to release the bus.
    const fn abort_for_halted_fault(self) -> Abort {
        match self {
            Self::StartPending { before, .. } => before,
            Self::FirstReceivePending => Abort::ReadAddressed,
            Self::Stable(abort) => abort,
            Self::StopPending { before } => before,
            Self::StopFinalized => Abort::General,
        }
    }
}

/// An opaque snapshot of the phase that must survive a session drop until
/// recovery has classified any late halt. It keeps phase selection confined
/// to this module; no outer driver code can construct, inspect, or overwrite
/// the underlying [`SessionPhase`].
#[derive(Clone, Copy)]
struct PendingRecovery(SessionPhase);

impl PendingRecovery {
    const fn abort_for_cancellation(self) -> Abort {
        self.0.abort_for_cancellation()
    }

    const fn abort_for_halted_fault(self) -> Abort {
        self.0.abort_for_halted_fault()
    }
}

/// Preserve first-RECEIVE evidence observed during recovery without exposing
/// a boolean-to-phase transition. When no session phase is being recovered,
/// do not observe or mint a proof at all.
fn retain_recovery_rx_progress(phase: Option<PendingRecovery>, regs: &ControllerRegisters) -> Option<PendingRecovery> {
    phase.map(|phase| match regs.observe_rx_progress() {
        Some(progress) => PendingRecovery(phase.0.after_read_progress(progress, regs.identity())),
        None => phase,
    })
}

/// The discarded byte is intentionally not delivered, but it still proves a
/// pending first RECEIVE executed and therefore must be threaded through the
/// same typed phase transition as a normal read.
fn retain_recovery_discarded_rx_progress(
    phase: Option<PendingRecovery>,
    regs: &ControllerRegisters,
) -> Option<PendingRecovery> {
    match regs.discard_rx() {
        Some(progress) => match phase {
            Some(phase) => Some(PendingRecovery(phase.0.after_read_progress(progress, regs.identity()))),
            // Recovery without a live session must still drain the FIFO to
            // release SCL flow control. It has no phase to retain, but the
            // owner-branded evidence is consumed explicitly rather than
            // silently dropped.
            None => {
                progress.consume_for(regs.identity());
                None
            }
        },
        None => phase,
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
    assert!(SessionPhase::Stable(Abort::General).permits_transmit());
    assert!(!SessionPhase::Stable(Abort::ReadAddressed).permits_transmit());
    assert!(
        !SessionPhase::StartPending {
            before: Abort::General,
            after: Abort::General,
        }
        .permits_transmit()
    );
    assert!(!SessionPhase::StopPending { before: Abort::General }.permits_transmit());
    assert!(matches!(
        SessionPhase::StartPending {
            before: Abort::General,
            after: Abort::General,
        }
        .after_start_settled(),
        Some(SessionPhase::Stable(Abort::General))
    ));
    assert!(matches!(
        SessionPhase::StartPending {
            before: Abort::General,
            after: Abort::ReadAddressed,
        }
        .after_start_settled(),
        Some(SessionPhase::Stable(Abort::ReadAddressed))
    ));
    assert!(matches!(
        SessionPhase::Stable(Abort::General).after_start_settled(),
        None
    ));
    assert!(matches!(SessionPhase::FirstReceivePending.after_start_settled(), None));
    assert!(matches!(
        SessionPhase::StopPending { before: Abort::General }.after_start_settled(),
        None
    ));
    assert!(matches!(SessionPhase::StopFinalized.after_start_settled(), None));
    assert!(matches!(
        SessionPhase::Stable(Abort::General).after_stop_enqueue(true),
        Some(SessionPhase::StopPending { before: Abort::General })
    ));
    assert!(matches!(
        SessionPhase::Stable(Abort::ReadStreaming).after_stop_enqueue(true),
        Some(SessionPhase::StopPending {
            before: Abort::ReadStreaming
        })
    ));
    assert!(matches!(
        SessionPhase::Stable(Abort::General).after_stop_enqueue(false),
        Some(SessionPhase::Stable(Abort::General))
    ));
    assert!(matches!(
        SessionPhase::Stable(Abort::ReadStreaming).after_stop_enqueue(false),
        Some(SessionPhase::Stable(Abort::ReadStreaming))
    ));
    assert!(matches!(
        SessionPhase::Stable(Abort::ReadAddressed).after_stop_enqueue(true),
        None
    ));
    assert!(matches!(
        SessionPhase::StartPending {
            before: Abort::General,
            after: Abort::General,
        }
        .after_stop_enqueue(true),
        None
    ));
    assert!(matches!(
        SessionPhase::FirstReceivePending.after_stop_enqueue(true),
        None
    ));
    assert!(matches!(
        SessionPhase::StopPending { before: Abort::General }.after_stop_enqueue(true),
        None
    ));
    assert!(matches!(SessionPhase::StopFinalized.after_stop_enqueue(true), None));
    assert!(matches!(
        SessionPhase::StopPending { before: Abort::General }.abort_for_cancellation(),
        Abort::General
    ));
    assert!(matches!(
        SessionPhase::StopPending {
            before: Abort::ReadStreaming
        }
        .abort_for_halted_fault(),
        Abort::ReadStreaming
    ));
    assert!(matches!(
        SessionPhase::StopPending {
            before: Abort::ReadStreaming
        }
        .abort_for_cancellation(),
        Abort::ReadStreaming
    ));
    assert!(matches!(
        SessionPhase::StopPending { before: Abort::General }.abort_for_halted_fault(),
        Abort::General
    ));
    assert!(matches!(
        SessionPhase::StopFinalized.abort_for_cancellation(),
        Abort::General
    ));
    assert!(matches!(
        SessionPhase::FirstReceivePending.after_read_progress_inner(),
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
/// [`StartTransitionPermit`] and STOP has [`StopTransitionPermit`], so a
/// future sibling-module edit cannot enqueue a START, TRANSMIT, or STOP
/// without choosing its matching ownership path.
#[must_use]
pub(super) struct CommandPermit<'a> {
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

    pub(super) fn owner(&self) -> usize {
        self.owner
    }
}

/// Capability consumed by a START action. It carries the mutable phase that
/// must become `StartPending` exactly when the facade accepted that START.
#[must_use]
pub(super) struct StartTransitionPermit<'a> {
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

    pub(super) fn owner(&self) -> usize {
        self.owner
    }

    pub(super) fn finish_enqueue(self, action: StartAction, queued: bool, _seal: FacadeSeal) {
        *self.phase = (*self.phase)
            .after_start_enqueue(action.is_read(), queued)
            .expect("i2c: a START was committed from the wrong phase");
    }
}

/// Capability consumed by the typed drain/status sequence of a queued START.
///
/// This owns the mutable session borrow through every pending poll, so no
/// clean status witness can outlive its specific `StartPending` phase or be
/// replayed into a later same-controller transaction.
#[must_use]
pub(super) struct StartStatusPermit<'a> {
    owner: usize,
    session: &'a mut Session,
}

impl<'a> StartStatusPermit<'a> {
    fn new(owner: usize, session: &'a mut Session) -> Self {
        assert!(
            matches!(session.phase, SessionPhase::StartPending { .. }),
            "i2c: a START status was consumed outside a pending START phase"
        );
        Self { owner, session }
    }

    pub(super) fn owner(&self) -> usize {
        self.owner
    }

    pub(super) fn commit_settled(self, _seal: FacadeSeal) {
        self.session.phase = self
            .session
            .phase
            .after_start_settled()
            .expect("i2c: a START drained outside the pending transition phase");
    }
}

/// Capability consumed by a normal trailing STOP action. It records the
/// exact recovery predecessor when MTDR accepts STOP, so neither an
/// addressed-only read nor an already-pending command can be closed through
/// the regular terminal path.
#[must_use]
pub(super) struct StopTransitionPermit<'a> {
    owner: usize,
    phase: &'a mut SessionPhase,
}

impl<'a> StopTransitionPermit<'a> {
    fn new(owner: usize, phase: &'a mut SessionPhase) -> Self {
        assert!(
            matches!(*phase, SessionPhase::Stable(Abort::General | Abort::ReadStreaming)),
            "i2c: a normal STOP was requested outside a settled transaction phase"
        );
        Self { owner, phase }
    }

    pub(super) fn owner(&self) -> usize {
        self.owner
    }

    pub(super) fn finish_enqueue(self, queued: bool, _seal: FacadeSeal) {
        *self.phase = (*self.phase)
            .after_stop_enqueue(queued)
            .expect("i2c: a normal STOP was committed from the wrong phase");
    }
}

/// Owns a STOP-pending session through all completion polls. This is not a
/// borrow-based permit: moving the session into this value makes a completed
/// or finalized STOP proof linear, while dropping it keeps the ordinary
/// session recovery behavior intact on cancellation/error paths.
#[must_use]
pub(super) struct StopWait {
    session: Session,
}

impl StopWait {
    fn new(session: Session) -> Self {
        assert!(
            matches!(session.phase, SessionPhase::StopPending { .. }),
            "i2c: STOP completion began without a queued normal STOP"
        );
        Self { session }
    }

    pub(super) fn owner(&self) -> usize {
        self.session.info.controller_registers().identity()
    }

    pub(super) fn into_completed(self, _seal: FacadeSeal) -> StopCompleted {
        StopCompleted { stop: self }
    }

    fn bind_fault(mut self, fault: TransferFault) -> IOError {
        self.session.bind_fault(fault)
    }
}

/// A fault observed while an ownership-carrying STOP wait was live.
/// Returning this rather than a bare [`TransferFault`] keeps the session
/// attached until the caller binds the halt proof and lets Drop recover.
#[must_use]
pub(super) struct StopFault {
    stop: StopWait,
    fault: TransferFault,
}

impl StopFault {
    pub(super) fn new(stop: StopWait, fault: TransferFault, _seal: FacadeSeal) -> Self {
        Self { stop, fault }
    }

    pub(super) fn into_error(self) -> IOError {
        self.stop.bind_fault(self.fault)
    }
}

/// Proof that the normal STOP this session queued has physically completed.
/// It owns the same `StopWait`, so an idle snapshot cannot be transplanted
/// into a second session or a second controller.
#[must_use]
pub(super) struct StopCompleted {
    stop: StopWait,
}

impl StopCompleted {
    pub(super) fn owner(&self) -> usize {
        self.stop.owner()
    }

    pub(super) fn into_wait(self) -> StopWait {
        self.stop
    }

    pub(super) fn commit_finalized(mut self, _seal: FacadeSeal) -> StopFinalized {
        self.stop.session.phase = SessionPhase::StopFinalized;
        StopFinalized {
            session: self.stop.session,
        }
    }
}

/// A clean terminal-status proof carrying the actual completed session. Its
/// sole terminal operation consumes that session without recovery.
#[must_use]
pub(super) struct StopFinalized {
    session: Session,
}

impl StopFinalized {
    pub(super) fn defuse(self) {
        self.session.defuse_after_stop();
    }
}

/// A fault in the final snapshot after physical STOP completion. The
/// completed session stays attached so error conversion cannot discard the
/// recovery owner in the stop-step/finish-stop polling gap.
#[must_use]
pub(super) struct StopFinalizeFault {
    stop: StopWait,
    fault: TransferFault,
}

impl StopFinalizeFault {
    pub(super) fn new(stop: StopWait, fault: TransferFault, _seal: FacadeSeal) -> Self {
        Self { stop, fault }
    }

    pub(super) fn into_error(self) -> IOError {
        self.stop.bind_fault(self.fault)
    }
}

/// Authority to use recovery's deliberate active-fault bypass. Only the
/// controller's self-contained remediation code can mint this token, so a
/// sibling I2C module cannot turn the recovery batch into a general raw-MTDR
/// command path.
#[must_use]
pub(super) struct RecoveryPermit {
    owner: usize,
}

impl RecoveryPermit {
    fn for_registers(regs: &ControllerRegisters) -> Self {
        Self { owner: regs.identity() }
    }

    pub(super) fn owner(&self) -> usize {
        self.owner
    }
}

/// Runtime reservation made before a fresh START is accepted. Its drop
/// releases the single-session slot on every pre-command error/cancellation
/// path; converting it into `Session` transfers that responsibility without
/// a manual unreserve call.
#[must_use]
pub(super) struct StartReservation {
    info: &'static Info,
    armed: bool,
    phase: SessionPhase,
}

impl StartReservation {
    pub(super) fn acquire(info: &'static Info) -> Self {
        info.reserve_session();
        Self {
            info,
            armed: true,
            phase: SessionPhase::Stable(Abort::General),
        }
    }

    pub(super) fn start_transition_permit(&mut self) -> StartTransitionPermit<'_> {
        assert!(self.armed, "i2c: a fresh START used a released reservation");
        StartTransitionPermit::new(self.info.controller_registers().identity(), &mut self.phase)
    }

    pub(super) fn into_pending_session(mut self, timeout: embassy_time::Duration) -> Session {
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
            fault: FaultSlot::empty(),
        }
    }
}

impl Drop for StartReservation {
    fn drop(&mut self) {
        if self.armed {
            self.info.release_session();
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
pub(super) struct FirstReceivePermit<'a> {
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

    pub(super) fn owner(&self) -> usize {
        self.owner
    }

    /// Consume this permit with the single gate outcome. Both success and
    /// failure assign through the same transition table, making the
    /// addressed-state preservation on Full/fault an explicit invariant.
    pub(super) fn finish_enqueue(self, queued: bool, _seal: FacadeSeal) {
        *self.phase = (*self.phase)
            .after_first_receive_enqueue(queued)
            .expect("i2c: a first read command was committed from the wrong phase");
    }
}

/// Capability for follow-on RECEIVE commands after the first command
/// entered the command FIFO. It remains valid while that command is pending
/// and after a received byte proves it executed.
#[must_use]
pub(super) struct ReadReceivePermit<'a> {
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

    pub(super) fn owner(&self) -> usize {
        self.owner
    }
}

/// Capability consumed by a controller RX-DMA lease. It is minted only
/// after a first RECEIVE entered the FIFO (or while that read is already
/// streaming), and the lease calls `note_read_progress` only after it has
/// stopped the channel and observed the final transfer state.
#[must_use]
pub(super) struct RxDmaPermit<'a> {
    owner: usize,
    session: &'a mut Session,
}

impl<'a> RxDmaPermit<'a> {
    fn new(owner: usize, session: &'a mut Session) -> Self {
        assert!(
            matches!(
                session.phase,
                SessionPhase::FirstReceivePending | SessionPhase::Stable(Abort::ReadStreaming)
            ),
            "i2c: RX DMA was requested before the first read command"
        );
        Self { owner, session }
    }

    pub(super) fn owner(&self) -> usize {
        self.owner
    }

    /// Preserve evidence that the first RECEIVE executed after the DMA
    /// channel is genuinely idle. This is intentionally unavailable to a
    /// caller before it has performed the lease's paired cleanup.
    pub(super) fn note_read_progress(&mut self, seal: FacadeSeal) {
        self.session.note_dma_read_progress(seal);
    }
}

/// Capability consumed by a controller TX-DMA lease. Its session borrow
/// pins the recovery owner in place until the lease has disabled MDER and
/// quiesced the eDMA channel.
#[must_use]
pub(super) struct TxDmaPermit<'a> {
    owner: usize,
    _session: PhantomData<&'a mut Session>,
}

impl<'a> TxDmaPermit<'a> {
    fn new(owner: usize, _session: &'a mut Session) -> Self {
        Self {
            owner,
            _session: PhantomData,
        }
    }

    pub(super) fn owner(&self) -> usize {
        self.owner
    }
}
