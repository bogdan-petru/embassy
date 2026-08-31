//! Safe controller-side operations over the LPI2C register block.
//!
//! Two layers meet here. [`self::lpi2c_regs`] supplies `tock-registers`
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
//!
//! Scope: every PROTOCOL register — status, interrupts, DMA enables,
//! commands, data, FIFO status — is reachable only through this
//! facade; the driver holds no generic read/write/modify on any of
//! them. The one deliberate exception is `set_configuration`, which
//! touches init-only configuration registers (MCR/MCFGR*/MCCR0)
//! through the PAC: they are outside the hot-path map, written once at
//! construction, and never part of a transfer-time sequence.

#[path = "lpi2c_regs.rs"]
mod lpi2c_regs;

use self::lpi2c_regs::LpI2cRegisters;
use super::session::{
    CommandPermit, FirstReceivePermit, ReadReceivePermit, RecoveryPermit, RxDmaPermit, StartStatusPermit,
    StartTransitionPermit, StopCompleted, StopFault, StopFinalizeFault, StopFinalized, StopTransitionPermit, StopWait,
    TxDmaPermit,
};
use super::{ControllerRxDma, ControllerTxDma};
use crate::dma::{DMA_MAX_TRANSFER_SIZE, DmaChannel, InvalidParameters, TransferOptions};
use crate::pac;
use crate::pac::lpi2c::Cmd as ControllerCommand;
use crate::pac::lpi2c::{
    Alf, Dmf, Epf, Mbf, Mcr, McrRrf, McrRtf, Mder, Mfsr, Mier, Mrdr, Msr, MsrFef, MsrSdf, Mtdr, Ndf, Param, Pltf, Stf,
};
use tock_registers::interfaces::{Readable, Writeable};

/// Unforgeable authority for semantic session transitions performed by this
/// register facade. Its tuple field and constructor stay private here, so a
/// call site outside this facade can hold a permit but cannot invoke its
/// sealed phase-transition method without going through the matching
/// fault/FIFO/MMIO operation below.
pub(super) struct FacadeSeal(());

impl FacadeSeal {
    fn new() -> Self {
        Self(())
    }
}

/// A typed snapshot of the controller error flags relevant to transfers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct ControllerStatus {
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

    fn error(self) -> Option<ControllerStatusError> {
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

/// Proof that an NDF or FEF snapshot is still latched in hardware.
///
/// This is deliberately neither `Copy` nor `Clone`. Only this module can
/// mint it, and recovery operations consume it before they release a
/// halted engine or discard its frozen pipeline. Debug/test builds also
/// fail deterministically if a proof reaches `Drop` unresolved: Rust is
/// affine rather than linear, so this catches the remaining explicit
/// discard escape during development.
#[derive(Debug, PartialEq, Eq)]
#[must_use]
pub(super) struct HaltedFault {
    owner: usize,
    error: ControllerStatusError,
    #[cfg(debug_assertions)]
    armed: bool,
}

impl HaltedFault {
    /// The error class of this still-live halt. Recovery may inspect it to
    /// select a protocol close, but cannot extract or resolve the proof
    /// except through this facade.
    pub(super) fn error(&self) -> ControllerStatusError {
        self.error
    }

    fn resolve(&mut self) {
        #[cfg(debug_assertions)]
        {
            self.armed = false;
        }
    }
}

#[cfg(debug_assertions)]
impl Drop for HaltedFault {
    fn drop(&mut self) {
        assert!(
            !self.armed,
            "i2c: a halted-fault proof was dropped without session or recovery ownership"
        );
    }
}

/// A transfer-time fault classified from one status snapshot.
///
/// Ordinary errors are cleared from the snapshot that observed them.
/// NDF/FEF instead produce a non-copyable [`HaltedFault`]: they freeze
/// the queued command pipeline, so the owning controller session must receive
/// the proof before it can return an `IOError`.
#[derive(Debug, PartialEq, Eq)]
#[must_use]
pub(super) struct TransferFault(TransferFaultKind);

/// Private so an I2C driver cannot destructure a fault into an `IOError`
/// and accidentally drop the corresponding halt proof.
#[derive(Debug, PartialEq, Eq)]
enum TransferFaultKind {
    Error(ControllerStatusError),
    Halted(HaltedFault),
}

/// The only container allowed to extract a public error from a
/// [`TransferFault`]. It owns a halt until session drop or start recovery
/// consumes it; no borrowed `error()` accessor exists on the carrier.
#[must_use]
pub(super) struct HaltSlot(Option<HaltedFault>);

impl HaltSlot {
    pub(super) const fn empty() -> Self {
        Self(None)
    }

    pub(super) fn is_empty(&self) -> bool {
        self.0.is_none()
    }

    pub(super) fn capture(&mut self, fault: TransferFault) -> ControllerStatusError {
        match fault.0 {
            TransferFaultKind::Error(error) => error,
            TransferFaultKind::Halted(proof) => {
                assert!(
                    self.0.is_none(),
                    "i2c: a transfer fault was bound while another halt proof was unresolved"
                );
                let error = proof.error();
                self.0 = Some(proof);
                error
            }
        }
    }

    pub(super) fn take(&mut self) -> Option<HaltedFault> {
        self.0.take()
    }
}

/// One step of receive progress. Data drains before faults surface:
/// bytes that arrived before an error are valid and must not be lost.
///
/// Its fault arm carries the same [`TransferFault`] as every active
/// transfer path. In particular, NDF/FEF cannot be flattened to an
/// `IOError` before the live session owns the corresponding halt proof.
///
/// The byte arm also carries [`RxProgress`], minted only after this facade
/// has popped MRDR. This keeps the CPU read state's transition coupled to
/// actual FIFO consumption rather than a caller's interpretation of a
/// readiness flag.
#[derive(Debug, PartialEq, Eq)]
#[must_use]
pub(super) enum RxStep {
    Byte {
        byte: u8,
        progress: RxProgress,
    },
    Fault(TransferFault),
    /// The transfer terminated with no data pending and no fault
    /// flagged. Observed on FRDM-MCXA577 during chained multi-command
    /// reads under interrupt-latency stress: the transfer ends mid-read
    /// with the remaining queued commands discarded — the same silicon
    /// quirk family as the spurious arbitration loss. Without this
    /// variant the reader waits forever for bytes that never arrive.
    Ended,
}

/// Evidence that the first queued RECEIVE executed for one controller.
///
/// This is intentionally neither `Copy` nor `Clone`. Only the register
/// facade can mint it, immediately after its read-only popping access to
/// MRDR or a non-empty FIFO observation. Consuming it is the only CPU or
/// recovery path that may advance a session into its read-streaming cleanup
/// state. The controller identity prevents evidence observed on one instance
/// from being applied to another.
#[derive(Debug, PartialEq, Eq)]
#[must_use]
pub(super) struct RxProgress {
    owner: usize,
    #[cfg(debug_assertions)]
    armed: bool,
}

impl RxProgress {
    fn observed(owner: usize) -> Self {
        Self {
            owner,
            #[cfg(debug_assertions)]
            armed: true,
        }
    }

    /// Consume this evidence for its owning controller. The constructor is
    /// private to this facade; the identity check rejects cross-instance
    /// evidence reuse, while the debug-only drop guard catches ordinary
    /// accidental discards. Session adapters pair this consumption with the
    /// corresponding state transition.
    pub(super) fn consume_for(self, owner: usize) {
        assert_eq!(
            self.owner, owner,
            "i2c: RX progress evidence was applied to a different controller"
        );
        #[cfg(debug_assertions)]
        {
            let mut progress = self;
            progress.armed = false;
        }
    }
}

#[cfg(debug_assertions)]
impl Drop for RxProgress {
    fn drop(&mut self) {
        assert!(
            !self.armed,
            "i2c: RX progress evidence was dropped without required owner-side handling"
        );
    }
}

/// An ordinary non-START command whose wire-level shape has been checked by
/// this facade.
///
/// The controller implementation never writes a raw `Cmd`/`DATA` pair.
/// Keeping the PAC encoding private here prevents a new call site from
/// accidentally emitting a receive count of zero or a command whose
/// FIFO/fault precondition was checked elsewhere. START has a separate
/// [`StartAction`] type, and STOP has [`StopAction`], so neither can enter
/// the ordinary command gate.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[must_use]
pub(super) struct ControllerAction(ControllerActionKind);

/// A semantic START action. It is separate from ordinary active commands
/// because the facade consumes it with a [`StartTransitionPermit`], making
/// the queued-START phase transition inseparable from its MTDR write.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[must_use]
pub(super) struct StartAction(StartSpec);

/// A semantic trailing STOP action. It is intentionally separate from
/// [`ControllerAction`] because a normal STOP is valid only from the
/// session's stable write/read-streaming states and must leave an explicit
/// `StopPending` recovery owner as soon as MTDR accepts it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[must_use]
pub(super) struct StopAction;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ControllerActionKind {
    Transmit(u8),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct StartSpec {
    address: u8,
    read: bool,
    high_speed: bool,
}

impl ControllerAction {
    pub(super) const fn transmit(byte: u8) -> Self {
        Self(ControllerActionKind::Transmit(byte))
    }

    fn encode(self) -> (ControllerCommand, u8) {
        match self.0 {
            ControllerActionKind::Transmit(byte) => (ControllerCommand::TRANSMIT, byte),
        }
    }
}

impl StartAction {
    /// Construct a seven-bit START action. Invalid addresses cannot be
    /// represented by this type.
    pub(super) const fn new(address: u8, read: bool, high_speed: bool) -> Option<Self> {
        if address < 0x80 {
            Some(Self(StartSpec {
                address,
                read,
                high_speed,
            }))
        } else {
            None
        }
    }

    fn encode(self) -> (ControllerCommand, u8) {
        (
            if self.0.high_speed {
                ControllerCommand::START_HS
            } else {
                ControllerCommand::START
            },
            self.0.address << 1 | u8::from(self.0.read),
        )
    }

    pub(super) fn is_read(self) -> bool {
        self.0.read
    }
}

impl StopAction {
    pub(super) const fn new() -> Self {
        Self
    }

    fn encode(self) -> (ControllerCommand, u8) {
        (ControllerCommand::STOP, 0)
    }
}

/// The indivisible outcome of trying to emit one ordinary CPU command.
///
/// `try_enqueue_active` takes the fault snapshot, checks FIFO capacity,
/// and writes MTDR in one facade operation. Callers cannot observe room and
/// later bypass error classification with a raw write.
#[derive(Debug, PartialEq, Eq)]
#[must_use]
pub(super) enum CommandStep {
    Queued,
    Full,
    Fault(TransferFault),
}

/// Opaque witness that this facade observed a clean, drained queued START.
/// Its constructor and session-transition method stay private to this module;
/// callers can obtain one only as the `Drained` arm of [`StartDrainStep`] and
/// can consume it only through [`ControllerRegisters::finish_start_status`].
#[must_use]
pub(super) struct StartDrained<'a>(StartStatusPermit<'a>);

impl<'a> StartDrained<'a> {
    fn new(permit: StartStatusPermit<'a>) -> Self {
        Self(permit)
    }

    fn owner(&self) -> usize {
        self.0.owner()
    }

    fn commit_settled(self) {
        self.0.commit_settled(FacadeSeal::new());
    }
}

/// A typed poll of a queued START's command-FIFO drain. The capability
/// remains attached to the pending session while waiting, and can become
/// [`StartDrained`] only after this facade observed both a clean status
/// snapshot and an empty TX FIFO.
#[must_use]
pub(super) enum StartDrainStep<'a> {
    Pending(StartStatusPermit<'a>),
    Drained(StartDrained<'a>),
    Fault(TransferFault),
}

/// A typed poll of a normal trailing STOP. The completion permit proves the
/// STOP entered MTDR for this session; an idle observation without it can
/// never become a completed-STOP proof.
#[must_use]
pub(super) enum StopStep {
    Pending(StopWait),
    Completed(StopCompleted),
    Fault(StopFault),
}

/// The pure decision at the ordinary CPU-command gate. It deliberately
/// models fault and FIFO-full as simultaneous facts, even though the live
/// path refuses to read FIFO state after obtaining a fault proof. The
/// combined case keeps the required priority executable in a const table.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct ActiveGateInput {
    fault: bool,
    pending: usize,
    capacity: usize,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ActiveGate {
    Fault,
    Full,
    Ready,
}

const fn decide_active_gate(input: ActiveGateInput) -> ActiveGate {
    if input.fault {
        ActiveGate::Fault
    } else if input.pending >= input.capacity {
        ActiveGate::Full
    } else {
        ActiveGate::Ready
    }
}

/// The only command sequences recovery may emit while a fault can still be
/// latched. This replaces a boolean flag so both protocol shapes stay
/// auditable at the sole active-fault bypass.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum RecoveryClose {
    Stop,
    ReleaseAddressedRead,
}

/// The literal recovery command plan. Capacity and emission both consume
/// this one value, so a close can neither reserve one slot and emit two
/// commands nor accidentally reorder the manual receive and STOP.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum RecoveryCommand {
    ReceiveOne,
    Stop,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct RecoveryBatch {
    first: Option<RecoveryCommand>,
    second: RecoveryCommand,
}

impl RecoveryBatch {
    const fn slots(self) -> usize {
        1 + self.first.is_some() as usize
    }

    const fn fits(self, pending: usize, capacity: usize) -> bool {
        capacity.saturating_sub(pending) >= self.slots()
    }
}

impl RecoveryClose {
    const fn batch(self) -> RecoveryBatch {
        match self {
            Self::Stop => RecoveryBatch {
                first: None,
                second: RecoveryCommand::Stop,
            },
            Self::ReleaseAddressedRead => RecoveryBatch {
                first: Some(RecoveryCommand::ReceiveOne),
                second: RecoveryCommand::Stop,
            },
        }
    }
}

// These are compiled for every supported embedded target, rather than only
// by a host test harness. The production methods below call the same pure
// classifiers and plans, so they reject a regression in these decision and
// batch rules. They complement — rather than replace — hardware timing
// tests for the MMIO observations surrounding the rules.
const _: () = {
    assert!(matches!(
        decide_active_gate(ActiveGateInput {
            fault: true,
            pending: 4,
            capacity: 4,
        }),
        ActiveGate::Fault
    ));
    assert!(matches!(
        decide_active_gate(ActiveGateInput {
            fault: false,
            pending: 4,
            capacity: 4,
        }),
        ActiveGate::Full
    ));
    assert!(matches!(
        decide_active_gate(ActiveGateInput {
            fault: false,
            pending: 5,
            capacity: 4,
        }),
        ActiveGate::Full
    ));
    assert!(matches!(
        decide_active_gate(ActiveGateInput {
            fault: false,
            pending: 3,
            capacity: 4,
        }),
        ActiveGate::Ready
    ));

    assert!(matches!(
        RecoveryClose::Stop.batch(),
        RecoveryBatch {
            first: None,
            second: RecoveryCommand::Stop
        }
    ));
    assert!(matches!(
        RecoveryClose::ReleaseAddressedRead.batch(),
        RecoveryBatch {
            first: Some(RecoveryCommand::ReceiveOne),
            second: RecoveryCommand::Stop
        }
    ));
    assert!(RecoveryClose::Stop.batch().fits(3, 4));
    assert!(!RecoveryClose::Stop.batch().fits(4, 4));
    assert!(RecoveryClose::ReleaseAddressedRead.batch().fits(2, 4));
    assert!(!RecoveryClose::ReleaseAddressedRead.batch().fits(3, 4));
    assert!(!RecoveryClose::ReleaseAddressedRead.batch().fits(5, 4));

    assert!(matches!(StartAction::new(0x7f, false, false), Some(_)));
    assert!(matches!(StartAction::new(0x80, false, false), None));
    assert!(matches!(ControllerRegisters::encode_receive(1), Some(0)));
    assert!(matches!(ControllerRegisters::encode_receive(256), Some(255)));
    assert!(matches!(ControllerRegisters::encode_receive(0), None));
    assert!(matches!(ControllerRegisters::encode_receive(257), None));
};

/// Safe controller-specific operations over the LPI2C register block.
#[derive(Clone, Copy)]
pub(super) struct ControllerRegisters {
    regs: &'static LpI2cRegisters,
}

/// A live RX DMA handoff to the controller's read-only FIFO port.
///
/// It owns the pairing that raw pointer APIs cannot express: MDER is off
/// and the channel is quiesced before the destination borrow and its session
/// capability are released. The only explicit completion path and `Drop`
/// use the same cleanup routine.
#[must_use]
pub(super) struct RxDmaLease<'s, 'channel, 'dma, 'buf> {
    regs: ControllerRegisters,
    channel: &'channel DmaChannel<'dma>,
    permit: RxDmaPermit<'s>,
    total: usize,
    armed: bool,
    _buffer: core::marker::PhantomData<&'buf mut [u8]>,
}

impl RxDmaLease<'_, '_, '_, '_> {
    /// Register for this lease's own completion event and report the
    /// level-latched DONE state. The caller cannot accidentally wait on a
    /// different controller DMA channel after it has armed this lease.
    pub(super) fn poll_complete(&self, cx: &mut core::task::Context<'_>) -> bool {
        while self.channel.wait_cell().poll_wait(cx).is_ready() {}
        self.channel.is_done()
    }

    /// Quiesce the exact request path this lease armed and retain a
    /// first-read proof if DMA moved a byte or a byte remains in MRDR.
    fn quiesce_and_note(&mut self) -> bool {
        cortex_m::asm::dsb();
        self.regs.set_rx_dma(false);
        let complete = self.channel.quiesce();
        let moved = if complete {
            self.total
        } else {
            self.channel.transferred_bytes()
        };
        if moved != 0 || self.regs.rx_pending() {
            self.permit.note_read_progress(FacadeSeal::new());
        }
        complete
    }

    /// Finish the DMA handoff before inspecting status or returning from the
    /// transfer. Dropping an unfinished lease performs the same operation.
    pub(super) fn finish(mut self) -> bool {
        let complete = self.quiesce_and_note();
        self.armed = false;
        complete
    }
}

impl Drop for RxDmaLease<'_, '_, '_, '_> {
    fn drop(&mut self) {
        if self.armed {
            let _ = self.quiesce_and_note();
        }
    }
}

/// A live TX DMA handoff to the controller's write-only command/data port.
/// See [`RxDmaLease`] for why this is a lease rather than a bare address.
#[must_use]
pub(super) struct TxDmaLease<'s, 'channel, 'dma, 'buf> {
    regs: ControllerRegisters,
    channel: &'channel DmaChannel<'dma>,
    _permit: TxDmaPermit<'s>,
    armed: bool,
    _buffer: core::marker::PhantomData<&'buf [u8]>,
}

impl TxDmaLease<'_, '_, '_, '_> {
    /// Register for this lease's own completion event; see
    /// [`RxDmaLease::poll_complete`].
    pub(super) fn poll_complete(&self, cx: &mut core::task::Context<'_>) -> bool {
        while self.channel.wait_cell().poll_wait(cx).is_ready() {}
        self.channel.is_done()
    }

    fn quiesce(&mut self) -> bool {
        cortex_m::asm::dsb();
        self.regs.set_tx_dma(false);
        self.channel.quiesce()
    }

    /// Finish the DMA handoff before status is inspected or its source
    /// buffer can be released. Dropping an unfinished lease does the same.
    pub(super) fn finish(mut self) -> bool {
        let complete = self.quiesce();
        self.armed = false;
        complete
    }
}

impl Drop for TxDmaLease<'_, '_, '_, '_> {
    fn drop(&mut self) {
        if self.armed {
            let _ = self.quiesce();
        }
    }
}

/// Match the eDMA setup APIs' buffer precondition before an arm method
/// mutates any channel state. That keeps a rejected future call site
/// completely side-effect free rather than relying on a cleanup lease that
/// cannot exist until after setup succeeds.
const fn dma_buffer_fits(length: usize) -> bool {
    length != 0 && length <= DMA_MAX_TRANSFER_SIZE
}

const _: () = {
    assert!(!dma_buffer_fits(0));
    assert!(dma_buffer_fits(1));
    assert!(dma_buffer_fits(DMA_MAX_TRANSFER_SIZE));
    assert!(!dma_buffer_fits(DMA_MAX_TRANSFER_SIZE.saturating_add(1)));
};

impl ControllerRegisters {
    pub(super) fn new(regs: pac::lpi2c::Lpi2c) -> Self {
        Self {
            regs: lpi2c_regs::from_pac(regs),
        }
    }

    /// Cross-check the hidden raw layout against the linked PAC before
    /// a controller is configured. Keeping this entry point on the
    /// facade means driver code never needs access to raw MMIO cells.
    pub(super) fn check_layout(regs: pac::lpi2c::Lpi2c) {
        lpi2c_regs::check_layout(regs);
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

    fn register_address(&self) -> usize {
        self.regs as *const LpI2cRegisters as usize
    }

    /// Stable identity used by non-copyable protocol capabilities that
    /// cross from the session layer into this facade.
    pub(super) fn identity(&self) -> usize {
        self.register_address()
    }

    /// Mint first-RECEIVE execution evidence for this controller only.
    fn rx_progress(&self) -> RxProgress {
        RxProgress::observed(self.identity())
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
    fn enable_error_interrupts(&self) {
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

    /// Arm the transmit-path sources and report whether a CPU command may
    /// make progress. This is deliberately non-consuming: the caller must
    /// immediately retry [`Self::try_enqueue_active`] so the actual MTDR
    /// write still shares the fault/capacity check with every other command.
    pub(super) fn tx_room_wake(&self) -> bool {
        self.enable_transmit_interrupts();
        self.tx_pending() < self.tx_fifo_capacity() || self.read_status().error().is_some()
    }

    /// Arm the receive-path interrupt set and evaluate its wake
    /// condition, as one operation — see [`Self::tx_settle_wake`].
    pub(super) fn rx_wake(&self) -> bool {
        self.enable_receive_interrupts();
        self.rx_ready()
    }

    /// Arm the error interrupt set (for DMA-driven transfers, where
    /// TDF/RDF service the engine) and take a latched transfer fault,
    /// as one operation — see [`Self::tx_settle_wake`].
    ///
    /// The DMA poller returns the non-copyable [`TransferFault`] to its
    /// caller; that caller binds a halting proof to the live session
    /// before returning its public `IOError`.
    pub(super) fn error_wake(&self) -> Option<TransferFault> {
        self.enable_error_interrupts();
        self.take_active_fault()
    }

    fn enable_receive_interrupts(&self) {
        // No EPIE/SDIE — see `enable_error_interrupts`.
        self.write_mier(|w| {
            w.set_rdie(true);
            w.set_ndie(true);
            w.set_alie(true);
            w.set_feie(true);
            w.set_pltie(true);
        });
    }

    fn enable_transmit_interrupts(&self) {
        self.write_mier(|w| {
            w.set_tdie(true);
            w.set_ndie(true);
            w.set_alie(true);
            w.set_feie(true);
            w.set_pltie(true);
        });
    }

    /// Reset both FIFOs while the controller is disabled during setup.
    ///
    /// Transfer-time resets must instead flow through recovery methods
    /// that prove the engine is halted, idle, or terminal.
    pub(super) fn reset_while_disabled(&self) {
        self.reset_fifos();
    }

    fn reset_fifos(&self) {
        critical_section::with(|_| {
            self.modify_mcr(|w| {
                w.set_rtf(McrRtf::Reset);
                w.set_rrf(McrRrf::Reset);
            });
        });
    }

    /// Clear every controller status flag.
    ///
    /// This is intentionally private: clearing NDF/FEF while commands
    /// remain queued resumes a halted controller. Public callers must
    /// prove the phase in which that is safe through one of the narrow
    /// semantic operations below.
    fn clear_all_status(&self) {
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

    /// Clear the power-on/configuration residue before any transaction
    /// has been opened.
    pub(super) fn clear_after_init(&self) {
        self.clear_all_status();
    }

    /// Discard state for a controller that was observed idle before
    /// recovery began.
    ///
    /// An idle master owns no live command pipeline, so resetting the
    /// FIFOs and clearing every W1C flag cannot release stale work.
    pub(super) fn discard_idle_recovery_state(&self) {
        self.reset_fifos();
        self.clear_all_status();
    }

    /// Discard the terminal state of a halt whose controller already
    /// reached idle before its recovery owner ran.
    fn discard_idle_halt(&self, mut halt: HaltedFault) {
        self.assert_halt_owner(&halt);
        assert!(
            !self.master_busy(),
            "i2c: an idle-halt proof was discarded while the controller was busy"
        );
        self.reset_fifos();
        self.clear_all_status();
        halt.resolve();
        halt.resolve();
    }

    /// Release a halted controller only after recovery established that
    /// no command remains queued.
    fn release_halted_empty_fifo(&self, mut halt: HaltedFault) {
        self.assert_halt_owner(&halt);
        self.clear_all_status();
        halt.resolve();
        halt.resolve();
    }

    /// Discard a frozen command pipeline after the auto-STOP grace
    /// window expired.
    fn discard_frozen_halt(&self, mut halt: HaltedFault) {
        self.assert_halt_owner(&halt);
        self.reset_fifos();
        self.clear_all_status();
        halt.resolve();
        halt.resolve();
    }

    fn confirm_auto_stopped_halt(&self, mut halt: HaltedFault) {
        self.assert_halt_owner(&halt);
        assert!(
            !self.master_busy(),
            "i2c: auto-STOP halt was confirmed while the controller was still busy"
        );
        halt.resolve();
        halt.resolve();
    }

    /// Close out recovery after the recovery drain reached a terminal
    /// state. The caller has already made the master idle or escalated
    /// it, so no live command can be released by this cleanup.
    pub(super) fn finish_recovery(&self) {
        self.reset_fifos();
        self.clear_current_status();
    }

    fn halted_from_snapshot(&self, msr: &Msr) -> Option<HaltedFault> {
        if msr.ndf() == Ndf::IntYes {
            Some(HaltedFault {
                owner: self.register_address(),
                error: ControllerStatusError::AddressNack,
                #[cfg(debug_assertions)]
                armed: true,
            })
        } else if msr.fef() == MsrFef::IntYes {
            Some(HaltedFault {
                owner: self.register_address(),
                error: ControllerStatusError::Fifo,
                #[cfg(debug_assertions)]
                armed: true,
            })
        } else {
            None
        }
    }

    fn assert_halt_owner(&self, halt: &HaltedFault) {
        assert!(
            self.register_address() == halt.owner,
            "i2c: a halted-fault proof was resolved through a different controller"
        );
    }

    /// Clear only non-halting W1C flags from a sampled status word.
    ///
    /// NDF/FEF are deliberately masked out: either one freezes the
    /// command pipeline, and recovery must consume a [`HaltedFault`]
    /// before releasing or discarding it.
    fn clear_snapshot_preserving_halts(&self, msr: Msr) {
        let mut wb = msr;
        wb.set_ndf(Ndf::IntNo);
        wb.set_fef(MsrFef::IntNo);
        self.write_msr(wb);
    }

    /// Take a transfer fault from an already-sampled status word.
    ///
    /// A halt wins over every ordinary-error priority, including a
    /// co-latched ALF. The snapshot clear intentionally leaves NDF/FEF
    /// latched; their proof is transferred to recovery through the
    /// returned [`TransferFault`]. A clean snapshot is left untouched so
    /// event state such as EPF remains available to the caller.
    fn take_active_fault_from_snapshot(&self, msr: &Msr) -> Option<TransferFault> {
        if let Some(halt) = self.halted_from_snapshot(msr) {
            self.clear_snapshot_preserving_halts(Msr(msr.0));
            Some(TransferFault(TransferFaultKind::Halted(halt)))
        } else if let Some(error) = ControllerStatus::from_snapshot(msr).error() {
            // Writing the sampled snapshot clears only flags observed by
            // this read, so a fault that races in afterward remains
            // available to the next observation.
            self.write_msr(Msr(msr.0));
            Some(TransferFault(TransferFaultKind::Error(error)))
        } else {
            None
        }
    }

    /// Take one currently active transfer fault.
    ///
    /// Unlike a bare status read, this either returns the halt proof the
    /// caller must thread to its session or clears only the ordinary
    /// fault observed in the same snapshot. It deliberately does
    /// nothing on a clean snapshot.
    pub(super) fn take_active_fault(&self) -> Option<TransferFault> {
        let msr = self.msr();
        self.take_active_fault_from_snapshot(&msr)
    }

    /// Poll a queued START until its command FIFO entry has drained.
    ///
    /// The returned [`StartDrained`] witness is the only input accepted by
    /// [`Self::finish_start_status`]. It is therefore impossible to consume
    /// the START-boundary W1C snapshot from this facade just because some
    /// unrelated caller observed a clean status word: the same pending
    /// session must first survive an empty-TX-FIFO observation here.
    pub(super) fn poll_start_drain<'a>(&self, permit: StartStatusPermit<'a>) -> StartDrainStep<'a> {
        assert!(
            self.register_address() == permit.owner(),
            "i2c: a START-drain permit was used through a different controller"
        );
        let msr = self.msr();
        if let Some(fault) = self.take_active_fault_from_snapshot(&msr) {
            StartDrainStep::Fault(fault)
        } else if self.mfsr().txcount() == 0 {
            StartDrainStep::Drained(StartDrained::new(permit))
        } else {
            StartDrainStep::Pending(permit)
        }
    }

    /// Classify and clear the final status snapshot after a typed queued
    /// START drain. The witness carries the pending session borrow, and a
    /// clean result commits that session's `StartPending -> Stable` mapping
    /// before the borrow is released. A fault leaves the phase pending so
    /// its normal recovery owner remains intact.
    pub(super) fn finish_start_status(&self, drained: StartDrained<'_>) -> Result<(), TransferFault> {
        assert!(
            self.register_address() == drained.owner(),
            "i2c: a START-drain witness was finalized through a different controller"
        );
        let msr = self.msr();
        if let Some(fault) = self.take_active_fault_from_snapshot(&msr) {
            Err(fault)
        } else {
            self.write_msr(msr);
            drained.commit_settled();
            Ok(())
        }
    }

    /// Observe a halting fault during the recovery drain.
    ///
    /// This deliberately reports NDF/FEF before any ordinary error
    /// priority. Scrub-safe ALF/PLTF bits from the same snapshot are
    /// cleared, but the halting bits stay latched until a recovery
    /// operation consumes the returned proof.
    pub(super) fn observe_recovery_halt(&self) -> Option<HaltedFault> {
        let msr = self.msr();
        if let Some(halt) = self.halted_from_snapshot(&msr) {
            self.clear_snapshot_preserving_halts(msr);
            return Some(halt);
        }

        if matches!(
            ControllerStatus::from_snapshot(&msr).error(),
            Some(ControllerStatusError::ArbitrationLoss) | Some(ControllerStatusError::PinLowTimeout)
        ) {
            self.write_msr(msr);
        }

        None
    }

    /// Resolve an NDF/FEF halt without inferring the fault-time FIFO
    /// state from a later TXCOUNT sample.
    ///
    /// If the hardware's auto-STOP reaches idle during a short grace
    /// window, recovery is finished. Otherwise the controller is still
    /// frozen: its pipeline is discarded while halted, then the caller
    /// can queue the manual recovery close. Keeping the observation,
    /// FIFO predicates, and token consumption together prevents an
    /// active transfer from being reset or un-halted by a reordered
    /// controller-side call.
    pub(super) fn resolve_halted_fault(
        &self,
        halt: HaltedFault,
        timeout: embassy_time::Duration,
        deadline: embassy_time::Instant,
    ) -> bool {
        self.assert_halt_owner(&halt);

        // Empty NOW means no command can be replayed if the halt is
        // cleared, regardless of the auto-STOP condition at the fault
        // instant. Let the recovery drain form a manual close.
        if self.tx_pending() == 0 {
            self.release_halted_empty_fifo(halt);
            return false;
        }

        // A STOP is normally a bit-time. The grace is deliberately
        // scaled for configurations that permit longer clock stretching,
        // and clamped to the caller's remaining recovery budget.
        let grace_len = core::cmp::max(embassy_time::Duration::from_millis(2), timeout / 8);
        let now = embassy_time::Instant::now();
        let grace_end = core::cmp::min(now + grace_len, deadline);
        while self.master_busy() {
            if let Some(progress) = self.discard_rx() {
                // This helper has no live session phase to advance: it is
                // waiting for an already-halted engine's autonomous
                // auto-STOP. Still consume the proof explicitly so a future
                // caller cannot silently discard RX execution evidence.
                progress.consume_for(self.identity());
            }
            if embassy_time::Instant::now() > grace_end {
                self.discard_frozen_halt(halt);
                return false;
            }
        }

        self.confirm_auto_stopped_halt(halt);
        true
    }

    /// Resolve a halt which was observed by the transaction owner
    /// before recovery began.
    ///
    /// This preserves the proof across an error return instead of
    /// relying on a later status re-read to rediscover the same latch.
    pub(super) fn resolve_owned_halt(
        &self,
        halt: HaltedFault,
        timeout: embassy_time::Duration,
        deadline: embassy_time::Instant,
    ) -> bool {
        if self.master_busy() {
            self.resolve_halted_fault(halt, timeout, deadline)
        } else {
            self.discard_idle_halt(halt);
            true
        }
    }

    fn read_status(&self) -> ControllerStatus {
        ControllerStatus::from_snapshot(&self.msr())
    }

    fn clear_current_status(&self) {
        let msr = self.msr();
        self.write_msr(msr);
    }

    /// One step of receive progress: a received byte (popped from the RX
    /// FIFO), a fault that means no more data will arrive, an early
    /// termination, or `None` to keep waiting. There is deliberately no
    /// data-only variant of this wait — see the module docs.
    pub(super) fn rx_step(&self) -> Option<RxStep> {
        if self.mfsr().rxcount() != 0 {
            return Some(RxStep::Byte {
                byte: Mrdr(self.regs.mrdr.get()).data(),
                progress: self.rx_progress(),
            });
        }
        let msr = self.msr();
        // A final byte can arrive between the first FIFO observation and
        // the status snapshot. Preserve data-before-fault ordering by
        // checking once more before consuming that snapshot: the next
        // step will classify its still-latched fault or termination.
        if self.mfsr().rxcount() != 0 {
            return Some(RxStep::Byte {
                byte: Mrdr(self.regs.mrdr.get()).data(),
                progress: self.rx_progress(),
            });
        }
        if let Some(fault) = self.take_active_fault_from_snapshot(&msr) {
            return Some(RxStep::Fault(fault));
        }
        if Self::transfer_ended_from_snapshot(&msr) {
            return Some(RxStep::Ended);
        }
        None
    }

    /// Non-consuming readiness check for [`Self::rx_step`], for use in
    /// wake conditions. True when a byte, a fault, or an early transfer
    /// termination is pending.
    fn rx_ready(&self) -> bool {
        self.mfsr().rxcount() != 0 || self.read_status().error().is_some() || self.transfer_ended()
    }

    /// Non-consuming observation that at least one received byte remains
    /// in the hardware FIFO. DMA cleanup uses this after it has disabled
    /// requests and quiesced the channel: a byte that reached the FIFO but
    /// not memory still proves that the first RECEIVE executed.
    fn rx_pending(&self) -> bool {
        self.mfsr().rxcount() != 0
    }

    /// Non-consuming proof that a first RECEIVE executed. This captures the
    /// same hardware fact as [`Self::rx_pending`] but prevents a recovery or
    /// fault path from turning a bare boolean into a read-streaming session
    /// transition.
    pub(super) fn observe_rx_progress(&self) -> Option<RxProgress> {
        if self.rx_pending() {
            Some(self.rx_progress())
        } else {
            None
        }
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
    /// transaction are cleared by the START's own
    /// [`Self::finish_start_status`] check.
    pub(super) fn transfer_ended(&self) -> bool {
        let msr = self.msr();
        Self::transfer_ended_from_snapshot(&msr)
    }

    fn transfer_ended_from_snapshot(msr: &Msr) -> bool {
        msr.epf() == Epf::IntYes && msr.mbf() == Mbf::Idle
    }

    /// Capacity of the shared command/transmit FIFO, in entries.
    pub(super) fn tx_fifo_capacity(&self) -> usize {
        1usize << Param(self.regs.param.get()).mtxfifo()
    }

    /// True when the command FIFO has fully drained *or* a fault is
    /// pending — i.e. the wait for a queued command is over, one way or
    /// the other. Callers classify with [`Self::take_active_fault`] or
    /// `read_status`.
    fn tx_settled(&self) -> bool {
        self.mfsr().txcount() == 0 || self.read_status().error().is_some()
    }

    /// True when the RECOVERY drain is over: the command FIFO is empty
    /// AND this controller's bus engine is idle (MBF clear).
    ///
    /// `tx_settled` is the wrong wait for recovery: it exits on any
    /// latched fault, and an aborted transfer's in-flight command —
    /// which a FIFO reset deliberately does NOT abort — completes
    /// autonomously, overflowing the un-drained RX FIFO and latching
    /// FEF mid-drain. Exiting there lets the trailing FIFO reset
    /// discard the still-queued recovery STOP, and lets the command's
    /// remaining bytes land in the RX FIFO *after* the final reset, to
    /// be served as the stale head of the next transfer (observed on
    /// hardware via drop-cancellation: a later read returned the
    /// aborted read's tail bytes first). This predicate ignores faults
    /// entirely — recovery has no caller to classify for — and insists
    /// on genuine idleness, so the trailing cleanup runs after the
    /// last autonomous byte and the STOP.
    pub(super) fn recovery_settled(&self) -> bool {
        self.mfsr().txcount() == 0 && self.msr().mbf() == Mbf::Idle
    }

    /// True while this controller's bus engine is mid-transfer (MBF).
    /// Recovery keys on this: a recovery STOP is meaningful ONLY for
    /// an open transfer — queued onto an idle controller it is a
    /// protocol violation the engine refuses (FEF) and, if the fault
    /// is then scrubbed, retries forever: a livelock that burns the
    /// whole recovery deadline. (The old fault-exit drain masked this
    /// by bailing on the FEF and silently discarding the bogus STOP.)
    pub(super) fn master_busy(&self) -> bool {
        self.msr().mbf() == Mbf::Busy
    }

    /// Number of commands currently waiting in the transmit FIFO.
    pub(super) fn tx_pending(&self) -> usize {
        self.mfsr().txcount() as usize
    }

    /// One typed step of a trailing-STOP completion wait. A normal
    /// [`StopWait`] exists only after a `StopAction` entered
    /// MTDR for the owning session, so an otherwise idle controller cannot
    /// manufacture a terminal proof for an unrelated transaction.
    ///
    /// "Pulled from the FIFO" is NOT completion: the wire condition
    /// follows later, and a fault in that window belongs to the
    /// transaction that requested the STOP — a wait that ends at FIFO
    /// drain hands such faults to whoever runs next, with no recovery
    /// owner. Deliberately not SDF-based either: this silicon raises
    /// SDF spuriously mid-transfer (see `transfer_ended`), while
    /// MBF reflects the engine actually going idle. The fault arm carries
    /// the live transaction's proof; terminal status is cleared only after
    /// the [`StopCompleted`] proof is returned to [`Self::finish_stop`].
    pub(super) fn stop_step(&self, stop: StopWait) -> StopStep {
        assert!(
            self.register_address() == stop.owner(),
            "i2c: a STOP completion owner was used through a different controller"
        );
        let msr = self.msr();
        if let Some(fault) = self.take_active_fault_from_snapshot(&msr) {
            return StopStep::Fault(StopFault::new(stop, fault, FacadeSeal::new()));
        }
        if self.mfsr().txcount() == 0 && msr.mbf() == Mbf::Idle {
            return StopStep::Completed(stop.into_completed(FacadeSeal::new()));
        }
        StopStep::Pending(stop)
    }

    /// Classify and clear the terminal status snapshot after a
    /// physically completed STOP.
    ///
    /// [`StopCompleted`] is intentionally consumed here, so terminal
    /// status is cleared only after the physical STOP proof. A fault that
    /// races in after the final `stop_step` is still returned through the
    /// same typed path, rather than being erased by terminal cleanup.
    pub(super) fn finish_stop(&self, completed: StopCompleted) -> Result<StopFinalized, StopFinalizeFault> {
        assert!(
            self.register_address() == completed.owner(),
            "i2c: a completed-STOP proof was finalized through a different controller"
        );
        let msr = self.msr();
        if let Some(fault) = self.take_active_fault_from_snapshot(&msr) {
            return Err(StopFinalizeFault::new(completed.into_wait(), fault, FacadeSeal::new()));
        }
        self.write_msr(msr);
        Ok(completed.commit_finalized(FacadeSeal::new()))
    }

    /// Recovery helper: discard everything currently in the RX FIFO.
    ///
    /// An aborted read's in-flight RECEIVE — which a FIFO reset
    /// deliberately does not touch — keeps clocking with nobody
    /// popping, and once the RX FIFO fills the engine holds SCL in
    /// flow control (no fault latches: it is not an error). The
    /// abandoned command then never finishes and the recovery STOP
    /// queued behind it never executes (hardware-observed: MBF busy
    /// forever, rxcount pinned at the FIFO depth, zero faults). The
    /// recovery drain must keep popping to let it run to its
    /// auto-NACKed end.
    ///
    /// Returns execution evidence when at least one byte was removed. A
    /// recovery owner consumes that evidence before retaining the first-
    /// RECEIVE phase transition, even though it intentionally discards the
    /// byte rather than delivering it.
    pub(super) fn discard_rx(&self) -> Option<RxProgress> {
        let mut discarded = false;
        while self.mfsr().rxcount() != 0 {
            let _ = self.regs.mrdr.get();
            discarded = true;
        }
        if discarded { Some(self.rx_progress()) } else { None }
    }

    /// Last-resort recovery escalation: reset the master engine by
    /// toggling MCR[MEN], aborting a wedged transfer and releasing the
    /// bus. Unlike MCR[RST] this resets only the master logic — the
    /// speed/pin/FIFO-watermark configuration is preserved, so no
    /// re-init is needed.
    ///
    /// Needed because a FIFO reset issued while the master is mid
    /// transfer can wedge the engine (observed on hardware via
    /// drop-cancellation between the address phase and the first data
    /// byte of a read: the engine holds the bus, never executes the
    /// queued recovery STOP, and every later transfer runs with the
    /// RX FIFO permanently lagging by its own depth). The graceful
    /// recovery STOP stays the first choice — this runs only when
    /// that provably cannot drain.
    /// Escalate a recovery drain that exceeded its bounded deadline.
    pub(super) fn reset_after_recovery_timeout(&self) {
        self.reset_engine();
    }

    fn reset_engine(&self) {
        self.modify_mcr(|w| w.set_men(false));
        self.modify_mcr(|w| w.set_men(true));
    }

    /// Try to enqueue one ordinary CPU command.
    ///
    /// This is the only active-transfer path to MTDR. It observes and
    /// retains a halting fault before it considers FIFO space, then emits
    /// the already-validated semantic action immediately when space exists.
    /// A full FIFO is deliberately distinct from a fault so blocking and
    /// async callers can choose their own bounded wait without flattening a
    /// live [`TransferFault`].
    pub(super) fn try_enqueue_active(&self, permit: CommandPermit<'_>, action: ControllerAction) -> CommandStep {
        assert!(
            self.register_address() == permit.owner(),
            "i2c: a command permit was used through a different controller"
        );
        let (command, data) = action.encode();
        self.try_enqueue_encoded(command, data)
    }

    /// Queue a START and atomically record its pending transition in the
    /// owning session/reservation. Ordinary command permits cannot enter
    /// this path, so a START cannot be emitted without its recovery phase.
    pub(super) fn try_enqueue_start(&self, permit: StartTransitionPermit<'_>, action: StartAction) -> CommandStep {
        assert!(
            self.register_address() == permit.owner(),
            "i2c: a START-transition permit was used through a different controller"
        );
        let (command, data) = action.encode();
        let step = self.try_enqueue_encoded(command, data);
        permit.finish_enqueue(action, matches!(&step, CommandStep::Queued), FacadeSeal::new());
        step
    }

    /// Queue a normal trailing STOP and atomically record its pending
    /// terminal transition in the owning session. Ordinary command permits
    /// cannot enter this path, so an idle observation elsewhere cannot be
    /// mistaken for completion of a STOP this session never queued.
    pub(super) fn try_enqueue_stop(&self, permit: StopTransitionPermit<'_>, action: StopAction) -> CommandStep {
        assert!(
            self.register_address() == permit.owner(),
            "i2c: a STOP-transition permit was used through a different controller"
        );
        let (command, data) = action.encode();
        let step = self.try_enqueue_encoded(command, data);
        permit.finish_enqueue(matches!(&step, CommandStep::Queued), FacadeSeal::new());
        step
    }

    fn try_enqueue_encoded(&self, command: ControllerCommand, data: u8) -> CommandStep {
        match self.take_active_fault() {
            Some(fault) => match decide_active_gate(ActiveGateInput {
                // Do not sample FIFO state on this path. The model's
                // fault+full table verifies that this priority stays the
                // correct decision even if both facts exist in hardware.
                fault: true,
                pending: 0,
                capacity: 0,
            }) {
                ActiveGate::Fault => CommandStep::Fault(fault),
                ActiveGate::Full | ActiveGate::Ready => unreachable!("i2c: fault gate classification drifted"),
            },
            None => match decide_active_gate(ActiveGateInput {
                fault: false,
                pending: self.tx_pending(),
                capacity: self.tx_fifo_capacity(),
            }) {
                ActiveGate::Fault => unreachable!("i2c: FIFO gate classification drifted"),
                ActiveGate::Full => CommandStep::Full,
                ActiveGate::Ready => {
                    self.emit_encoded(command, data);
                    CommandStep::Queued
                }
            },
        }
    }

    /// Queue the first RECEIVE behind a read START and atomically record
    /// the recovery-phase transition in its owning session.
    ///
    /// `FirstReceivePermit` can be minted only by an addressed read session.
    /// A `Full` or `Fault` result leaves that session in the addressed phase,
    /// while `Queued` moves it to the explicit first-RECEIVE-pending phase;
    /// byte/FIFO evidence later promotes it to streaming.
    pub(super) fn try_enqueue_first_receive(&self, permit: FirstReceivePermit<'_>, bytes: usize) -> CommandStep {
        assert!(
            self.register_address() == permit.owner(),
            "i2c: a first-read permit was used through a different controller"
        );
        let step = self.try_enqueue_encoded(ControllerCommand::RECEIVE, Self::receive_data(bytes));
        permit.finish_enqueue(matches!(&step, CommandStep::Queued), FacadeSeal::new());
        step
    }

    /// Queue a follow-on RECEIVE. The phase-specific permit means a normal
    /// active action can never accidentally begin a read stream: only the
    /// first-read transition above may do that.
    pub(super) fn try_enqueue_read_receive(&self, permit: ReadReceivePermit<'_>, bytes: usize) -> CommandStep {
        assert!(
            self.register_address() == permit.owner(),
            "i2c: a streaming-read permit was used through a different controller"
        );
        self.try_enqueue_encoded(ControllerCommand::RECEIVE, Self::receive_data(bytes))
    }

    /// Queue recovery's only allowed close sequence once its entire batch
    /// fits. Recovery intentionally runs with a fault latched, so it must
    /// not use [`Self::try_enqueue_active`]; exposing this narrow operation
    /// keeps that exception from becoming a second raw-MTDR escape hatch.
    pub(super) fn try_enqueue_recovery_close(&self, permit: RecoveryPermit, close: RecoveryClose) -> bool {
        assert!(
            self.register_address() == permit.owner(),
            "i2c: a recovery permit was used through a different controller"
        );
        let batch = close.batch();
        if !batch.fits(self.tx_pending(), self.tx_fifo_capacity()) {
            return false;
        }
        if let Some(command) = batch.first {
            self.emit_recovery_command(command);
        }
        self.emit_recovery_command(batch.second);
        true
    }

    /// Validate and encode the LPI2C `RECEIVE` byte count. This stays
    /// private so only the first-read and streaming-read capability paths
    /// can ever reach the raw RECEIVE command.
    const fn encode_receive(bytes: usize) -> Option<u8> {
        if bytes != 0 && bytes <= 256 {
            Some((bytes - 1) as u8)
        } else {
            None
        }
    }

    fn receive_data(bytes: usize) -> u8 {
        Self::encode_receive(bytes).expect("i2c: RECEIVE length must be in the LPI2C range 1..=256")
    }

    fn emit_recovery_command(&self, command: RecoveryCommand) {
        match command {
            RecoveryCommand::ReceiveOne => self.emit_encoded(ControllerCommand::RECEIVE, 0),
            RecoveryCommand::Stop => self.emit_encoded(ControllerCommand::STOP, 0),
        }
    }

    /// Raw MTDR emission. Keeping it private means every normal command is
    /// checked by `try_enqueue_active`, while recovery is constrained to the
    /// dedicated close batch above.
    fn emit_encoded(&self, command: ControllerCommand, data: u8) {
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

    // DMA plumbing. The FIFO addresses remain entirely inside the facade:
    // callers receive a directional lease rather than a raw pointer, so the
    // setup order and inverse cleanup cannot drift apart.

    fn set_rx_dma(&self, enable: bool) {
        self.modify_mder(|w| w.set_rdde(enable));
    }

    fn set_tx_dma(&self, enable: bool) {
        self.modify_mder(|w| w.set_tdde(enable));
    }

    /// Configure and arm a controller-to-memory DMA transfer.
    ///
    /// This is the sole safe owner of the MRDR endpoint. Its return value
    /// retains both the live session proof and destination borrow until the
    /// paired `MDER off -> eDMA quiesce` cleanup runs.
    pub(super) fn arm_rx_dma<'s, 'channel, 'dma, 'buf>(
        &self,
        permit: RxDmaPermit<'s>,
        port: ControllerRxDma<'channel, 'dma>,
        buffer: &'buf mut [u8],
    ) -> Result<RxDmaLease<'s, 'channel, 'dma, 'buf>, InvalidParameters> {
        assert!(
            self.register_address() == permit.owner(),
            "i2c: an RX DMA permit was used through a different controller"
        );
        assert!(
            self.register_address() == port.owner(),
            "i2c: an RX DMA port was used through a different controller"
        );
        if !dma_buffer_fits(buffer.len()) {
            return Err(InvalidParameters);
        }
        let total = buffer.len();
        let channel = port.channel();
        let request = port.request();

        // SAFETY: the capability proves this controller has a live read
        // session, MRDR is the Tock-typed read-only FIFO port, and the
        // returned lease retains `buffer` until it disables MDER and waits
        // for an active minor loop to finish.
        unsafe {
            channel.disable_request();
            channel.clear_done();
            channel.clear_interrupt();
            channel.set_request_source(request);
            channel.setup_read_from_peripheral(
                &self.regs.mrdr as *const _ as *const u8,
                buffer,
                false,
                TransferOptions::COMPLETE_INTERRUPT,
            )?;
            self.set_rx_dma(true);
            channel.enable_request();
        }

        Ok(RxDmaLease {
            regs: *self,
            channel,
            permit,
            total,
            armed: true,
            _buffer: core::marker::PhantomData,
        })
    }

    /// Configure and arm a memory-to-controller DMA transfer.
    ///
    /// This is the sole safe owner of the MTDR endpoint. The directional
    /// permit prevents an RX phase from accidentally writing commands/data,
    /// and the returned lease owns the matching shutdown.
    pub(super) fn arm_tx_dma<'s, 'channel, 'dma, 'buf>(
        &self,
        permit: TxDmaPermit<'s>,
        port: ControllerTxDma<'channel, 'dma>,
        buffer: &'buf [u8],
    ) -> Result<TxDmaLease<'s, 'channel, 'dma, 'buf>, InvalidParameters> {
        assert!(
            self.register_address() == permit.owner(),
            "i2c: a TX DMA permit was used through a different controller"
        );
        assert!(
            self.register_address() == port.owner(),
            "i2c: a TX DMA port was used through a different controller"
        );
        if !dma_buffer_fits(buffer.len()) {
            return Err(InvalidParameters);
        }
        let channel = port.channel();
        let request = port.request();

        // SAFETY: the capability proves this controller has a live write
        // session, MTDR is the Tock-typed write-only FIFO port, and the
        // returned lease retains `buffer` until the request path is shut
        // down and eDMA has become inactive.
        unsafe {
            channel.disable_request();
            channel.clear_done();
            channel.clear_interrupt();
            channel.set_request_source(request);
            channel.setup_write_to_peripheral(
                buffer,
                &self.regs.mtdr as *const _ as *mut u8,
                false,
                TransferOptions::COMPLETE_INTERRUPT,
            )?;
            self.set_tx_dma(true);
            channel.enable_request();
        }

        Ok(TxDmaLease {
            regs: *self,
            channel,
            _permit: permit,
            armed: true,
            _buffer: core::marker::PhantomData,
        })
    }
}
