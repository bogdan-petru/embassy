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
//!
//! Scope: every PROTOCOL register — status, interrupts, DMA enables,
//! data, address status — is reachable only through this facade; the
//! driver holds no generic read/write/modify on any of them. The one
//! deliberate exception is `set_configuration`, which touches
//! init-only configuration registers (SCR/SCFGR1/SCFGR2/SAMR) through
//! the PAC: they are outside the hot-path map, written once at
//! construction, and never part of a transfer-time sequence.

use core::marker::PhantomData;
use core::sync::atomic::{Ordering, fence};

use tock_registers::interfaces::{Readable, Writeable};

use super::lpi2c_regs::{self, LpI2cRegisters};
use crate::dma::{DMA_MAX_TRANSFER_SIZE, InvalidParameters, TransferOptions};
use crate::i2c::target::{TargetRxDma, TargetTxDma};
use crate::pac;
use crate::pac::lpi2c::{Sasr, Scr, ScrRrf, ScrRtf, Sder, Sier, Srdr, Ssr, Stdr};

/// Hardware faults the target status register can report mid-transfer.
///
/// The interrupt masks enable BEIE/FEIE, so these MUST be part of every
/// readiness/event check: a fault that wakes the waiter without
/// surfacing as an event would re-arm and wake forever.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(in crate::i2c) enum TargetFault {
    /// Bit error: the target saw a bus level conflicting with what it
    /// was driving.
    Bit,
    /// FIFO error (receive overflow / transmit underflow).
    Fifo,
}

/// One step of target receive progress.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[must_use]
pub(in crate::i2c) enum TargetRxEvent {
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
pub(in crate::i2c) enum TargetTxStep {
    /// The transmit register has room for the next byte.
    Room,
    /// A fault; the transfer is compromised, push nothing more.
    Fault(TargetFault),
    /// The transfer ended (STOP or repeated START); push nothing more.
    Ended,
}

/// One classified listen event — see
/// [`TargetRegisters::take_listen_event`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[must_use]
pub(in crate::i2c) enum ListenEvent {
    /// A fault surfaced while listening.
    Fault(TargetFault),
    /// Address match for one of the configured addresses.
    AddressValid(u16),
    /// General-call address match.
    GeneralCall,
    /// SMBus-alert address match.
    SmbusAlert,
    /// Transmit-ACK request.
    TransmitAck,
    /// Repeated START observed (address from the last match).
    RepeatedStart(u16),
    /// STOP observed (address from the last match).
    Stop(u16),
    /// Nothing classifiable was latched.
    None,
}

/// Safe target-specific operations over the LPI2C register block.
#[derive(Clone, Copy)]
pub(in crate::i2c) struct TargetRegisters {
    regs: &'static LpI2cRegisters,
}

/// A live target RX-DMA handoff to the read-only SRDR FIFO port.
///
/// The lease ties the destination borrow to the exact shutdown protocol:
/// target DMA requests are disabled and the eDMA channel is quiesced before
/// a caller can inspect the count or release the buffer. `Drop` runs the
/// same protocol, so cancellation cannot leave a DMA write live.
#[must_use]
pub(in crate::i2c) struct TargetRxDmaLease<'channel, 'dma, 'buf> {
    port: TargetRxDma<'channel, 'dma>,
    total: usize,
    armed: bool,
    _buffer: PhantomData<&'buf mut [u8]>,
}

impl TargetRxDmaLease<'_, '_, '_> {
    /// Register for this exact channel's completion and report its
    /// level-latched DONE state. The caller cannot accidentally wait on a
    /// different target DMA channel after arming this lease.
    fn poll_complete(&self, cx: &mut core::task::Context<'_>) -> bool {
        while self.port.channel().wait_cell().poll_wait(cx).is_ready() {}
        self.port.channel().is_done()
    }

    /// Arm/check the exact I2C interrupt set and this lease's DMA channel
    /// together. The live port owns an exclusive target-operation borrow,
    /// so no other target register sequence can run during the handoff.
    pub(in crate::i2c) fn poll_wake(&self, cx: &mut core::task::Context<'_>) -> bool {
        while self.port.wait_cell().poll_wait(cx).is_ready() {}
        self.port.registers().dma_transfer_wake() || self.poll_complete(cx)
    }

    /// Disable the matching peripheral request, wait until the exact
    /// channel is inactive, then return a stable transfer count. RX needs
    /// an acquire fence before its destination is visible to the CPU.
    fn quiesce(&mut self) -> usize {
        cortex_m::asm::dsb();
        self.port.registers().set_rx_dma(false);
        self.port.registers().disarm_dma_transfer_interrupts();
        let complete = self.port.channel().quiesce();
        fence(Ordering::Acquire);
        let moved = if complete {
            self.total
        } else {
            self.port.channel().transferred_bytes()
        };
        moved
    }

    /// Finish the handoff before status classification or releasing the
    /// caller buffer. Dropping an unfinished lease has identical cleanup.
    pub(in crate::i2c) fn finish(mut self) -> usize {
        let moved = self.quiesce();
        self.armed = false;
        moved
    }
}

impl Drop for TargetRxDmaLease<'_, '_, '_> {
    fn drop(&mut self) {
        if self.armed {
            let _ = self.quiesce();
        }
    }
}

/// A live target TX-DMA handoff to the write-only STDR FIFO port. See
/// [`TargetRxDmaLease`] for why this is a lease instead of a raw pointer.
#[must_use]
pub(in crate::i2c) struct TargetTxDmaLease<'channel, 'dma, 'buf> {
    port: TargetTxDma<'channel, 'dma>,
    total: usize,
    armed: bool,
    _buffer: PhantomData<&'buf [u8]>,
}

impl TargetTxDmaLease<'_, '_, '_> {
    /// Register for this exact channel's completion; see
    /// [`TargetRxDmaLease::poll_complete`].
    fn poll_complete(&self, cx: &mut core::task::Context<'_>) -> bool {
        while self.port.channel().wait_cell().poll_wait(cx).is_ready() {}
        self.port.channel().is_done()
    }

    /// See [`TargetRxDmaLease::poll_wake`].
    pub(in crate::i2c) fn poll_wake(&self, cx: &mut core::task::Context<'_>) -> bool {
        while self.port.wait_cell().poll_wait(cx).is_ready() {}
        self.port.registers().dma_transfer_wake() || self.poll_complete(cx)
    }

    fn quiesce(&mut self) -> usize {
        cortex_m::asm::dsb();
        self.port.registers().set_tx_dma(false);
        self.port.registers().disarm_dma_transfer_interrupts();
        let complete = self.port.channel().quiesce();
        if complete {
            self.total
        } else {
            self.port.channel().transferred_bytes()
        }
    }

    /// Finish the handoff before status classification or releasing the
    /// source buffer. Dropping an unfinished lease has identical cleanup.
    pub(in crate::i2c) fn finish(mut self) -> usize {
        let moved = self.quiesce();
        self.armed = false;
        moved
    }
}

impl Drop for TargetTxDmaLease<'_, '_, '_> {
    fn drop(&mut self) {
        if self.armed {
            let _ = self.quiesce();
        }
    }
}

/// Match the eDMA setup APIs' buffer precondition before an arm method
/// mutates channel state. A rejected arm is completely side-effect free.
const fn dma_buffer_fits(length: usize) -> bool {
    length != 0 && length <= DMA_MAX_TRANSFER_SIZE
}

const _: () = {
    assert!(!dma_buffer_fits(0));
    assert!(dma_buffer_fits(1));
    assert!(dma_buffer_fits(DMA_MAX_TRANSFER_SIZE));
    assert!(!dma_buffer_fits(DMA_MAX_TRANSFER_SIZE.saturating_add(1)));
};

impl TargetRegisters {
    pub(in crate::i2c) fn new(regs: pac::lpi2c::Lpi2c) -> Self {
        Self {
            regs: lpi2c_regs::from_pac(regs),
        }
    }

    /// Cross-check the hidden raw layout against the linked PAC before
    /// a target is configured. Driver code only receives this facade,
    /// never the cells themselves.
    pub(in crate::i2c) fn check_layout(regs: pac::lpi2c::Lpi2c) {
        lpi2c_regs::check_layout(regs);
    }

    fn register_address(&self) -> usize {
        self.regs as *const LpI2cRegisters as usize
    }

    /// Stable identity used to bind a target's fixed DMA channel/request
    /// pair to this exact LPI2C register block.
    pub(in crate::i2c) fn identity(&self) -> usize {
        self.register_address()
    }

    fn ssr(&self) -> Ssr {
        Ssr(self.regs.ssr.get())
    }

    fn write_sier(&self, f: impl FnOnce(&mut Sier)) {
        let mut v = Sier(0);
        f(&mut v);
        self.regs.sier.set(v.0);
    }

    fn modify_sder(&self, f: impl FnOnce(&mut Sder)) {
        let mut v = Sder(self.regs.sder.get());
        f(&mut v);
        self.regs.sder.set(v.0);
    }

    fn modify_scr(&self, f: impl FnOnce(&mut Scr)) {
        let mut v = Scr(self.regs.scr.get());
        f(&mut v);
        self.regs.scr.set(v.0);
    }

    /// Disable the target interrupt mask if any source is enabled.
    ///
    /// Returns whether the driver should wake its waiter.
    pub(in crate::i2c) fn disable_interrupts_if_enabled(&self) -> bool {
        if self.regs.sier.get() == 0 {
            return false;
        }

        self.regs.sier.set(0);
        true
    }

    /// Interrupts relevant while receiving data (respond_to_write).
    pub(in crate::i2c) fn enable_receive_interrupts(&self) {
        self.write_sier(|w| {
            w.set_feie(true);
            w.set_beie(true);
            w.set_sdie(true);
            w.set_rsie(true);
            w.set_rdie(true);
        });
    }

    /// Interrupts relevant while transmitting data (respond_to_read).
    pub(in crate::i2c) fn enable_transmit_interrupts(&self) {
        self.write_sier(|w| {
            w.set_feie(true);
            w.set_beie(true);
            w.set_sdie(true);
            w.set_rsie(true);
            w.set_tdie(true);
        });
    }

    /// Arm the receive interrupt set and evaluate its wake condition,
    /// as ONE operation: the armed set and the predicate are defined
    /// together so they cannot drift apart (an armed source outside
    /// the wake set re-arms and interrupts forever — the listen-side
    /// RSIE mismatch was exactly this class).
    pub(in crate::i2c) fn rx_wake(&self) -> bool {
        self.enable_receive_interrupts();
        self.rx_ready()
    }

    /// Arm the transmit interrupt set and evaluate its wake condition,
    /// as one operation — see [`Self::rx_wake`].
    pub(in crate::i2c) fn tx_wake(&self) -> bool {
        self.enable_transmit_interrupts();
        self.tx_ready()
    }

    /// Enable or disable the RX DMA request path (SDER[RDDE]). This stays
    /// private: only an RX DMA lease may pair it with channel ownership.
    fn set_rx_dma(&self, enable: bool) {
        self.modify_sder(|w| w.set_rdde(enable));
    }

    /// Enable or disable the TX DMA request path (SDER[TDDE]). See
    /// [`Self::set_rx_dma`].
    fn set_tx_dma(&self, enable: bool) {
        self.modify_sder(|w| w.set_tdde(enable));
    }

    /// Stop the DMA transfer interrupt sources before the matching channel
    /// is quiesced. A cancelled response must not leave SIER armed to wake
    /// a future waiter after its lease has gone away.
    fn disarm_dma_transfer_interrupts(&self) {
        self.regs.sier.set(0);
    }

    /// Configure and arm one controller-to-target DMA transfer. The
    /// read-only FIFO endpoint, channel/request pairing, and inverse
    /// teardown are all retained by the returned lease.
    pub(in crate::i2c) fn arm_rx_dma<'channel, 'dma, 'buf>(
        &self,
        port: TargetRxDma<'channel, 'dma>,
        buffer: &'buf mut [u8],
    ) -> Result<TargetRxDmaLease<'channel, 'dma, 'buf>, InvalidParameters> {
        assert!(
            self.register_address() == port.owner(),
            "i2c: an RX DMA port was used through a different target"
        );
        if !dma_buffer_fits(buffer.len()) {
            return Err(InvalidParameters);
        }
        let total = buffer.len();
        let channel = port.channel();
        let request = port.request();

        // SAFETY: `port` proves the fixed RX channel/request pairing for
        // this target; SRDR is the Tock-typed read-only FIFO port; and the
        // returned lease retains `buffer` until SDER is off and eDMA is
        // quiesced.
        unsafe {
            channel.disable_request();
            channel.clear_done();
            channel.clear_interrupt();
            channel.set_request_source(request);
            channel.setup_read_from_peripheral(
                &self.regs.srdr as *const _ as *const u8,
                buffer,
                false,
                TransferOptions::COMPLETE_INTERRUPT,
            )?;
            self.set_rx_dma(true);
            channel.enable_request();
        }

        Ok(TargetRxDmaLease {
            port,
            total,
            armed: true,
            _buffer: PhantomData,
        })
    }

    /// Configure and arm one target-to-controller DMA transfer. See
    /// [`Self::arm_rx_dma`] for the ownership/cleanup contract.
    pub(in crate::i2c) fn arm_tx_dma<'channel, 'dma, 'buf>(
        &self,
        port: TargetTxDma<'channel, 'dma>,
        buffer: &'buf [u8],
    ) -> Result<TargetTxDmaLease<'channel, 'dma, 'buf>, InvalidParameters> {
        assert!(
            self.register_address() == port.owner(),
            "i2c: a TX DMA port was used through a different target"
        );
        if !dma_buffer_fits(buffer.len()) {
            return Err(InvalidParameters);
        }
        let total = buffer.len();
        let channel = port.channel();
        let request = port.request();

        // SAFETY: `port` proves the fixed TX channel/request pairing for
        // this target; STDR is the Tock-typed write-only FIFO port; and the
        // returned lease retains `buffer` until SDER is off and eDMA is
        // quiesced.
        unsafe {
            channel.disable_request();
            channel.clear_done();
            channel.clear_interrupt();
            channel.set_request_source(request);
            channel.setup_write_to_peripheral(
                buffer,
                &self.regs.stdr as *const _ as *mut u8,
                false,
                TransferOptions::COMPLETE_INTERRUPT,
            )?;
            fence(Ordering::Release);
            self.set_tx_dma(true);
            channel.enable_request();
        }

        Ok(TargetTxDmaLease {
            port,
            total,
            armed: true,
            _buffer: PhantomData,
        })
    }

    /// Interrupts for listening for a new transaction. Deliberately no
    /// RSIE: the armed set must equal [`Self::listen_ready`]'s wake
    /// set, or a source that fires without satisfying the predicate
    /// re-arms and interrupts forever. A repeated START is not itself
    /// a listen event — the address phase that follows it raises AVF
    /// (armed), which is what classification keys on; an RSF-only wake
    /// would classify against a stale SASR. The general-call and
    /// SMBus-alert sources are driver configuration, passed in.
    pub(in crate::i2c) fn enable_listen_interrupts(&self, general_call: bool, smbus_alert: bool) {
        self.write_sier(|w| {
            w.set_sarie(smbus_alert);
            w.set_gcie(general_call);
            w.set_am1ie(true);
            w.set_am0ie(true);
            w.set_feie(true);
            w.set_beie(true);
            w.set_sdie(true);
            w.set_avie(true);
        });
    }

    /// Unconditionally clear the W1C event flags. ONLY for the entry
    /// to `listen`, where everything still latched is by definition
    /// stale (the previous transaction's lifecycle is complete).
    /// Anywhere a snapshot is being classified,
    /// [`Self::take_listen_event`]'s same-snapshot clear is the API —
    /// there is deliberately no generic clear.
    pub(in crate::i2c) fn clear_stale_events(&self) {
        let mut v = Ssr(0);
        v.set_rsf(true);
        v.set_sdf(true);
        v.set_bef(true);
        v.set_fef(true);
        self.regs.ssr.set(v.0);
    }

    /// Take one classified listen event: reads ONE status snapshot,
    /// clears only the W1C flags that snapshot observed (a constant-
    /// mask clear would erase a flag latching between read and write,
    /// unseen), and — for address-class events — consumes SASR, which
    /// releases an ADRSTALL stretch. The whole protocol action, in the
    /// single priority order: faults, then the address family (GCF and
    /// SARF are classification tags on address-valid), then transmit-
    /// ACK, repeated START, STOP.
    pub(in crate::i2c) fn take_listen_event(&self) -> ListenEvent {
        let ssr = self.ssr();
        let mut w = Ssr(0);
        w.set_rsf(ssr.rsf());
        w.set_sdf(ssr.sdf());
        w.set_bef(ssr.bef());
        w.set_fef(ssr.fef());
        self.regs.ssr.set(w.0);

        if ssr.bef() {
            ListenEvent::Fault(TargetFault::Bit)
        } else if ssr.fef() {
            ListenEvent::Fault(TargetFault::Fifo)
        } else if ssr.avf() || ssr.gcf() || ssr.sarf() {
            // Read SASR to consume the address-valid state regardless
            // of which classification tag triggered the match.
            let addr = Sasr(self.regs.sasr.get()).raddr();
            if ssr.gcf() {
                ListenEvent::GeneralCall
            } else if ssr.sarf() {
                ListenEvent::SmbusAlert
            } else {
                ListenEvent::AddressValid(addr)
            }
        } else if ssr.taf() {
            ListenEvent::TransmitAck
        } else if ssr.rsf() {
            ListenEvent::RepeatedStart(Sasr(self.regs.sasr.get()).raddr())
        } else if ssr.sdf() {
            ListenEvent::Stop(Sasr(self.regs.sasr.get()).raddr())
        } else {
            ListenEvent::None
        }
    }

    pub(in crate::i2c) fn reset_fifos(&self) {
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
    pub(in crate::i2c) fn rx_event(&self) -> Option<TargetRxEvent> {
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
    pub(in crate::i2c) fn rx_ready(&self) -> bool {
        let ssr = self.ssr();
        ssr.rdf() || ssr.sdf() || ssr.rsf() || ssr.bef() || ssr.fef()
    }

    /// One step of transmit progress: faults first (the transfer is
    /// compromised), then termination, then room.
    pub(in crate::i2c) fn tx_step(&self) -> Option<TargetTxStep> {
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
    pub(in crate::i2c) fn tx_ready(&self) -> bool {
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
    pub(in crate::i2c) fn listen_ready(&self) -> bool {
        let ssr = self.ssr();
        ssr.avf() || ssr.sarf() || ssr.gcf() || ssr.sdf() || ssr.bef() || ssr.fef()
    }

    /// Push one byte into the target transmit register.
    pub(in crate::i2c) fn push_tx(&self, byte: u8) {
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

    /// Arm the DMA-transfer interrupt set (fault and termination
    /// sources only — RDF/TDF service the DMA engine, not firmware)
    /// and evaluate its wake condition, as ONE operation, like the
    /// other `*_wake` pairs. The DMA-completion disjunct is the
    /// channel's own wait cell and is OR'd in by the caller — it is
    /// not an SIER source and cannot drift from this set.
    pub(in crate::i2c) fn dma_transfer_wake(&self) -> bool {
        self.enable_dma_transfer_interrupts();
        self.dma_transfer_event()
    }

    /// Interrupts for a DMA-driven transfer — see
    /// [`Self::dma_transfer_wake`], the only intended caller.
    fn enable_dma_transfer_interrupts(&self) {
        self.write_sier(|w| {
            w.set_feie(true);
            w.set_beie(true);
            w.set_sdie(true);
            w.set_rsie(true);
        });
    }

    /// Wake condition paired with the arm set above — see
    /// [`Self::dma_transfer_wake`], the only intended caller.
    fn dma_transfer_event(&self) -> bool {
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
    pub(in crate::i2c) fn drain_rx(&self, data: &mut [u8], mut from: usize) -> usize {
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

    /// Terminal classification of an RX DMA chunk from ONE status
    /// snapshot, in the vocabulary's single flag-priority order: data
    /// residue with no room left outranks termination (the caller must
    /// defer, or the residue is marooned into the next transaction),
    /// after faults, before STOP/repeated-START. Consumes nothing.
    pub(in crate::i2c) fn rx_chunk_end(&self, chunk_full: bool) -> RxChunkEnd {
        let ssr = self.ssr();
        if ssr.bef() {
            RxChunkEnd::Fault(TargetFault::Bit)
        } else if ssr.fef() {
            RxChunkEnd::Fault(TargetFault::Fifo)
        } else if ssr.rdf() && chunk_full {
            RxChunkEnd::ResiduePending
        } else if ssr.sdf() {
            RxChunkEnd::Stopped
        } else if ssr.rsf() {
            RxChunkEnd::Restarted
        } else {
            RxChunkEnd::Continue
        }
    }

    /// Terminal classification of a TX DMA chunk, in the vocabulary's
    /// single flag-priority order: bit error before FIFO error
    /// (matching [`Self::rx_event`]/[`Self::tx_step`]), faults before
    /// termination. Consumes nothing.
    pub(in crate::i2c) fn chunk_end(&self) -> ChunkEnd {
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

/// Terminal classification of an RX DMA chunk — see
/// [`TargetRegisters::rx_chunk_end`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[must_use]
pub(in crate::i2c) enum RxChunkEnd {
    /// A fault; the transfer is compromised.
    Fault(TargetFault),
    /// Data is still pending with no room left in the chunk — defer
    /// (report the chunk full); a latched termination stays latched
    /// for the follow-up.
    ResiduePending,
    /// The controller issued a STOP.
    Stopped,
    /// The controller issued a repeated START.
    Restarted,
    /// No fault or termination latched.
    Continue,
}

/// Terminal classification of a TX DMA chunk transfer — see
/// [`TargetRegisters::chunk_end`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[must_use]
pub(in crate::i2c) enum ChunkEnd {
    /// A fault; the transfer is compromised.
    Fault(TargetFault),
    /// The controller issued a STOP.
    Stopped,
    /// The controller issued a repeated START.
    Restarted,
    /// No fault or termination latched.
    Continue,
}
