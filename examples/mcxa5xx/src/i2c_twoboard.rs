//! Shared harness for the i2c-twoboard-* examples: a two-board I2C test
//! between separate MCXA577s wired to the same LPI2C3 bus.
//!
//! Wiring (board A ↔ board B): P3_20 ↔ P3_20 (SDA), P3_21 ↔ P3_21 (SCL),
//! GND ↔ GND. The bus needs pull-ups to 3V3 on both lines.
//!
//! The target exposes a 40-byte RAM buffer at address 0x2A behind a
//! **persistent read cursor**, modelling a real device (EEPROM, FIFO,
//! sensor block) rather than a stateless one: each byte served advances
//! the cursor, the cursor survives STOP and re-addressing, and a write
//! stores to the front of the buffer and rewinds it. The controller
//! keeps an exact shadow of the buffer — not of the cursor, which it
//! cannot track (see below); reads are anchored instead — and checks
//! every byte it reads back, so data integrity is verified end to end
//! against real written payloads instead of a constant fill.
//!
//! The cursor is what makes non-atomic reads detectable. A read that is
//! silently retried after a partial transfer resumes from an advanced
//! cursor while the caller's buffer restarts at zero, so the payload
//! comes back shifted — with a stateless target that mismatch is
//! invisible, which is exactly how an earlier revision of this suite
//! passed a driver that could corrupt data on real devices.
//!
//! The controller does not try to track the cursor across transactions:
//! it cannot. The target driver's `ReadStatus` count is (per its own
//! docs) bytes *queued* into the transmit register, not bytes the
//! controller took, and the difference — up to one stranded byte per
//! terminated transfer — is not recoverable: this IP exposes no target
//! FIFO status register, and the measured same-snapshot TDF correction
//! is raceable past the next address phase. So every verified read is
//! instead **anchored**: `op_read` rewrites the
//! buffer first, which rewinds the target's cursor to a known zero.
//! Detection is unaffected, because a silent retry happens *inside* the
//! read being verified, after the anchor.
//!
//! Long-transfer coverage: reads of 40/80 (exact view multiples) and
//! 255/256/257/260/300/512 bytes cross
//! the LPI2C's 256-byte RECEIVE-command boundary. The driver chains
//! adjacent RECEIVE commands under a single address phase (the
//! controller ACKs across a command boundary only when the next command
//! is already queued), which is exactly the mechanism with a documented
//! silent-data-loss hazard for >256-byte reads — so these tests verify
//! byte-for-byte, against the shadow model, that nothing is lost or
//! duplicated across the command seams, in the interrupt-async, DMA,
//! and blocking paths separately.
//!
//! Interrupt-latency stress: `interference::task` periodically blocks all
//! interrupts for ~0.5 ms. The controller binary gates it around
//! `t_isr_latency`; the target binary runs a lighter version constantly,
//! so every test also exercises target-side ACK/stall handling under
//! delayed ISR entry (the target relies on RXSTALL/TXDSTALL stretching
//! whenever firmware is late).
//!
//! The suite starts with a sync write that resets the target buffer to a
//! known state, so it is deterministic across re-runs and across phases
//! without resetting the target board.
//!
//! Chunked reads are left at their default (disabled), so a chained read
//! that the silicon terminates early surfaces as an error instead of a
//! silent re-read. `t_over_capacity` covers reads past the DMA chaining
//! ceiling (which must be refused, not split) and `t_chunked_optin`
//! covers the opt-in path, including one such read.
//!
//! Retry policy: `ArbitrationLoss` is retried up to 15 times per operation
//! and counted — the FRDM-MCXA577 LPI2C flags a spurious arbitration loss
//! on roughly every other read whose first data byte has the MSB set (the
//! target releasing SDA for a 1-bit reads as a foreign STOP). The suite
//! reports the total retry count and fails if any operation exhausts its
//! retries; every other error stays fatal, and every byte read is still
//! verified against the model.

use embassy_embedded_hal::SetConfig;
use embassy_mcxa as hal;
use embassy_time::Instant;
use hal::Peri;
use hal::i2c::Blocking;
use hal::i2c::controller::{
    Config as CtrlConfig, I2c as ControllerI2c, IOError as ControllerIOError, SetupError, Speed,
};
use hal::i2c::target::{self, Address, Config as TargetConfig, InterruptHandler, ReadStatus, Request, WriteStatus};
use hal::interrupt::typelevel::Binding;
use hal::peripherals::{DMA0_CH0, DMA0_CH1, LPI2C3, P3_20, P3_21};

pub const TARGET_ADDR: u8 = 0x2A;

/// Target buffer size.
///
/// Deliberately **not** a divisor of 256. With a 32-byte buffer a
/// re-addressed 256-byte seam lands exactly back at buffer offset 0, so
/// a chunked read and an atomic one produce identical bytes and the
/// seam is undetectable by construction. At 40 bytes a seam lands at
/// offset 16, so any implementation that restarts rather than continues
/// shows up as mismatched data.
pub const BUF_LEN: usize = 40;

const FILL: u8 = 0x55;
/// An address nothing on the bus answers to.
const BAD_ADDR: u8 = 0x33;

/// A read longer than the DMA path can chain.
///
/// The DMA path can only queue as many RECEIVE commands as the transmit
/// FIFO holds (4 on this part → 1024 bytes), because nothing refills it
/// while the CPU sleeps on the DMA completion. This length is past that
/// ceiling, so it exercises the branch that must refuse rather than
/// silently split — untested while every read stopped at 512.
const OVER_CAPACITY: usize = 1100;

/// Longest read the tests perform.
const MAX_READ: usize = OVER_CAPACITY;

/// Control message: a write of exactly [`CTRL_LEN`] bytes whose first
/// four match this magic is a command, not data. The last byte selects:
/// 0 = persistent-cursor mode, 1 = stateless-read mode, 2 = arm the
/// one-shot overflow probe (serve the next write with a
/// [`OVERFLOW_PROBE_LEN`]-byte buffer — see `t_overflow_write`). The
/// magic is unreachable by every generated test pattern: those are all
/// arithmetic sequences xor a constant, and these bytes xor any
/// constant are not consecutive.
const CTRL_MAGIC: [u8; 4] = [0x5C, 0xC5, 0x3A, 0xA3];
const CTRL_LEN: usize = 5;
const CTRL_STATELESS_OFF: u8 = 0;
const CTRL_STATELESS_ON: u8 = 1;
const CTRL_ARM_OVERFLOW_PROBE: u8 = 2;
/// Serve the audit counters on the next Read request (WITHOUT
/// resetting them — see [`CTRL_RESET_STATS`]).
const CTRL_SERVE_STATS: u8 = 3;
/// Reset the audit counters, the stats latch, and every test mode
/// (stateless read, overflow probe). Separate from the
/// serve so a stats read that fails controller-side (the audit read is
/// retried) cannot destroy the evidence it was fetching.
const CTRL_RESET_STATS: u8 = 4;

/// Stats view layout: echo marker, then the premature-NeedMore counter
/// (LE u32). Marker constraints, all load-bearing: first byte MSB
/// CLEAR (an MSB-set first data byte trips the ~every-other-read
/// spurious-ALF quirk — on the audit read itself); not the forward
/// magic prefix (a partially committed control write can leave that in
/// the buffer); adjacent-byte XORs not of the form 2^k - 1 (every
/// generated test pattern is an additive sequence xor a constant, so
/// its adjacent XORs are exactly that form — this marker is therefore
/// unreachable as committed data).
const STATS_LEN: usize = 8;
const STATS_ECHO: [u8; 4] = [0x3C, 0x5A, 0xC3, 0xA5];

/// Buffer size the overflow probe serves the next write with — small
/// enough that a probe-length-plus-one write overflows it.
const OVERFLOW_PROBE_LEN: usize = 8;

/// Per-phase driver capabilities the tests key their expectations on.
/// Typed and passed from the binary, where the engine type is concrete
/// — never derived from a display string.
#[derive(Clone, Copy)]
pub struct PhaseCaps {
    /// The engine's atomic-read ceiling in bytes, if it has one. The
    /// DMA engine cannot refill the command FIFO while the CPU sleeps
    /// on the DMA completion, so reads past `tx_fifo_capacity * 256`
    /// are refused (`ChunkingRequired`) rather than chained. `None`:
    /// the engine chains without a length limit and must never refuse.
    pub dma_chain_ceiling: Option<usize>,
}

/// Interrupt-latency interference: a task that periodically blocks all
/// interrupts, delaying ISR entry to model a busy system.
pub mod interference {
    use core::sync::atomic::{AtomicBool, Ordering};

    use embassy_time::Timer;

    /// Gate for the interference task. When false the task idles.
    pub static ACTIVE: AtomicBool = AtomicBool::new(false);

    /// Spawn at executor startup. Every `period_us` microseconds, if
    /// [`ACTIVE`] is set, holds a critical section for `spin` busy-wait
    /// units (~3 CPU cycles each), blocking every interrupt handler for
    /// the duration.
    #[embassy_executor::task]
    pub async fn task(spin: u32, period_us: u64) {
        loop {
            Timer::after_micros(period_us).await;
            if ACTIVE.load(Ordering::Relaxed) {
                critical_section::with(|_| cortex_m::asm::delay(spin));
            }
        }
    }
}

/// Target task: a [`BUF_LEN`]-byte RAM buffer behind a persistent read cursor,
/// served over I2C at [`TARGET_ADDR`].
///
/// Reads serve `buf[cursor..]` wrapping, advancing the cursor by the
/// driver-reported queued count, and the cursor **survives STOP and
/// re-addressing** — so a re-addressed read continues where the previous
/// one stopped, exactly like a device with an auto-incrementing pointer.
/// Writes commit in `BUF_LEN`-byte chunks to the front of the buffer and rewind
/// the cursor to 0.
///
/// A [`CTRL_MAGIC`] control write switches the target into
/// **stateless-read mode**: every read transaction serves from the
/// buffer start, modelling the stateless-read device class that chunked
/// reads are actually intended for. `t_chunked_optin` uses this to make
/// its expectations fully deterministic and byte-exact.
pub async fn target_task(
    peri: Peri<'static, LPI2C3>,
    scl: Peri<'static, P3_21>,
    sda: Peri<'static, P3_20>,
    irq: impl Binding<<LPI2C3 as hal::i2c::Instance>::Interrupt, InterruptHandler<LPI2C3>> + 'static,
) -> ! {
    let mut config = TargetConfig::default();
    config.address = Address::Single(TARGET_ADDR as u16);

    let mut tgt = target::I2c::new_async(peri, scl, sda, irq, config).unwrap();
    serve(&mut tgt).await
}

/// [`target_task`], but constructed over the DMA target driver, so the
/// suite exercises the target's DMA respond paths — the half of the
/// closed vocabulary the interrupt-mode target never touches (and where
/// the terminated-transfer RDF drain lives).
pub async fn target_task_dma(
    peri: Peri<'static, LPI2C3>,
    scl: Peri<'static, P3_21>,
    sda: Peri<'static, P3_20>,
    tx_dma: Peri<'static, DMA0_CH0>,
    rx_dma: Peri<'static, DMA0_CH1>,
    irq: impl Binding<<LPI2C3 as hal::i2c::Instance>::Interrupt, InterruptHandler<LPI2C3>> + 'static,
) -> ! {
    let mut config = TargetConfig::default();
    config.address = Address::Single(TARGET_ADDR as u16);

    let mut tgt = target::I2c::new_async_with_dma(peri, scl, sda, tx_dma, rx_dma, irq, config).unwrap();
    serve(&mut tgt).await
}

/// Abstraction over the two async target constructions, so one serve
/// loop runs against both. The driver's mode-generic bounds are
/// private, hence a harness-side trait over the concrete types.
#[allow(async_fn_in_trait)]
pub trait TargetPort {
    async fn listen(&mut self) -> Result<Request, target::IOError>;
    async fn respond_read(&mut self, buf: &[u8]) -> Result<ReadStatus, target::IOError>;
    async fn respond_write(&mut self, buf: &mut [u8]) -> Result<WriteStatus, target::IOError>;
}

impl TargetPort for target::I2c<'static, hal::i2c::Async> {
    async fn listen(&mut self) -> Result<Request, target::IOError> {
        self.async_listen().await
    }
    async fn respond_read(&mut self, buf: &[u8]) -> Result<ReadStatus, target::IOError> {
        self.async_respond_to_read(buf).await
    }
    async fn respond_write(&mut self, buf: &mut [u8]) -> Result<WriteStatus, target::IOError> {
        self.async_respond_to_write(buf).await
    }
}

impl TargetPort for target::I2c<'static, hal::i2c::Dma<'static>> {
    async fn listen(&mut self) -> Result<Request, target::IOError> {
        self.async_listen().await
    }
    async fn respond_read(&mut self, buf: &[u8]) -> Result<ReadStatus, target::IOError> {
        self.async_respond_to_read(buf).await
    }
    async fn respond_write(&mut self, buf: &mut [u8]) -> Result<WriteStatus, target::IOError> {
        self.async_respond_to_write(buf).await
    }
}

/// The emulated device's serve loop — see [`target_task`] for the
/// behavioral contract (cursor, writes, stateless mode).
async fn serve<T: TargetPort>(tgt: &mut T) -> ! {
    let mut buf = [FILL; BUF_LEN];
    // Persistent read cursor: survives STOP and re-addressing.
    let mut cursor = 0usize;
    // Stateless-read mode (see the task docs): toggled by the control
    // write, off by default.
    let mut stateless = false;
    // One-shot overflow probe (see `t_overflow_write`): armed by the
    // control write, consumed by the next Write request.
    let mut overflow_probe = false;
    // Settle-audit counter (see `t_settle_audit`): incremented on
    // every NeedMore whose follow-up terminates with zero bytes;
    // served and reset by the one-shot stats mode.
    let mut premature_needmore: u32 = 0;
    let mut stats_pending = false;

    loop {
        let request = tgt.listen().await.unwrap();
        defmt::trace!("[T] event {}", request);
        match request {
            Request::Write(_) if overflow_probe => {
                overflow_probe = false;
                // Serve this write with a deliberately small buffer so
                // the controller's probe-length-plus-one write
                // overflows it. On the DMA target the first respond
                // must take the full-chunk-with-RDF-pending branch
                // (`BufferFull` with the termination still latched)
                // and the follow-up respond must take the terminal
                // drain branch — the two residue paths the normal
                // oversized scratch can never reach; the interrupt
                // target follows the same contract via `rx_event`.
                // The concatenated stream commits like a normal
                // write, so the controller's read-back verifies the
                // residue byte end to end.
                let mut small = [0u8; OVERFLOW_PROBE_LEN + 8];
                // Latency spike, joined with the first respond: block
                // all interrupts for ~600us starting ~600us in — right
                // when the 8th byte's DMA completion fires (~765us
                // after the address phase released). Without this the
                // maroon branches are unreachable in practice:
                // ADRSTALL phase-locks every transaction to the
                // periodic interference grid (the address phase
                // stretches THROUGH a blackout, so the data phase runs
                // in the clear — measured 0/48 branch hits), and an
                // un-delayed poll classifies the DMA completion before
                // the ninth byte and STOP even arrive. With the spike,
                // the completion, the residue byte, and the
                // termination are all latched by the time firmware
                // looks — the residue-deferral branch, deterministic.
                // The eDMA and the bus need no CPU service meanwhile.
                let spike = async {
                    embassy_time::Timer::after_micros(600).await;
                    critical_section::with(|_| cortex_m::asm::delay(20_000));
                };
                let (s1, ()) =
                    embassy_futures::join::join(tgt.respond_write(&mut small[..OVERFLOW_PROBE_LEN]), spike).await;
                let mut status = s1.unwrap();
                let mut total = match status {
                    WriteStatus::Stopped(n) | WriteStatus::Restarted(n) | WriteStatus::BufferFull(n) => n,
                    _ => 0,
                }
                .min(OVERFLOW_PROBE_LEN);
                let first_full = matches!(status, WriteStatus::BufferFull(_));
                // Service to a clean termination NO MATTER how much
                // arrives. With raw whole-sequence retries the data
                // write is always 9 bytes, so anything longer reaching
                // an armed probe would be a harness bug — but
                // abandoning a still-open transaction would wedge the
                // bus under RXSTALL, so this stays robust to any
                // length: collect what fits, discard the rest.
                while matches!(status, WriteStatus::BufferFull(_)) {
                    if total < small.len() {
                        status = tgt.respond_write(&mut small[total..]).await.unwrap();
                        let n = match status {
                            WriteStatus::Stopped(n) | WriteStatus::Restarted(n) | WriteStatus::BufferFull(n) => n,
                            _ => 0,
                        };
                        total = (total + n).min(small.len());
                    } else {
                        let mut waste = [0u8; 64];
                        status = tgt.respond_write(&mut waste).await.unwrap();
                    }
                }
                if first_full {
                    defmt::info!(
                        "[T] overflow probe: BufferFull({}) then {} more",
                        OVERFLOW_PROBE_LEN,
                        total - OVERFLOW_PROBE_LEN.min(total)
                    );
                } else {
                    defmt::info!("[T] overflow probe: no overflow ({})", total);
                }
                // Commands are commands on EVERY path: a retried
                // control write can land on the armed probe (e.g. the
                // arm reported a bus error to the controller but
                // completed on the wire, and the retry arrives here).
                // Handling it — instead of committing magic bytes as
                // data — keeps the arm idempotent.
                if total == CTRL_LEN && small[..4] == CTRL_MAGIC {
                    match small[4] {
                        CTRL_ARM_OVERFLOW_PROBE => {
                            overflow_probe = true;
                            defmt::info!("[T] overflow probe re-armed");
                        }
                        CTRL_SERVE_STATS => {
                            stats_pending = true;
                        }
                        CTRL_RESET_STATS => {
                            // Clears ALL audit and test-mode state:
                            // an interrupted run must not leave a
                            // stale counter, a pending stats latch (it
                            // would serve a stats payload to the next
                            // data read), a stateless-read mode, or an
                            // armed overflow probe
                            // behind for the next run's phases.
                            premature_needmore = 0;
                            stats_pending = false;
                            stateless = false;
                            overflow_probe = false;
                        }
                        m => {
                            stateless = m != CTRL_STATELESS_OFF;
                            defmt::info!("[T] stateless-read mode: {}", stateless);
                        }
                    }
                    continue;
                }
                for chunk in small[..total].chunks(BUF_LEN) {
                    buf[..chunk.len()].copy_from_slice(chunk);
                }
                cursor = 0;
            }
            Request::Read(_) if stats_pending => {
                // One-shot: serve the audit counters instead of the
                // buffer. Deliberately NOT resetting here — the
                // controller-side read can fail after this serve (the
                // ALF quirk, an early termination), and a
                // reset-on-serve would destroy the evidence before
                // the auditor saw it; the auditor acknowledges with
                // [`CTRL_RESET_STATS`] once the count is safely read.
                // Cursor and buffer are untouched.
                stats_pending = false;
                let mut view = [0u8; STATS_LEN];
                view[..4].copy_from_slice(&STATS_ECHO);
                view[4..8].copy_from_slice(&premature_needmore.to_le_bytes());
                let _ = tgt.respond_read(&view).await.unwrap();
            }
            Request::Read(_) => {
                if stateless {
                    // Every read transaction starts at the front; the
                    // cursor still advances *within* the transaction
                    // (a NeedMore continuation is the same transfer).
                    cursor = 0;
                }
                let mut after_needmore = false;
                loop {
                    // Serve the buffer rotated to the cursor, then advance by
                    // however many bytes the driver reports as queued.
                    let mut view = [0u8; BUF_LEN];
                    for (i, b) in view.iter_mut().enumerate() {
                        *b = buf[(cursor + i) % BUF_LEN];
                    }
                    defmt::trace!("[T] R serve @{} {:02x}", cursor, view[..2]);
                    let status = tgt.respond_read(&view).await.unwrap();
                    let (n, more) = match status {
                        ReadStatus::NeedMore(n) => (n, true),
                        ReadStatus::Complete(n) | ReadStatus::EarlyStop(n) => (n, false),
                        _ => (0, false),
                    };
                    // Discriminator for the TX-settle contract: NeedMore
                    // claims the controller wants more, so a follow-up
                    // that immediately reports termination with ZERO
                    // bytes means the NeedMore was premature (the
                    // pre-settle DMA defect fired this on every read
                    // ending exactly at a view boundary — the suite's
                    // exact-multiple lengths force that case). The one
                    // legitimate producer is the razor TDF-then-NACK
                    // race, so a correct driver logs ~zero of these.
                    if after_needmore && n == 0 && !more {
                        premature_needmore += 1;
                        defmt::info!("[T] premature NeedMore (0-byte follow-up)");
                    }
                    after_needmore = more;
                    // NOTE: `ReadStatus` documents this count as bytes
                    // *queued*, and explicitly warns against using it to
                    // advance a device-side position — at a terminated
                    // transfer it overshoots by the discarded FIFO residue.
                    // This emulated device does it anyway because there is
                    // no better source; verified reads are anchored, and
                    // the chunked test runs in stateless-read mode, where
                    // the per-transaction rewind makes the residue moot.
                    cursor = (cursor + n) % BUF_LEN;
                    if !more {
                        break;
                    }
                }
            }
            Request::Write(_) => {
                // One respond call per Write event, with headroom above
                // the longest transfer the tests perform. `BufferFull` on
                // an exact-multiple-length write is ambiguous ("full" vs
                // "full and stopped"), and calling respond_to_write again
                // would clear the pending STOP flag and misattribute the
                // *next* transaction's bytes to this one.
                let mut scratch = [0u8; MAX_READ + 1];
                let n = match tgt.respond_write(&mut scratch).await.unwrap() {
                    WriteStatus::Stopped(n) | WriteStatus::Restarted(n) | WriteStatus::BufferFull(n) => n,
                    _ => 0,
                };
                let n = n.min(scratch.len());
                // Control message: toggle stateless-read mode; not
                // data, so no commit and no cursor rewind. (A control
                // write that terminated early misses the length/magic
                // match and is committed as data — harmless: on the
                // controller side that attempt errored, which retries
                // only on arbitration loss and otherwise fails the
                // test loudly, and the partial bytes are overwritten
                // by the next anchored write before anything is
                // verified.)
                if n == CTRL_LEN && scratch[..4] == CTRL_MAGIC {
                    match scratch[4] {
                        CTRL_ARM_OVERFLOW_PROBE => {
                            overflow_probe = true;
                            defmt::info!("[T] overflow probe armed");
                        }
                        CTRL_SERVE_STATS => {
                            stats_pending = true;
                        }
                        CTRL_RESET_STATS => {
                            // Clears ALL audit and test-mode state:
                            // an interrupted run must not leave a
                            // stale counter, a pending stats latch (it
                            // would serve a stats payload to the next
                            // data read), a stateless-read mode, or an
                            // armed overflow probe
                            // behind for the next run's phases.
                            premature_needmore = 0;
                            stats_pending = false;
                            stateless = false;
                            overflow_probe = false;
                        }
                        m => {
                            stateless = m != CTRL_STATELESS_OFF;
                            defmt::info!("[T] stateless-read mode: {}", stateless);
                        }
                    }
                    continue;
                }
                // Commit in `BUF_LEN`-byte chunks, mirroring `Model::write`,
                // and rewind the read cursor like a device whose address
                // pointer is set by the write.
                for chunk in scratch[..n].chunks(BUF_LEN) {
                    buf[..chunk.len()].copy_from_slice(chunk);
                }
                cursor = 0;
                defmt::trace!("[T] W {} -> {:02x}", n, buf[..2]);
            }
            _ => {}
        }
    }
}

/// Trait abstracting an async I2C controller for the test suite.
///
/// Both `I2c<'_, Async>` and `I2c<'_, Dma<'_>>` implement
/// `embedded_hal_async::i2c::I2c` and `SetConfig`, so the same suite runs
/// against the interrupt-driven and the DMA controller. The blocking
/// controller runs a parallel battery via [`harness::run_blocking`].
pub trait Controller:
    embedded_hal_async::i2c::I2c<Error = ControllerIOError> + SetConfig<Config = CtrlConfig, ConfigError = SetupError>
{
}
impl<
    T: embedded_hal_async::i2c::I2c<Error = ControllerIOError> + SetConfig<Config = CtrlConfig, ConfigError = SetupError>,
> Controller for T
{
}

/// Exact shadow of the target's RAM buffer.
///
/// Deliberately does *not* mirror the target's read cursor — see the
/// module docs: reads are anchored instead, so the cursor is zero
/// whenever a verified read begins.
pub struct Model {
    buf: [u8; BUF_LEN],
}

impl Model {
    fn new() -> Self {
        Self { buf: [FILL; BUF_LEN] }
    }

    /// Mirror of the target's write handling: data is committed in
    /// `BUF_LEN`-byte chunks, each overwriting the front of the buffer, so a
    /// long write leaves the last chunk (plus any surviving tail of the
    /// chunk before it) in place. The write rewinds the read cursor.
    fn write(&mut self, data: &[u8]) {
        for chunk in data.chunks(BUF_LEN) {
            self.buf[..chunk.len()].copy_from_slice(chunk);
        }
    }

    /// Verify a chunked read served by the target in **stateless-read
    /// mode**: fully deterministic and byte-exact, no tolerance.
    ///
    /// The stateless target serves every bus transaction from the
    /// buffer start, so each 256-byte window of the payload has exactly
    /// two legal shapes, distinguished by its first byte (buffer values
    /// are distinct): it either **continues** the previous position
    /// exactly — the same transaction, a chained atomic read — or
    /// **restarts** at position zero exactly — a new re-addressed
    /// chunk. The transmit-register residue that forces a slip
    /// tolerance against the cursor target is moot here (the restart
    /// discards it deterministically), and a fallback engaged after a
    /// partially-consumed chained attempt also restarts at zero — so
    /// first-window placement is verified exactly too. The first window
    /// must sit at position zero unconditionally.
    ///
    /// The payload's **shape is uniform**: a correct driver either
    /// served the whole read as one chained transaction (every window
    /// continues) or as re-addressed chunks (every window restarts —
    /// the seamed path and its fallback always re-read the entire
    /// buffer from chunk zero). Window 1 decides which; the rest must
    /// match. Accepting a per-window mix would reopen mod-`BUF_LEN`
    /// aliasing: dropping one whole 256-byte chunk at the final
    /// chained seam of an 1100-byte read shifts the next continuation
    /// position to 1280 ≡ 0 (mod 40), byte-identical to a restart.
    /// (The 512-byte analogue — a seam defect whose payload exactly
    /// equals a legal all-restart read — is indistinguishable by any
    /// payload check on a stateless device and is covered by the
    /// cursor-mode atomic tests instead.)
    ///
    /// `require_restart` narrows the accepted shapes to all-restart
    /// alone: a read past the engine's chaining ceiling can only have
    /// been served by the split path, so a continuation-shaped payload
    /// there means the driver chained past its declared limit.
    fn check_chunked_stateless(&self, read: &[u8], require_restart: bool) -> bool {
        // Shape, decided by window 1: true = continuation (atomic
        // chained), false = restart-per-window (seamed chunks).
        let mut continuation = true;
        let mut pos = 0usize;
        for (ci, chunk) in read.chunks(256).enumerate() {
            let start = if ci == 0 {
                // First window: exactly at the front, unconditionally.
                0
            } else if ci == 1 {
                // `buf[16]` vs `buf[0]` — distinct values, unambiguous.
                continuation = chunk[0] == self.buf[pos % BUF_LEN];
                if continuation && require_restart {
                    defmt::error!("chunked: payload is one chained transaction, but this length must be split");
                    return false;
                }
                if continuation {
                    pos
                } else if chunk[0] == self.buf[0] {
                    0
                } else {
                    defmt::error!(
                        "chunked: window 1 misplaced (got {:02x}, want {:02x} cont or {:02x} restart)",
                        chunk[0],
                        self.buf[pos % BUF_LEN],
                        self.buf[0]
                    );
                    return false;
                }
            } else if continuation {
                pos
            } else {
                0
            };
            for (i, b) in chunk.iter().enumerate() {
                let expect = self.buf[(start + i) % BUF_LEN];
                if *b != expect {
                    defmt::error!(
                        "chunked: break at {} (got {:02x}, want {:02x})",
                        ci * 256 + i,
                        *b,
                        expect
                    );
                    return false;
                }
            }
            pos = start + chunk.len();
        }
        true
    }

    /// Current buffer contents, for re-establishing a known state after
    /// a failed transfer (see `resync`).
    fn snapshot(&self) -> [u8; BUF_LEN] {
        self.buf
    }

    /// Verify a completed read, which always begins at the anchored
    /// cursor position zero.
    ///
    /// Position `i` must equal `buf[i % BUF_LEN]`. A read that the
    /// driver silently restarted after a partial transfer resumes from
    /// the target's advanced cursor and therefore lands shifted, which
    /// fails here — the property this whole arrangement exists to test.
    fn check_read(&mut self, read: &[u8]) -> bool {
        read.iter().enumerate().all(|(i, b)| *b == self.buf[i % BUF_LEN])
    }
}

pub mod harness {
    use super::*;

    /// Run the full suite through `ctrl` against the remote target board.
    ///
    /// `mode` labels the log lines (e.g. "async", "dma") — display
    /// only; test expectations come from the typed `caps`, which the
    /// binary fills in where the engine type is concrete. Logs
    /// `[mode] <test> PASS (<ms>)` per test and panics on the first
    /// failure, so a failing run exits through the panic handler.
    pub async fn run<C: Controller>(mode: &str, ctrl: &mut C, caps: PhaseCaps) {
        defmt::info!("== two-board i2c suite [{=str}] start ==", mode);

        let mut model = Model::new();
        let mut stats = tests::RetryStats::default();

        // Scrub audit and test-mode state possibly left by an
        // interrupted previous run BEFORE the sync write: a stale
        // armed probe would otherwise partially swallow the sync
        // payload (the probe path handles control writes as commands,
        // so this scrub gets through regardless of stale state).
        if let Err(e) = tests::t_audit_reset(ctrl, &mut stats).await {
            defmt::error!("[{=str}] audit reset failed: {=str}", mode, e);
            panic!("audit reset failed");
        }

        // Reset the target buffer to a known state so the model is exact
        // even if the target kept state from a previous run or phase.
        if ctrl.write(TARGET_ADDR, &model.buf).await.is_err() {
            defmt::error!("[{=str}] sync write failed — is the target board up?", mode);
            panic!("target sync failed");
        }

        macro_rules! run_test {
            ($name:literal, $fut:expr) => {{
                let t0 = Instant::now();
                match $fut.await {
                    Ok(()) => {
                        defmt::info!(
                            "[{=str}] {=str} PASS ({=u64} ms)",
                            mode,
                            $name,
                            t0.elapsed().as_millis()
                        );
                    }
                    Err(e) => {
                        defmt::error!("[{=str}] {=str} FAIL: {=str}", mode, $name, e);
                        panic!("test failure");
                    }
                }
            }};
        }

        run_test!("basic_rw", tests::t_basic_rw(ctrl, &mut model, &mut stats));
        run_test!("lengths", tests::t_lengths(ctrl, &mut model, &mut stats));
        run_test!("burst", tests::t_burst(ctrl, &mut model, &mut stats));
        run_test!("edges", tests::t_edges(ctrl, &mut model, &mut stats));
        run_test!("speed_sweep", tests::t_speed_sweep(ctrl, &mut model, &mut stats));
        run_test!("long_transfers", tests::t_long_transfers(ctrl, &mut model, &mut stats));
        run_test!("isr_latency", tests::t_isr_latency(ctrl, &mut model, &mut stats));
        run_test!(
            "over_capacity",
            tests::t_over_capacity(ctrl, &mut model, &mut stats, caps)
        );
        run_test!("overflow_write", tests::t_overflow_write(ctrl, &mut model, &mut stats));
        run_test!(
            "chunked_optin",
            tests::t_chunked_optin(ctrl, &mut model, &mut stats, caps)
        );
        run_test!("soak", tests::t_soak(ctrl, &mut model, &mut stats));
        run_test!("settle_audit", tests::t_settle_audit(ctrl, &mut stats));
        // AFTER the settle audit: the cancellation probe ends by
        // scrubbing the audit counters (its aborts inflate them by
        // design), and running it earlier would wipe the evidence the
        // phase-end audit exists to judge.
        run_test!("cancellation", tests::t_cancellation(ctrl, &mut model, &mut stats));
        run_test!("data_nack", tests::t_data_nack(ctrl, &mut model, &mut stats));

        defmt::info!(
            "== two-board i2c suite [{=str}] ALL PASS ({=u32} ALF / {=u32} FEF / {=u32} END retries) ==",
            mode,
            stats.alf_retries,
            stats.fef_retries,
            stats.end_retries
        );
    }

    /// Blocking-path battery: the polled driver has no interrupt path, so
    /// this focuses on transfer correctness — basic traffic, every long
    /// length, consecutive reads, repeated-START, and NACK recovery.
    /// Panics on failure, like [`run`].
    pub fn run_blocking(mode: &str, ctrl: &mut ControllerI2c<'_, Blocking>) {
        defmt::info!("== two-board i2c suite [{=str}] start ==", mode);

        let mut model = Model::new();
        let mut stats = tests::RetryStats::default();

        // Scrub before the sync write — see `run`.
        if let Err(e) = tests::b_audit_reset(ctrl, &mut stats) {
            defmt::error!("[{=str}] audit reset failed: {=str}", mode, e);
            panic!("audit reset failed");
        }

        if ctrl.blocking_write(TARGET_ADDR, &model.buf).is_err() {
            defmt::error!("[{=str}] sync write failed — is the target board up?", mode);
            panic!("target sync failed");
        }

        let t0 = Instant::now();
        match tests::t_blocking_battery(ctrl, &mut model, &mut stats) {
            Ok(()) => defmt::info!("[{=str}] battery PASS ({=u64} ms)", mode, t0.elapsed().as_millis()),
            Err(e) => {
                defmt::error!("[{=str}] battery FAIL: {=str}", mode, e);
                panic!("test failure");
            }
        }

        match tests::b_settle_audit(ctrl, &mut stats) {
            Ok(()) => {}
            Err(e) => {
                defmt::error!("[{=str}] settle_audit FAIL: {=str}", mode, e);
                panic!("test failure");
            }
        }

        defmt::info!(
            "== two-board i2c suite [{=str}] ALL PASS ({=u32} ALF / {=u32} FEF / {=u32} END retries) ==",
            mode,
            stats.alf_retries,
            stats.fef_retries,
            stats.end_retries
        );
    }
}

pub mod tests {
    use core::sync::atomic::Ordering;

    use super::*;

    type TestResult = Result<(), &'static str>;

    /// Read lengths that straddle the driver's 256-byte RECEIVE-command
    /// chunk boundary, where >256-byte reads are documented to risk
    /// silent data loss if the controller chains receive commands.
    // 40 and 80 are exact multiples of the target's BUF_LEN: the read
    // terminates exactly at a served-view boundary, which is the case
    // that discriminates a settled NeedMore/Complete decision from a
    // premature one (see the serve loop's premature-NeedMore log).
    const LONG_LENGTHS: &[usize] = &[40, 80, 255, 256, 257, 260, 300, 512];

    /// Spurious-error retry accounting for one suite run.
    #[derive(Default)]
    pub struct RetryStats {
        /// Total operations retried after an `ArbitrationLoss`.
        pub alf_retries: u32,
        /// Total operations retried after a latched `FifoError` (blocking
        /// path: a transient FEF left by the auto-NACK ending a fully
        /// consumed RECEIVE can trip the next transfer's START check; the
        /// reporting status read also clears it, so a retry runs clean).
        pub fef_retries: u32,
        /// Total operations retried after an `UnexpectedStop` (a chained
        /// read terminated early with SDF/EPF latched and no fault —
        /// same spurious-flag silicon family as the ALF quirk) or a
        /// `Timeout` (no progress for a full window; the same
        /// silent-termination family caught by the bounded waits).
        pub end_retries: u32,
    }

    /// Retries per operation before giving up on `ArbitrationLoss`.
    const MAX_RETRIES: u32 = 15;

    /// Re-establish a known state on both sides after a failed transfer.
    ///
    /// A transfer that died part-way may have clocked out an unknown
    /// number of bytes, advancing the target's read cursor without the
    /// model seeing it — and a partial write may have stored unknown
    /// data. Rewriting the model's buffer restores both: the contents,
    /// and (because a write rewinds the cursor) the position. Writes do
    /// not exhibit the spurious-ALF quirk, so this is reliable.
    async fn resync<C: Controller>(ctrl: &mut C, model: &mut Model) -> TestResult {
        let snap = model.snapshot();
        for _ in 0..=MAX_RETRIES {
            if ctrl.write(TARGET_ADDR, &snap).await.is_ok() {
                model.write(&snap);
                return Ok(());
            }
        }
        Err("resync failed")
    }

    async fn op_write<C: Controller>(
        ctrl: &mut C,
        data: &[u8],
        model: &mut Model,
        stats: &mut RetryStats,
    ) -> TestResult {
        for _ in 0..=MAX_RETRIES {
            match ctrl.write(TARGET_ADDR, data).await {
                Ok(()) => return Ok(()),
                Err(ControllerIOError::ArbitrationLoss) => {
                    stats.alf_retries += 1;
                    resync(ctrl, model).await?;
                }
                Err(_) => return Err("write failed"),
            }
        }
        Err("write: retries exhausted")
    }

    async fn op_read<C: Controller>(
        ctrl: &mut C,
        buf: &mut [u8],
        model: &mut Model,
        stats: &mut RetryStats,
    ) -> TestResult {
        // Anchor: rewind the target's read cursor to zero so the read
        // below has a known origin (see the module docs).
        resync(ctrl, model).await?;
        for _ in 0..=MAX_RETRIES {
            match ctrl.read(TARGET_ADDR, buf).await {
                Ok(()) => return Ok(()),
                Err(ControllerIOError::ArbitrationLoss) => {
                    stats.alf_retries += 1;
                    resync(ctrl, model).await?;
                }
                Err(ControllerIOError::UnexpectedStop) | Err(ControllerIOError::Timeout) => {
                    stats.end_retries += 1;
                    resync(ctrl, model).await?;
                }
                Err(_) => return Err("read failed"),
            }
        }
        Err("read: retries exhausted")
    }

    async fn op_write_read<C: Controller>(
        ctrl: &mut C,
        w: &[u8],
        r: &mut [u8],
        model: &mut Model,
        stats: &mut RetryStats,
    ) -> TestResult {
        for _ in 0..=MAX_RETRIES {
            match ctrl.write_read(TARGET_ADDR, w, r).await {
                Ok(()) => return Ok(()),
                Err(ControllerIOError::ArbitrationLoss) => {
                    stats.alf_retries += 1;
                    resync(ctrl, model).await?;
                }
                Err(ControllerIOError::UnexpectedStop) | Err(ControllerIOError::Timeout) => {
                    stats.end_retries += 1;
                    resync(ctrl, model).await?;
                }
                Err(_) => return Err("write_read failed"),
            }
        }
        Err("write_read: retries exhausted")
    }

    /// 100 iters of {write 2 bytes, read 2 bytes} with exact payload check.
    pub async fn t_basic_rw<C: Controller>(ctrl: &mut C, model: &mut Model, stats: &mut RetryStats) -> TestResult {
        for i in 0..100u16 {
            let w = [i as u8, (i >> 8) as u8];
            op_write(ctrl, &w, model, stats).await?;
            model.write(&w);

            let mut r = [0u8; 2];
            op_read(ctrl, &mut r, model, stats).await?;
            if !model.check_read(&r) {
                defmt::error!("iter {}: read mismatch got={:02x} want={:02x}", i, r, model.buf[..2]);
                // Diagnostic: re-read twice to distinguish a corrupted
                // target buffer (stable wrong data) from a transient
                // read corruption (subsequent reads correct).
                let mut r2 = [0u8; 4];
                let _ = ctrl.read(TARGET_ADDR, &mut r2).await;
                defmt::error!("  re-read 1: {:02x}", r2);
                let _ = ctrl.read(TARGET_ADDR, &mut r2).await;
                defmt::error!("  re-read 2: {:02x}", r2);
                return Err("read mismatch");
            }
        }
        Ok(())
    }

    /// Transfer lengths {1,2,4,8,16,32}: write L distinct bytes, read L
    /// back, then write_read with an (L/2)-byte write and L-byte read.
    pub async fn t_lengths<C: Controller>(ctrl: &mut C, model: &mut Model, stats: &mut RetryStats) -> TestResult {
        const LENGTHS: &[usize] = &[1, 2, 4, 8, 16, 32];
        let mut rbuf = [0u8; BUF_LEN];

        for (round, &l) in LENGTHS.iter().enumerate() {
            let mut payload = [0u8; BUF_LEN];
            for (i, b) in payload.iter_mut().enumerate() {
                *b = (round as u8) << 5 | i as u8;
            }

            op_write(ctrl, &payload[..l], model, stats).await?;
            model.write(&payload[..l]);

            let r = &mut rbuf[..l];
            op_read(ctrl, r, model, stats).await?;
            if !model.check_read(r) {
                defmt::error!("L={}: read mismatch got={:02x}", l, r);
                return Err("read mismatch");
            }

            let wlen = core::cmp::max(1, l / 2);
            let w = &payload[..wlen];
            let r = &mut rbuf[..l];
            op_write_read(ctrl, w, r, model, stats).await?;
            model.write(w);
            if !model.check_read(r) {
                defmt::error!("L={} wr({},{}): mismatch got={:02x}", l, wlen, l, r);
                return Err("wr mismatch");
            }
        }
        Ok(())
    }

    /// Back-to-back stress: 500 iters of {W2, R2, WR(1,2)} with exact
    /// payload check on every read.
    pub async fn t_burst<C: Controller>(ctrl: &mut C, model: &mut Model, stats: &mut RetryStats) -> TestResult {
        const N: u16 = 500;
        for i in 0..N {
            let w = [i as u8, (i >> 8) as u8];
            op_write(ctrl, &w, model, stats).await?;
            model.write(&w);

            let mut r = [0u8; 2];
            op_read(ctrl, &mut r, model, stats).await?;
            if !model.check_read(&r) {
                defmt::error!("burst i={}: read mismatch got={:02x}", i, r);
                return Err("read mismatch");
            }

            let w = [!(i as u8)];
            op_write_read(ctrl, &w, &mut r, model, stats).await?;
            model.write(&w);
            if !model.check_read(&r) {
                defmt::error!("burst i={}: wr mismatch got={:02x}", i, r);
                return Err("wr mismatch");
            }
        }
        Ok(())
    }

    /// Error-path checks against a live bus:
    /// E1 write to an unoccupied address must NACK;
    /// E2 read from an unoccupied address must NACK;
    /// E3 the target's buffer must be untouched by the failed transfers;
    /// E4 normal traffic must work immediately after the NACKs.
    ///
    /// The bad-address probes are deliberate raw calls (no retry): any
    /// error is the expected outcome there.
    pub async fn t_edges<C: Controller>(ctrl: &mut C, model: &mut Model, stats: &mut RetryStats) -> TestResult {
        // Exact class, not merely "some error": an address NACK is the
        // one deterministic outcome here (no data phase ever starts,
        // so the spurious read quirks cannot fire), and accepting any
        // error would let a misclassified fault pose as the NACK.
        match ctrl.write(BAD_ADDR, &[0x00]).await {
            Err(ControllerIOError::AddressNack) => {}
            Ok(()) => return Err("E1: expected NACK on bad-addr write"),
            Err(_) => return Err("E1: wrong error class for bad-addr write"),
        }

        let mut r = [0u8; 2];
        match ctrl.read(BAD_ADDR, &mut r).await {
            Err(ControllerIOError::AddressNack) => {}
            Ok(()) => return Err("E2: expected NACK on bad-addr read"),
            Err(_) => return Err("E2: wrong error class for bad-addr read"),
        }

        let mut full = [0u8; BUF_LEN];
        op_read(ctrl, &mut full, model, stats)
            .await
            .map_err(|_| "E3 read failed")?;
        if !model.check_read(&full) {
            defmt::error!("E3: target buffer changed after NACKs got={:02x}", full);
            return Err("E3 mismatch");
        }

        let w = [0xAB, 0xCD];
        op_write(ctrl, &w, model, stats).await.map_err(|_| "E4 write failed")?;
        model.write(&w);
        op_read(ctrl, &mut r, model, stats)
            .await
            .map_err(|_| "E4 read failed")?;
        if !model.check_read(&r) {
            defmt::error!("E4: got={:02x}", r);
            return Err("E4 mismatch");
        }

        Ok(())
    }

    /// t_basic_rw-style traffic at Standard / Fast / FastPlus. Restores
    /// Standard before returning so later tests are unaffected.
    pub async fn t_speed_sweep<C: Controller>(ctrl: &mut C, model: &mut Model, stats: &mut RetryStats) -> TestResult {
        for &speed in &[Speed::Standard, Speed::Fast, Speed::FastPlus] {
            let mut cfg = CtrlConfig::default();
            cfg.speed = speed;
            ctrl.set_config(&cfg).map_err(|_| "set_config failed")?;

            let t0 = Instant::now();
            for i in 0..50u16 {
                let w = [i as u8, (i >> 8) as u8];
                op_write(ctrl, &w, model, stats).await?;
                model.write(&w);

                let mut r = [0u8; 2];
                op_read(ctrl, &mut r, model, stats).await?;
                if !model.check_read(&r) {
                    defmt::error!("speed {} iter {}: got={:02x}", speed, i, r);
                    return Err("speed: mismatch");
                }

                let w = [!(i as u8)];
                op_write_read(ctrl, &w, &mut r, model, stats).await?;
                model.write(&w);
                if !model.check_read(&r) {
                    defmt::error!("speed {} wr i={}: got={:02x}", speed, i, r);
                    return Err("speed: wr mismatch");
                }
            }
            defmt::info!("  speed {}: 50 iters in {=u64} ms", speed, t0.elapsed().as_millis());
        }

        let cfg = CtrlConfig::default();
        ctrl.set_config(&cfg).map_err(|_| "set_config restore failed")?;
        Ok(())
    }

    /// Long transfers across the 256-byte RECEIVE chunk boundary, with
    /// every byte checked against the cyclic model:
    /// - reads of 40/80/255/256/257/260/300/512 bytes;
    /// - consecutive long reads separated by STOP;
    /// - repeated-START (write_read) into a 300-byte read;
    /// - wrong-address NACK immediately followed by a 257-byte read;
    /// - a 512-byte write streamed in one transaction, read back.
    pub async fn t_long_transfers<C: Controller>(
        ctrl: &mut C,
        model: &mut Model,
        stats: &mut RetryStats,
    ) -> TestResult {
        // MSB-rich cyclic pattern so the spurious-ALF path stays stressed.
        let mut pat = [0u8; BUF_LEN];
        for (i, b) in pat.iter_mut().enumerate() {
            *b = 0x80 | (i as u8).wrapping_mul(7);
        }
        op_write(ctrl, &pat, model, stats).await?;
        model.write(&pat);

        let mut big = [0u8; MAX_READ];
        for &l in LONG_LENGTHS {
            let r = &mut big[..l];
            op_read(ctrl, r, model, stats).await?;
            if !model.check_read(r) {
                defmt::error!("long L={}: mismatch head={:02x}", l, r[..8]);
                return Err("long read mismatch");
            }
        }

        // Consecutive long reads, each anchored by `op_read` (so this
        // exercises back-to-back long transactions rather than a shared
        // cursor walk).
        op_read(ctrl, &mut big[..257], model, stats).await?;
        if !model.check_read(&big[..257]) {
            return Err("consecutive read 1 mismatch");
        }
        op_read(ctrl, &mut big[..31], model, stats).await?;
        if !model.check_read(&big[..31]) {
            return Err("consecutive read 2 mismatch");
        }

        // Repeated START into a long read. The 4-byte write re-sends the
        // pattern's own prefix, so the model is unchanged by design.
        let w = [pat[0], pat[1], pat[2], pat[3]];
        op_write_read(ctrl, &w, &mut big[..300], model, stats).await?;
        model.write(&w);
        if !model.check_read(&big[..300]) {
            return Err("wr long read mismatch");
        }

        // Error recovery into a long read.
        if ctrl.write(BAD_ADDR, &[0x00]).await.is_ok() {
            return Err("expected NACK before long read");
        }
        op_read(ctrl, &mut big[..257], model, stats).await?;
        if !model.check_read(&big[..257]) {
            return Err("post-NACK long read mismatch");
        }

        // Long write: 512 bytes streamed in one transaction; the target
        // commits `BUF_LEN`-byte chunks, so the buffer ends as the final chunk.
        let mut w512 = [0u8; 512];
        for (i, b) in w512.iter_mut().enumerate() {
            *b = (i as u8) ^ 0x5A;
        }
        op_write(ctrl, &w512, model, stats).await?;
        model.write(&w512);
        op_read(ctrl, &mut big[..BUF_LEN], model, stats).await?;
        if !model.check_read(&big[..BUF_LEN]) {
            defmt::error!("long write readback: got={:02x}", big[..BUF_LEN]);
            return Err("long write mismatch");
        }

        Ok(())
    }

    /// Re-runs short and long traffic while `interference::task` blocks
    /// all interrupts ~0.5 ms at a time, delaying LPI2C and DMA ISR entry
    /// on the controller. (The target board runs its own interference
    /// constantly.) Requires the controller binary to have spawned the
    /// task; without it this is a plain re-run, which is harmless.
    pub async fn t_isr_latency<C: Controller>(ctrl: &mut C, model: &mut Model, stats: &mut RetryStats) -> TestResult {
        interference::ACTIVE.store(true, Ordering::Relaxed);
        let res = isr_latency_inner(ctrl, model, stats).await;
        interference::ACTIVE.store(false, Ordering::Relaxed);
        res
    }

    async fn isr_latency_inner<C: Controller>(ctrl: &mut C, model: &mut Model, stats: &mut RetryStats) -> TestResult {
        for i in 0..20u16 {
            let w = [i as u8, 0xE0 | (i as u8 & 0x0F)];
            op_write(ctrl, &w, model, stats).await?;
            model.write(&w);
            let mut r = [0u8; 2];
            op_read(ctrl, &mut r, model, stats).await?;
            if !model.check_read(&r) {
                defmt::error!("isr i={}: got={:02x}", i, r);
                return Err("isr: mismatch");
            }
        }

        // Long chained reads under sustained interrupt-blocking are the
        // one case this silicon can terminate early (see
        // `IOError::UnexpectedStop`). With chunked reads disabled — the
        // default — the contract is *not* that they always succeed, but
        // that they either succeed with correct data or fail cleanly.
        // Silently returning shifted data is the failure this asserts
        // against.
        let mut big = [0u8; MAX_READ];
        for &l in &[257usize, 512] {
            // The spurious-ALF quirk is retryable and unrelated; an
            // early termination is the outcome under test.
            for attempt in 0..=MAX_RETRIES {
                resync(ctrl, model).await?;
                match ctrl.read(TARGET_ADDR, &mut big[..l]).await {
                    Ok(()) => {
                        if !model.check_read(&big[..l]) {
                            defmt::error!("isr long {}: mismatch head={:02x}", l, big[..8]);
                            return Err("isr: long read mismatch");
                        }
                        break;
                    }
                    Err(ControllerIOError::UnexpectedStop) | Err(ControllerIOError::Timeout) => {
                        stats.end_retries += 1;
                        defmt::info!("  isr long {}: terminated early (reported, not hidden)", l);
                        break;
                    }
                    Err(ControllerIOError::ArbitrationLoss) => {
                        stats.alf_retries += 1;
                        if attempt == MAX_RETRIES {
                            return Err("isr: long read ALF retries exhausted");
                        }
                    }
                    Err(e) => {
                        defmt::error!("isr long {}: unexpected error {}", l, e);
                        return Err("isr: long read failed");
                    }
                }
            }
        }

        // Same contract as the long reads above: the repeated-START read
        // is 300 bytes, so it can also terminate early under sustained
        // interference. Correct-or-clean-failure, never silent shift.
        let w = [model.buf[0], model.buf[1]];
        for attempt in 0..=MAX_RETRIES {
            match ctrl.write_read(TARGET_ADDR, &w, &mut big[..300]).await {
                Ok(()) => {
                    model.write(&w);
                    if !model.check_read(&big[..300]) {
                        return Err("isr: wr mismatch");
                    }
                    break;
                }
                Err(ControllerIOError::UnexpectedStop) | Err(ControllerIOError::Timeout) => {
                    stats.end_retries += 1;
                    defmt::info!("  isr wr 300: terminated early (reported, not hidden)");
                    break;
                }
                Err(ControllerIOError::ArbitrationLoss) => {
                    stats.alf_retries += 1;
                    resync(ctrl, model).await?;
                    if attempt == MAX_RETRIES {
                        return Err("isr: wr ALF retries exhausted");
                    }
                }
                Err(e) => {
                    defmt::error!("isr wr 300: unexpected error {}", e);
                    return Err("isr: wr failed");
                }
            }
        }

        Ok(())
    }

    /// A read longer than the DMA path can chain must either complete
    /// atomically (the interrupt-driven and blocking paths refill the
    /// command FIFO as it drains, so length is not a limit for them) or
    /// be refused with `ChunkingRequired`. What it must never do is
    /// silently split into re-addressed chunks, which is what the DMA
    /// path did before chunking became opt-in.
    ///
    /// `caps` carries the phase's expectation, and the two outcomes are
    /// mutually exclusive: an engine with a chaining ceiling MUST
    /// refuse (the length is asserted to exceed it, and completing
    /// would mean it chained past its own declared limit); an engine
    /// without one MUST complete atomically, byte-correct.
    pub async fn t_over_capacity<C: Controller>(
        ctrl: &mut C,
        model: &mut Model,
        stats: &mut RetryStats,
        caps: PhaseCaps,
    ) -> TestResult {
        let must_refuse = match caps.dma_chain_ceiling {
            Some(ceiling) => {
                defmt::assert!(
                    OVER_CAPACITY > ceiling,
                    "over-capacity length must exceed the engine's chaining ceiling"
                );
                true
            }
            None => false,
        };
        let mut pat = [0u8; BUF_LEN];
        for (i, b) in pat.iter_mut().enumerate() {
            *b = 0x80 | (i as u8).wrapping_mul(7);
        }
        op_write(ctrl, &pat, model, stats).await?;
        model.write(&pat);

        let mut big = [0u8; MAX_READ];
        for attempt in 0..=MAX_RETRIES {
            resync(ctrl, model).await?;
            match ctrl.read(TARGET_ADDR, &mut big[..OVER_CAPACITY]).await {
                Ok(()) => {
                    if must_refuse {
                        defmt::error!("over-capacity: completed, but this engine's ceiling requires a refusal");
                        return Err("over_capacity: missing refusal");
                    }
                    if !model.check_read(&big[..OVER_CAPACITY]) {
                        defmt::error!("over-capacity: mismatch head={:02x}", big[..8]);
                        return Err("over_capacity: mismatch");
                    }
                    defmt::info!("  over-capacity {}: chained atomically", OVER_CAPACITY);
                    break;
                }
                Err(ControllerIOError::ChunkingRequired) => {
                    if !must_refuse {
                        defmt::error!("over-capacity: refused, but this engine chains without a capacity limit");
                        return Err("over_capacity: unexpected refusal");
                    }
                    defmt::info!("  over-capacity {}: refused (ChunkingRequired)", OVER_CAPACITY);
                    break;
                }
                // The silicon quirks are orthogonal to what this checks.
                Err(ControllerIOError::ArbitrationLoss) => {
                    stats.alf_retries += 1;
                    if attempt == MAX_RETRIES {
                        return Err("over_capacity: ALF retries exhausted");
                    }
                }
                // Early termination is a clean, reported failure of one
                // attempt (the always-on target interference makes the
                // occasional one legitimate) — but it must not be
                // accepted as the test's outcome: a chained path that
                // *always* dies early would otherwise be
                // indistinguishable from one that works. Retry like the
                // ALF arm; a persistent failure exhausts the budget and
                // fails the test.
                Err(ControllerIOError::UnexpectedStop) | Err(ControllerIOError::Timeout) => {
                    stats.end_retries += 1;
                    if attempt == MAX_RETRIES {
                        return Err("over_capacity: early-termination retries exhausted");
                    }
                }
                Err(e) => {
                    defmt::error!("over-capacity: unexpected error {}", e);
                    return Err("over_capacity: unexpected error");
                }
            }
        }

        Ok(())
    }

    /// Toggle the target's stateless-read mode via the control write.
    /// Not committed to the model — the target intercepts it as a
    /// command, not data.
    async fn set_stateless<C: Controller>(
        ctrl: &mut C,
        model: &mut Model,
        stats: &mut RetryStats,
        on: bool,
    ) -> TestResult {
        let mode = if on { CTRL_STATELESS_ON } else { CTRL_STATELESS_OFF };
        let msg = [CTRL_MAGIC[0], CTRL_MAGIC[1], CTRL_MAGIC[2], CTRL_MAGIC[3], mode];
        op_write(ctrl, &msg, model, stats).await
    }

    /// How many times the overflow probe runs per phase — the residue
    /// branches it aims at are timing-dependent (see below), so the
    /// probe repeats to give them many chances per run.
    const OVERFLOW_PROBE_REPS: usize = 24;

    /// Overflow probe: arm the target to serve the next write with a
    /// deliberately small buffer, then write one byte more than it
    /// holds, and read back WITHOUT re-anchoring — the read-back is
    /// the proof, and an anchor resync would repair a dropped ninth
    /// byte before validation could see it (each retry restarts the
    /// whole arm/write/read sequence instead).
    ///
    /// What this proves deterministically: the `BufferFull` follow-up
    /// contract end to end, on both target modes — no byte of the
    /// overflowing write is lost or misattributed, whichever internal
    /// path served it. What it exercises statistically: the DMA
    /// target's residue branches. Whether the ninth byte is marooned
    /// (its request rescinded or never granted with the termination
    /// already latched) or simply collected by the follow-up
    /// respond's fresh DMA depends on executor latency relative to
    /// the last two byte times, so the probe repeats
    /// [`OVERFLOW_PROBE_REPS`] times per phase under the target's
    /// interference blackouts, and the driver logs each residue
    /// branch at debug level — branch counts are MEASURED from the
    /// target's RTT and reported, not asserted, because they depend
    /// on bus timing; the correctness contract above is what must
    /// always hold.
    pub async fn t_overflow_write<C: Controller>(
        ctrl: &mut C,
        model: &mut Model,
        stats: &mut RetryStats,
    ) -> TestResult {
        let arm = [
            CTRL_MAGIC[0],
            CTRL_MAGIC[1],
            CTRL_MAGIC[2],
            CTRL_MAGIC[3],
            CTRL_ARM_OVERFLOW_PROBE,
        ];
        let mut w = [0u8; OVERFLOW_PROBE_LEN + 1];
        // Monotone across every attempt of every rep: CONSECUTIVE
        // payloads always differ at every index, so a dropped ninth
        // byte can never be masked by the previously committed value.
        // (Deriving the base from rep and attempt separately collides
        // — e.g. rep*0x21 ^ attempt*0x0B repeats 0x81 — which is
        // exactly the masking this exists to prevent.)
        let mut seq: u8 = 0;

        'reps: for _rep in 0..OVERFLOW_PROBE_REPS {
            for attempt in 0..=MAX_RETRIES {
                seq = seq.wrapping_add(1);
                let base = 0xA0 ^ seq;
                for (i, b) in w.iter_mut().enumerate() {
                    *b = base ^ i as u8;
                }

                // RAW writes throughout: `op_write` retries internally
                // (with a resync write), which can consume the one-shot
                // arm and route the data write through the ordinary
                // oversized handler without the test noticing. Here any
                // failure restarts the WHOLE arm/write/read sequence,
                // and a retried arm landing on an already-armed probe
                // re-arms it (the probe path intercepts the magic).
                if let Err(e) = ctrl.write(TARGET_ADDR, &arm).await {
                    match e {
                        ControllerIOError::ArbitrationLoss => stats.alf_retries += 1,
                        ControllerIOError::UnexpectedStop | ControllerIOError::Timeout => stats.end_retries += 1,
                        _ => {
                            defmt::error!("overflow write: arm failed {}", e);
                            return Err("overflow_write: arm failed");
                        }
                    }
                    if attempt == MAX_RETRIES {
                        return Err("overflow_write: retries exhausted");
                    }
                    continue;
                }
                // The overflowing write; commits and rewinds cursor.
                if let Err(e) = ctrl.write(TARGET_ADDR, &w).await {
                    match e {
                        ControllerIOError::ArbitrationLoss => stats.alf_retries += 1,
                        ControllerIOError::UnexpectedStop | ControllerIOError::Timeout => stats.end_retries += 1,
                        _ => {
                            defmt::error!("overflow write: write failed {}", e);
                            return Err("overflow_write: write failed");
                        }
                    }
                    if attempt == MAX_RETRIES {
                        return Err("overflow_write: retries exhausted");
                    }
                    continue;
                }
                model.write(&w);

                // UNANCHORED read-back: the write above rewound the
                // cursor, and nothing may rewrite the buffer between
                // write and verify. A dropped ninth byte fails here.
                let mut r = [0u8; OVERFLOW_PROBE_LEN + 1];
                match ctrl.read(TARGET_ADDR, &mut r).await {
                    Ok(()) => {
                        if !model.check_read(&r) {
                            defmt::error!("overflow write: read-back mismatch got={:02x}", r);
                            return Err("overflow_write: mismatch");
                        }
                        continue 'reps;
                    }
                    // The failed read advanced the cursor by however
                    // many bytes it clocked, so an unanchored re-read
                    // would be shifted: restart the whole sequence.
                    Err(ControllerIOError::ArbitrationLoss) => stats.alf_retries += 1,
                    Err(ControllerIOError::UnexpectedStop) | Err(ControllerIOError::Timeout) => {
                        stats.end_retries += 1;
                    }
                    Err(e) => {
                        defmt::error!("overflow write: read failed {}", e);
                        return Err("overflow_write: read failed");
                    }
                }
                if attempt == MAX_RETRIES {
                    return Err("overflow_write: retries exhausted");
                }
            }
        }
        Ok(())
    }

    /// Exercise the opt-in non-atomic path: with
    /// `Config::allow_chunked_reads` enabled, a long read may be split
    /// into re-addressed, STOP-separated chunks.
    ///
    /// Runs against the target in **stateless-read mode** — the device
    /// class chunked reads are actually intended for (each transaction
    /// re-serves from the front, so re-addressing loses nothing).
    /// That makes every expectation deterministic and byte-exact:
    /// `check_chunked_stateless` verifies each 256-byte window as
    /// either an exact continuation (chained atomic transaction) or an
    /// exact restart at zero (a re-addressed chunk), with first-window
    /// placement asserted unconditionally — no slip tolerance, no
    /// variable origin.
    ///
    /// For a length past the phase's chaining ceiling (`caps`), the
    /// payload must additionally be the **all-restart** shape: over
    /// the ceiling only the split path exists, so a continuation-
    /// shaped payload would mean the driver chained past its own
    /// declared limit — the test proves the split actually happened,
    /// not merely that some correct-looking bytes came back.
    ///
    /// Restores the default config and cursor mode before returning,
    /// unconditionally: every fallible step after the opt-in config is
    /// enabled routes through the result merge, so a failed run cannot
    /// leak chunked mode or stateless mode into later tests or reruns.
    pub async fn t_chunked_optin<C: Controller>(
        ctrl: &mut C,
        model: &mut Model,
        stats: &mut RetryStats,
        caps: PhaseCaps,
    ) -> TestResult {
        let mut cfg = CtrlConfig::default();
        cfg.speed = Speed::Standard;
        cfg.allow_chunked_reads = true;
        // Nothing is enabled if this fails, so the early return is safe.
        ctrl.set_config(&cfg).map_err(|_| "set_config failed")?;

        let mut result = set_stateless(ctrl, model, stats, true).await;

        if result.is_ok() {
            // Distinct values (BUF_LEN < 0x80, so `0x80 | i` is unique
            // per index) with the high bit set, which both enables the
            // contiguity check below and keeps the MSB-related ALF
            // quirk exercised.
            let mut pat = [0u8; BUF_LEN];
            for (i, b) in pat.iter_mut().enumerate() {
                *b = 0x80 | i as u8;
            }
            result = op_write(ctrl, &pat, model, stats).await;
            if result.is_ok() {
                model.write(&pat);
            }
        }

        if result.is_ok() {
            let mut big = [0u8; MAX_READ];
            // Includes a length past the DMA chaining ceiling: with the
            // opt-in enabled that read *is* split, and must still come
            // back byte-correct (non-atomic is permitted; incorrect is
            // not).
            for &l in &[257usize, 512, OVER_CAPACITY] {
                // Scrub with a value outside the pattern (all pattern
                // bytes have the MSB set): a lost chunk must read as
                // garbage, not as plausible data left over from the
                // previous length's read.
                big[..l].fill(0x00);
                if let Err(e) = op_read(ctrl, &mut big[..l], model, stats).await {
                    result = Err(e);
                    break;
                }
                // Deterministic in stateless mode: exact continuation
                // or exact restart-at-zero, uniform shape — and past
                // the ceiling, restart shape only (see above).
                let must_split = caps.dma_chain_ceiling.is_some_and(|c| l > c);
                if !model.check_chunked_stateless(&big[..l], must_split) {
                    defmt::error!("chunked L={}: seam break, head={:02x}", l, big[..8]);
                    result = Err("chunked read mismatch");
                    break;
                }
            }
        }

        // Unconditional restores; a restore failure outranks a pass
        // but must not mask a test failure.
        if set_stateless(ctrl, model, stats, false).await.is_err() && result.is_ok() {
            result = Err("stateless-mode restore failed");
        }
        let mut cfg = CtrlConfig::default();
        cfg.speed = Speed::Standard;
        if ctrl.set_config(&cfg).is_err() && result.is_ok() {
            result = Err("set_config restore failed");
        }
        result
    }

    /// Drop-cancellation probe: races transfers against staggered
    /// timer deadlines so their futures are dropped in every
    /// transaction phase, then proves the driver recovered.
    ///
    /// The driver's contract under cancellation is that dropping a
    /// transfer future — at ANY await point — recovers the bus: the
    /// transaction session's drop runs remediation, and on the DMA
    /// engine the channel is quiesced before the buffer borrow ends.
    /// At the suite's Standard-speed base rate (~90 µs/byte) the
    /// deadlines land inside the START/address phase, inside the
    /// first data bytes, mid-transfer, and (the largest) racing
    /// natural completion and the trailing STOP. Each race is followed
    /// by an anchored exact read: if cancellation left the bus held,
    /// commands queued, or a DMA channel live, that verify — or its
    /// anchoring resync write — fails.
    ///
    /// Scope: the proof is END-TO-END recoverability. From outside the
    /// driver a rig cannot attribute the recovery to the session's
    /// drop specifically — a later transition's own recovery arms
    /// clean up some abandonment shapes too — but a recovery path
    /// that wedges the controller or corrupts later transfers fails
    /// here regardless of which owner was supposed to run (this test
    /// caught exactly such a wedge on first contact with hardware).
    pub async fn t_cancellation<C: Controller>(ctrl: &mut C, model: &mut Model, stats: &mut RetryStats) -> TestResult {
        use embassy_futures::select::{Either, select};
        use embassy_time::Timer;

        // On this rig a 32-byte read runs ~5–6 ms end to end (the
        // emulated target stretches SCL between bytes); 30/80 µs
        // cancel inside the START/address phase (before and after the
        // driver queues its commands), 150–1500 µs land across the
        // early data bytes, 2400–4500 µs walk the middle and late
        // data, and 7000 µs sits past natural completion against the
        // DMA target, so some transfers genuinely finish first there —
        // the cancellation-vs-completion race at the close is probed
        // from both sides. (The interrupt target's per-byte service
        // stretches reads past even this; its races all cancel, which
        // is fine — the completion side is covered by the DMA-target
        // runs.)
        const DEADLINES_US: &[u64] = &[30, 80, 150, 300, 700, 1500, 2400, 2900, 3200, 3800, 4500, 7000];
        const REPS: usize = 3;
        /// Deadlines at or below this must always beat a 32-byte
        /// transfer on this rig (measured ≥ ~4.3 ms for both kinds):
        /// a completion there means the race never actually raced.
        const SHORT_MAX_US: u64 = 3200;

        // Make every race outcome DETERMINISTIC: the one legitimate
        // spontaneous fault on this rig is the spurious-ALF quirk,
        // which needs an MSB-SET first read data byte — so the races
        // run over an all-MSB-clear buffer pattern (distinct values,
        // 0x20..=0x47), and every race payload below is MSB-clear
        // too. With faults physically excluded, "cancel" is the only
        // legal outcome for any deadline that undercuts the transfer,
        // and that is asserted per workload and per band rather than
        // settling for "something cancelled somewhere".
        let mut pat = [0u8; BUF_LEN];
        for (i, b) in pat.iter_mut().enumerate() {
            *b = 0x20 + i as u8;
        }
        op_write(ctrl, &pat, model, stats).await?;
        model.write(&pat);

        let mut cancelled = 0u32;
        let mut completed = 0u32;
        let mut faulted = 0u32;
        // Per-band bookkeeping. Wire arithmetic per band:
        // * 32-byte read/write races: transfer ≥ ~4.3 ms, so every
        //   deadline ≤ SHORT_MAX must cancel;
        // * write_read: its read half begins ≥ ~1 ms in, so deadlines
        //   ≤ 700 µs land in the write half or the repeated-START
        //   continue transition and must cancel there;
        // * long chained reads: all three deadlines undercut the
        //   ~27+ ms transfer and must cancel (200 µs additionally
        //   lands pre-data, i.e. in the multi-RECEIVE pipeline before
        //   any byte moved).
        let mut short_completed = 0u32;
        let mut write_cancels = [0u32; DEADLINES_US.len()];
        let mut read_cancels = [0u32; DEADLINES_US.len()];
        let mut wr_cancels = [0u32; DEADLINES_US.len()];
        const WR_SHORT_MAX_US: u64 = 700;
        for _rep in 0..REPS {
            for (di, &d) in DEADLINES_US.iter().enumerate() {
                // Read race, anchored so a completed read is exactly
                // checkable.
                resync(ctrl, model).await?;
                let t_race = Instant::now();
                let mut buf = [0u8; 32];
                match select(ctrl.read(TARGET_ADDR, &mut buf), Timer::after_micros(d)).await {
                    Either::First(Ok(())) => {
                        completed += 1;
                        if d <= SHORT_MAX_US {
                            short_completed += 1;
                        }
                        if !model.check_read(&buf) {
                            return Err("cancellation: completed read mismatch");
                        }
                    }
                    // With the MSB-clear pattern a race fault should
                    // be impossible; counted so the band asserts (and
                    // the log) expose one if the premise ever breaks.
                    Either::First(Err(_)) => faulted += 1,
                    Either::Second(()) => {
                        cancelled += 1;
                        read_cancels[di] += 1;
                    }
                }
                defmt::debug!(
                    "[c] d={=u64} read race+recovery took {=u64}us",
                    d,
                    t_race.elapsed().as_micros()
                );

                // Recovery proof for the read race.
                let mut verify = [0u8; 32];
                op_read(ctrl, &mut verify, model, stats).await?;
                if !model.check_read(&verify) {
                    defmt::error!(
                        "cancellation d={=u64}us post-read: got={:02x} want={:02x}",
                        d,
                        verify[..32],
                        model.buf[..8]
                    );
                    // Raw follow-up reads (no resync): distinguish
                    // drained-once staleness from a persistent shift.
                    let mut r2 = [0u8; 8];
                    let _ = ctrl.read(TARGET_ADDR, &mut r2).await;
                    defmt::error!("  raw re-read 1: {:02x}", r2);
                    let _ = ctrl.read(TARGET_ADDR, &mut r2).await;
                    defmt::error!("  raw re-read 2: {:02x}", r2);
                    return Err("cancellation: post-read-cancel verify mismatch");
                }

                // Write race: cancels mid-TRANSMIT and during the
                // address phase. A cancelled write leaves the target
                // buffer partially committed; the verify's anchoring
                // resync rewrites it, so the model stays exact.
                // MSB-clear so a COMPLETED write cannot arm the ALF
                // quirk for the next race's read of this buffer.
                let w = [0x45u8; 32];
                let t_race = Instant::now();
                match select(ctrl.write(TARGET_ADDR, &w), Timer::after_micros(d)).await {
                    Either::First(Ok(())) => {
                        completed += 1;
                        if d <= SHORT_MAX_US {
                            short_completed += 1;
                        }
                        model.write(&w);
                    }
                    Either::First(Err(_)) => faulted += 1,
                    Either::Second(()) => {
                        cancelled += 1;
                        write_cancels[di] += 1;
                    }
                }
                defmt::debug!(
                    "[c] d={=u64} write race+recovery took {=u64}us",
                    d,
                    t_race.elapsed().as_micros()
                );

                // Recovery proof for the write race.
                let mut verify = [0u8; 32];
                op_read(ctrl, &mut verify, model, stats).await?;
                if !model.check_read(&verify) {
                    defmt::error!(
                        "cancellation d={=u64}us post-write: got={:02x} want={:02x}",
                        d,
                        verify[..8],
                        model.buf[..8]
                    );
                    return Err("cancellation: post-write-cancel verify mismatch");
                }

                // Repeated-START race: write_read's read half rides a
                // repeated START on the write half's open transaction,
                // so the drop can land in the write, in the continue
                // transition itself, or in the read half — the paths
                // the plain races above never touch. ~4–5 ms total on
                // this rig, so the ladder spans it like the others.
                resync(ctrl, model).await?;
                let wr_w = [0x11u8, 0x22];
                let mut wr_r = [0u8; 24];
                match select(ctrl.write_read(TARGET_ADDR, &wr_w, &mut wr_r), Timer::after_micros(d)).await {
                    Either::First(Ok(())) => {
                        completed += 1;
                        // The write half committed and rewound the
                        // cursor; the read half served from zero.
                        model.write(&wr_w);
                        if !model.check_read(&wr_r) {
                            return Err("cancellation: completed write_read mismatch");
                        }
                    }
                    Either::First(Err(_)) => faulted += 1,
                    Either::Second(()) => {
                        cancelled += 1;
                        wr_cancels[di] += 1;
                    }
                }

                // Recovery proof for the repeated-START race.
                let mut verify = [0u8; 32];
                op_read(ctrl, &mut verify, model, stats).await?;
                if !model.check_read(&verify) {
                    defmt::error!(
                        "cancellation d={=u64}us post-write_read: got={:02x} want={:02x}",
                        d,
                        verify[..8],
                        model.buf[..8]
                    );
                    return Err("cancellation: post-write_read-cancel verify mismatch");
                }
            }

            // Continue-transition races: an EMPTY-write write_read
            // reaches its repeated START almost immediately (the write
            // half is one addressed probe, ~0.3 ms), so these
            // deadlines bracket the continue transition's own settle
            // await — forcing drops INSIDE `async_start_continue`,
            // which the wide ladder above cannot pin. No data phase
            // precedes them and the transaction runs ≥ ~2.7 ms, so
            // cancel is the only legal outcome, asserted in place.
            for &d in &[250u64, 350, 450] {
                resync(ctrl, model).await?;
                let mut wr_r = [0u8; 24];
                match select(ctrl.write_read(TARGET_ADDR, &[], &mut wr_r), Timer::after_micros(d)).await {
                    Either::Second(()) => cancelled += 1,
                    Either::First(Ok(())) => {
                        return Err("cancellation: continue race completed before it physically could");
                    }
                    Either::First(Err(_)) => {
                        return Err("cancellation: continue race faulted");
                    }
                }

                let mut verify = [0u8; 32];
                op_read(ctrl, &mut verify, model, stats).await?;
                if !model.check_read(&verify) {
                    defmt::error!(
                        "cancellation continue d={=u64}us: got={:02x} want={:02x}",
                        d,
                        verify[..8],
                        model.buf[..8]
                    );
                    return Err("cancellation: post-continue-cancel verify mismatch");
                }
            }
        }

        // Long chained-read races: a >256-byte read puts multiple
        // RECEIVE commands in flight/pending at once, so cancellation
        // aborts a session holding a live command PIPELINE — recovery
        // must run it out (draining RX the whole way) and close behind
        // its auto-NACK, the path a 32-byte race can never reach.
        // Recovery costs the remaining pipeline on the wire (tens of
        // ms), so this ladder is short. 300 bytes chains on every
        // engine without the chunking opt-in.
        const LONG_DEADLINES_US: &[u64] = &[200, 5000, 20_000];
        for &d in LONG_DEADLINES_US {
            resync(ctrl, model).await?;
            let mut big = [0u8; 300];
            match select(ctrl.read(TARGET_ADDR, &mut big), Timer::after_micros(d)).await {
                // Every long deadline undercuts the ~27+ ms transfer,
                // and the MSB-clear pattern excludes the quirk:
                // cancellation is the only legal outcome, asserted
                // in place.
                Either::First(Ok(())) => {
                    return Err("cancellation: a long read completed before it physically could");
                }
                Either::First(Err(_)) => {
                    return Err("cancellation: a long chained read race faulted");
                }
                Either::Second(()) => cancelled += 1,
            }

            let mut verify = [0u8; 32];
            op_read(ctrl, &mut verify, model, stats).await?;
            if !model.check_read(&verify) {
                defmt::error!(
                    "cancellation long d={=u64}us: got={:02x} want={:02x}",
                    d,
                    verify[..8],
                    model.buf[..8]
                );
                return Err("cancellation: post-long-cancel verify mismatch");
            }
        }

        // Deterministic claims, asserted rather than logged (the wire
        // arithmetic is at the band bookkeeping above; the MSB-clear
        // pattern is what makes "must cancel" — not merely "must not
        // complete" — provable for the read-bearing workloads):
        // * neither 32-byte race may complete under SHORT_MAX;
        // * EVERY 32-byte read AND write race with d ≤ SHORT_MAX must
        //   have cancelled, per deadline band;
        // * EVERY write_read race with d ≤ 700 µs must have cancelled
        //   — those drops land in the write half or the repeated-
        //   START continue transition, so this pins the continue-path
        //   cancellations the composite race exists for;
        // * every long chained-read race asserted in place above.
        if short_completed != 0 {
            return Err("cancellation: a transfer completed before it physically could");
        }
        for (di, &d) in DEADLINES_US.iter().enumerate() {
            if d <= SHORT_MAX_US && (write_cancels[di] != REPS as u32 || read_cancels[di] != REPS as u32) {
                defmt::error!(
                    "cancellation: d={=u64}us cancels write {=u32}/{=u32} read {=u32}/{=u32}",
                    d,
                    write_cancels[di],
                    REPS as u32,
                    read_cancels[di],
                    REPS as u32
                );
                return Err("cancellation: a short-deadline race did not cancel");
            }
            if d <= WR_SHORT_MAX_US && wr_cancels[di] != REPS as u32 {
                defmt::error!(
                    "cancellation: d={=u64}us write_read races cancelled {=u32}/{=u32}",
                    d,
                    wr_cancels[di],
                    REPS as u32
                );
                return Err("cancellation: a write-half/repeated-START race did not cancel");
            }
        }
        defmt::info!(
            "[cancellation] {=u32} dropped mid-flight, {=u32} completed, {=u32} faulted",
            cancelled,
            completed,
            faulted
        );

        // The aborts above are intentional protocol violations and can
        // legitimately bump the target's premature-NeedMore counter (a
        // recovery STOP can land at the razor boundary the audit
        // watches for). This phase's settle_audit already ran — the
        // scrub protects whatever comes NEXT (the following phase, or
        // a rerun without a target reboot) from inheriting counts this
        // test manufactured, defense in depth alongside the
        // audit-reset every phase opens with.
        t_audit_reset(ctrl, stats).await
    }

    /// Budget for premature-NeedMore events per phase. The documented
    /// razor race — TDF asserting as the final byte enters the shifter
    /// just before the controller NACK+STOPs — is legitimate and rare;
    /// the pre-settle defect fires on EVERY read ending exactly at a
    /// served-view boundary (several per phase via the 40/80-byte
    /// lengths), far above this.
    const PREMATURE_NEEDMORE_BUDGET: u32 = 1;

    /// Query the target's premature-NeedMore counter, acknowledge it
    /// (an explicit reset control write), and FAIL the phase if it
    /// exceeds the budget. This is the enforcement for the TX-settle
    /// contract — reverting the settle fix turns the suite red here;
    /// the serve-side log line remains the diagnostic. The
    /// serve/acknowledge split matters: the audit read itself is
    /// retried on bus errors, and a reset-on-serve would let a failed
    /// first read (the MSB-quirk class) destroy the very evidence
    /// under audit, turning the gate probabilistically green.
    pub async fn t_settle_audit<C: Controller>(ctrl: &mut C, stats: &mut RetryStats) -> TestResult {
        let arm = [
            CTRL_MAGIC[0],
            CTRL_MAGIC[1],
            CTRL_MAGIC[2],
            CTRL_MAGIC[3],
            CTRL_SERVE_STATS,
        ];
        for attempt in 0..=MAX_RETRIES {
            // Classified arm write, then read, whole-sequence retry: a
            // failed read consumed the one-shot stats view, so re-arm
            // each time.
            ctl_write(ctrl, &arm, stats)
                .await
                .map_err(|_| "settle_audit: arm failed")?;
            let mut r = [0u8; STATS_LEN];
            match ctrl.read(TARGET_ADDR, &mut r).await {
                Ok(()) => {
                    if r[..4] != STATS_ECHO {
                        // The arm never reached the target (its write
                        // failed on the wire), so this read served
                        // buffer data: restart the sequence.
                        if attempt == MAX_RETRIES {
                            return Err("settle_audit: retries exhausted");
                        }
                        continue;
                    }
                    let count = u32::from_le_bytes(r[4..8].try_into().unwrap());
                    // Acknowledge BEFORE the verdict: reset the
                    // counter so the next phase audits independently
                    // whatever this one concludes. Loud on failure.
                    let ack = [
                        CTRL_MAGIC[0],
                        CTRL_MAGIC[1],
                        CTRL_MAGIC[2],
                        CTRL_MAGIC[3],
                        CTRL_RESET_STATS,
                    ];
                    ctl_write(ctrl, &ack, stats)
                        .await
                        .map_err(|_| "settle_audit: reset ack failed")?;
                    if count > PREMATURE_NEEDMORE_BUDGET {
                        defmt::error!(
                            "settle audit: {} premature NeedMore events (budget {})",
                            count,
                            PREMATURE_NEEDMORE_BUDGET
                        );
                        return Err("settle_audit: premature NeedMore over budget");
                    }
                    defmt::info!(
                        "  settle audit: {} premature NeedMore (budget {})",
                        count,
                        PREMATURE_NEEDMORE_BUDGET
                    );
                    return Ok(());
                }
                Err(ControllerIOError::ArbitrationLoss) => stats.alf_retries += 1,
                Err(ControllerIOError::UnexpectedStop) | Err(ControllerIOError::Timeout) => {
                    stats.end_retries += 1;
                }
                Err(ControllerIOError::FifoError) => stats.fef_retries += 1,
                Err(e) => {
                    defmt::error!("settle audit: read failed {}", e);
                    return Err("settle_audit: read failed");
                }
            }
            if attempt == MAX_RETRIES {
                return Err("settle_audit: retries exhausted");
            }
        }
        Err("settle_audit: retries exhausted")
    }

    /// One control write, retried with the harness's honest error
    /// classification: every recoverable class — arbitration loss,
    /// the early-termination/timeout family, and the documented
    /// transient FIFO fault — retries in its own telemetry bucket;
    /// anything else (a NACK, a write failure) is fatal and loud.
    /// Control writes are idempotent, so retrying one that may
    /// already have landed is always safe.
    async fn ctl_write<C: Controller>(ctrl: &mut C, msg: &[u8], stats: &mut RetryStats) -> TestResult {
        for attempt in 0..=MAX_RETRIES {
            match ctrl.write(TARGET_ADDR, msg).await {
                Ok(()) => return Ok(()),
                Err(ControllerIOError::ArbitrationLoss) => stats.alf_retries += 1,
                Err(ControllerIOError::UnexpectedStop) | Err(ControllerIOError::Timeout) => {
                    stats.end_retries += 1;
                }
                Err(ControllerIOError::FifoError) => stats.fef_retries += 1,
                Err(e) => {
                    defmt::error!("control write failed {}", e);
                    return Err("control write failed");
                }
            }
            if attempt == MAX_RETRIES {
                break;
            }
        }
        Err("control write: retries exhausted")
    }

    /// Blocking twin of [`ctl_write`].
    fn b_ctl_write(ctrl: &mut ControllerI2c<'_, Blocking>, msg: &[u8], stats: &mut RetryStats) -> TestResult {
        for attempt in 0..=MAX_RETRIES {
            match ctrl.blocking_write(TARGET_ADDR, msg) {
                Ok(()) => return Ok(()),
                Err(ControllerIOError::ArbitrationLoss) => stats.alf_retries += 1,
                Err(ControllerIOError::UnexpectedStop) | Err(ControllerIOError::Timeout) => {
                    stats.end_retries += 1;
                }
                Err(ControllerIOError::FifoError) => stats.fef_retries += 1,
                Err(e) => {
                    defmt::error!("control write (blocking) failed {}", e);
                    return Err("control write failed");
                }
            }
            if attempt == MAX_RETRIES {
                break;
            }
        }
        Err("control write: retries exhausted")
    }

    /// Scrub the target's audit state (counter AND pending stats
    /// latch). Run at every phase start: the target task persists
    /// across controller reruns, so an interrupted run must not
    /// contaminate the next one's audit or serve a stale stats
    /// payload to its first read.
    pub async fn t_audit_reset<C: Controller>(ctrl: &mut C, stats: &mut RetryStats) -> TestResult {
        let msg = [
            CTRL_MAGIC[0],
            CTRL_MAGIC[1],
            CTRL_MAGIC[2],
            CTRL_MAGIC[3],
            CTRL_RESET_STATS,
        ];
        ctl_write(ctrl, &msg, stats).await.map_err(|_| "audit_reset: failed")
    }

    /// Late-NACK (mid-write NDF) coverage — the halting-fault recovery
    /// paths, exercised without target cooperation: a MULTI-BYTE write
    /// to an absent address. The settle wake fires when the START is
    /// pulled from the FIFO — a full address-time (~90 µs at the
    /// suite's Standard speed) before the NACK bit — so under any
    /// realistic scheduling the session mints and the write body
    /// queues its first byte(s) before NDF latches MID-WRITE with a
    /// queued suffix. (A margin, not an architectural guarantee: an
    /// interleaving that loses it degrades this into a plain
    /// address-NACK probe with identical assert outcomes — both
    /// shapes classify AddressNack and both must recover clean — so
    /// the late-NDF claim rests on the timing arithmetic, and the
    /// asserts prove outcome-safety across ALL interleavings.)
    /// From the driver's side that is exactly the data-phase-NACK
    /// shape: the halt-preserving classify must freeze the suffix
    /// (nothing may reach the wire after the failure returns), and
    /// remediate must discriminate the auto-STOP sub-cases by
    /// observation, since the fault instant's FIFO state varies with
    /// byte timing.
    ///
    /// A true wire-level DATA NACK from the emulated target is not
    /// producible — hardware-measured on FRDM-MCXA577: STAR[TXNACK]
    /// raised from idle NACKs the next ADDRESS (the transaction never
    /// matches), and raised at the address-release window or mid-data
    /// it changes nothing (8 further bytes still ACKed). NXP-ticket
    /// material; noted so nobody re-attempts it.
    pub async fn t_data_nack<C: Controller>(ctrl: &mut C, model: &mut Model, stats: &mut RetryStats) -> TestResult {
        use embassy_futures::select::{Either, select};
        use embassy_time::Timer;

        // MSB-clear payload. NOTE the verifies below prove BUS
        // recovery, not suffix death directly — a BAD_ADDR suffix can
        // never land in the target's buffer in any interleaving, and
        // a replayed stale TRANSMIT's wire signature (FEF on the next
        // transaction) would be absorbed by the anchoring resync's
        // retries. Suffix death is the driver-side property enforced
        // by the halt-preserving classify, which this test drives
        // through its paths; it is not independently observable from
        // this rig.
        let mut w = [0u8; 12];
        for (i, b) in w.iter_mut().enumerate() {
            *b = 0x50 + i as u8;
        }

        // Straight late-NACK: exact class, then prove the bus fully
        // recovered with an anchored byte-exact read.
        match ctrl.write(BAD_ADDR, &w).await {
            Err(ControllerIOError::AddressNack) => {}
            Ok(()) => return Err("data_nack: write to an absent address succeeded"),
            Err(_) => return Err("data_nack: wrong error class for the NACK"),
        }
        let mut verify = [0u8; 32];
        op_read(ctrl, &mut verify, model, stats).await?;
        if !model.check_read(&verify) {
            return Err("data_nack: post-NACK verify mismatch");
        }

        // Cancellation racing the late NACK: the drop lands before,
        // around, and after NDF latches (~0.3-0.6 ms in), so recovery
        // variously runs with the fault already latched, latching
        // concurrently, or never — the wait_out_halting_fault
        // sub-cases. Legal outcomes are ONLY cancel or the exact
        // class.
        for &d in &[150u64, 400, 800] {
            match select(ctrl.write(BAD_ADDR, &w), Timer::after_micros(d)).await {
                Either::First(Err(ControllerIOError::AddressNack)) => {}
                Either::Second(()) => {}
                Either::First(Ok(())) => {
                    return Err("data_nack: raced write to an absent address succeeded");
                }
                Either::First(Err(_)) => return Err("data_nack: raced write failed with wrong class"),
            }
            let mut verify = [0u8; 32];
            op_read(ctrl, &mut verify, model, stats).await?;
            if !model.check_read(&verify) {
                defmt::error!(
                    "data_nack d={=u64}us: got={:02x} want={:02x}",
                    d,
                    verify[..8],
                    model.buf[..8]
                );
                return Err("data_nack: post-race verify mismatch");
            }
        }

        Ok(())
    }

    /// Blocking-phase variant of [`t_audit_reset`].
    pub fn b_audit_reset(ctrl: &mut ControllerI2c<'_, Blocking>, stats: &mut RetryStats) -> TestResult {
        let msg = [
            CTRL_MAGIC[0],
            CTRL_MAGIC[1],
            CTRL_MAGIC[2],
            CTRL_MAGIC[3],
            CTRL_RESET_STATS,
        ];
        b_ctl_write(ctrl, &msg, stats).map_err(|_| "audit_reset(b): failed")
    }

    /// Blocking-phase settle audit — same protocol as
    /// [`t_settle_audit`] over the blocking API. Also prevents
    /// blocking-phase boundary reads from leaking their counts into a
    /// later run's async audit when the target is not rebooted in
    /// between.
    pub fn b_settle_audit(ctrl: &mut ControllerI2c<'_, Blocking>, stats: &mut RetryStats) -> TestResult {
        let arm = [
            CTRL_MAGIC[0],
            CTRL_MAGIC[1],
            CTRL_MAGIC[2],
            CTRL_MAGIC[3],
            CTRL_SERVE_STATS,
        ];
        let ack = [
            CTRL_MAGIC[0],
            CTRL_MAGIC[1],
            CTRL_MAGIC[2],
            CTRL_MAGIC[3],
            CTRL_RESET_STATS,
        ];
        for attempt in 0..=MAX_RETRIES {
            b_ctl_write(ctrl, &arm, stats).map_err(|_| "settle_audit(b): arm failed")?;
            let mut r = [0u8; STATS_LEN];
            match ctrl.blocking_read(TARGET_ADDR, &mut r) {
                Ok(()) if r[..4] == STATS_ECHO => {
                    let count = u32::from_le_bytes(r[4..8].try_into().unwrap());
                    b_ctl_write(ctrl, &ack, stats).map_err(|_| "settle_audit(b): reset ack failed")?;
                    if count > PREMATURE_NEEDMORE_BUDGET {
                        defmt::error!(
                            "settle audit (blocking): {} premature NeedMore (budget {})",
                            count,
                            PREMATURE_NEEDMORE_BUDGET
                        );
                        return Err("settle_audit(b): over budget");
                    }
                    defmt::info!(
                        "  settle audit (blocking): {} premature NeedMore (budget {})",
                        count,
                        PREMATURE_NEEDMORE_BUDGET
                    );
                    return Ok(());
                }
                Ok(()) => {
                    // Arm never landed; buffer data served. Retry.
                }
                Err(ControllerIOError::ArbitrationLoss) => stats.alf_retries += 1,
                Err(ControllerIOError::UnexpectedStop) | Err(ControllerIOError::Timeout) => {
                    stats.end_retries += 1;
                }
                Err(ControllerIOError::FifoError) => stats.fef_retries += 1,
                Err(e) => {
                    defmt::error!("settle audit (blocking): read failed {}", e);
                    return Err("settle_audit(b): read failed");
                }
            }
            if attempt == MAX_RETRIES {
                return Err("settle_audit(b): retries exhausted");
            }
        }
        Err("settle_audit(b): retries exhausted")
    }

    /// Long soak: 2000 iters of a randomized op mix (W / R / WR) with
    /// exact payload check. Deterministic xorshift PRNG, so every run and
    /// both phases replay the same op sequence. Reports throughput.
    pub async fn t_soak<C: Controller>(ctrl: &mut C, model: &mut Model, stats: &mut RetryStats) -> TestResult {
        const N: u32 = 2000;
        let mut state: u32 = 0xC0FFEE_u32;
        let mut next = || {
            state ^= state << 13;
            state ^= state >> 17;
            state ^= state << 5;
            state
        };

        let t0 = Instant::now();
        let mut bytes: u32 = 0;
        for i in 0..N {
            let op = next() % 3;
            let len = ((next() % 8) as usize) + 1; // 1..=8
            let mut buf = [0u8; 8];
            for (j, b) in buf.iter_mut().enumerate() {
                *b = (i as u8).wrapping_add(j as u8) ^ 0xA5;
            }
            let mut rbuf = [0u8; 8];

            match op {
                0 => {
                    op_write(ctrl, &buf[..len], model, stats).await?;
                    model.write(&buf[..len]);
                    bytes += len as u32;
                }
                1 => {
                    let r = &mut rbuf[..len];
                    op_read(ctrl, r, model, stats).await?;
                    if !model.check_read(r) {
                        defmt::error!("soak i={} R: got={:02x}", i, r);
                        return Err("soak: read mismatch");
                    }
                    bytes += len as u32;
                }
                _ => {
                    let wlen = core::cmp::max(1, len / 2);
                    let r = &mut rbuf[..len];
                    op_write_read(ctrl, &buf[..wlen], r, model, stats).await?;
                    model.write(&buf[..wlen]);
                    if !model.check_read(r) {
                        defmt::error!("soak i={} WR: got={:02x}", i, r);
                        return Err("soak: wr mismatch");
                    }
                    bytes += (wlen + len) as u32;
                }
            }
        }

        let ms = t0.elapsed().as_millis().max(1);
        defmt::info!(
            "  soak: {=u32} ops, {=u32} payload bytes, {=u64} bytes/s",
            N,
            bytes,
            bytes as u64 * 1000 / ms
        );
        Ok(())
    }

    // ---- blocking-path battery ------------------------------------------

    /// Blocking twin of `resync`.
    fn b_resync(ctrl: &mut ControllerI2c<'_, Blocking>, model: &mut Model) -> TestResult {
        let snap = model.snapshot();
        for _ in 0..=MAX_RETRIES {
            if ctrl.blocking_write(TARGET_ADDR, &snap).is_ok() {
                model.write(&snap);
                return Ok(());
            }
        }
        Err("resync failed")
    }

    fn b_write(
        ctrl: &mut ControllerI2c<'_, Blocking>,
        data: &[u8],
        model: &mut Model,
        stats: &mut RetryStats,
    ) -> TestResult {
        for _ in 0..=MAX_RETRIES {
            match ctrl.blocking_write(TARGET_ADDR, data) {
                Ok(()) => return Ok(()),
                Err(ControllerIOError::ArbitrationLoss) => {
                    stats.alf_retries += 1;
                    b_resync(ctrl, model)?;
                }
                Err(ControllerIOError::FifoError) => {
                    stats.fef_retries += 1;
                    b_resync(ctrl, model)?;
                }
                Err(e) => {
                    defmt::error!("blocking write err: {} (len {})", e, data.len());
                    return Err("write failed");
                }
            }
        }
        Err("write: retries exhausted")
    }

    fn b_read(
        ctrl: &mut ControllerI2c<'_, Blocking>,
        buf: &mut [u8],
        model: &mut Model,
        stats: &mut RetryStats,
    ) -> TestResult {
        // Anchor — see `op_read`.
        b_resync(ctrl, model)?;
        for _ in 0..=MAX_RETRIES {
            match ctrl.blocking_read(TARGET_ADDR, buf) {
                Ok(()) => return Ok(()),
                Err(ControllerIOError::ArbitrationLoss) => {
                    stats.alf_retries += 1;
                    b_resync(ctrl, model)?;
                }
                Err(ControllerIOError::FifoError) => {
                    stats.fef_retries += 1;
                    b_resync(ctrl, model)?;
                }
                Err(ControllerIOError::UnexpectedStop) | Err(ControllerIOError::Timeout) => {
                    stats.end_retries += 1;
                    b_resync(ctrl, model)?;
                }
                Err(e) => {
                    defmt::error!("blocking read err: {} (len {})", e, buf.len());
                    return Err("read failed");
                }
            }
        }
        Err("read: retries exhausted")
    }

    fn b_write_read(
        ctrl: &mut ControllerI2c<'_, Blocking>,
        w: &[u8],
        r: &mut [u8],
        model: &mut Model,
        stats: &mut RetryStats,
    ) -> TestResult {
        for _ in 0..=MAX_RETRIES {
            match ctrl.blocking_write_read(TARGET_ADDR, w, r) {
                Ok(()) => return Ok(()),
                Err(ControllerIOError::ArbitrationLoss) => {
                    stats.alf_retries += 1;
                    b_resync(ctrl, model)?;
                }
                Err(ControllerIOError::FifoError) => {
                    stats.fef_retries += 1;
                    b_resync(ctrl, model)?;
                }
                Err(ControllerIOError::UnexpectedStop) | Err(ControllerIOError::Timeout) => {
                    stats.end_retries += 1;
                    b_resync(ctrl, model)?;
                }
                Err(_) => return Err("write_read failed"),
            }
        }
        Err("write_read: retries exhausted")
    }

    /// Blocking-path condensation of the async suite: short W/R traffic,
    /// every long read length, consecutive reads, repeated-START into a
    /// long read, NACK recovery into a long read, and a 512-byte write.
    pub fn t_blocking_battery(
        ctrl: &mut ControllerI2c<'_, Blocking>,
        model: &mut Model,
        stats: &mut RetryStats,
    ) -> TestResult {
        for i in 0..20u16 {
            let w = [i as u8, 0xB0 | (i as u8 & 0x0F)];
            b_write(ctrl, &w, model, stats)?;
            model.write(&w);
            let mut r = [0u8; 2];
            b_read(ctrl, &mut r, model, stats)?;
            if !model.check_read(&r) {
                defmt::error!("blk i={}: got={:02x}", i, r);
                return Err("mismatch");
            }
        }

        let mut pat = [0u8; BUF_LEN];
        for (i, b) in pat.iter_mut().enumerate() {
            *b = 0x80 | (i as u8).wrapping_mul(11);
        }
        b_write(ctrl, &pat, model, stats)?;
        model.write(&pat);

        let mut big = [0u8; MAX_READ];
        // The blocking path refills the command FIFO as it drains, so it
        // chains any length — including past the DMA ceiling.
        for &l in LONG_LENGTHS.iter().chain(core::iter::once(&OVER_CAPACITY)) {
            b_read(ctrl, &mut big[..l], model, stats)?;
            if !model.check_read(&big[..l]) {
                defmt::error!("blk long L={}: head={:02x}", l, big[..8]);
                return Err("long read mismatch");
            }
        }

        b_read(ctrl, &mut big[..257], model, stats)?;
        if !model.check_read(&big[..257]) {
            return Err("consecutive read 1 mismatch");
        }
        b_read(ctrl, &mut big[..31], model, stats)?;
        if !model.check_read(&big[..31]) {
            return Err("consecutive read 2 mismatch");
        }

        let w = [pat[0], pat[1], pat[2], pat[3]];
        b_write_read(ctrl, &w, &mut big[..300], model, stats)?;
        model.write(&w);
        if !model.check_read(&big[..300]) {
            return Err("wr long read mismatch");
        }

        if ctrl.blocking_write(BAD_ADDR, &[0x00]).is_ok() {
            return Err("expected NACK");
        }
        b_read(ctrl, &mut big[..257], model, stats)?;
        if !model.check_read(&big[..257]) {
            return Err("post-NACK long read mismatch");
        }

        let mut w512 = [0u8; 512];
        for (i, b) in w512.iter_mut().enumerate() {
            *b = (i as u8) ^ 0xC3;
        }
        b_write(ctrl, &w512, model, stats)?;
        model.write(&w512);
        b_read(ctrl, &mut big[..BUF_LEN], model, stats)?;
        if !model.check_read(&big[..BUF_LEN]) {
            return Err("long write mismatch");
        }

        Ok(())
    }
}
