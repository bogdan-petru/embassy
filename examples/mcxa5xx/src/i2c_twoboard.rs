//! Shared harness for the i2c-twoboard-* examples: a two-board I2C test
//! between separate MCXA577s wired to the same LPI2C3 bus.
//!
//! Wiring (board A ↔ board B): P3_20 ↔ P3_20 (SDA), P3_21 ↔ P3_21 (SCL),
//! GND ↔ GND. The bus needs pull-ups to 3V3 on both lines.
//!
//! The target exposes a 32-byte RAM buffer at address 0x2A: controller
//! writes store into the buffer in 32-byte chunks, controller reads serve
//! the buffer cyclically for arbitrarily long transfers. The controller
//! keeps an exact shadow model of that buffer and checks every byte it
//! reads back against the model, so data integrity is verified end to end
//! against real written payloads instead of a constant fill.
//!
//! Long-transfer coverage: reads of 255/256/257/260/300/512 bytes cross
//! the driver's 256-byte RECEIVE-command chunk boundary, where the LPI2C
//! controller would otherwise depend on back-to-back receive-command
//! chaining (the documented silent-data-loss hazard for >256-byte reads).
//! This driver instead issues each chunk as its own repeated-START
//! transfer; these tests verify no byte is lost or duplicated across the
//! seams, in the interrupt-async, DMA, and blocking paths separately.
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
//! Retry policy: `ArbitrationLoss` is retried up to 3 times per operation
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
use hal::peripherals::{LPI2C3, P3_20, P3_21};

pub const TARGET_ADDR: u8 = 0x2A;
pub const BUF_LEN: usize = 32;
const FILL: u8 = 0x55;
/// An address nothing on the bus answers to.
const BAD_ADDR: u8 = 0x33;
/// Longest read the tests perform.
const MAX_READ: usize = 512;

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

/// Target task: a 32-byte RAM buffer served over I2C at [`TARGET_ADDR`].
///
/// Reads serve the buffer cyclically for as long as the controller keeps
/// clocking (`NeedMore` responds with the buffer again). Writes commit in
/// 32-byte chunks, each overwriting the front of the buffer. The buffer
/// persists between transactions, which is what lets the controller model
/// it byte for byte.
pub async fn target_task(
    peri: Peri<'static, LPI2C3>,
    scl: Peri<'static, P3_21>,
    sda: Peri<'static, P3_20>,
    irq: impl Binding<<LPI2C3 as hal::i2c::Instance>::Interrupt, InterruptHandler<LPI2C3>> + 'static,
) -> ! {
    let mut config = TargetConfig::default();
    config.address = Address::Single(TARGET_ADDR as u16);

    let mut tgt = target::I2c::new_async(peri, scl, sda, irq, config).unwrap();
    let mut buf = [FILL; BUF_LEN];

    loop {
        let request = tgt.async_listen().await.unwrap();
        defmt::trace!("[T] event {}", request);
        match request {
            Request::Read(_) => loop {
                defmt::trace!("[T] R serve {:02x}", buf[..2]);
                match tgt.async_respond_to_read(&buf).await.unwrap() {
                    // Controller ACKed the whole buffer and wants more:
                    // serve it again (cyclic).
                    ReadStatus::NeedMore(_) => continue,
                    ReadStatus::Complete(_) | ReadStatus::EarlyStop(_) => break,
                    _ => break,
                }
            },
            Request::Write(_) => {
                // One respond call per Write event, with headroom above
                // the longest transfer the tests perform. `BufferFull` on
                // an exact-multiple-length write is ambiguous ("full" vs
                // "full and stopped"), and calling respond_to_write again
                // would clear the pending STOP flag and misattribute the
                // *next* transaction's bytes to this one.
                let mut scratch = [0u8; MAX_READ + 1];
                let n = match tgt.async_respond_to_write(&mut scratch).await.unwrap() {
                    WriteStatus::Stopped(n) | WriteStatus::Restarted(n) | WriteStatus::BufferFull(n) => n,
                    _ => 0,
                };
                let n = n.min(scratch.len());
                // Commit in 32-byte chunks, mirroring `Model::write`.
                for chunk in scratch[..n].chunks(BUF_LEN) {
                    buf[..chunk.len()].copy_from_slice(chunk);
                }
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
pub struct Model {
    buf: [u8; BUF_LEN],
}

impl Model {
    /// Mirror of the target's write handling: data is committed in
    /// 32-byte chunks, each overwriting the front of the buffer, so a
    /// long write leaves the last chunk (plus any surviving tail of the
    /// chunk before it) in place.
    fn write(&mut self, data: &[u8]) {
        for chunk in data.chunks(BUF_LEN) {
            self.buf[..chunk.len()].copy_from_slice(chunk);
        }
    }

    /// Expected read data: the target serves its buffer cyclically and
    /// restarts from the front on every (repeated) START. The driver's
    /// 256-byte read chunks are a multiple of the 32-byte buffer, so
    /// position `i` of any read must equal `buf[i % BUF_LEN]` regardless
    /// of where the chunk seams fall.
    fn check(&self, read: &[u8]) -> bool {
        read.iter().enumerate().all(|(i, b)| *b == self.buf[i % BUF_LEN])
    }
}

pub mod harness {
    use super::*;

    /// Run the full suite through `ctrl` against the remote target board.
    ///
    /// `mode` labels the log lines (e.g. "async", "dma"). Logs
    /// `[mode] <test> PASS (<ms>)` per test and panics on the first
    /// failure, so a failing run exits through the panic handler.
    pub async fn run<C: Controller>(mode: &str, ctrl: &mut C) {
        defmt::info!("== two-board i2c suite [{=str}] start ==", mode);

        // Reset the target buffer to a known state so the model is exact
        // even if the target kept state from a previous run or phase.
        let mut model = Model { buf: [FILL; BUF_LEN] };
        if ctrl.write(TARGET_ADDR, &model.buf).await.is_err() {
            defmt::error!("[{=str}] sync write failed — is the target board up?", mode);
            panic!("target sync failed");
        }

        let mut stats = tests::RetryStats::default();

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
        run_test!("soak", tests::t_soak(ctrl, &mut model, &mut stats));

        defmt::info!(
            "== two-board i2c suite [{=str}] ALL PASS ({=u32} ALF / {=u32} FEF retries) ==",
            mode,
            stats.alf_retries,
            stats.fef_retries
        );
    }

    /// Blocking-path battery: the polled driver has no interrupt path, so
    /// this focuses on transfer correctness — basic traffic, every long
    /// length, consecutive reads, repeated-START, and NACK recovery.
    /// Panics on failure, like [`run`].
    pub fn run_blocking(mode: &str, ctrl: &mut ControllerI2c<'_, Blocking>) {
        defmt::info!("== two-board i2c suite [{=str}] start ==", mode);

        let mut model = Model { buf: [FILL; BUF_LEN] };
        if ctrl.blocking_write(TARGET_ADDR, &model.buf).is_err() {
            defmt::error!("[{=str}] sync write failed — is the target board up?", mode);
            panic!("target sync failed");
        }

        let mut stats = tests::RetryStats::default();
        let t0 = Instant::now();
        match tests::t_blocking_battery(ctrl, &mut model, &mut stats) {
            Ok(()) => defmt::info!("[{=str}] battery PASS ({=u64} ms)", mode, t0.elapsed().as_millis()),
            Err(e) => {
                defmt::error!("[{=str}] battery FAIL: {=str}", mode, e);
                panic!("test failure");
            }
        }

        defmt::info!(
            "== two-board i2c suite [{=str}] ALL PASS ({=u32} ALF / {=u32} FEF retries) ==",
            mode,
            stats.alf_retries,
            stats.fef_retries
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
    const LONG_LENGTHS: &[usize] = &[255, 256, 257, 260, 300, 512];

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
    }

    /// Retries per operation before giving up on `ArbitrationLoss`.
    const MAX_RETRIES: u32 = 15;

    async fn op_write<C: Controller>(ctrl: &mut C, data: &[u8], stats: &mut RetryStats) -> TestResult {
        for _ in 0..=MAX_RETRIES {
            match ctrl.write(TARGET_ADDR, data).await {
                Ok(()) => return Ok(()),
                Err(ControllerIOError::ArbitrationLoss) => stats.alf_retries += 1,
                Err(_) => return Err("write failed"),
            }
        }
        Err("write: retries exhausted")
    }

    async fn op_read<C: Controller>(ctrl: &mut C, buf: &mut [u8], stats: &mut RetryStats) -> TestResult {
        for _ in 0..=MAX_RETRIES {
            match ctrl.read(TARGET_ADDR, buf).await {
                Ok(()) => return Ok(()),
                Err(ControllerIOError::ArbitrationLoss) => stats.alf_retries += 1,
                Err(_) => return Err("read failed"),
            }
        }
        Err("read: retries exhausted")
    }

    async fn op_write_read<C: Controller>(ctrl: &mut C, w: &[u8], r: &mut [u8], stats: &mut RetryStats) -> TestResult {
        for _ in 0..=MAX_RETRIES {
            match ctrl.write_read(TARGET_ADDR, w, r).await {
                Ok(()) => return Ok(()),
                Err(ControllerIOError::ArbitrationLoss) => stats.alf_retries += 1,
                Err(_) => return Err("write_read failed"),
            }
        }
        Err("write_read: retries exhausted")
    }

    /// 100 iters of {write 2 bytes, read 2 bytes} with exact payload check.
    pub async fn t_basic_rw<C: Controller>(ctrl: &mut C, model: &mut Model, stats: &mut RetryStats) -> TestResult {
        for i in 0..100u16 {
            let w = [i as u8, (i >> 8) as u8];
            op_write(ctrl, &w, stats).await?;
            model.write(&w);

            let mut r = [0u8; 2];
            op_read(ctrl, &mut r, stats).await?;
            if !model.check(&r) {
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

            op_write(ctrl, &payload[..l], stats).await?;
            model.write(&payload[..l]);

            let r = &mut rbuf[..l];
            op_read(ctrl, r, stats).await?;
            if !model.check(r) {
                defmt::error!("L={}: read mismatch got={:02x}", l, r);
                return Err("read mismatch");
            }

            let wlen = core::cmp::max(1, l / 2);
            let w = &payload[..wlen];
            let r = &mut rbuf[..l];
            op_write_read(ctrl, w, r, stats).await?;
            model.write(w);
            if !model.check(r) {
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
            op_write(ctrl, &w, stats).await?;
            model.write(&w);

            let mut r = [0u8; 2];
            op_read(ctrl, &mut r, stats).await?;
            if !model.check(&r) {
                defmt::error!("burst i={}: read mismatch got={:02x}", i, r);
                return Err("read mismatch");
            }

            let w = [!(i as u8)];
            op_write_read(ctrl, &w, &mut r, stats).await?;
            model.write(&w);
            if !model.check(&r) {
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
        if ctrl.write(BAD_ADDR, &[0x00]).await.is_ok() {
            return Err("E1: expected NACK on bad-addr write");
        }

        let mut r = [0u8; 2];
        if ctrl.read(BAD_ADDR, &mut r).await.is_ok() {
            return Err("E2: expected NACK on bad-addr read");
        }

        let mut full = [0u8; BUF_LEN];
        op_read(ctrl, &mut full, stats).await.map_err(|_| "E3 read failed")?;
        if !model.check(&full) {
            defmt::error!("E3: target buffer changed after NACKs got={:02x}", full);
            return Err("E3 mismatch");
        }

        let w = [0xAB, 0xCD];
        op_write(ctrl, &w, stats).await.map_err(|_| "E4 write failed")?;
        model.write(&w);
        op_read(ctrl, &mut r, stats).await.map_err(|_| "E4 read failed")?;
        if !model.check(&r) {
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
                op_write(ctrl, &w, stats).await?;
                model.write(&w);

                let mut r = [0u8; 2];
                op_read(ctrl, &mut r, stats).await?;
                if !model.check(&r) {
                    defmt::error!("speed {} iter {}: got={:02x}", speed, i, r);
                    return Err("speed: mismatch");
                }

                let w = [!(i as u8)];
                op_write_read(ctrl, &w, &mut r, stats).await?;
                model.write(&w);
                if !model.check(&r) {
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
    /// - reads of 255/256/257/260/300/512 bytes;
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
        op_write(ctrl, &pat, stats).await?;
        model.write(&pat);

        let mut big = [0u8; MAX_READ];
        for &l in LONG_LENGTHS {
            let r = &mut big[..l];
            op_read(ctrl, r, stats).await?;
            if !model.check(r) {
                defmt::error!("long L={}: mismatch head={:02x}", l, r[..8]);
                return Err("long read mismatch");
            }
        }

        // Consecutive long reads separated by STOP.
        op_read(ctrl, &mut big[..257], stats).await?;
        if !model.check(&big[..257]) {
            return Err("consecutive read 1 mismatch");
        }
        op_read(ctrl, &mut big[..31], stats).await?;
        if !model.check(&big[..31]) {
            return Err("consecutive read 2 mismatch");
        }

        // Repeated START into a long read. The 4-byte write re-sends the
        // pattern's own prefix, so the model is unchanged by design.
        let w = [pat[0], pat[1], pat[2], pat[3]];
        op_write_read(ctrl, &w, &mut big[..300], stats).await?;
        model.write(&w);
        if !model.check(&big[..300]) {
            return Err("wr long read mismatch");
        }

        // Error recovery into a long read.
        if ctrl.write(BAD_ADDR, &[0x00]).await.is_ok() {
            return Err("expected NACK before long read");
        }
        op_read(ctrl, &mut big[..257], stats).await?;
        if !model.check(&big[..257]) {
            return Err("post-NACK long read mismatch");
        }

        // Long write: 512 bytes streamed in one transaction; the target
        // commits 32-byte chunks, so the buffer ends as the final chunk.
        let mut w512 = [0u8; 512];
        for (i, b) in w512.iter_mut().enumerate() {
            *b = (i as u8) ^ 0x5A;
        }
        op_write(ctrl, &w512, stats).await?;
        model.write(&w512);
        op_read(ctrl, &mut big[..BUF_LEN], stats).await?;
        if !model.check(&big[..BUF_LEN]) {
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
            op_write(ctrl, &w, stats).await?;
            model.write(&w);
            let mut r = [0u8; 2];
            op_read(ctrl, &mut r, stats).await?;
            if !model.check(&r) {
                defmt::error!("isr i={}: got={:02x}", i, r);
                return Err("isr: mismatch");
            }
        }

        let mut big = [0u8; MAX_READ];
        for &l in &[257usize, 512] {
            op_read(ctrl, &mut big[..l], stats).await?;
            if !model.check(&big[..l]) {
                return Err("isr: long read mismatch");
            }
        }

        let w = [model.buf[0], model.buf[1]];
        op_write_read(ctrl, &w, &mut big[..300], stats).await?;
        model.write(&w);
        if !model.check(&big[..300]) {
            return Err("isr: wr mismatch");
        }

        Ok(())
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
                    op_write(ctrl, &buf[..len], stats).await?;
                    model.write(&buf[..len]);
                    bytes += len as u32;
                }
                1 => {
                    let r = &mut rbuf[..len];
                    op_read(ctrl, r, stats).await?;
                    if !model.check(r) {
                        defmt::error!("soak i={} R: got={:02x}", i, r);
                        return Err("soak: read mismatch");
                    }
                    bytes += len as u32;
                }
                _ => {
                    let wlen = core::cmp::max(1, len / 2);
                    let r = &mut rbuf[..len];
                    op_write_read(ctrl, &buf[..wlen], r, stats).await?;
                    model.write(&buf[..wlen]);
                    if !model.check(r) {
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

    fn b_write(ctrl: &mut ControllerI2c<'_, Blocking>, data: &[u8], stats: &mut RetryStats) -> TestResult {
        for _ in 0..=MAX_RETRIES {
            match ctrl.blocking_write(TARGET_ADDR, data) {
                Ok(()) => return Ok(()),
                Err(ControllerIOError::ArbitrationLoss) => stats.alf_retries += 1,
                Err(ControllerIOError::FifoError) => stats.fef_retries += 1,
                Err(e) => {
                    defmt::error!("blocking write err: {} (len {})", e, data.len());
                    return Err("write failed");
                }
            }
        }
        Err("write: retries exhausted")
    }

    fn b_read(ctrl: &mut ControllerI2c<'_, Blocking>, buf: &mut [u8], stats: &mut RetryStats) -> TestResult {
        for _ in 0..=MAX_RETRIES {
            match ctrl.blocking_read(TARGET_ADDR, buf) {
                Ok(()) => return Ok(()),
                Err(ControllerIOError::ArbitrationLoss) => stats.alf_retries += 1,
                Err(ControllerIOError::FifoError) => stats.fef_retries += 1,
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
        stats: &mut RetryStats,
    ) -> TestResult {
        for _ in 0..=MAX_RETRIES {
            match ctrl.blocking_write_read(TARGET_ADDR, w, r) {
                Ok(()) => return Ok(()),
                Err(ControllerIOError::ArbitrationLoss) => stats.alf_retries += 1,
                Err(ControllerIOError::FifoError) => stats.fef_retries += 1,
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
            b_write(ctrl, &w, stats)?;
            model.write(&w);
            let mut r = [0u8; 2];
            b_read(ctrl, &mut r, stats)?;
            if !model.check(&r) {
                defmt::error!("blk i={}: got={:02x}", i, r);
                return Err("mismatch");
            }
        }

        let mut pat = [0u8; BUF_LEN];
        for (i, b) in pat.iter_mut().enumerate() {
            *b = 0x80 | (i as u8).wrapping_mul(11);
        }
        b_write(ctrl, &pat, stats)?;
        model.write(&pat);

        let mut big = [0u8; MAX_READ];
        for &l in LONG_LENGTHS {
            b_read(ctrl, &mut big[..l], stats)?;
            if !model.check(&big[..l]) {
                defmt::error!("blk long L={}: head={:02x}", l, big[..8]);
                return Err("long read mismatch");
            }
        }

        b_read(ctrl, &mut big[..257], stats)?;
        if !model.check(&big[..257]) {
            return Err("consecutive read 1 mismatch");
        }
        b_read(ctrl, &mut big[..31], stats)?;
        if !model.check(&big[..31]) {
            return Err("consecutive read 2 mismatch");
        }

        let w = [pat[0], pat[1], pat[2], pat[3]];
        b_write_read(ctrl, &w, &mut big[..300], stats)?;
        model.write(&w);
        if !model.check(&big[..300]) {
            return Err("wr long read mismatch");
        }

        if ctrl.blocking_write(BAD_ADDR, &[0x00]).is_ok() {
            return Err("expected NACK");
        }
        b_read(ctrl, &mut big[..257], stats)?;
        if !model.check(&big[..257]) {
            return Err("post-NACK long read mismatch");
        }

        let mut w512 = [0u8; 512];
        for (i, b) in w512.iter_mut().enumerate() {
            *b = (i as u8) ^ 0xC3;
        }
        b_write(ctrl, &w512, stats)?;
        model.write(&w512);
        b_read(ctrl, &mut big[..BUF_LEN], stats)?;
        if !model.check(&big[..BUF_LEN]) {
            return Err("long write mismatch");
        }

        Ok(())
    }
}
