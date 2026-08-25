//! Shared harness for the i2c-twoboard-* examples: a two-board I2C test
//! between separate MCXA577s wired to the same LPI2C3 bus.
//!
//! Wiring (board A ↔ board B): P3_20 ↔ P3_20 (SDA), P3_21 ↔ P3_21 (SCL),
//! GND ↔ GND. The bus needs pull-ups to 3V3 on both lines.
//!
//! The target exposes a 32-byte RAM buffer at address 0x2A: controller
//! writes store into the front of the buffer, controller reads return the
//! current contents. The controller keeps an exact shadow model of that
//! buffer and checks every byte it reads back against the model, so data
//! integrity is verified end to end against real written payloads instead
//! of a constant fill.
//!
//! The suite starts with a sync write that resets the target buffer to a
//! known state, so it is deterministic across re-runs and across the
//! async/DMA phases without resetting the target board.
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
use hal::i2c::controller::{Config as CtrlConfig, IOError as ControllerIOError, SetupError, Speed};
use hal::i2c::target::{self, Address, Config as TargetConfig, InterruptHandler, ReadStatus, Request, WriteStatus};
use hal::interrupt::typelevel::Binding;
use hal::peripherals::{LPI2C3, P3_20, P3_21};

pub const TARGET_ADDR: u8 = 0x2A;
pub const BUF_LEN: usize = 32;
const FILL: u8 = 0x55;

/// Target task: a 32-byte RAM buffer served over I2C at [`TARGET_ADDR`].
///
/// Writes overwrite the front of the buffer; reads return the current
/// contents. The buffer persists between transactions, which is what lets
/// the controller model it byte for byte.
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
            Request::Read(addr) => {
                let count = match tgt.async_respond_to_read(&buf).await.unwrap() {
                    ReadStatus::Complete(n) | ReadStatus::NeedMore(n) | ReadStatus::EarlyStop(n) => n,
                    _ => 0,
                };
                defmt::trace!("[T R {:02x}] -> {:02x}", addr, buf[..count]);
            }
            Request::Write(addr) => {
                let mut scratch = [0u8; BUF_LEN];
                let count = match tgt.async_respond_to_write(&mut scratch).await.unwrap() {
                    WriteStatus::Stopped(n) | WriteStatus::Restarted(n) | WriteStatus::BufferFull(n) => n,
                    _ => 0,
                };
                let count = count.min(BUF_LEN);
                buf[..count].copy_from_slice(&scratch[..count]);
                defmt::trace!("[T W {:02x}] <- {:02x}", addr, buf[..count]);
            }
            _ => {}
        }
    }
}

/// Trait abstracting an async I2C controller for the test suite.
///
/// Both `I2c<'_, Async>` and `I2c<'_, Dma<'_>>` implement
/// `embedded_hal_async::i2c::I2c` and `SetConfig`, so the same suite runs
/// against the interrupt-driven and the DMA controller.
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
    fn write(&mut self, data: &[u8]) {
        self.buf[..data.len()].copy_from_slice(data);
    }

    fn check(&self, read: &[u8]) -> bool {
        read == &self.buf[..read.len()]
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
        run_test!("soak", tests::t_soak(ctrl, &mut model, &mut stats));

        defmt::info!(
            "== two-board i2c suite [{=str}] ALL PASS ({=u32} arbitration-loss retries) ==",
            mode,
            stats.alf_retries
        );
    }
}

pub mod tests {
    use super::*;

    type TestResult = Result<(), &'static str>;

    /// Spurious-ArbitrationLoss retry accounting for one suite run.
    #[derive(Default)]
    pub struct RetryStats {
        /// Total operations retried after an `ArbitrationLoss`.
        pub alf_retries: u32,
    }

    /// Retries per operation before giving up on `ArbitrationLoss`.
    const MAX_RETRIES: u32 = 3;

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
                defmt::error!("iter {}: read mismatch got={:02x}", i, r);
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
        const BAD: u8 = 0x33;

        if ctrl.write(BAD, &[0x00]).await.is_ok() {
            return Err("E1: expected NACK on bad-addr write");
        }

        let mut r = [0u8; 2];
        if ctrl.read(BAD, &mut r).await.is_ok() {
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
}
