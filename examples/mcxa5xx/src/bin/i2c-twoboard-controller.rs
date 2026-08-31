//! i2c-twoboard-controller — the controller (master) half of the two-board
//! I2C test.
//!
//! Flash this to board A with `probe-rs run`. It drives the shared
//! `i2c_twoboard::harness` suite against the other board running
//! `i2c-twoboard-target` (interrupt-driven) or `i2c-twoboard-target-dma`
//! (DMA respond paths) — the suite is identical against either — in
//! three phases over the same physical bus: interrupt-driven async,
//! DMA, then a blocking-path battery.
//!
//! Coverage per async/DMA phase (see `i2c_twoboard::tests`): basic_rw,
//! lengths, burst, edges (wrong-address NACK on write and read, recovery,
//! target state preserved), speed_sweep (Standard / Fast / FastPlus),
//! long_transfers (reads of 255..512 bytes across the 256-byte RECEIVE
//! chunk boundary, consecutive reads, repeated-START, NACK recovery into
//! a long read, 512-byte write), isr_latency (the same traffic with
//! interrupts periodically blocked ~0.5 ms) and a 2000-iteration
//! randomized soak — every byte read back is checked against a shadow
//! model of the target's buffer. The blocking phase repeats the transfer
//! battery through the polled API.
//!
//! Wiring (board A ↔ board B): P3_20 ↔ P3_20 (SDA), P3_21 ↔ P3_21 (SCL),
//! GND ↔ GND, with pull-ups to 3V3 on both lines.
//!
//! Bring the target board up first, otherwise the initial sync write
//! NACKs. Exits via semihosting, so `probe-rs run` returns 0 when every
//! test passed and nonzero (panic → HardFault) on any failure.
//! `run-two-board.ps1` in this crate captures the validated run
//! procedure (probe roles, target-reset timing, SWD speed, build
//! recipe).

#![no_std]
#![no_main]

// Only the harness half of the shared module is used here; the target
// binary uses `target_task`.
#[allow(dead_code)]
#[path = "../i2c_twoboard.rs"]
mod i2c_twoboard;

use cortex_m_semihosting::debug;
use embassy_executor::Spawner;
use hal::bind_interrupts;
use hal::clocks::config::Div8;
use hal::config::Config;
use hal::i2c::controller::{self, I2c, InterruptHandler as ControllerIH, Speed};
use hal::peripherals::LPI2C3;
use {defmt_rtt as _, embassy_mcxa as hal, panic_probe as _};

bind_interrupts!(
    struct Irqs {
        LPI2C3 => ControllerIH<LPI2C3>;
    }
);

/// Build-time phase filter: `SUITE_PHASES=async,dma,blocking` (default
/// all). Lets a phase run as its own short probe session when the
/// debug-probe environment cannot sustain a full-suite session (the
/// board-A probe drops RTT-heavy sessions after 10–40 s of uptime).
fn phase_enabled(phase: &str) -> bool {
    match option_env!("SUITE_PHASES") {
        None => true,
        Some(list) => list.split(',').any(|p| p.trim() == phase),
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = Config::default();
    config.clock_cfg.sirc.fro_lf_div = Div8::from_divisor(1);

    let mut p = hal::init(config);
    defmt::info!("i2c-twoboard-controller: driving 0x2a on LPI2C3 (P3_21 SCL / P3_20 SDA)");

    // A phase-filtered build must say so loudly (a leaked env var in a
    // CI build would otherwise skip phases yet exit success), a
    // filter that enables NO phase is a footgun, not a run — and so
    // is a misspelled token, which would silently drop the phase it
    // meant to name.
    if let Some(list) = option_env!("SUITE_PHASES") {
        defmt::warn!(
            "SUITE_PHASES={=str}: phase-filtered build, NOT a full validation run",
            list
        );
        for tok in list.split(',') {
            let t = tok.trim();
            // A trailing or doubled comma is a formatting artifact,
            // not a phase name — only real tokens must match.
            if t.is_empty() {
                continue;
            }
            assert!(
                matches!(t, "async" | "dma" | "blocking" | "pin_low"),
                "SUITE_PHASES: unknown phase name"
            );
        }
        assert!(
            ["async", "dma", "blocking", "pin_low"].iter().any(|p| phase_enabled(p)),
            "SUITE_PHASES enables no phase"
        );
    }

    // Quiet window: this binary's own flash/reset can glitch a listening
    // target into a half-addressed ADRSTALL stretch that wedges the bus.
    // Idle long enough for the test flow to reset the target board after
    // the controller is up, so the suite always starts on a clean bus.
    defmt::info!("2s quiet window for target reset");
    embassy_time::Timer::after_secs(2).await;

    // Interrupt-latency interference for t_isr_latency: ~0.5 ms with all
    // interrupts blocked, every ~1.5 ms, while the test has it enabled.
    spawner.spawn(i2c_twoboard::interference::task(160_000, 1_500).unwrap());

    // Phase 1: interrupt-driven async controller.
    if phase_enabled("async") {
        let mut ccfg = controller::Config::default();
        ccfg.speed = Speed::Standard;
        let mut ctrl = I2c::new_async(p.LPI2C3.reborrow(), p.P3_21.reborrow(), p.P3_20.reborrow(), Irqs, ccfg).unwrap();
        // The interrupt engine refills the command FIFO as it drains:
        // no chaining ceiling, refusing a long read is a failure.
        let caps = i2c_twoboard::PhaseCaps {
            dma_chain_ceiling: None,
        };
        i2c_twoboard::harness::run("async", &mut ctrl, caps).await;
    }

    // Phase 2: DMA controller over the same bus.
    if phase_enabled("dma") {
        let mut ccfg = controller::Config::default();
        ccfg.speed = Speed::Standard;
        let mut ctrl = I2c::new_async_with_dma(
            p.LPI2C3.reborrow(),
            p.P3_21.reborrow(),
            p.P3_20.reborrow(),
            p.DMA0_CH0.reborrow(),
            p.DMA0_CH1.reborrow(),
            Irqs,
            ccfg,
        )
        .unwrap();
        // The DMA engine queues every RECEIVE up front in the 4-entry
        // command FIFO (PARAM[MTXFIFO]=2 on this part) and cannot
        // refill it while the CPU sleeps: reads past 4 * 256 bytes are
        // refused with `ChunkingRequired` unless chunking is opted in.
        let caps = i2c_twoboard::PhaseCaps {
            dma_chain_ceiling: Some(4 * 256),
        };
        i2c_twoboard::harness::run("dma", &mut ctrl, caps).await;
    }

    // Phase 3: blocking (polled) controller.
    if phase_enabled("blocking") {
        let mut ccfg = controller::Config::default();
        ccfg.speed = Speed::Standard;
        let mut ctrl = I2c::new_blocking(p.LPI2C3.reborrow(), p.P3_21.reborrow(), p.P3_20.reborrow(), ccfg).unwrap();
        i2c_twoboard::harness::run_blocking("blocking", &mut ctrl);
    }

    // LAST, deliberately: this probe takes the bus through a real terminal
    // pin-low fault, then asserts target cancellation plus GPIO bus clear
    // restore traffic. Keeping it final isolates a physical-fault regression
    // from the ordinary phase diagnostics that precede it.
    if phase_enabled("pin_low") {
        let mut ccfg = controller::Config::default();
        ccfg.speed = Speed::Standard;
        let mut ctrl = I2c::new_async(p.LPI2C3.reborrow(), p.P3_21.reborrow(), p.P3_20.reborrow(), Irqs, ccfg).unwrap();
        i2c_twoboard::harness::run_pin_low("pin_low", &mut ctrl).await;
    }

    defmt::info!("== two-board i2c test: all enabled phases passed ==");
    // Let the host drain the final RTT lines before the semihosting
    // exit tears the session down — the tail (including the verdict
    // line above) was otherwise observed truncated, with only the
    // exit code carrying the result.
    embassy_time::Timer::after_millis(100).await;
    debug::exit(debug::EXIT_SUCCESS);
    unreachable!();
}
