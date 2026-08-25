//! i2c-twoboard-controller — the controller (master) half of the two-board
//! I2C test.
//!
//! Flash this to board A with `probe-rs run`. It drives the shared
//! `i2c_twoboard::harness` suite against the other board running
//! `i2c-twoboard-target`, twice: first with the interrupt-driven async
//! controller, then with the DMA controller, so both transfer engines are
//! exercised over the same physical bus in one run.
//!
//! Coverage per phase (see `i2c_twoboard::tests`): basic_rw, lengths,
//! burst, edges (wrong-address NACK on write and read, recovery, target
//! state preserved), speed_sweep (Standard / Fast / FastPlus) and a
//! 2000-iteration randomized soak — every byte read back is checked
//! against a shadow model of the target's buffer.
//!
//! Wiring (board A ↔ board B): P3_20 ↔ P3_20 (SDA), P3_21 ↔ P3_21 (SCL),
//! GND ↔ GND, with pull-ups to 3V3 on both lines.
//!
//! Bring the target board up first, otherwise the initial sync write
//! NACKs. Exits via semihosting, so `probe-rs run` returns 0 when every
//! test passed and nonzero (panic → HardFault) on any failure.

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

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = Config::default();
    config.clock_cfg.sirc.fro_lf_div = Div8::from_divisor(1);

    let mut p = hal::init(config);
    defmt::info!("i2c-twoboard-controller: driving 0x2a on LPI2C3 (P3_21 SCL / P3_20 SDA)");

    // Phase 1: interrupt-driven async controller.
    {
        let mut ccfg = controller::Config::default();
        ccfg.speed = Speed::Standard;
        let mut ctrl = I2c::new_async(p.LPI2C3.reborrow(), p.P3_21.reborrow(), p.P3_20.reborrow(), Irqs, ccfg).unwrap();
        i2c_twoboard::harness::run("async", &mut ctrl).await;
    }

    // Phase 2: DMA controller over the same bus.
    {
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
        i2c_twoboard::harness::run("dma", &mut ctrl).await;
    }

    defmt::info!("== two-board i2c test: all phases passed ==");
    debug::exit(debug::EXIT_SUCCESS);
    unreachable!();
}
