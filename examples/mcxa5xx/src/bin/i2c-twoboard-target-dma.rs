//! i2c-twoboard-target-dma — the target (slave) half of the two-board
//! I2C test, served through the target driver's **DMA** respond paths.
//!
//! Identical device behavior to `i2c-twoboard-target` (40-byte RAM
//! buffer at 0x2A, persistent read cursor, stateless-read control
//! write), but every respond runs over eDMA — covering the target DMA
//! chunk paths the interrupt-mode binary never touches, including the
//! terminated-transfer RDF drain (a byte delivered with the STOP can
//! sit in the FIFO with its DMA request not yet granted; the driver
//! must drain it, not maroon it).
//!
//! Flash this to board B and reset it; board A runs
//! `i2c-twoboard-controller` unchanged against either target binary.
//!
//! Wiring (board A ↔ board B): P3_20 ↔ P3_20 (SDA), P3_21 ↔ P3_21 (SCL),
//! GND ↔ GND, with pull-ups to 3V3 on both lines.

#![no_std]
#![no_main]

// Only the target half of the shared module is used here; the controller
// binary uses the harness half.
#[allow(dead_code)]
#[path = "../i2c_twoboard.rs"]
mod i2c_twoboard;

use embassy_executor::Spawner;
use hal::bind_interrupts;
use hal::clocks::config::Div8;
use hal::config::Config;
use hal::i2c::target::InterruptHandler as TargetIH;
use hal::peripherals::LPI2C3;
use {defmt_rtt as _, embassy_mcxa as hal, panic_probe as _};

bind_interrupts!(
    struct Irqs {
        LPI2C3 => TargetIH<LPI2C3>;
    }
);

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = Config::default();
    config.clock_cfg.sirc.fro_lf_div = Div8::from_divisor(1);

    let p = hal::init(config);
    defmt::info!("i2c-twoboard-target-dma: serving 0x2a on LPI2C3 (P3_21 SCL / P3_20 SDA), DMA respond paths");

    // Constant light interrupt-latency interference (~0.25 ms blocked
    // every ~2 ms), so the whole suite exercises target-side ACK/stall
    // handling under delayed ISR entry.
    i2c_twoboard::interference::ACTIVE.store(true, core::sync::atomic::Ordering::Relaxed);
    spawner.spawn(i2c_twoboard::interference::task(80_000, 2_000).unwrap());

    i2c_twoboard::target_task_dma(p.LPI2C3, p.P3_21, p.P3_20, p.DMA0_CH0, p.DMA0_CH1, Irqs).await
}
