//! i2c-twoboard-target — the target (slave) half of the two-board I2C test.
//!
//! Flash this to board B and reset it; it runs standalone with no debugger
//! attached. It serves a 32-byte RAM buffer at address 0x2A on LPI2C3:
//! controller writes store into the buffer, controller reads return it.
//! Board A runs `i2c-twoboard-controller` against it.
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
async fn main(_spawner: Spawner) {
    let mut config = Config::default();
    config.clock_cfg.sirc.fro_lf_div = Div8::from_divisor(1);

    let p = hal::init(config);
    defmt::info!("i2c-twoboard-target: serving 0x2a on LPI2C3 (P3_21 SCL / P3_20 SDA)");

    i2c_twoboard::target_task(p.LPI2C3, p.P3_21, p.P3_20, Irqs).await
}
