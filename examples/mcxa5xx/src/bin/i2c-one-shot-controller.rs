//! One new four-layer controller, one async DMA write, one verdict.
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_mcxa::clocks::config::Div8;
use embassy_mcxa::i2c::one_shot::{ClockConfig, I2c, InterruptHandler, McxaI2cPort};
use embassy_mcxa::{bind_interrupts, dma, peripherals};
use embassy_time::Timer;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    LPI2C3 => InterruptHandler<peripherals::LPI2C3>;
    DMA0_CH0 => dma::InterruptHandler<peripherals::DMA0_CH0>;
});

static TX: StaticCell<[u8; 16]> = StaticCell::new();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_mcxa::config::Config::default();
    config.clock_cfg.sirc.fro_lf_div = Div8::from_divisor(1);
    let p = embassy_mcxa::init(config);
    // Give the operator/runner time to reset the receiver after this boot.
    defmt::info!("ONE-SHOT CONTROLLER: 10s quiet window");
    Timer::after_secs(10).await;
    let port = McxaI2cPort::acquire(p.LPI2C3, p.P3_21, p.P3_20, p.DMA0_CH0, Irqs, ClockConfig::default()).unwrap();
    let i2c = I2c::new(port).unwrap();
    nxp_shared_examples::i2c::write(TX.init([0; 16]), |address, bytes| i2c.write(address, bytes))
        .await
        .unwrap();
    defmt::info!("ONE-SHOT CONTROLLER PASS: 16 bytes, DMA finished, STOP observed");
    cortex_m_semihosting::debug::exit(cortex_m_semihosting::debug::EXIT_SUCCESS);
}
