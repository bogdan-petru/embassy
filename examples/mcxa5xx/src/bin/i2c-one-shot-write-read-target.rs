//! Peer asserts the actual repeated START, not merely matching response bytes.
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_mcxa::clocks::config::Div8;
use embassy_mcxa::i2c::target::{self, Address, InterruptHandler};
use embassy_mcxa::{bind_interrupts, dma, peripherals};
use {defmt_rtt as _, panic_probe as _};

#[path = "i2c_one_shot/peer.rs"]
mod peer;

bind_interrupts!(struct Irqs {
    LPI2C3 => InterruptHandler<peripherals::LPI2C3>;
    DMA0_CH0 => dma::InterruptHandler<peripherals::DMA0_CH0>;
    DMA0_CH1 => dma::InterruptHandler<peripherals::DMA0_CH1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_mcxa::config::Config::default();
    config.clock_cfg.sirc.fro_lf_div = Div8::from_divisor(1);
    let p = embassy_mcxa::init(config);
    let mut config = target::Config::default();
    config.address = Address::Single(u16::from(nxp_shared_examples::i2c::ADDRESS));
    let i2c =
        target::I2c::new_async_with_dma(p.LPI2C3, p.P3_21, p.P3_20, p.DMA0_CH0, p.DMA0_CH1, Irqs, config).unwrap();
    let mut peer = peer::Peer(i2c);
    defmt::info!("ONE-SHOT WRITE-READ TARGET READY");
    nxp_shared_examples::i2c::serve_write_read(&mut peer).await.unwrap();
    defmt::info!(
        "ONE-SHOT WRITE-READ TARGET PASS: register 0x10, repeated START, no intermediate STOP, termination observed"
    );
    cortex_m_semihosting::debug::exit(cortex_m_semihosting::debug::EXIT_SUCCESS);
}
