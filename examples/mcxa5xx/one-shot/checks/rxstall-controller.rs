//! Separate write-service-withholding probe; the normal application is unchanged.
//! I2C_ONE_SHOT_RXSTALL=short requires success; long requires exactly Timeout.
//! A timeout is not proof of a continuously low SCL waveform or peer recovery.
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_mcxa::clocks::config::Div8;
use embassy_mcxa::i2c::one_shot::{ClockConfig, Error, I2c, InterruptHandler, McxaI2cPort};
use embassy_mcxa::interrupt::typelevel::Interrupt;
use embassy_mcxa::{bind_interrupts, dma, pac, peripherals};
use embassy_time::{Instant, Timer};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    LPI2C3 => InterruptHandler<peripherals::LPI2C3>;
    DMA0_CH0 => dma::InterruptHandler<peripherals::DMA0_CH0>;
});

static TX: StaticCell<[u8; 16]> = StaticCell::new();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let long = match option_env!("I2C_ONE_SHOT_RXSTALL") {
        Some("short") => false,
        Some("long") => true,
        _ => panic!("Build with I2C_ONE_SHOT_RXSTALL=short or long"),
    };
    let mut config = embassy_mcxa::config::Config::default();
    config.clock_cfg.sirc.fro_lf_div = Div8::from_divisor(1);
    let p = embassy_mcxa::init(config);
    defmt::info!("RXSTALL CONTROLLER: 10s quiet window");
    Timer::after_secs(10).await;
    let port = McxaI2cPort::acquire(p.LPI2C3, p.P3_21, p.P3_20, p.DMA0_CH0, Irqs, ClockConfig::default()).unwrap();
    let i2c = I2c::new(port).unwrap();
    let bytes = TX.init(*b"Hello from DMA!\n");
    let started = Instant::now();
    let result = i2c.write(0x2a, bytes).await;
    let elapsed_us = started.elapsed().as_micros();
    defmt::info!("RXSTALL CONTROLLER result={:?}, elapsed_us={}", result, elapsed_us);

    if long {
        assert_eq!(result, Err(Error::Timeout));
        // This is a selected-board observation window, not a hard real-time
        // promise. It separates the 100 ms deadline from target release at 150 ms.
        assert!((95_000..=125_000).contains(&elapsed_us));
    } else {
        assert_eq!(result, Ok(()));
        assert!((5_000..=50_000).contains(&elapsed_us));
    }
    // Read-only audit after the consumed owner's Drop. Do not repair anything
    // before these assertions or the probe would hide missing local cleanup.
    let dma = pac::EDMA_0_TCD.tcd(0).ch_csr().read();
    assert!(!dma.erq() && !dma.active());
    assert!(!pac::LPI2C3.mder().read().tdde());
    assert!(!pac::LPI2C3.mcr().read().men());
    assert!(!embassy_mcxa::interrupt::typelevel::LPI2C3::is_enabled());
    if long {
        defmt::info!("RXSTALL LONG CONTROLLER PASS: exact Timeout and local cleanup");
    } else {
        defmt::info!("RXSTALL SHORT CONTROLLER PASS");
    }
    cortex_m_semihosting::debug::exit(cortex_m_semihosting::debug::EXIT_SUCCESS);
}
