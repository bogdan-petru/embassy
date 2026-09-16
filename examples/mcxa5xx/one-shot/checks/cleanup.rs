//! Separate validation firmware, NOT part of the one-shot application's flow.
//! Read-only PAC inspection checks that the new owner's Drop actually shut down.
//! Build with I2C_ONE_SHOT_CHECK=nack or cancel; reset the peer between cases.
#![no_std]
#![no_main]

use core::future::{Future, poll_fn};
use core::task::Poll;

use embassy_executor::Spawner;
use embassy_mcxa::clocks::config::Div8;
use embassy_mcxa::i2c::one_shot::{ClockConfig, Error, I2c, InterruptHandler, McxaI2cPort};
use embassy_mcxa::interrupt::typelevel::Interrupt;
use embassy_mcxa::{bind_interrupts, dma, pac, peripherals};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    LPI2C3 => InterruptHandler<peripherals::LPI2C3>;
    DMA0_CH0 => dma::InterruptHandler<peripherals::DMA0_CH0>;
});

static TX: StaticCell<[u8; 32]> = StaticCell::new();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_mcxa::config::Config::default();
    config.clock_cfg.sirc.fro_lf_div = Div8::from_divisor(1);
    let p = embassy_mcxa::init(config);
    let port = McxaI2cPort::acquire(p.LPI2C3, p.P3_21, p.P3_20, p.DMA0_CH0, Irqs, ClockConfig::default()).unwrap();
    let i2c = I2c::new(port).unwrap();
    let bytes = TX.init([0xa5; 32]);
    match option_env!("I2C_ONE_SHOT_CHECK") {
        Some("nack") => {
            assert_eq!(i2c.write(0x2b, bytes).await, Err(Error::Nack));
            defmt::info!("ONE-SHOT NACK: exact error observed");
        }
        Some("cancel") => {
            let mut transfer = core::pin::pin!(i2c.write(0x2a, bytes));
            poll_fn(|cx| {
                assert!(transfer.as_mut().poll(cx).is_pending());
                assert!(pac::LPI2C3.mder().read().tdde(), "must cancel while DMA is armed");
                Poll::Ready(())
            })
            .await;
            // End this scope: drop the POLLED future while DMA is still armed.
        }
        _ => panic!("Set I2C_ONE_SHOT_CHECK to nack or cancel before building."),
    }
    let csr = pac::EDMA_0_TCD.tcd(0).ch_csr().read();
    assert!(!csr.erq() && !csr.active());
    assert!(!pac::LPI2C3.mder().read().tdde());
    assert!(!pac::LPI2C3.mcr().read().men());
    assert!(!embassy_mcxa::interrupt::typelevel::LPI2C3::is_enabled());
    defmt::info!("ONE-SHOT CLEANUP PASS: DMA inactive, requests off, MEN off, IRQ disabled");
    cortex_m_semihosting::debug::exit(cortex_m_semihosting::debug::EXIT_SUCCESS);
}
