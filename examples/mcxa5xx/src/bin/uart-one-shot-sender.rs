//! One UART owner, one static DMA write, one verdict after the final stop bit.
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_mcxa::clocks::config::Div8;
use embassy_mcxa::lpuart::one_shot::{ClockConfig, InterruptHandler, McxaUartPort, Uart};
use embassy_mcxa::{bind_interrupts, dma, peripherals};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    LPUART5 => InterruptHandler<peripherals::LPUART5>;
    DMA0_CH0 => dma::InterruptHandler<peripherals::DMA0_CH0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_mcxa::config::Config::default();
    config.clock_cfg.sirc.fro_12m_enabled = true;
    config.clock_cfg.sirc.fro_lf_div = Some(Div8::no_div());
    let p = embassy_mcxa::init(config);

    // Drive the TX idle level before the receiver enables RX. Leaving the pin
    // unconfigured throughout startup can look like a break/framing error.
    let clock = ClockConfig::default();
    #[cfg(feature = "uart-port-clock-div2")]
    let clock = ClockConfig {
        div: embassy_mcxa::clocks::periph_helpers::Div4::from_divisor(2).unwrap(),
        ..clock
    };
    let port = McxaUartPort::acquire(p.LPUART5, p.P0_25, p.DMA0_CH0, Irqs, clock).unwrap();
    defmt::info!("UART PORT CLOCK: {} Hz", port.clock_hz());
    let uart = Uart::new(port).unwrap();
    defmt::info!("UART PILOT SENDER: 10s quiet window; LPUART5/P0_25, 115200 8N1");
    Timer::after_secs(10).await;
    nxp_shared_examples::uart::send(|bytes| uart.write(bytes))
        .await
        .unwrap();
    defmt::info!("UART PILOT SENDER PASS: 16 bytes, DMA quiescent, transmission complete");
    cortex_m_semihosting::debug::exit(cortex_m_semihosting::debug::EXIT_SUCCESS);
}
