//! Independent test peer using the existing RX-only UART driver.
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_mcxa::clocks::config::Div8;
use embassy_mcxa::lpuart::{Config, Error, LpuartRx};
use embassy_time::Instant;
use nxp_shared_examples::uart::{ReceiveError, verify_receiver};
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_mcxa::config::Config::default();
    config.clock_cfg.sirc.fro_12m_enabled = true;
    config.clock_cfg.sirc.fro_lf_div = Some(Div8::no_div());
    let p = embassy_mcxa::init(config);
    let uart_config = Config {
        baudrate_bps: 115_200,
        ..Default::default()
    };
    let mut uart = LpuartRx::new_blocking(p.LPUART5, p.P0_24, uart_config).unwrap();
    defmt::info!("UART PILOT RECEIVER READY: LPUART5/P0_24, 115200 8N1");

    let result = verify_receiver(
        || {
            let mut byte = [0];
            match uart.read(&mut byte) {
                Ok(()) => Ok(Some(byte[0])),
                Err(Error::RxFifoEmpty) => Ok(None),
                Err(error) => Err(error),
            }
        },
        || Instant::now().as_micros(),
    );
    match result {
        Ok(()) => {}
        Err(ReceiveError::Timeout { received }) => {
            defmt::panic!("UART PILOT RECEIVER FAIL: receive timeout after {} bytes", received)
        }
        Err(ReceiveError::Mismatch {
            index,
            expected,
            actual,
        }) => {
            defmt::panic!(
                "UART PILOT RECEIVER FAIL: byte {} expected {} got {}",
                index,
                expected,
                actual
            )
        }
        Err(ReceiveError::ExtraByte(byte)) => defmt::panic!("UART PILOT RECEIVER FAIL: extra byte {}", byte),
        Err(ReceiveError::Peripheral(error)) => {
            defmt::panic!("UART PILOT RECEIVER FAIL: UART receive error {:?}", error)
        }
    }
    drop(uart);
    defmt::info!("UART PILOT RECEIVER PASS: exact 16 bytes, no extra bytes for 20ms");
    cortex_m_semihosting::debug::exit(cortex_m_semihosting::debug::EXIT_SUCCESS);
}
