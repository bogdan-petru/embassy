//! One short UART transmission demonstrating the four-layer contract recipe.
//! 115200 baud, 8N1, static source, 1..=32 bytes, one consuming async operation.
//! This pilot does not replace Lpuart, implement embedded-io, retry or receive.

mod hardware;

#[forbid(unsafe_code)]
mod service {
    use embassy_futures::select::{Either, select};
    use uart_one_shot_interface::Cpu;
    use uart_one_shot_vocabulary::Hardware;

    use super::hardware::DeviceHardware;
    pub use super::hardware::{ClockConfig, InterruptHandler, McxaUartPort};
    pub use uart_one_shot_core::Error;

    /// Owns this UART, TX pin, DMA channel, IRQ bindings and one transaction.
    /// Success includes UART TC, not merely DMA DONE. Error/cancellation consumes
    /// the driver; forgetting the future retains its immutable static source.
    pub struct Uart(Hardware<DeviceHardware>);

    impl Uart {
        /// Consume a SoC binding, preserving its clock/power lease through
        /// every pending state and cleanup. No raw clock frequency is accepted.
        pub fn new(port: McxaUartPort) -> Result<Self, Error> {
            DeviceHardware::new(port).map(|device| Self(Hardware::new(Cpu::new(device))))
        }

        pub async fn write(self, bytes: &'static [u8]) -> Result<(), Error> {
            // One deadline includes both DMA and final wire completion. It is
            // cooperative cancellation, not preemption of synchronous cleanup.
            let transfer = uart_one_shot_core::run_write(self.0, bytes);
            let deadline = embassy_time::Timer::after_millis(100);
            match select(transfer, deadline).await {
                Either::First(result) => result,
                Either::Second(()) => Err(Error::Timeout),
            }
        }
    }
}

pub use service::{ClockConfig, Error, InterruptHandler, McxaUartPort, Uart};
