//! Layer 4: one short async/DMA write or combined write/read.
//!
//! This is intentionally NOT the full `controller::I2c` API: one
//! 1..=32 bytes per phase, 7-bit address, 100 kHz, 12 MHz functional clock.
//! The driver and static buffer cannot be reused unsafely after cancellation.
//! Errors shut the local engine down; reset the peer before another attempt.
//! No chunking, retries, bus clear, blocking mode, or embedded-hal I2c trait.

mod session;

use i2c_one_shot_core::Idle;
pub use session::InterruptHandler;
pub(in crate::i2c) use session::Registers;
pub use session::{ClockConfig, McxaI2cPort};
use session::{Hardware, new_hardware, with_deadline, write_read_transfer, write_transfer};

/// Shared errors are defined below the HAL, without creating a dependency cycle.
pub use i2c_one_shot_core::Error;

/// Owns one peripheral, two pins, one bound DMA channel, and one transaction.
///
/// `write` consumes `self`: even on error this is not a reusable bus driver.
///
/// Correct: one owner, one static source, one awaited write.
///
/// ```no_run
/// use embassy_mcxa::i2c::one_shot::{I2c, Error};
/// async fn once(bus: I2c, message: &'static [u8]) -> Result<(), Error> {
///     bus.write(0x2a, message).await
/// }
/// ```
///
/// A second write through the consumed owner does not compile:
///
/// ```compile_fail
/// use embassy_mcxa::i2c::one_shot::I2c;
/// async fn twice(bus: I2c, message: &'static [u8]) {
///     bus.write(0x2a, message).await.unwrap();
///     bus.write(0x2a, message).await.unwrap(); // use of moved value
/// }
/// ```
///
/// Stack storage cannot satisfy this deliberately static-only DMA contract:
///
/// ```compile_fail
/// use embassy_mcxa::i2c::one_shot::I2c;
/// async fn stack_source(bus: I2c) {
///     let message = [42u8; 16];
///     bus.write(0x2a, &message).await.unwrap(); // does not live long enough
/// }
/// ```
pub struct I2c(Idle<Hardware>);

impl I2c {
    /// Consume an owned MCXA binding. Acquisition derives the mapping, clock
    /// and directional DMA requests from the same Embassy singleton.
    pub fn new(port: McxaI2cPort) -> Result<Self, Error> {
        new_hardware(port).map(|hardware| Self(Idle::new(hardware)))
    }

    /// Send one static buffer, returning only after DMA completion AND STOP.
    /// The bus transaction is closed; whether the addressed device has finished
    /// acting on the bytes is the device driver's concern.
    ///
    /// Static storage is deliberate: even `mem::forget` of a polled future
    /// cannot make the source memory disappear while DMA may still read it.
    /// Ordinary cancellation drops the owner and shuts down DMA and I2C.
    pub async fn write(self, address: u8, bytes: &'static [u8]) -> Result<(), Error> {
        validate(address, bytes.len())?;
        with_deadline(
            write_transfer(self.0, address, bytes),
            embassy_time::Timer::after(embassy_time::Duration::from_millis(100)),
        )
        .await
    }

    /// Write, repeated START, read, STOP using the same DMA channel in turn.
    /// Both lengths are checked before START; there are no intermediate STOPs.
    /// Returns exclusive receive storage only after DMA quiescence AND STOP.
    ///
    /// This one-shot API consumes both the driver and the RX buffer on error
    /// or cancellation. It does not provide a buffer-reclaim/retry API. A
    /// forgotten future retains static storage indefinitely: safe Rust cannot
    /// access it while the forgotten DMA may still write. Use the returned
    /// reference on success, never a second alias to the static allocation.
    ///
    /// Stack-backed RX storage cannot meet the forgotten-future guarantee:
    /// ```compile_fail
    /// use embassy_mcxa::i2c::one_shot::I2c;
    /// async fn stack_rx(bus: I2c) {
    ///     let mut rx = [0; 4];
    ///     bus.write_read(0x2a, &[0x10], &mut rx).await.unwrap();
    /// }
    /// ```
    /// A moved RX buffer cannot be inspected even if the future is forgotten:
    /// ```compile_fail
    /// use embassy_mcxa::i2c::one_shot::I2c;
    /// fn leaked_rx(bus: I2c, rx: &'static mut [u8]) {
    ///     let transfer = bus.write_read(0x2a, &[0x10], rx);
    ///     core::mem::forget(transfer);
    ///     rx[0] = 0; // exclusive borrow is still required for 'static
    /// }
    /// ```
    pub async fn write_read(
        self,
        address: u8,
        write: &'static [u8],
        read: &'static mut [u8],
    ) -> Result<&'static mut [u8], Error> {
        validate(address, write.len())?;
        validate(address, read.len())?;
        with_deadline(
            write_read_transfer(self.0, address, write, read),
            embassy_time::Timer::after(embassy_time::Duration::from_millis(100)),
        )
        .await
    }
}

fn validate(address: u8, length: usize) -> Result<(), Error> {
    i2c_one_shot_core::validate_transfer(address, length)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn one_shot_rejects_unsupported_transfers_before_start() {
        assert_eq!(validate(0x2a, 16), Ok(()));
        for (address, length) in [(0x00, 16), (0x78, 16), (0x2a, 0), (0x2a, 33)] {
            assert_eq!(validate(address, length), Err(Error::InvalidInput));
        }
    }
}
