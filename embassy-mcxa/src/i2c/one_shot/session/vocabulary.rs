//! HAL binding for the hardware-independent protocol vocabulary.
//! The core has no dependency on this crate, its PAC, MMIO or timer.

mod registers;

use i2c_one_shot_interface::{Cpu, Error};

pub(in crate::i2c::one_shot) use registers::Hardware;
pub use registers::InterruptHandler;
pub(in crate::i2c) use registers::Registers;
pub use registers::{ClockConfig, McxaI2cPort};

pub(in crate::i2c::one_shot) fn new_hardware(port: McxaI2cPort) -> Result<Hardware, Error> {
    let device = registers::DeviceHardware::new(port)?;
    Ok(Hardware::new(Cpu::new(device)))
}

#[cfg(all(test, feature = "one-shot-host-tests"))]
mod effect_tests;
