//! Shared example bodies, not a second implementation of peripheral drivers.
//! Board binaries construct the hardware and supply operations and a clock.
#![no_std]
#![forbid(unsafe_code)]

pub mod i2c;
pub mod uart;
