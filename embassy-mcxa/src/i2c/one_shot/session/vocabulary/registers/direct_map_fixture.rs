//! Compile-only mutation anchors for the actual generated LPI2C owner.
//!
//! These are deliberately legal uses of the checked-in map. Mutation cases
//! replace one marked operation at a time and run the HAL's host suite, which
//! links the real safe-mmio recorder instead of a stand-alone fake register
//! block.

use super::mmio;

#[allow(dead_code, unused_mut)]
fn direct_enabled_surface(mut registers: mmio::Lpi2cRegisters<mmio::Enabled>) {
    registers.write_mtdr(0);
    let _ = registers.read_msr();
    let _ = registers.read_mfsr();
    let _ = registers.read_mrdr();
}

#[allow(dead_code, unused_mut)]
fn direct_off_surface(mut registers: mmio::Lpi2cRegisters<mmio::Off>) {
    registers.write_mier(0);
    registers.write_mcfgr1(0);
    registers.write_mccr0(0);
}

#[allow(dead_code)]
fn direct_arm_once(registers: mmio::Lpi2cRegisters<mmio::Enabled>) {
    let streaming = registers.into_arm_dma();
    let _ = streaming;
}

#[allow(dead_code)]
fn direct_owner_identity(registers: mmio::Lpi2cRegisters<mmio::Enabled>) {
    let _ = registers;
}

#[allow(dead_code, unused_mut)]
fn direct_streaming_surface(mut registers: mmio::Lpi2cRegisters<mmio::Streaming>) {
    let _ = registers.read_msr();
    let _ = registers.read_mfsr();
    registers.write_mier(0);
}
