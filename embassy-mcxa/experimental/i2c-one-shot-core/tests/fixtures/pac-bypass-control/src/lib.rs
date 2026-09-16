//! COMPILE ONLY. Never execute or link this fixture into firmware.
//!
//! These operations would violate the one-shot ISR's ownership policy, yet they
//! require no Rust unsafe operation. The wake-only source gate rejects the same
//! accesses. A successful check is the lint's negative control, not a safety
//! endorsement of touching an unowned peripheral.
#![no_std]
#![forbid(unsafe_code)]

pub fn safe_pac_access_compiles() {
    embassy_mcxa::pac::LPI2C0.mtdr().write(|word| word.set_data(0));
    let _discarded = embassy_mcxa::pac::LPI2C0.mrdr().read();
}
