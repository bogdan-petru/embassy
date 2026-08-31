//! GPIO bus recovery for a controller that has already relinquished LPI2C.
//!
//! I2C is open drain: software can pull a line low but it can never force a
//! peer-held line high. Therefore a standard bus clear is meaningful only
//! when SCL can rise. For a clockable bus with SDA held low, it emits up to
//! nine SCL pulses and then a STOP. A target that actively clock-stretches
//! SCL must release the line itself before this sequence can proceed.

use embedded_hal_1::delay::DelayNs;

use super::BusClearError;
use crate::gpio::{AnyPin, TemporaryOpenDrain};

/// I2C standard-mode minimum high/low timing is 4.0/4.7 us. Use a small,
/// conservative five-microsecond half period independent of the controller
/// baud setting; GPIO recovery is a fault path, not normal bus traffic.
const HALF_PERIOD_US: u32 = 5;

/// Bound a peer's clock stretch while validating each released SCL edge.
/// A failed bus clear must return rather than turn a physical short into an
/// unbounded blocking call. Five milliseconds is intentionally much longer
/// than a recovery pulse but short enough to make a held-SCL diagnosis useful
/// to the caller.
const SCL_RISE_POLLS: usize = 1_000;

/// Wait for the released SCL line to actually rise. A GPIO high latch only
/// releases an open-drain line; reading PDIR is the physical-bus proof.
fn wait_scl_high(scl: &TemporaryOpenDrain<'_>, delay: &mut impl DelayNs) -> bool {
    for _ in 0..SCL_RISE_POLLS {
        if scl.is_high() {
            return true;
        }
        delay.delay_us(HALF_PERIOD_US);
    }
    false
}

/// Attempt the standard GPIO bus-clear sequence while the controller MMIO
/// lease keeps MEN, MIER, and MDER disabled.
///
/// The temporary GPIO guards restore the caller's exact saved pad register
/// value on every return path, including a physical-line error. The caller
/// decides whether to re-enable LPI2C only after this function returns `Ok`.
pub(super) fn clear(scl: &AnyPin, sda: &AnyPin, delay: &mut impl DelayNs) -> Result<(), BusClearError> {
    let mut scl = TemporaryOpenDrain::new(scl);
    let mut sda = TemporaryOpenDrain::new(sda);

    // Start from both lines released. If another device is holding SCL low,
    // we cannot manufacture a rising edge or a valid STOP, so report that
    // fact rather than driving an invalid pulse sequence.
    scl.release();
    sda.release();
    if !wait_scl_high(&scl, delay) {
        return Err(BusClearError::SclHeldLow);
    }

    // An already-idle bus needs no clocks and, importantly, no synthetic
    // START/STOP traffic visible to a listening peer.
    if sda.is_high() {
        return Ok(());
    }

    // A target which lost a transfer may be waiting for up to nine clocks to
    // finish its byte state. Stop as soon as SDA is physically released.
    for _ in 0..9 {
        scl.drive_low();
        delay.delay_us(HALF_PERIOD_US);
        scl.release();
        if !wait_scl_high(&scl, delay) {
            return Err(BusClearError::SclHeldLow);
        }
        delay.delay_us(HALF_PERIOD_US);

        if sda.is_high() {
            break;
        }
    }

    if !sda.is_high() {
        return Err(BusClearError::SdaHeldLow);
    }

    // Form STOP as GPIO: SDA low while SCL is low, release SCL and verify
    // the physical high level, then release SDA while SCL is high.
    scl.drive_low();
    delay.delay_us(HALF_PERIOD_US);
    sda.drive_low();
    delay.delay_us(HALF_PERIOD_US);
    scl.release();
    if !wait_scl_high(&scl, delay) {
        return Err(BusClearError::SclHeldLow);
    }
    delay.delay_us(HALF_PERIOD_US);
    sda.release();
    delay.delay_us(HALF_PERIOD_US);

    if sda.is_high() {
        Ok(())
    } else {
        Err(BusClearError::SdaHeldLow)
    }
}
