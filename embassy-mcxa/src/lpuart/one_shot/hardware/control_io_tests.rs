//! Independent checks of the generated private adapter, not its metadata.
//! Live host memory substitutes only for the MMIO region; the real adapter
//! still performs every field-sized safe-mmio operation.

extern crate std;

use core::cell::Cell;
use std::panic::{AssertUnwindSafe, catch_unwind};

use super::{ControlIo, ControlRegister, ControlRegisterIo, UartRegisters, Unconfirmed};

#[test]
fn control_reads_use_the_documented_uart_offsets() {
    let original: [u32; 12] = core::array::from_fn(|index| 0xa500_0000 + index as u32);
    let mut words = original;
    // SAFETY: this aligned, live 0x30-byte region is exclusively assigned to
    // the owner. No array access, IRQ, DMA, or competing owner exists until
    // it is dropped; all accesses below go through the private adapter.
    let mut registers = unsafe { UartRegisters::<Unconfirmed>::new(words.as_mut_ptr().cast()) };
    {
        let mut io = ControlIo {
            registers: &mut registers,
            now: || 0,
        };
        // Literal offsets deliberately do not use generated constants/metadata.
        for (register, offset) in [
            (ControlRegister::Global, 0x08),
            (ControlRegister::Baud, 0x10),
            (ControlRegister::Stat, 0x14),
            (ControlRegister::Ctrl, 0x18),
        ] {
            assert_eq!(io.read(register), original[offset / 4], "UART_CONTROL_READ_OFFSET");
        }
    }
    drop(registers);
    assert_eq!(words, original, "UART_CONTROL_READ_MUST_NOT_WRITE");
}

#[test]
fn control_writes_touch_only_the_three_allowed_words() {
    let mut words = [0x5a5a_a5a5; 12];
    let mut expected = words;
    // SAFETY: same live/exclusive host-region contract as the read fixture;
    // expected is a separate copy, never a reference into the owned region.
    let mut registers = unsafe { UartRegisters::<Unconfirmed>::new(words.as_mut_ptr().cast()) };
    {
        let mut io = ControlIo {
            registers: &mut registers,
            now: || 0,
        };
        for (register, offset, value) in [
            (ControlRegister::Global, 0x08, 0x1234_5678),
            (ControlRegister::Baud, 0x10, 0x8765_4321),
            (ControlRegister::Ctrl, 0x18, 0xfedc_ba98),
        ] {
            io.write(register, value);
            expected[offset / 4] = value;
        }
    }
    drop(registers);
    assert_eq!(words, expected, "UART_CONTROL_WRITE_OFFSET_AND_ISOLATION");
}

#[test]
fn stat_write_is_rejected_before_any_memory_changes() {
    let original: [u32; 12] = core::array::from_fn(|index| 0x6b00_0000 + index as u32);
    let mut words = original;
    // SAFETY: the owner alone accesses this aligned live region, including
    // during the caught panic. Inspection waits until the owner is dropped.
    let mut registers = unsafe { UartRegisters::<Unconfirmed>::new(words.as_mut_ptr().cast()) };
    let rejected = {
        let mut io = ControlIo {
            registers: &mut registers,
            now: || 0,
        };
        catch_unwind(AssertUnwindSafe(|| io.write(ControlRegister::Stat, u32::MAX)))
    };
    drop(registers);
    assert!(rejected.is_err(), "UART_CONTROL_STAT_WRITE_MUST_REJECT");
    assert_eq!(words, original, "UART_CONTROL_STAT_REJECTION_BEFORE_EFFECT");
}

#[test]
fn control_clock_forwards_each_current_callback_value() {
    let mut words = [0u32; 12];
    let clock = Cell::new(17u64);
    let calls = Cell::new(0);
    // SAFETY: same exclusive, live and aligned host-region contract. The
    // callback captures only independent clock cells, never the MMIO region.
    let mut registers = unsafe { UartRegisters::<Unconfirmed>::new(words.as_mut_ptr().cast()) };
    {
        let io = ControlIo {
            registers: &mut registers,
            now: || {
                calls.set(calls.get() + 1);
                clock.get()
            },
        };
        assert_eq!(io.now_us(), 17);
        clock.set(u64::MAX - 9);
        assert_eq!(io.now_us(), u64::MAX - 9);
        assert_eq!(calls.get(), 2, "UART_CONTROL_CLOCK_IS_NOT_CACHED");
    }
    drop(registers);
    assert_eq!(words, [0; 12], "UART_CONTROL_CLOCK_HAS_NO_MMIO_EFFECT");
}
