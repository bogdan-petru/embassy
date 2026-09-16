//! Abacus declaration using the shared safe-mmio backend.
//! UART STAT remains exclusive: its hardware also has write-one-to-clear bits.
#![allow(dead_code)]

#[cfg(test)]
#[test]
fn abacus_layout_matches_pinned_pac_without_device_access() {
    // Addresses are compared only. Neither this test nor check_layout reads MMIO.
    let backing = [0u32; 64];
    let pac = unsafe { crate::pac::lpuart::Lpuart::from_ptr(backing.as_ptr().cast_mut().cast()) };
    check_layout(pac);
}
#[path = "control_boundary.rs"]
mod control_boundary;
#[cfg(all(test, not(feature = "one-shot-host-tests")))]
#[path = "control_io_tests.rs"]
mod control_io_tests;

#[abacus_registers::process_mmio_block(
    peripheral_name = "Uart", register_base_addr = 0,
    states = [(Unconfirmed), (Off), (Enabled), (Streaming)]
)]
#[mmio_initial_state(Unconfirmed)]
#[repr(C)]
struct Layout {
    _reserved0: [u8; 8],
    #[mmio_private]
    global: ReadWrite<u32>,
    _reserved1: [u8; 4],
    #[mmio_private]
    baud: ReadWrite<u32>,
    #[RegAttributes([Enabled, Streaming], ReadOnly)]
    stat: ReadOnly<u32>,
    #[mmio_private]
    ctrl: ReadWrite<u32>,
    #[mmio_private]
    data: WriteOnly<u32>,
    _reserved2: [u8; 8],
    #[RegAttributes([Off], WriteOnly)]
    fifo: WriteOnly<u32>,
    #[RegAttributes([Off], WriteOnly)]
    water: WriteOnly<u32>,
}
// Private adapter for the reviewed hardware recipe. No public
// register access, state cast, polling policy or success evidence is generated.
use super::reset_sequence::{Register as ControlRegister, RegisterIo as ControlRegisterIo};

struct ControlIo<'a, S: State, F> {
    registers: &'a mut UartRegisters<S>,
    now: F,
}

impl<S: State, F: Fn() -> u64> ControlRegisterIo for ControlIo<'_, S, F> {
    fn read(&mut self, register: ControlRegister) -> u32 {
        #[allow(unreachable_patterns)]
        let offset = match register {
            ControlRegister::Global => GLOBAL_OFFSET,
            ControlRegister::Baud => BAUD_OFFSET,
            ControlRegister::Ctrl => CTRL_OFFSET,
            ControlRegister::Stat => STAT_OFFSET,
            _ => panic!("unmapped control I/O read"),
        };
        // SAFETY: the whitelist selects one reviewed field of this exclusive owner;
        // no retained shared handle or DMA FIFO access overlaps this temporary.
        unsafe { self.registers.field_mut::<safe_mmio::fields::ReadOnly<u32>>(offset) }.read()
    }

    fn write(&mut self, register: ControlRegister, word: u32) {
        #[allow(unreachable_patterns)]
        let offset = match register {
            ControlRegister::Global => GLOBAL_OFFSET,
            ControlRegister::Baud => BAUD_OFFSET,
            ControlRegister::Ctrl => CTRL_OFFSET,
            _ => panic!("unmapped control I/O write"),
        };
        // SAFETY: the whitelist selects one reviewed field of this exclusive owner;
        // no retained shared handle or DMA FIFO access overlaps this temporary.
        unsafe { self.registers.field_mut::<safe_mmio::fields::WriteOnly<u32>>(offset) }.write(word)
    }

    fn now_us(&self) -> u64 {
        (self.now)()
    }
}

/// Address comparison only: this reads no device register.
pub(super) fn check_layout(pac: crate::pac::lpuart::Lpuart) {
    // Compile-time PAC direction checks; these functions perform no MMIO.
    use crate::pac::common::{Read, Reg, Write};
    fn readable<T: Copy, A: Read>(_: Reg<T, A>) {}
    fn writable<T: Copy, A: Write>(_: Reg<T, A>) {}
    readable(pac.global());
    readable(pac.baud());
    readable(pac.stat());
    readable(pac.ctrl());
    writable(pac.global());
    writable(pac.baud());
    writable(pac.ctrl());
    writable(pac.data());
    writable(pac.fifo());
    writable(pac.water());
    macro_rules! check {
        ($field:ident, $offset:ident) => {
            assert!(
                pac.$field().as_ptr() as usize == pac.as_ptr() as usize + $offset,
                concat!("UART one-shot MMIO drift: ", stringify!($field))
            );
        };
    }
    check!(global, GLOBAL_OFFSET);
    check!(baud, BAUD_OFFSET);
    check!(stat, STAT_OFFSET);
    check!(ctrl, CTRL_OFFSET);
    check!(data, DATA_OFFSET);
    check!(fifo, FIFO_OFFSET);
    check!(water, WATER_OFFSET);
}
