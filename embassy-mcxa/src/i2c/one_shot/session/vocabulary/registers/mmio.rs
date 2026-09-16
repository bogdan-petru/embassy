//! Abacus declaration backed by safe-mmio, not Tock register cells.
//! The macro emits owned accessors; the private companions alone confirm reset
//! and restore CPU authority after DMA quiescence. PAC types retain bit semantics.
#![allow(dead_code)]

#[cfg(test)]
#[test]
fn abacus_layout_matches_pinned_pac_without_device_access() {
    // Addresses are compared only. Neither this test nor check_layout reads MMIO.
    let backing = [0u32; 64];
    let pac = unsafe { crate::pac::lpi2c::Lpi2c::from_ptr(backing.as_ptr().cast_mut().cast()) };
    check_layout(pac);
}
use crate::pac::lpi2c::{Cmd, Mder, Mtdr};

#[path = "control_boundary.rs"]
mod control_boundary;
#[path = "dma_boundary.rs"]
mod dma_boundary;

// Offsets are checked against the pinned PAC before ownership is acquired.
#[abacus_registers::process_mmio_block(
    peripheral_name = "Lpi2c", register_base_addr = 0,
    states = [(Unconfirmed), (Off), (Enabled), (Streaming)]
)]
#[mmio_initial_state(Unconfirmed)]
#[repr(C)]
struct Layout {
    _reserved0: [u8; 0x10],
    #[mmio_private]
    mcr: ReadWrite<u32>,
    #[RegAttributes([Enabled, Streaming], ReadOnly)]
    msr: ReadPure<u32>,
    #[RegAttributes([Off, Enabled, Streaming], WriteOnly)]
    mier: WriteOnly<u32>,
    #[mmio_private]
    mder: WriteOnly<u32>,
    _reserved1: [u8; 4],
    #[RegAttributes([Off], WriteOnly)]
    mcfgr1: WriteOnly<u32>,
    _reserved2: [u8; 0x20],
    #[RegAttributes([Off], WriteOnly)]
    mccr0: WriteOnly<u32>,
    _reserved3: [u8; 0x10],
    #[RegAttributes([Enabled, Streaming], ReadOnly)]
    mfsr: ReadPure<u32>,
    #[RegAttributes([Enabled], WriteOnly)]
    mtdr: WriteOnly<u32>,
    _reserved4: [u8; 0x0c],
    #[RegAttributes([Enabled], ReadOnly)]
    mrdr: ReadOnly<u32>,
}

// This multi-register recipe is deliberately reviewed driver code, not a
// generic generated setter. The RX path orders RDDE before START and RECEIVE.
impl Lpi2cRegisters<Enabled> {
    /// Fixed Enabled -> Streaming transition. No arbitrary MDER setter exists.
    pub(super) fn into_arm_dma(mut self) -> Lpi2cRegisters<Streaming> {
        // SAFETY: this is the reviewed fixed MDER arm word.
        unsafe { self.field_mut::<safe_mmio::fields::WriteOnly<u32>>(MDER_OFFSET) }.write(arm_dma_word());
        Lpi2cRegisters {
            base: self.base,
            __read_pure_msr: self.__read_pure_msr,
            __read_pure_mfsr: self.__read_pure_mfsr,
            state: PhantomData,
            _not_send_sync: PhantomData,
        }
    }

    /// Fixed Enabled -> Streaming RX transition. It enables RDDE before
    /// queuing the one reviewed START-read/RECEIVE command pair.
    pub(super) fn into_arm_rx_dma(mut self, address: u8, count_minus_one: u8) -> Lpi2cRegisters<Streaming> {
        // SAFETY: this is the reviewed fixed MDER RDDE arm word.
        unsafe { self.field_mut::<safe_mmio::fields::WriteOnly<u32>>(MDER_OFFSET) }.write(arm_receive_dma_word());
        let mut start = Mtdr(0);
        start.set_cmd(Cmd::START);
        start.set_data((address << 1) | 1);
        // SAFETY: this Enabled owner queues the fixed read START only after RDDE.
        unsafe { self.field_mut::<safe_mmio::fields::WriteOnly<u32>>(MTDR_OFFSET) }.write(start.0);
        let mut receive = Mtdr(0);
        receive.set_cmd(Cmd::RECEIVE);
        receive.set_data(count_minus_one);
        // SAFETY: this is the paired fixed RECEIVE command; Streaming exposes no MTDR writer.
        unsafe { self.field_mut::<safe_mmio::fields::WriteOnly<u32>>(MTDR_OFFSET) }.write(receive.0);
        Lpi2cRegisters {
            base: self.base,
            __read_pure_msr: self.__read_pure_msr,
            __read_pure_mfsr: self.__read_pure_mfsr,
            state: PhantomData,
            _not_send_sync: PhantomData,
        }
    }
}
fn arm_dma_word() -> u32 {
    let mut word = Mder(0);
    word.set_tdde(true);
    word.0
}

fn arm_receive_dma_word() -> u32 {
    let mut word = Mder(0);
    word.set_rdde(true);
    word.0
}

// Private adapter for the reviewed hardware recipe. No public
// register access, state cast, polling policy or success evidence is generated.
use crate::i2c::reset_sequence::{Register as ControlRegister, RegisterIo as ControlRegisterIo};

struct ControlIo<'a, S: State, F> {
    registers: &'a mut Lpi2cRegisters<S>,
    now: F,
}

impl<S: State, F: Fn() -> u64> ControlRegisterIo for ControlIo<'_, S, F> {
    fn read(&mut self, register: ControlRegister) -> u32 {
        #[allow(unreachable_patterns)]
        let offset = match register {
            ControlRegister::Control => MCR_OFFSET,
            _ => panic!("unmapped control I/O read"),
        };
        // SAFETY: the whitelist selects one reviewed field of this exclusive owner;
        // no retained shared handle or DMA FIFO access overlaps this temporary.
        unsafe { self.registers.field_mut::<safe_mmio::fields::ReadOnly<u32>>(offset) }.read()
    }

    fn write(&mut self, register: ControlRegister, word: u32) {
        #[allow(unreachable_patterns)]
        let offset = match register {
            ControlRegister::Control => MCR_OFFSET,
            ControlRegister::InterruptEnable => MIER_OFFSET,
            ControlRegister::DmaEnable => MDER_OFFSET,
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

/// Verify every declared offset against the linked PAC before making
/// an owned handle. This compares addresses only; no MMIO is read or
/// borrowed. The assertion also runs in release builds.
pub(super) fn check_layout(pac: crate::pac::lpi2c::Lpi2c) {
    // Compile-time PAC direction checks; these functions perform no MMIO.
    use crate::pac::common::{Read, Reg, Write};
    fn readable<T: Copy, A: Read>(_: Reg<T, A>) {}
    fn writable<T: Copy, A: Write>(_: Reg<T, A>) {}
    readable(pac.mcr());
    readable(pac.msr());
    readable(pac.mfsr());
    readable(pac.mrdr());
    writable(pac.mcr());
    writable(pac.mier());
    writable(pac.mder());
    writable(pac.mcfgr1());
    writable(pac.mccr0());
    writable(pac.mtdr());
    macro_rules! check {
        ($field:ident, $offset:ident) => {
            assert!(
                pac.$field().as_ptr() as usize == pac.as_ptr() as usize + $offset,
                concat!("one-shot MMIO layout drift vs PAC: ", stringify!($field))
            );
        };
    }
    check!(mcr, MCR_OFFSET);
    check!(msr, MSR_OFFSET);
    check!(mier, MIER_OFFSET);
    check!(mder, MDER_OFFSET);
    check!(mcfgr1, MCFGR1_OFFSET);
    check!(mccr0, MCCR0_OFFSET);
    check!(mfsr, MFSR_OFFSET);
    check!(mtdr, MTDR_OFFSET);
    check!(mrdr, MRDR_OFFSET);
}
