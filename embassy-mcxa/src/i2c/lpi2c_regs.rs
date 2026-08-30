//! Tock-style (`tock-registers`) MMIO cells for the LPI2C hot paths.
//!
//! This module supplies **only** the two things `nxp-pac` does not give
//! the safe wrappers: a `register_structs!` layout whose padding and
//! offsets are checked by the compiler, and per-register access typing
//! ([`ReadOnly`] / [`WriteOnly`] / [`ReadWrite`]) that makes a wrong-
//! direction access unrepresentable — `MRDR`/`SRDR` reads *pop* a FIFO
//! and must never be written, `MTDR`/`STDR` must never be read back.
//!
//! Everything *semantic* stays in the PAC. There are deliberately no
//! `register_bitfields!` here: field offsets, widths, enumerated values
//! and command encodings would then exist in two places, and an address
//! check cannot catch a bit that moved. The cells are therefore untyped
//! `u32`, and the wrappers convert each raw word through the PAC's own
//! value types (`pac::lpi2c::Msr`, `Mier`, `Cmd`, …), which are plain
//! `pub struct X(pub u32)` newtypes over exactly this word. The PAC
//! remains the single source of truth for what the bits mean.
//!
//! Layout drift is guarded three ways: the `register_structs!` block
//! and the `offset_of!` assertions below are **generated from the
//! PAC's own accessors** by `tools/gen_lpi2c_regs.py` (offsets are
//! never written by hand; rerun the script after a PAC bump, or run it
//! with `--check` to verify), the assertions pin the generated map at
//! compile time, and [`check_layout`] compares every mapped register's
//! address against the PAC's generated accessors at driver init — so
//! drift at any layer panics at first construction instead of
//! corrupting MMIO.

use tock_registers::register_structs;
use tock_registers::registers::{ReadOnly, ReadWrite, WriteOnly};

// BEGIN GENERATED (gen_lpi2c_regs.py): register_structs
register_structs! {
    /// LPI2C register block (hot-path subset; gaps are configuration
    /// registers still accessed through the PAC).
    pub LpI2cRegisters {
        (0x000 => _reserved0),
        /// Parameter Register (FIFO sizes). PAC type: `Param`.
        (0x004 => pub param: ReadOnly<u32>),
        (0x008 => _reserved1),
        /// Controller Control Register. PAC type: `Mcr`.
        (0x010 => pub mcr: ReadWrite<u32>),
        /// Controller Status Register (W1C flags). PAC type: `Msr`.
        (0x014 => pub msr: ReadWrite<u32>),
        /// Controller Interrupt Enable Register. PAC type: `Mier`.
        (0x018 => pub mier: ReadWrite<u32>),
        /// Controller DMA Enable Register. PAC type: `Mder`.
        (0x01c => pub mder: ReadWrite<u32>),
        (0x020 => _reserved2),
        /// Controller Configuration 1. PAC type: `Mcfgr1`.
        (0x024 => pub mcfgr1: ReadWrite<u32>),
        (0x028 => _reserved3),
        /// Controller FIFO Status Register. PAC type: `Mfsr`.
        (0x05c => pub mfsr: ReadOnly<u32>),
        /// Controller Transmit Data Register (command + data).
        /// Write-only: reading it back is meaningless. PAC type: `Mtdr`.
        (0x060 => pub mtdr: WriteOnly<u32>),
        (0x064 => _reserved4),
        /// Controller Receive Data Register. Read-only, and a read
        /// *pops* the RX FIFO. PAC type: `Mrdr`.
        (0x070 => pub mrdr: ReadOnly<u32>),
        (0x074 => _reserved5),
        /// Target Control Register. PAC type: `Scr`.
        (0x110 => pub scr: ReadWrite<u32>),
        /// Target Status Register (W1C flags). PAC type: `Ssr`.
        (0x114 => pub ssr: ReadWrite<u32>),
        /// Target Interrupt Enable Register. PAC type: `Sier`.
        (0x118 => pub sier: ReadWrite<u32>),
        /// Target DMA Enable Register. PAC type: `Sder`.
        (0x11c => pub sder: ReadWrite<u32>),
        (0x120 => _reserved6),
        /// Target Address Status Register. Read-only, and a read
        /// *consumes* the address-valid state (releasing an ADRSTALL
        /// stretch), so reading it is a protocol action. PAC type:
        /// `Sasr`.
        (0x150 => pub sasr: ReadOnly<u32>),
        (0x154 => _reserved7),
        /// Target Transmit Data Register. Write-only. PAC type: `Stdr`.
        (0x160 => pub stdr: WriteOnly<u32>),
        (0x164 => _reserved8),
        /// Target Receive Data Register. Read-only, and a read *pops*
        /// the RX FIFO. PAC type: `Srdr`.
        (0x170 => pub srdr: ReadOnly<u32>),
        (0x174 => @END),
    }
}
// END GENERATED: register_structs

/// View the LPI2C block behind a PAC handle through the Tock map.
pub(super) fn from_pac(regs: crate::pac::lpi2c::Lpi2c) -> &'static LpI2cRegisters {
    // SAFETY: `regs` wraps the peripheral's MMIO base address, valid for
    // the whole address span this struct covers; the offsets are pinned
    // by the assertions below and by `check_layout`, and every field is
    // a volatile accessor.
    unsafe { &*(regs.as_ptr() as *const LpI2cRegisters) }
}

/// Assert, against the PAC's *generated* accessors, that this map and
/// the PAC agree on every mapped register address.
///
/// The map and the `offset_of!` block below are generated by
/// `tools/gen_lpi2c_regs.py` from a PAC checkout — but the PAC this
/// binary LINKS can differ from the one the map was generated against
/// (a dependency bump without regeneration). This check compares
/// against the linked PAC's own accessor pointers at runtime, so that
/// drift trips a deterministic panic instead of corrupting MMIO.
/// Called once per driver construction; a handful of pointer
/// comparisons, active in release builds too.
///
/// Field *positions* need no equivalent check: they are never
/// transcribed here, only read through the PAC's own value types.
pub(super) fn check_layout(regs: crate::pac::lpi2c::Lpi2c) {
    let tock = from_pac(regs);

    macro_rules! check {
        ($accessor:ident, $field:ident) => {
            assert!(
                regs.$accessor().as_ptr() as usize == &tock.$field as *const _ as usize,
                concat!("lpi2c_regs layout drift vs PAC: ", stringify!($field))
            );
        };
    }

    // BEGIN GENERATED (gen_lpi2c_regs.py): layout checks
    check!(param, param);
    check!(mcr, mcr);
    check!(msr, msr);
    check!(mier, mier);
    check!(mder, mder);
    check!(mcfgr1, mcfgr1);
    check!(mfsr, mfsr);
    check!(mtdr, mtdr);
    check!(mrdr, mrdr);
    check!(scr, scr);
    check!(ssr, ssr);
    check!(sier, sier);
    check!(sder, sder);
    check!(sasr, sasr);
    check!(stdr, stdr);
    check!(srdr, srdr);
    // END GENERATED: layout checks
}

// BEGIN GENERATED (gen_lpi2c_regs.py): offset assertions
// Offsets generated from the PAC's own accessors
// (nxp-pac/src/meta_peripherals/mcxa/LPI2C.rs).
const _: () = {
    use core::mem::offset_of;
    assert!(offset_of!(LpI2cRegisters, param) == 0x004);
    assert!(offset_of!(LpI2cRegisters, mcr) == 0x010);
    assert!(offset_of!(LpI2cRegisters, msr) == 0x014);
    assert!(offset_of!(LpI2cRegisters, mier) == 0x018);
    assert!(offset_of!(LpI2cRegisters, mder) == 0x01c);
    assert!(offset_of!(LpI2cRegisters, mcfgr1) == 0x024);
    assert!(offset_of!(LpI2cRegisters, mfsr) == 0x05c);
    assert!(offset_of!(LpI2cRegisters, mtdr) == 0x060);
    assert!(offset_of!(LpI2cRegisters, mrdr) == 0x070);
    assert!(offset_of!(LpI2cRegisters, scr) == 0x110);
    assert!(offset_of!(LpI2cRegisters, ssr) == 0x114);
    assert!(offset_of!(LpI2cRegisters, sier) == 0x118);
    assert!(offset_of!(LpI2cRegisters, sder) == 0x11c);
    assert!(offset_of!(LpI2cRegisters, sasr) == 0x150);
    assert!(offset_of!(LpI2cRegisters, stdr) == 0x160);
    assert!(offset_of!(LpI2cRegisters, srdr) == 0x170);
};
// END GENERATED: offset assertions
