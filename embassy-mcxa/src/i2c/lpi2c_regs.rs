//! Tock-style (`tock-registers`) register map for the LPI2C hot paths.
//!
//! This is a deliberate, second definition of the LPI2C register layout
//! next to `nxp-pac`, limited to the registers the transfer hot paths
//! touch. The PAC remains the layer for one-time configuration (clocks,
//! timing, addresses); this map exists so the safe wrappers on top of it
//! can use `tock-registers`' access typing:
//!
//! - `MRDR`/`SRDR` are [`ReadOnly`]: a read *pops* the RX FIFO, and the
//!   type makes accidental writes unrepresentable.
//! - `MTDR`/`STDR` are [`WriteOnly`]: reading them back is meaningless
//!   and now impossible.
//! - `MSR`/`SSR` writes are W1C; [`LocalRegisterCopy`] makes the
//!   read-once/write-back-the-same-snapshot pattern a first-class value.
//!
//! Layout drift is guarded twice: `offset_of!` assertions at the bottom
//! pin this map against transcribed offsets at compile time, and
//! [`check_layout`] compares every mapped register's address against the
//! PAC's *generated* accessors at driver init — so a regenerated PAC
//! with a changed layout (which the transcribed literals cannot see)
//! panics at first construction instead of corrupting MMIO.

use tock_registers::registers::{ReadOnly, ReadWrite, WriteOnly};
use tock_registers::{register_bitfields, register_structs};

register_structs! {
    /// LPI2C register block (hot-path subset; gaps are config registers
    /// still owned by the PAC).
    pub LpI2cRegisters {
        (0x000 => _reserved0),
        /// Parameter Register (FIFO sizes).
        (0x004 => pub param: ReadOnly<u32, PARAM::Register>),
        (0x008 => _reserved1),
        /// Controller Control Register.
        (0x010 => pub mcr: ReadWrite<u32, MCR::Register>),
        /// Controller Status Register (W1C flags).
        (0x014 => pub msr: ReadWrite<u32, MSR::Register>),
        /// Controller Interrupt Enable Register.
        (0x018 => pub mier: ReadWrite<u32, MIER::Register>),
        /// Controller DMA Enable Register.
        (0x01c => pub mder: ReadWrite<u32, MDER::Register>),
        (0x020 => _reserved2),
        /// Controller Configuration 1 (read here only for AUTOSTOP).
        (0x024 => pub mcfgr1: ReadWrite<u32, MCFGR1::Register>),
        (0x028 => _reserved3),
        /// Controller FIFO Status Register.
        (0x05c => pub mfsr: ReadOnly<u32, MFSR::Register>),
        /// Controller Transmit Data Register (command + data).
        (0x060 => pub mtdr: WriteOnly<u32, MTDR::Register>),
        (0x064 => _reserved4),
        /// Controller Receive Data Register (a read pops the RX FIFO).
        (0x070 => pub mrdr: ReadOnly<u32, MRDR::Register>),
        (0x074 => _reserved5),
        /// Target Control Register.
        (0x110 => pub scr: ReadWrite<u32, SCR::Register>),
        /// Target Status Register (W1C flags).
        (0x114 => pub ssr: ReadWrite<u32, SSR::Register>),
        /// Target Interrupt Enable Register.
        (0x118 => pub sier: ReadWrite<u32, SIER::Register>),
        (0x11c => _reserved6),
        /// Target Transmit Data Register.
        (0x160 => pub stdr: WriteOnly<u32, STDR::Register>),
        (0x164 => _reserved7),
        /// Target Receive Data Register (a read pops the RX FIFO).
        (0x170 => pub srdr: ReadOnly<u32, SRDR::Register>),
        (0x174 => @END),
    }
}

register_bitfields![u32,
    pub PARAM [
        MTXFIFO OFFSET(0) NUMBITS(4) [],
        MRXFIFO OFFSET(8) NUMBITS(4) [],
    ],
    pub MCR [
        MEN OFFSET(0) NUMBITS(1) [],
        RST OFFSET(1) NUMBITS(1) [],
        DOZEN OFFSET(2) NUMBITS(1) [],
        DBGEN OFFSET(3) NUMBITS(1) [],
        RTF OFFSET(8) NUMBITS(1) [],
        RRF OFFSET(9) NUMBITS(1) [],
    ],
    pub MSR [
        TDF OFFSET(0) NUMBITS(1) [],
        RDF OFFSET(1) NUMBITS(1) [],
        EPF OFFSET(8) NUMBITS(1) [],
        SDF OFFSET(9) NUMBITS(1) [],
        NDF OFFSET(10) NUMBITS(1) [],
        ALF OFFSET(11) NUMBITS(1) [],
        FEF OFFSET(12) NUMBITS(1) [],
        PLTF OFFSET(13) NUMBITS(1) [],
        DMF OFFSET(14) NUMBITS(1) [],
        STF OFFSET(15) NUMBITS(1) [],
        MBF OFFSET(24) NUMBITS(1) [],
        BBF OFFSET(25) NUMBITS(1) [],
    ],
    pub MIER [
        TDIE OFFSET(0) NUMBITS(1) [],
        RDIE OFFSET(1) NUMBITS(1) [],
        EPIE OFFSET(8) NUMBITS(1) [],
        SDIE OFFSET(9) NUMBITS(1) [],
        NDIE OFFSET(10) NUMBITS(1) [],
        ALIE OFFSET(11) NUMBITS(1) [],
        FEIE OFFSET(12) NUMBITS(1) [],
        PLTIE OFFSET(13) NUMBITS(1) [],
        DMIE OFFSET(14) NUMBITS(1) [],
        STIE OFFSET(15) NUMBITS(1) [],
    ],
    pub MDER [
        TDDE OFFSET(0) NUMBITS(1) [],
        RDDE OFFSET(1) NUMBITS(1) [],
    ],
    pub MCFGR1 [
        PRESCALE OFFSET(0) NUMBITS(3) [],
        AUTOSTOP OFFSET(8) NUMBITS(1) [],
        IGNACK OFFSET(9) NUMBITS(1) [],
    ],
    pub MFSR [
        TXCOUNT OFFSET(0) NUMBITS(3) [],
        RXCOUNT OFFSET(16) NUMBITS(3) [],
    ],
    pub MTDR [
        DATA OFFSET(0) NUMBITS(8) [],
        CMD OFFSET(8) NUMBITS(3) [
            Transmit = 0,
            Receive = 1,
            Stop = 2,
            ReceiveAndDiscard = 3,
            Start = 4,
            StartExpectNack = 5,
            StartHs = 6,
            StartHsExpectNack = 7,
        ],
    ],
    pub MRDR [
        DATA OFFSET(0) NUMBITS(8) [],
        RXEMPTY OFFSET(14) NUMBITS(1) [],
    ],
    pub SCR [
        SEN OFFSET(0) NUMBITS(1) [],
        RST OFFSET(1) NUMBITS(1) [],
        FILTEN OFFSET(4) NUMBITS(1) [],
        FILTDZ OFFSET(5) NUMBITS(1) [],
        RTF OFFSET(8) NUMBITS(1) [],
        RRF OFFSET(9) NUMBITS(1) [],
    ],
    pub SSR [
        TDF OFFSET(0) NUMBITS(1) [],
        RDF OFFSET(1) NUMBITS(1) [],
        AVF OFFSET(2) NUMBITS(1) [],
        TAF OFFSET(3) NUMBITS(1) [],
        RSF OFFSET(8) NUMBITS(1) [],
        SDF OFFSET(9) NUMBITS(1) [],
        BEF OFFSET(10) NUMBITS(1) [],
        FEF OFFSET(11) NUMBITS(1) [],
        AM0F OFFSET(12) NUMBITS(1) [],
        AM1F OFFSET(13) NUMBITS(1) [],
        GCF OFFSET(14) NUMBITS(1) [],
        SARF OFFSET(15) NUMBITS(1) [],
        SBF OFFSET(24) NUMBITS(1) [],
        BBF OFFSET(25) NUMBITS(1) [],
    ],
    pub SIER [
        TDIE OFFSET(0) NUMBITS(1) [],
        RDIE OFFSET(1) NUMBITS(1) [],
        AVIE OFFSET(2) NUMBITS(1) [],
        TAIE OFFSET(3) NUMBITS(1) [],
        RSIE OFFSET(8) NUMBITS(1) [],
        SDIE OFFSET(9) NUMBITS(1) [],
        BEIE OFFSET(10) NUMBITS(1) [],
        FEIE OFFSET(11) NUMBITS(1) [],
        AM0IE OFFSET(12) NUMBITS(1) [],
        AM1IE OFFSET(13) NUMBITS(1) [],
        GCIE OFFSET(14) NUMBITS(1) [],
        SARIE OFFSET(15) NUMBITS(1) [],
    ],
    pub STDR [
        DATA OFFSET(0) NUMBITS(8) [],
    ],
    pub SRDR [
        DATA OFFSET(0) NUMBITS(8) [],
        RADDR OFFSET(8) NUMBITS(3) [],
        RXEMPTY OFFSET(14) NUMBITS(1) [],
        SOF OFFSET(15) NUMBITS(1) [],
    ],
];

/// View the LPI2C block behind a PAC handle through the Tock map.
///
/// # Safety-relevant invariants (checked below)
/// The struct layout must match the hardware layout the PAC describes;
/// every offset is asserted against the value transcribed from
/// `nxp-pac`'s generated accessors.
pub(super) fn from_pac(regs: crate::pac::lpi2c::Lpi2c) -> &'static LpI2cRegisters {
    // SAFETY: `regs` wraps the peripheral's MMIO base address, valid for
    // the whole address space this struct spans; the offset assertions
    // below pin the layout, and all register types are volatile accessors.
    unsafe { &*(regs.as_ptr() as *const LpI2cRegisters) }
}

/// Assert, against the PAC's *generated* accessors, that this map and
/// the PAC agree on every mapped register address.
///
/// The `offset_of!` block below pins the Tock side against transcribed
/// offsets at compile time, but both sides of that check come from the
/// same manual transcription — it cannot catch the PAC itself drifting
/// (a regenerated PAC with a changed layout). This check compares
/// against the PAC's own accessor pointers, so drift in either layer
/// trips it. Called once per driver construction; a handful of pointer
/// comparisons, active in release builds too.
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
    check!(stdr, stdr);
    check!(srdr, srdr);
}

// Offsets transcribed from nxp-pac (nxp-pac/src/meta_peripherals/mcxa/LPI2C.rs).
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
    assert!(offset_of!(LpI2cRegisters, stdr) == 0x160);
    assert!(offset_of!(LpI2cRegisters, srdr) == 0x170);
};
