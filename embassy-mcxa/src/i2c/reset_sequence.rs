//! Local-engine reset ordering, shared by the private I2C facades.
//!
//! A graceful MEN-off can wait for a stretching peer. Abrupt RST instead
//! resets the local engine immediately (RM 42.3.1.5/42.7.1.4). Confirm RST
//! assertion and release, not graceful transfer completion, before restoring
//! configuration. This does not generate STOP or establish physical bus idle.

#![forbid(unsafe_code)]

use crate::pac::lpi2c::{Mcfgr0, Mcr, Scr};
use crate::register_wait::{self, Timing};
#[cfg(test)]
use crate::register_wait::{HANDSHAKE_US, MAX_POLLS};

/// The adapter owns the mapping; no FIFO/data/status operation is exposed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Register {
    Control,
    InterruptEnable,
    DmaEnable,
    /// Controller: MCFGR0..3, MDMR, MCCR0..1, MFCR (indices 0..8).
    /// Target: SCFGR1, SCFGR2, SAMR (indices 0..3).
    Config(u8),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Role {
    Controller,
    Target,
}

/// Register-level effects, not a mock of the reset decision itself.
///
/// Implementations must keep this mapping bound to one owned peripheral and
/// role. Before entering `reset`, callers must quiesce any borrowed-buffer DMA
/// channel: masking peripheral requests alone does not stop an active minor loop.
pub(super) trait RegisterIo {
    fn read(&mut self, register: Register) -> u32;
    fn write(&mut self, register: Register, value: u32);
    fn now_us(&self) -> u64;
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum ResetError {
    Assert,
    Release,
    Configure,
    Enable,
}

impl Role {
    fn config_count(self) -> u8 {
        match self {
            Self::Controller => 8,
            Self::Target => 3,
        }
    }

    fn mode_word(self, current: u32) -> u32 {
        match self {
            Self::Controller => {
                let current = Mcr(current);
                let mut word = Mcr(0);
                word.set_dozen(current.dozen());
                word.set_dbgen(current.dbgen());
                word.0
            }
            Self::Target => {
                let current = Scr(current);
                let mut word = Scr(0);
                word.set_filtdz(current.filtdz());
                word.set_filten(current.filten());
                word.0
            }
        }
    }

    fn control_word(self, mode: u32, reset: bool, enable: bool) -> u32 {
        match self {
            Self::Controller => {
                let mut word = Mcr(mode);
                word.set_rst(reset);
                word.set_men(enable);
                word.0
            }
            Self::Target => {
                let mut word = Scr(mode);
                word.set_rst(reset);
                word.set_sen(enable);
                word.0
            }
        }
    }

    fn matches_control(self, word: u32, reset: bool, enable: bool) -> bool {
        match self {
            Self::Controller => Mcr(word).rst() == reset && Mcr(word).men() == enable,
            Self::Target => Scr(word).rst() == reset && Scr(word).sen() == enable,
        }
    }

    fn persistent_config(self, index: u8, word: u32) -> u32 {
        if self == Self::Controller && index == 0 {
            let mut word = Mcfgr0(word);
            // ABORT belongs to the discarded transaction, not configuration.
            word.set_abort(false);
            word.0
        } else {
            word
        }
    }
}

fn confirm(io: &mut impl RegisterIo, role: Role, mode: u32, reset: bool, enable: bool) -> bool {
    register_wait::confirm(
        io,
        Timing::SaturatingDeadline,
        |io| {
            let word = io.read(Register::Control);
            role.matches_control(word, reset, enable) && role.mode_word(word) == mode
        },
        RegisterIo::now_us,
    )
}

/// Evidence of reset released and the local engine disabled, not wire idle.
///
/// The mutable borrow binds `enable` to the same backend; no detached receipt
/// can enable another instance. The private adapter's role/register mapping is
/// still trusted. Dropping this value does not re-enable the engine.
#[must_use]
pub(super) struct Disabled<'a, I: RegisterIo> {
    io: &'a mut I,
    role: Role,
    mode: u32,
}

impl<I: RegisterIo> Disabled<'_, I> {
    /// Change only persistent control modes while disabled. The next enable
    /// uses this same confirmed word, not the pre-configuration snapshot.
    pub(super) fn configure_mode(mut self, requested: u32) -> Result<Self, ResetError> {
        self.mode = self.role.mode_word(requested);
        self.io
            .write(Register::Control, self.role.control_word(self.mode, false, false));
        if confirm(self.io, self.role, self.mode, false, false) {
            Ok(self)
        } else {
            self.io
                .write(Register::Control, self.role.control_word(self.mode, true, false));
            Err(ResetError::Configure)
        }
    }

    pub(super) fn enable(self) -> Result<(), ResetError> {
        self.io
            .write(Register::Control, self.role.control_word(self.mode, false, true));
        if confirm(self.io, self.role, self.mode, false, true) {
            Ok(())
        } else {
            self.io
                .write(Register::Control, self.role.control_word(self.mode, true, false));
            Err(ResetError::Enable)
        }
    }
}

/// Revalidate an already-disabled owner before its separately configured enable.
/// This is used by the one-shot's state-specific boundary: it does not reset
/// away the timing words that were written while Off, nor wait for a peer.
#[cfg(feature = "i2c-one-shot")]
pub(super) fn confirm_disabled<I: RegisterIo>(io: &mut I, role: Role) -> Result<Disabled<'_, I>, ResetError> {
    let mode = role.mode_word(io.read(Register::Control));
    if confirm(io, role, mode, false, false) {
        Ok(Disabled { io, role, mode })
    } else {
        io.write(Register::Control, role.control_word(mode, true, false));
        Err(ResetError::Configure)
    }
}

/// Abruptly reset one role, optionally restoring its static configuration.
///
/// DMA channels must already be quiescent. This function also masks peripheral
/// IRQ/DMA requests and never restores those masks. All control writes are
/// complete words derived from stable mode bits, never live-control RMWs.
/// On any handshake failure the final attempted write holds RST with enable=0;
/// no disabled evidence is returned and no configuration is restored. A caller
/// must not resolve fault ownership or report successful recovery on that path.
///
/// Config preservation does not change BUSIDLE, generate STOP, or guarantee
/// that the next transfer can start after the peer releases its lines.
pub(super) fn reset<I: RegisterIo>(io: &mut I, role: Role, preserve: bool) -> Result<Disabled<'_, I>, ResetError> {
    io.write(Register::InterruptEnable, 0);
    io.write(Register::DmaEnable, 0);
    let mode = role.mode_word(io.read(Register::Control));
    let mut config = [0; 8];
    if preserve {
        for index in 0..role.config_count() {
            config[index as usize] = role.persistent_config(index, io.read(Register::Config(index)));
        }
    }

    let held = role.control_word(mode, true, false);
    io.write(Register::Control, held);
    if !confirm(io, role, mode, true, false) {
        io.write(Register::Control, held);
        return Err(ResetError::Assert);
    }
    io.write(Register::Control, role.control_word(mode, false, false));
    if !confirm(io, role, mode, false, false) {
        io.write(Register::Control, held);
        return Err(ResetError::Release);
    }

    if preserve {
        for index in 0..role.config_count() {
            io.write(Register::Config(index), config[index as usize]);
        }
    }
    Ok(Disabled { io, role, mode })
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    enum Event {
        Read(Register, u32),
        Write(Register, u32),
    }

    struct Fake {
        role: Role,
        control: u32,
        config: [u32; 8],
        interrupt: u32,
        dma: u32,
        delays: [usize; 3],
        control_writes: usize,
        pending: Option<(u32, usize)>,
        time: u64,
        tick: u64,
        log: std::vec::Vec<Event>,
    }

    impl Fake {
        fn new(role: Role, delays: [usize; 3]) -> Self {
            let mode = match role {
                Role::Controller => 0x0c,
                Role::Target => 0x30,
            };
            let mut config = core::array::from_fn(|i| 0xa500_00ff + i as u32);
            if role == Role::Controller {
                let mut first = Mcfgr0(config[0]);
                first.set_abort(true);
                config[0] = first.0;
            }
            Self {
                role,
                // Deliberately includes transient/reserved bits. None may be
                // recopied into a reset or enable word.
                control: mode | 1 | 0x300 | 0x8000_0000,
                config,
                interrupt: 0xffff,
                dma: 3,
                delays,
                control_writes: 0,
                pending: None,
                time: 0,
                tick: 1,
                log: std::vec::Vec::new(),
            }
        }

        fn mode(&self) -> u32 {
            match self.role {
                Role::Controller => 0x0c,
                Role::Target => 0x30,
            }
        }

        fn writes(&self) -> std::vec::Vec<(Register, u32)> {
            self.log
                .iter()
                .filter_map(|e| match e {
                    Event::Write(r, v) => Some((*r, *v)),
                    _ => None,
                })
                .collect()
        }

        fn assert_failed_closed(&self) {
            assert_eq!(self.interrupt, 0);
            assert_eq!(self.dma, 0);
            assert_eq!(self.writes().last(), Some(&(Register::Control, self.mode() | 2)));
            assert!(
                !self
                    .log
                    .iter()
                    .any(|e| matches!(e, Event::Write(Register::Config(_), _)))
            );
            assert!(
                !self
                    .log
                    .iter()
                    .any(|e| matches!(e, Event::Write(Register::Control, v) if v & 1 != 0))
            );
        }
    }

    impl RegisterIo for Fake {
        fn read(&mut self, register: Register) -> u32 {
            self.time = self.time.saturating_add(self.tick);
            let value = match register {
                Register::Control => {
                    if let Some((value, remaining)) = self.pending.as_mut() {
                        if *remaining == 0 {
                            self.control = *value;
                            self.pending = None;
                        } else {
                            *remaining -= 1;
                        }
                    }
                    self.control
                }
                Register::InterruptEnable => self.interrupt,
                Register::DmaEnable => self.dma,
                Register::Config(index) => self.config[index as usize],
            };
            self.log.push(Event::Read(register, value));
            value
        }

        fn write(&mut self, register: Register, value: u32) {
            self.log.push(Event::Write(register, value));
            match register {
                Register::Control => {
                    let delay = self.delays.get(self.control_writes).copied().unwrap_or(0);
                    self.control_writes += 1;
                    self.pending = Some((value, delay));
                    if value & 2 != 0 {
                        // RST really destroys configuration in this fixture;
                        // preserving the pre-reset shadow cannot pass by luck.
                        self.config.fill(0);
                        self.interrupt = 0;
                        self.dma = 0;
                    }
                }
                Register::InterruptEnable => self.interrupt = value,
                Register::DmaEnable => self.dma = value,
                Register::Config(index) => self.config[index as usize] = value,
            }
        }

        fn now_us(&self) -> u64 {
            self.time
        }
    }

    #[test]
    fn reset_confirmation_precedes_release_and_restore_remove_wait_regression() {
        for role in [Role::Controller, Role::Target] {
            let mut io = Fake::new(role, [3, 4, 0]);
            let old = io.control;
            let mode = io.mode();
            drop(reset(&mut io, role, true).unwrap());

            let first_control = io
                .log
                .iter()
                .position(|e| matches!(e, Event::Write(Register::Control, _)))
                .unwrap();
            let mut expected = std::vec![Event::Write(Register::Control, mode | 2)];
            expected.extend([Event::Read(Register::Control, old); 3]);
            expected.push(Event::Read(Register::Control, mode | 2));
            expected.push(Event::Write(Register::Control, mode));
            expected.extend([Event::Read(Register::Control, mode | 2); 4]);
            expected.push(Event::Read(Register::Control, mode));
            assert_eq!(
                &io.log[first_control..first_control + expected.len()],
                expected,
                "RESET_CONFIRMATION_BEFORE_DEPENDENT_EFFECT"
            );
            assert!(matches!(
                io.log[first_control + expected.len()],
                Event::Write(Register::Config(0), _)
            ));
            assert_eq!(io.control, mode);
        }
    }

    #[test]
    fn reset_preserves_nonzero_configuration_but_not_abort_or_request_masks() {
        for role in [Role::Controller, Role::Target] {
            let mut io = Fake::new(role, [0; 3]);
            let mut expected = io.config;
            if role == Role::Controller {
                let mut first = Mcfgr0(expected[0]);
                assert!(first.abort());
                first.set_abort(false);
                expected[0] = first.0;
            }
            reset(&mut io, role, true).unwrap().enable().unwrap();
            let count = role.config_count() as usize;
            assert_eq!(&io.config[..count], &expected[..count]);
            assert_eq!((io.interrupt, io.dma), (0, 0));
            assert_eq!(io.control, io.mode() | 1);
            assert_eq!(
                &io.writes()[..2],
                &[(Register::InterruptEnable, 0), (Register::DmaEnable, 0)]
            );
            assert!(
                io.writes()
                    .iter()
                    .all(|(r, v)| *r != Register::Control || v & !0x3f == 0)
            );
        }
    }

    #[test]
    fn reset_discards_configuration_when_requested_and_drop_does_not_enable() {
        for role in [Role::Controller, Role::Target] {
            let mut io = Fake::new(role, [0; 3]);
            drop(reset(&mut io, role, false).unwrap());
            assert_eq!(io.config, [0; 8]);
            assert_eq!(io.control, io.mode());
            assert!(!io.log.iter().any(|e| matches!(
                e,
                Event::Read(Register::Config(_), _) | Event::Write(Register::Config(_), _)
            )));
            assert_eq!(io.control_writes, 2);
        }
    }

    #[test]
    fn reset_assertion_timeout_never_restores_or_enables() {
        for role in [Role::Controller, Role::Target] {
            let mut io = Fake::new(role, [usize::MAX, 0, 0]);
            let start = io.time;
            assert!(matches!(reset(&mut io, role, true), Err(ResetError::Assert)));
            io.assert_failed_closed();
            // Snapshot reads precede, and are not part of, the handshake.
            assert_eq!(io.time - start, 1 + u64::from(role.config_count()) + HANDSHAKE_US);
        }
    }

    #[test]
    fn reset_release_timeout_reasserts_reset_without_restoration() {
        for role in [Role::Controller, Role::Target] {
            let mut io = Fake::new(role, [0, usize::MAX, 0]);
            assert!(matches!(reset(&mut io, role, true), Err(ResetError::Release)));
            io.assert_failed_closed();
            assert_eq!(io.control_writes, 3);
        }
    }

    #[test]
    fn reset_confirmation_is_bounded_even_if_time_stops() {
        let mut io = Fake::new(Role::Controller, [usize::MAX, 0, 0]);
        io.tick = 0;
        assert!(matches!(
            reset(&mut io, Role::Controller, false),
            Err(ResetError::Assert)
        ));
        io.assert_failed_closed();
        let reads = io
            .log
            .iter()
            .filter(|e| matches!(e, Event::Read(Register::Control, _)))
            .count();
        assert_eq!(reads, 1 + MAX_POLLS);
        assert_eq!(io.time, 0);
    }

    #[test]
    fn reset_recipe_accepts_deadline_equality_but_rejects_late_matching_readback() {
        for role in [Role::Controller, Role::Target] {
            let mut at_deadline = Fake::new(role, [0; 3]);
            at_deadline.tick = HANDSHAKE_US;
            drop(reset(&mut at_deadline, role, false).unwrap());
            assert_eq!(at_deadline.control, at_deadline.mode());

            let mut late = Fake::new(role, [0; 3]);
            late.tick = HANDSHAKE_US + 1;
            assert!(matches!(reset(&mut late, role, false), Err(ResetError::Assert)));
            late.assert_failed_closed();
        }
    }

    #[test]
    fn enable_uses_retained_mode_word_and_waits_for_actual_confirmation() {
        for role in [Role::Controller, Role::Target] {
            let mut io = Fake::new(role, [0, 0, 3]);
            let mode = io.mode();
            reset(&mut io, role, true).unwrap().enable().unwrap();
            assert_eq!(
                &io.log[io.log.len() - 5..],
                &[
                    Event::Write(Register::Control, mode | 1),
                    Event::Read(Register::Control, mode),
                    Event::Read(Register::Control, mode),
                    Event::Read(Register::Control, mode),
                    Event::Read(Register::Control, mode | 1),
                ]
            );
        }
    }

    #[test]
    fn failed_enable_finishes_with_reset_held_not_a_live_engine() {
        for role in [Role::Controller, Role::Target] {
            let mut io = Fake::new(role, [0, 0, usize::MAX]);
            assert_eq!(reset(&mut io, role, true).unwrap().enable(), Err(ResetError::Enable));
            assert_eq!(io.writes().last(), Some(&(Register::Control, io.mode() | 2)));
            assert_eq!(io.config, [0; 8]);
            assert_eq!((io.interrupt, io.dma), (0, 0));
        }
    }

    #[test]
    fn configured_modes_survive_enable_without_restoring_the_old_modes() {
        for role in [Role::Controller, Role::Target] {
            let mut io = Fake::new(role, [0, 0, 3]);
            let requested = match role {
                Role::Controller => 0,
                Role::Target => {
                    let mut scr = Scr(0);
                    scr.set_filtdz(crate::pac::lpi2c::Filtdz::FilterDisabled);
                    scr.0
                }
            };
            reset(&mut io, role, false)
                .unwrap()
                // Even supplied transient/reserved bits are not authority.
                .configure_mode(requested | 0x8000_0303)
                .unwrap()
                .enable()
                .unwrap();
            assert_eq!(io.control, requested | 1);
            assert_eq!(io.writes().last(), Some(&(Register::Control, requested | 1)));
        }
    }
}
