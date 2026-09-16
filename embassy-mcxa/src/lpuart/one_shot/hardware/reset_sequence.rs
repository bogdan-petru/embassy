//! Bounded hardware handshake, with an effect seam below the real algorithm.
//! These are local UART confirmations, not claims that a peer received data.
#![forbid(unsafe_code)]

use uart_one_shot_interface::Error;

use crate::pac::lpuart::{Baud, Ctrl, Dozeen, Global, Rst, Stat, Tc};
#[cfg(test)]
use crate::register_wait::HANDSHAKE_US;
use crate::register_wait::{self, Timing};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Register {
    Global,
    Baud,
    Ctrl,
    Stat,
}

pub(super) trait RegisterIo {
    fn read(&mut self, register: Register) -> u32;
    fn write(&mut self, register: Register, word: u32);
    fn now_us(&self) -> u64;
}

/// Validated IP timing, independent of the SoC's clock source and mux names.
/// The same immutable word follows CPU -> DMA -> CPU; request toggles cannot
/// accidentally reinstate the old fixed 12 MHz configuration.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct BaudConfig(u32);

impl BaudConfig {
    pub(super) fn new(clock_hz: u32) -> Result<Self, Error> {
        const RATE: u64 = 115_200;
        if clock_hz == 0 {
            return Err(Error::Clock);
        }
        let clock = u64::from(clock_hz);
        let mut best: Option<(u64, u64, u8, u16)> = None;
        for osr in 4_u8..=32 {
            let floor = clock / (RATE * u64::from(osr));
            for sbr in [floor, floor + 1] {
                if !(1..=8191).contains(&sbr) {
                    continue;
                }
                let denominator = u64::from(osr) * sbr;
                let error = clock.abs_diff(RATE * denominator);
                // Reject clocks that cannot represent 115200 within 3%.
                // Rational comparison avoids integer-baud truncation. Equal
                // errors prefer the higher oversampling ratio.
                if error * 100 <= RATE * denominator * 3
                    && best.is_none_or(|(old_error, old_denominator, _, _)| {
                        error * old_denominator <= old_error * denominator
                    })
                {
                    best = Some((error, denominator, osr, sbr as u16));
                }
            }
        }
        let (_, _, osr, sbr) = best.ok_or(Error::Clock)?;
        let mut word = Baud(0);
        word.set_osr(osr - 1);
        word.set_sbr(sbr);
        word.set_bothedge(osr < 8);
        Ok(Self(word.0))
    }

    pub(super) fn word(self, dma: bool) -> u32 {
        let mut word = Baud(self.0);
        word.set_tdmae(dma);
        word.0
    }
}

pub(super) fn control(transmit: bool, interrupt: bool) -> u32 {
    let mut word = Ctrl(0);
    word.set_dozeen(Dozeen::Enabled);
    word.set_te(transmit);
    word.set_tcie(interrupt);
    word.0
}

fn held() -> u32 {
    let mut word = Global(0);
    word.set_rst(Rst::Reset);
    word.0
}

fn confirm(io: &mut impl RegisterIo, register: Register, accepts: impl Fn(u32) -> bool) -> bool {
    register_wait::confirm(
        io,
        Timing::ObservationFirstElapsed,
        |io| accepts(io.read(register)),
        RegisterIo::now_us,
    )
}

pub(super) fn reset(io: &mut impl RegisterIo) -> Result<(), Error> {
    // DMA is already quiescent when entering this sequence. Abrupt software
    // reset deliberately avoids a graceful TE-off wait on transmitter progress.
    io.write(Register::Global, held());
    if !confirm(io, Register::Global, |word| Global(word).rst() == Rst::Reset) {
        io.write(Register::Global, held());
        return Err(Error::Reset);
    }
    io.write(Register::Global, 0);
    if !confirm(io, Register::Global, |word| Global(word).rst() == Rst::NoEffect)
        || !confirm(io, Register::Ctrl, |word| !Ctrl(word).te() && !Ctrl(word).re())
    {
        // Attempt fail-closed reset; unresponsive hardware cannot be promised
        // to have accepted this fallback. No usable state is returned.
        io.write(Register::Global, held());
        return Err(Error::Reset);
    }
    Ok(())
}

pub(super) fn enable(io: &mut impl RegisterIo, timing: BaudConfig) -> Result<(), Error> {
    io.write(Register::Baud, timing.word(false));
    io.write(Register::Ctrl, control(true, false));
    if confirm(io, Register::Baud, |word| word == timing.word(false))
        && confirm(io, Register::Ctrl, |word| word == control(true, false))
    {
        Ok(())
    } else {
        io.write(Register::Global, held());
        Err(Error::Reset)
    }
}

/// Even a failed peripheral acknowledgement must not skip eDMA quiescence.
pub(super) fn stop_dma(
    io: &mut impl RegisterIo,
    timing: BaudConfig,
    quiesce: impl FnOnce() -> bool,
) -> Result<(), Error> {
    io.write(Register::Baud, timing.word(false));
    let disabled = confirm(io, Register::Baud, |word| !Baud(word).tdmae());
    let _was_done = quiesce();
    if disabled {
        Ok(())
    } else {
        // Channel is quiescent before any reset or returning an error. This
        // owner must not be restored as a successful DMA completion.
        io.write(Register::Global, held());
        Err(Error::Reset)
    }
}

/// The caller has registered its real WaitCell before entering this helper.
/// TC is sampled only after peripheral/NVIC wake enable, preventing the final
/// character from disappearing between an early observation and sleep.
pub(super) fn wire_complete(io: &mut impl RegisterIo, enable_irq: impl FnOnce()) -> bool {
    io.write(Register::Ctrl, control(true, true));
    enable_irq();
    Stat(io.read(Register::Stat)).tc() == Tc::Complete
}

#[cfg(test)]
mod tests {
    extern crate std;

    use std::cell::RefCell;
    use std::collections::VecDeque;
    use std::vec::Vec;

    use super::*;

    #[derive(Debug, PartialEq, Eq)]
    enum Effect {
        Read(Register),
        Write(Register, u32),
        Quiesce,
        EnableIrq,
    }

    struct Io<'a> {
        reads: VecDeque<(Register, u32)>,
        log: &'a RefCell<Vec<Effect>>,
        tick: core::cell::Cell<u64>,
    }

    impl RegisterIo for Io<'_> {
        fn read(&mut self, register: Register) -> u32 {
            self.log.borrow_mut().push(Effect::Read(register));
            let (expected, value) = self.reads.pop_front().expect("unexpected UART read");
            assert_eq!(register, expected);
            value
        }
        fn write(&mut self, register: Register, word: u32) {
            self.log.borrow_mut().push(Effect::Write(register, word));
        }
        fn now_us(&self) -> u64 {
            let now = self.tick.get();
            self.tick.set(now + HANDSHAKE_US);
            now
        }
    }

    #[test]
    fn clock_selection_preserves_original_wire_rate_without_fixed_source() {
        for (clock, osr, sbr) in [(12_000_000, 26, 4), (6_000_000, 26, 2), (24_000_000, 26, 8)] {
            let timing = BaudConfig::new(clock).unwrap();
            let plain = Baud(timing.word(false));
            assert_eq!(u32::from(plain.osr()) + 1, osr);
            assert_eq!(u32::from(plain.sbr()), sbr);
            assert!(!plain.tdmae());
            assert_eq!(timing.word(true) ^ timing.word(false), 1 << 23);
            assert_eq!(clock / (osr * sbr), 115_384);
        }
    }

    #[test]
    fn invalid_or_unrepresentable_clock_is_rejected() {
        for clock in [0, 1, 16_000, 1_000_000] {
            assert_eq!(BaudConfig::new(clock), Err(Error::Clock));
        }
    }

    #[test]
    fn timing_matches_an_independent_exhaustive_divisor_oracle() {
        // The oracle enumerates ALL legal divisors, not the production
        // floor/ceil search. This checks selection, rounding and tie priority.
        for clock in [460_800, 921_600, 3_000_000, 6_000_000, 12_000_000, 48_000_000, u32::MAX] {
            let mut best = (u64::MAX, 1_u64, 0_u8, 0_u16);
            for osr in 4_u8..=32 {
                for sbr in 1_u16..=8191 {
                    let denominator = u64::from(osr) * u64::from(sbr);
                    let error = u64::from(clock).abs_diff(115_200 * denominator);
                    if best.2 == 0 || error * best.1 <= best.0 * denominator {
                        best = (error, denominator, osr, sbr);
                    }
                }
            }
            let timing = BaudConfig::new(clock).unwrap();
            let actual = Baud(timing.word(false));
            assert_eq!((actual.osr() + 1, actual.sbr()), (best.2, best.3));
            assert_eq!(actual.bothedge(), best.2 < 8);
        }
    }

    #[test]
    fn enable_and_dma_shutdown_preserve_nondefault_clock_timing() {
        let timing = BaudConfig::new(6_000_000).unwrap();
        assert_ne!(timing.word(false), BaudConfig::new(12_000_000).unwrap().word(false));
        let log = RefCell::new(Vec::new());
        let mut io = Io {
            reads: [
                (Register::Baud, timing.word(false)),
                (Register::Ctrl, control(true, false)),
                (Register::Baud, timing.word(false)),
            ]
            .into(),
            log: &log,
            tick: core::cell::Cell::new(0),
        };
        enable(&mut io, timing).unwrap();
        stop_dma(&mut io, timing, || {
            log.borrow_mut().push(Effect::Quiesce);
            true
        })
        .unwrap();
        let baud_writes: Vec<_> = log
            .borrow()
            .iter()
            .filter_map(|effect| {
                if let Effect::Write(Register::Baud, word) = effect {
                    Some(*word)
                } else {
                    None
                }
            })
            .collect();
        assert_eq!(baud_writes, [timing.word(false), timing.word(false)]);
        assert!(io.reads.is_empty());
    }

    #[test]
    fn reset_observes_assertion_release_and_disabled_before_granting_off() {
        let log = RefCell::new(Vec::new());
        let mut io = Io {
            reads: [(Register::Global, held()), (Register::Global, 0), (Register::Ctrl, 0)].into(),
            log: &log,
            tick: core::cell::Cell::new(0),
        };
        assert_eq!(reset(&mut io), Ok(()));
        assert!(io.reads.is_empty());
        // Three confirmations each read time only at entry. A deadline-first
        // policy would add a post-success clock read and double this count.
        assert_eq!(io.tick.get(), 3 * HANDSHAKE_US, "UART_OBSERVATION_BEFORE_DEADLINE");
        assert_eq!(
            *log.borrow(),
            [
                Effect::Write(Register::Global, held()),
                Effect::Read(Register::Global),
                Effect::Write(Register::Global, 0),
                Effect::Read(Register::Global),
                Effect::Read(Register::Ctrl),
            ]
        );
    }

    #[test]
    fn request_off_is_confirmed_before_quiescence_on_success_and_failure() {
        let timing = BaudConfig::new(12_000_000).unwrap();
        for (readback, expected) in [(timing.word(false), Ok(())), (timing.word(true), Err(Error::Reset))] {
            let log = RefCell::new(Vec::new());
            let mut io = Io {
                reads: [(Register::Baud, readback)].into(),
                log: &log,
                tick: core::cell::Cell::new(0),
            };
            assert_eq!(
                stop_dma(&mut io, timing, || {
                    log.borrow_mut().push(Effect::Quiesce);
                    false
                }),
                expected
            );
            assert_eq!(
                &log.borrow()[..3],
                [
                    Effect::Write(Register::Baud, timing.word(false)),
                    Effect::Read(Register::Baud),
                    Effect::Quiesce,
                ]
            );
            if expected.is_err() {
                assert_eq!(log.borrow().last(), Some(&Effect::Write(Register::Global, held())));
            }
        }
    }

    #[test]
    fn unconfirmed_reset_never_grants_a_usable_owner() {
        let log = RefCell::new(Vec::new());
        let mut io = Io {
            reads: [(Register::Global, 0)].into(),
            log: &log,
            tick: core::cell::Cell::new(0),
        };
        assert_eq!(reset(&mut io), Err(Error::Reset));
        assert_eq!(log.borrow().last(), Some(&Effect::Write(Register::Global, held())));
    }

    #[test]
    fn wire_completion_is_fresh_tc_after_interrupt_arm_not_dma_done() {
        for ready in [false, true] {
            let log = RefCell::new(Vec::new());
            let mut status = Stat(0);
            status.set_tc(if ready { Tc::Complete } else { Tc::Active });
            let mut io = Io {
                reads: [(Register::Stat, status.0)].into(),
                log: &log,
                tick: core::cell::Cell::new(0),
            };
            assert_eq!(
                wire_complete(&mut io, || log.borrow_mut().push(Effect::EnableIrq)),
                ready
            );
            assert_eq!(
                *log.borrow(),
                [
                    Effect::Write(Register::Ctrl, control(true, true)),
                    Effect::EnableIrq,
                    Effect::Read(Register::Stat),
                ]
            );
        }
    }
}
