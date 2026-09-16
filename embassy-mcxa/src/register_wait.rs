//! Bounded readback polling, not a reset recipe or hardware-state capability.
//! Policies preserve the existing I2C and UART boundary behavior deliberately:
//! changing when time is sampled can change whether a matching read succeeds.
#![forbid(unsafe_code)]

pub(crate) const HANDSHAKE_US: u64 = 1_000;
pub(crate) const MAX_POLLS: usize = 65_536;

#[derive(Clone, Copy)]
pub(crate) enum Timing {
    /// I2C: observe, then sample time; accept equality but reject late matches.
    /// Saturate the absolute deadline and spin only while still pending.
    SaturatingDeadline,
    /// UART: a matching observation wins before another clock read. Otherwise
    /// stop at the wrapping elapsed-time bound, without a spin hint.
    #[cfg(any(test, feature = "uart-one-shot"))]
    ObservationFirstElapsed,
}

/// Borrow one effect owner throughout polling. Callers retain the register
/// selection, acceptance predicate, reset sequence and failure handling.
pub(crate) fn confirm<I>(
    io: &mut I,
    timing: Timing,
    mut observe: impl FnMut(&mut I) -> bool,
    now_us: impl Fn(&I) -> u64,
) -> bool {
    let start = now_us(io);
    let deadline = start.saturating_add(HANDSHAKE_US);
    for _ in 0..MAX_POLLS {
        let observed = observe(io);
        match timing {
            Timing::SaturatingDeadline => {
                let now = now_us(io);
                if now > deadline {
                    return false;
                }
                if observed {
                    return true;
                }
                if now == deadline {
                    return false;
                }
                core::hint::spin_loop();
            }
            #[cfg(any(test, feature = "uart-one-shot"))]
            Timing::ObservationFirstElapsed => {
                if observed {
                    return true;
                }
                if now_us(io).wrapping_sub(start) >= HANDSHAKE_US {
                    return false;
                }
            }
        }
    }
    false
}

#[cfg(test)]
mod tests {
    extern crate std;

    use core::cell::{Cell, RefCell};
    use std::collections::VecDeque;
    use std::vec::Vec;

    use super::*;

    struct Script {
        times: RefCell<VecDeque<u64>>,
        observations: VecDeque<bool>,
        events: RefCell<Vec<&'static str>>,
    }

    impl Script {
        fn new(times: &[u64], observations: &[bool]) -> Self {
            Self {
                times: RefCell::new(times.iter().copied().collect()),
                observations: observations.iter().copied().collect(),
                events: RefCell::new(Vec::new()),
            }
        }

        fn observe(&mut self) -> bool {
            self.events.borrow_mut().push("read");
            self.observations.pop_front().expect("unexpected readback")
        }

        fn now(&self) -> u64 {
            self.events.borrow_mut().push("clock");
            self.times.borrow_mut().pop_front().expect("unexpected clock read")
        }

        fn run(&mut self, timing: Timing) -> bool {
            confirm(self, timing, Self::observe, Self::now)
        }
    }

    #[test]
    fn policies_keep_their_distinct_observation_and_clock_order() {
        let mut deadline = Script::new(&[10, 11, 12], &[false, true]);
        assert!(deadline.run(Timing::SaturatingDeadline));
        assert_eq!(*deadline.events.borrow(), ["clock", "read", "clock", "read", "clock"]);

        let mut observation = Script::new(&[10, 11], &[false, true]);
        assert!(observation.run(Timing::ObservationFirstElapsed));
        assert_eq!(*observation.events.borrow(), ["clock", "read", "clock", "read"]);
    }

    #[test]
    fn deadline_accepts_matching_equality_but_rejects_late_or_unmatched_reads() {
        for (observed, elapsed, expected) in [
            (true, 999, true),
            (true, 1_000, true),
            (true, 1_001, false),
            (false, 1_000, false),
            (false, 1_001, false),
        ] {
            let mut io = Script::new(&[7, 7 + elapsed], &[observed]);
            assert_eq!(io.run(Timing::SaturatingDeadline), expected);
            assert_eq!(*io.events.borrow(), ["clock", "read", "clock"]);
        }
    }

    #[test]
    fn absolute_deadline_saturates_instead_of_wrapping() {
        let mut io = Script::new(&[u64::MAX - 500, u64::MAX - 100, u64::MAX], &[false, true]);
        assert!(io.run(Timing::SaturatingDeadline));
        let mut at_ceiling = Script::new(&[u64::MAX, u64::MAX], &[false]);
        assert!(!at_ceiling.run(Timing::SaturatingDeadline));
    }

    #[test]
    fn observation_first_uses_wrapping_elapsed_time_only_after_failed_reads() {
        // From MAX-500 through zero, 498 is 999 us and 499 is 1000 us.
        let mut pending = Script::new(&[u64::MAX - 500, 498], &[false, true]);
        assert!(pending.run(Timing::ObservationFirstElapsed));
        let mut expired = Script::new(&[u64::MAX - 500, 499], &[false]);
        assert!(!expired.run(Timing::ObservationFirstElapsed));
        // No post-observation clock read is allowed on immediate success.
        let mut accepted = Script::new(&[u64::MAX], &[true]);
        assert!(accepted.run(Timing::ObservationFirstElapsed));
        assert_eq!(*accepted.events.borrow(), ["clock", "read"]);
    }

    #[test]
    fn frozen_clock_is_bounded_and_last_allowed_observation_can_succeed() {
        assert_eq!((HANDSHAKE_US, MAX_POLLS), (1_000, 65_536));
        for timing in [Timing::SaturatingDeadline, Timing::ObservationFirstElapsed] {
            for last_read_succeeds in [false, true] {
                let clocks = Cell::new(0);
                let mut reads = 0;
                let result = confirm(
                    &mut reads,
                    timing,
                    |reads| {
                        *reads += 1;
                        last_read_succeeds && *reads == MAX_POLLS
                    },
                    |_| {
                        clocks.set(clocks.get() + 1);
                        0
                    },
                );
                assert_eq!(result, last_read_succeeds);
                assert_eq!(reads, MAX_POLLS);
                let early_success = matches!(timing, Timing::ObservationFirstElapsed) && last_read_succeeds;
                assert_eq!(clocks.get(), MAX_POLLS + 1 - usize::from(early_success));
            }
        }
    }
}
