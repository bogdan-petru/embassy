#![forbid(unsafe_code)]

use core::future::Future;
use core::pin::pin;
use core::task::{Context, Poll, Waker};
use std::cell::Cell;
use std::collections::VecDeque;
use std::future::ready;

use nxp_shared_examples::uart::{ReceiveError, send, verify_receiver};

// Independent expectation: changing the shared payload must not silently
// change the peer oracle in this host suite.
const EXPECTED: [u8; 16] = *b"UART pilot DMA!\n";

fn one_poll<F: Future>(future: F) -> Poll<F::Output> {
    pin!(future).poll(&mut Context::from_waker(Waker::noop()))
}

#[test]
fn sender_invokes_one_operation_with_exact_static_bytes_and_returns_its_result() {
    for result in [Ok(()), Err(7_u8)] {
        let calls = Cell::new(0);
        assert_eq!(
            one_poll(send(|bytes| {
                calls.set(calls.get() + 1);
                assert_eq!(bytes, EXPECTED);
                ready(result)
            })),
            Poll::Ready(result)
        );
        assert_eq!(calls.get(), 1);
    }
}

#[test]
fn pending_sender_retains_the_owned_future_until_cancellation() {
    struct Pending<'a>(&'a Cell<usize>);
    impl Future for Pending<'_> {
        type Output = Result<(), ()>;
        fn poll(self: core::pin::Pin<&mut Self>, _: &mut Context<'_>) -> Poll<Self::Output> {
            Poll::Pending
        }
    }
    impl Drop for Pending<'_> {
        fn drop(&mut self) {
            self.0.set(self.0.get() + 1);
        }
    }
    let drops = Cell::new(0);
    {
        let mut future = pin!(send(|_| Pending(&drops)));
        assert_eq!(
            future.as_mut().poll(&mut Context::from_waker(Waker::noop())),
            Poll::Pending
        );
        assert_eq!(drops.get(), 0);
    }
    assert_eq!(drops.get(), 1);
}

fn clock(step: u64) -> impl FnMut() -> u64 {
    let mut time: u64 = 0;
    move || {
        let result = time;
        time = time.wrapping_add(step);
        result
    }
}

fn reads(bytes: impl IntoIterator<Item = u8>) -> VecDeque<Result<Option<u8>, &'static str>> {
    bytes.into_iter().map(|byte| Ok(Some(byte))).collect()
}

#[test]
fn receiver_requires_exact_payload_and_a_full_quiet_window() {
    let mut script = reads(EXPECTED);
    let calls = Cell::new(0);
    assert_eq!(
        verify_receiver(
            || {
                calls.set(calls.get() + 1);
                script.pop_front().unwrap_or(Ok(None))
            },
            clock(1_000)
        ),
        Ok(())
    );
    assert!(script.is_empty());
    // 16 payload samples, then 19 samples at +1 through +19 ms. At +20 ms
    // the quiet window is complete; the body does not claim later silence.
    assert_eq!(calls.get(), 35);
}

#[test]
fn fifo_empty_is_not_an_error_or_permission_to_skip_a_byte() {
    let mut script: VecDeque<_> = EXPECTED
        .into_iter()
        .flat_map(|byte| [Ok(None), Ok(Some(byte))])
        .collect();
    assert_eq!(
        verify_receiver(|| script.pop_front().unwrap_or(Ok::<_, &str>(None)), clock(1_000)),
        Ok(())
    );
}

#[test]
fn missing_and_short_payloads_fail_at_the_receive_deadline() {
    for length in [0, 1, 15] {
        let mut script = reads(EXPECTED[..length].iter().copied());
        let calls = Cell::new(0);
        assert_eq!(
            verify_receiver(
                || {
                    calls.set(calls.get() + 1);
                    script.pop_front().unwrap_or(Ok(None))
                },
                clock(1_000_000)
            ),
            Err(ReceiveError::Timeout { received: length })
        );
        assert_eq!(calls.get(), 19, "no read at or after the 20 s deadline");
    }
}

#[test]
fn a_byte_at_the_receive_deadline_is_not_accepted() {
    let mut script = reads(EXPECTED);
    let mut times: VecDeque<_> = [0, 20_000_000].into();
    assert_eq!(
        verify_receiver(|| script.pop_front().unwrap(), || times.pop_front().unwrap()),
        Err(ReceiveError::Timeout { received: 0 })
    );
    assert_eq!(script.len(), EXPECTED.len());
}

#[test]
fn every_wrong_payload_position_fails_without_consuming_the_suffix() {
    for index in 0..EXPECTED.len() {
        let mut bytes = EXPECTED;
        bytes[index] ^= 0xff;
        let mut script = reads(bytes);
        assert_eq!(
            verify_receiver(|| script.pop_front().unwrap_or(Ok(None)), clock(1_000)),
            Err(ReceiveError::Mismatch {
                index,
                expected: EXPECTED[index],
                actual: bytes[index]
            })
        );
        assert_eq!(script.len(), EXPECTED.len() - index - 1);
    }
}

#[test]
fn any_extra_byte_during_the_quiet_window_fails() {
    for empty_samples in [0, 18] {
        let mut script = reads(EXPECTED);
        script.extend(core::iter::repeat_n(Ok(None), empty_samples));
        script.push_back(Ok(Some(0xa5)));
        assert_eq!(
            verify_receiver(|| script.pop_front().unwrap_or(Ok(None)), clock(1_000)),
            Err(ReceiveError::ExtraByte(0xa5))
        );
    }
}

#[test]
fn every_peripheral_error_in_receive_or_quiet_phase_fails_without_retry() {
    for before_error in 0..=EXPECTED.len() {
        let mut script = reads(EXPECTED[..before_error].iter().copied());
        script.push_back(Err("framing"));
        script.push_back(Ok(Some(0xff)));
        assert_eq!(
            verify_receiver(|| script.pop_front().unwrap_or(Ok(None)), clock(1_000)),
            Err(ReceiveError::Peripheral("framing"))
        );
        assert_eq!(script.len(), 1);
    }
}

#[test]
fn receive_and_quiet_deadlines_survive_microsecond_clock_wrap() {
    let mut script = reads(EXPECTED);
    let mut time = u64::MAX - 4_000;
    assert_eq!(
        verify_receiver(
            || script.pop_front().unwrap_or(Ok(None)),
            || {
                let now = time;
                time = time.wrapping_add(1_000);
                now
            }
        ),
        Ok(())
    );
}
