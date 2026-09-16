#![forbid(unsafe_code)]

use core::future::Future;
use core::pin::pin;
use core::task::{Context, Poll, Waker};
use std::cell::Cell;
use std::collections::VecDeque;
use std::future::ready;

use nxp_shared_examples::i2c::{self, CheckError, ReadEnd, Request, TargetPeer, WriteEnd};

// Independently maintained wire expectations; never imported from the body.
const MESSAGE: [u8; 16] = *b"Hello from DMA!\n";
const RESPONSE: [u8; 8] = [0x31, 0x82, 0xc7, 0x59, 0xa1, 0xb2, 0xc3, 0xd4];

fn one_poll<F: Future>(future: F) -> Poll<F::Output> {
    pin!(future).poll(&mut Context::from_waker(Waker::noop()))
}

#[test]
fn controller_write_invokes_exactly_one_static_operation_without_retry() {
    for result in [Ok(()), Err("nack")] {
        let calls = Cell::new(0);
        assert_eq!(
            one_poll(i2c::write(Box::leak(Box::new([0; 16])), |address, bytes| {
                calls.set(calls.get() + 1);
                assert_eq!(address, 0x2a);
                assert_eq!(bytes, MESSAGE);
                ready(result)
            })),
            Poll::Ready(result)
        );
        assert_eq!(calls.get(), 1);
    }
}

#[test]
fn controller_combined_request_is_one_register_write_and_four_byte_read() {
    let calls = Cell::new(0);
    assert_eq!(
        one_poll(i2c::write_read(
            Box::leak(Box::new([0; 4])),
            |address, register, receive| {
                calls.set(calls.get() + 1);
                assert_eq!(address, 0x2a);
                assert_eq!(register, &[0x10]);
                assert_eq!(receive, &[0; 4]);
                receive.copy_from_slice(&RESPONSE[..4]);
                ready(Ok::<_, &str>(receive))
            }
        )),
        Poll::Ready(Ok(()))
    );
    assert_eq!(calls.get(), 1);
}

#[test]
fn controller_inspects_only_successfully_returned_receive_bytes() {
    for index in 0..4 {
        assert_eq!(
            one_poll(i2c::write_read(Box::leak(Box::new([0; 4])), |_, _, receive| {
                receive.copy_from_slice(&RESPONSE[..4]);
                receive[index] ^= 0xff;
                ready(Ok::<_, &str>(receive))
            })),
            Poll::Ready(Err(CheckError::DataMismatch))
        );
    }
    // Even matching bytes cannot turn a driver error into a successful check.
    assert_eq!(
        one_poll(i2c::write_read(Box::leak(Box::new([0; 4])), |_, _, receive| {
            receive.copy_from_slice(&RESPONSE[..4]);
            ready(Err::<&'static mut [u8], _>("timeout"))
        })),
        Poll::Ready(Err(CheckError::Driver("timeout")))
    );
}

#[test]
fn controller_rejects_a_returned_slice_with_wrong_length() {
    assert_eq!(
        one_poll(i2c::write_read(Box::leak(Box::new([0; 4])), |_, _, _| {
            ready(Ok::<&'static mut [u8], &str>(Box::leak(Box::new(RESPONSE))))
        })),
        Poll::Ready(Err(CheckError::DataMismatch))
    );
}

#[test]
fn pending_combined_future_is_retained_then_dropped_on_cancellation() {
    struct Pending<'a>(&'a Cell<usize>, &'static mut [u8]);
    impl Future for Pending<'_> {
        type Output = Result<&'static mut [u8], &'static str>;
        fn poll(self: core::pin::Pin<&mut Self>, _: &mut Context<'_>) -> Poll<Self::Output> {
            assert_eq!(self.1.len(), 4);
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
        let mut future = pin!(i2c::write_read(Box::leak(Box::new([0; 4])), |_, _, rx| Pending(
            &drops, rx
        )));
        assert_eq!(
            future.as_mut().poll(&mut Context::from_waker(Waker::noop())),
            Poll::Pending
        );
        assert_eq!(drops.get(), 0);
    }
    assert_eq!(drops.get(), 1);
}

enum Step {
    Listen(Result<Request, &'static str>),
    Receive {
        capacity: usize,
        bytes: Vec<u8>,
        end: Result<WriteEnd, &'static str>,
    },
    Reply(Result<ReadEnd, &'static str>),
}

struct Peer(VecDeque<Step>);

impl TargetPeer for Peer {
    type Error = &'static str;

    fn listen(&mut self) -> impl Future<Output = Result<Request, Self::Error>> {
        let Step::Listen(result) = self.0.pop_front().expect("no unexpected listen/retry") else {
            panic!("wrong operation order: listen");
        };
        ready(result)
    }

    fn receive<'a>(&'a mut self, buffer: &'a mut [u8]) -> impl Future<Output = Result<WriteEnd, Self::Error>> + 'a {
        let Step::Receive { capacity, bytes, end } = self.0.pop_front().expect("no unexpected receive/retry") else {
            panic!("wrong operation order: receive");
        };
        assert_eq!(
            buffer.len(),
            capacity,
            "spare byte must keep receive pending until a bus event"
        );
        buffer[..bytes.len()].copy_from_slice(&bytes);
        ready(end)
    }

    fn reply<'a>(&'a mut self, bytes: &'a [u8]) -> impl Future<Output = Result<ReadEnd, Self::Error>> + 'a {
        let Step::Reply(end) = self.0.pop_front().expect("no unexpected reply/retry") else {
            panic!("wrong operation order: reply");
        };
        assert_eq!(bytes, RESPONSE, "spare suffix must prevent DMA-DONE-only completion");
        ready(end)
    }
}

fn write_peer(end: WriteEnd, bytes: Vec<u8>) -> Peer {
    Peer(
        [
            Step::Listen(Ok(Request::Write(0x2a))),
            Step::Receive {
                capacity: 17,
                bytes,
                end: Ok(end),
            },
        ]
        .into(),
    )
}

fn combined_peer(write_end: WriteEnd, read_end: ReadEnd) -> Peer {
    Peer(
        [
            Step::Listen(Ok(Request::Write(0x2a))),
            Step::Receive {
                capacity: 2,
                bytes: vec![0x10],
                end: Ok(write_end),
            },
            Step::Listen(Ok(Request::Read(0x2a))),
            Step::Reply(Ok(read_end)),
        ]
        .into(),
    )
}

#[test]
fn write_peer_requires_exact_bytes_and_stop_not_dma_fullness() {
    let mut peer = write_peer(WriteEnd::Stopped(16), MESSAGE.to_vec());
    assert_eq!(one_poll(i2c::receive_write(&mut peer)), Poll::Ready(Ok(())));
    assert!(peer.0.is_empty());
    for end in [
        WriteEnd::Stopped(15),
        WriteEnd::Stopped(17),
        WriteEnd::Restarted(16),
        WriteEnd::BufferFull(16),
        WriteEnd::Other,
    ] {
        let mut peer = write_peer(end, MESSAGE.to_vec());
        assert_eq!(
            one_poll(i2c::receive_write(&mut peer)),
            Poll::Ready(Err(CheckError::WriteEnd(end)))
        );
    }
}

#[test]
fn write_peer_rejects_every_wrong_byte() {
    for index in 0..MESSAGE.len() {
        let mut bytes = MESSAGE;
        bytes[index] ^= 0xff;
        let mut peer = write_peer(WriteEnd::Stopped(16), bytes.to_vec());
        assert_eq!(
            one_poll(i2c::receive_write(&mut peer)),
            Poll::Ready(Err(CheckError::DataMismatch))
        );
    }
}

#[test]
fn both_target_bodies_reject_wrong_initial_address_or_direction() {
    for actual in [Request::Write(0x2b), Request::Read(0x2a), Request::Other] {
        let mut peer = Peer([Step::Listen(Ok(actual))].into());
        assert_eq!(
            one_poll(i2c::receive_write(&mut peer)),
            Poll::Ready(Err(CheckError::Request(actual)))
        );
        let mut peer = Peer([Step::Listen(Ok(actual))].into());
        assert_eq!(
            one_poll(i2c::serve_write_read(&mut peer)),
            Poll::Ready(Err(CheckError::Request(actual)))
        );
    }
}

#[test]
fn combined_peer_requires_repeated_start_before_reply_even_when_data_matches() {
    for end in [
        WriteEnd::Stopped(1),
        WriteEnd::Restarted(0),
        WriteEnd::Restarted(2),
        WriteEnd::BufferFull(1),
        WriteEnd::Other,
    ] {
        let mut peer = combined_peer(end, ReadEnd::EarlyStop(4));
        assert_eq!(
            one_poll(i2c::serve_write_read(&mut peer)),
            Poll::Ready(Err(CheckError::WriteEnd(end)))
        );
        assert_eq!(peer.0.len(), 2, "no read/reply after a missing repeated START");
    }
}

#[test]
fn combined_peer_rejects_wrong_register_or_second_request() {
    let mut peer = combined_peer(WriteEnd::Restarted(1), ReadEnd::EarlyStop(4));
    let Step::Receive { bytes, .. } = &mut peer.0[1] else {
        unreachable!()
    };
    bytes[0] = 0x11;
    assert_eq!(
        one_poll(i2c::serve_write_read(&mut peer)),
        Poll::Ready(Err(CheckError::DataMismatch))
    );
    assert_eq!(peer.0.len(), 2);
    for actual in [Request::Read(0x2b), Request::Write(0x2a), Request::Other] {
        let mut peer = combined_peer(WriteEnd::Restarted(1), ReadEnd::EarlyStop(4));
        peer.0[2] = Step::Listen(Ok(actual));
        assert_eq!(
            one_poll(i2c::serve_write_read(&mut peer)),
            Poll::Ready(Err(CheckError::Request(actual)))
        );
        assert_eq!(peer.0.len(), 1);
    }
}

#[test]
fn target_reply_accepts_only_prior_characterized_early_stop_counts() {
    for count in [4, 5] {
        let mut peer = combined_peer(WriteEnd::Restarted(1), ReadEnd::EarlyStop(count));
        assert_eq!(one_poll(i2c::serve_write_read(&mut peer)), Poll::Ready(Ok(())));
        assert!(peer.0.is_empty());
    }
    for end in [
        ReadEnd::Complete(4),
        ReadEnd::NeedMore(4),
        ReadEnd::EarlyStop(0),
        ReadEnd::EarlyStop(3),
        ReadEnd::EarlyStop(6),
        ReadEnd::EarlyStop(8),
        ReadEnd::Other,
    ] {
        let mut peer = combined_peer(WriteEnd::Restarted(1), end);
        assert_eq!(
            one_poll(i2c::serve_write_read(&mut peer)),
            Poll::Ready(Err(CheckError::ReadEnd(end)))
        );
    }
}

#[test]
fn every_target_driver_error_is_terminal_without_retry_or_next_phase() {
    for stage in 0..4 {
        let mut peer = combined_peer(WriteEnd::Restarted(1), ReadEnd::EarlyStop(4));
        peer.0[stage] = match stage {
            0 | 2 => Step::Listen(Err("fault")),
            1 => Step::Receive {
                capacity: 2,
                bytes: vec![],
                end: Err("fault"),
            },
            _ => Step::Reply(Err("fault")),
        };
        assert_eq!(
            one_poll(i2c::serve_write_read(&mut peer)),
            Poll::Ready(Err(CheckError::Driver("fault")))
        );
        assert_eq!(peer.0.len(), 3 - stage);
    }
    for stage in 0..2 {
        let mut peer = write_peer(WriteEnd::Stopped(16), MESSAGE.to_vec());
        peer.0[stage] = match stage {
            0 => Step::Listen(Err("fault")),
            _ => Step::Receive {
                capacity: 17,
                bytes: vec![],
                end: Err("fault"),
            },
        };
        assert_eq!(
            one_poll(i2c::receive_write(&mut peer)),
            Poll::Ready(Err(CheckError::Driver("fault")))
        );
        assert_eq!(peer.0.len(), 1 - stage);
    }
}
