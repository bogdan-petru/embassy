//! Protocol decisions and retained ownership only. These mocks do not establish
//! MMIO correctness, real interrupt schedules, or HAL DMA quiescence ordering.

use std::cell::RefCell;
use std::future::Future;
use std::pin::Pin;
use std::rc::Rc;
use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::task::{Context, Poll, Wake, Waker};

use i2c_one_shot_core::{
    Backend, DmaPoll, Error, Idle, ReceivePoll, Status, Stopping, Streaming, TxDmaQuiescent, WakeOn,
};

static BYTES: [u8; 33] = [42; 33];

#[derive(Default)]
struct State {
    status: Status,
    dma_ready: bool,
    dma_error: bool,
    rx_ready: bool,
    rx_polls: usize,
    reads: Vec<(u8, u8)>,
    fifo_empty: bool,
    starts: Vec<u8>,
    arms: Vec<usize>,
    dma_polls: usize,
    stop_attempts: usize,
    drops: usize,
    waker: Option<Waker>,
}

#[derive(Default)]
struct WakeCount(AtomicUsize);
impl Wake for WakeCount {
    fn wake(self: Arc<Self>) {
        self.0.fetch_add(1, Ordering::Relaxed);
    }
    fn wake_by_ref(self: &Arc<Self>) {
        self.0.fetch_add(1, Ordering::Relaxed);
    }
}

#[derive(Default)]
struct Fixture {
    state: Rc<RefCell<State>>,
    wakes: Arc<WakeCount>,
}
impl Fixture {
    fn owner(&self) -> Idle<Mock> {
        Idle::new(Mock(self.state.clone()))
    }
    fn change(&self, update: impl FnOnce(&mut State)) {
        let wake = {
            let mut state = self.state.borrow_mut();
            update(&mut state);
            state.waker.clone()
        };
        if let Some(wake) = wake {
            wake.wake();
        }
    }
    fn poll<F: Future + ?Sized>(&self, future: &mut Pin<Box<F>>) -> Poll<F::Output> {
        let waker = Waker::from(self.wakes.clone());
        future.as_mut().poll(&mut Context::from_waker(&waker))
    }
}

struct Mock(Rc<RefCell<State>>);
struct Lease(Mock);
struct RxLease(Mock, &'static mut [u8]);
impl Drop for Mock {
    fn drop(&mut self) {
        self.0.borrow_mut().drops += 1;
    }
}

// No memory-mapped device or DMA exists in this fixture. The private
// lease owns the same single Mock on every pending poll, terminal failure drops
// it, completion returns it, and changed model observations wake a registered
// waker. This now implements a safe protocol service, not a hardware safety
// contract. Physical cleanup is checked separately below the real adapter.
impl Backend for Mock {
    type Streaming = Lease;
    type Receiving = RxLease;
    fn start(self, address: u8) -> Result<Self, Error> {
        self.0.borrow_mut().starts.push(address);
        Ok(self)
    }
    fn arm(self, bytes: &'static [u8]) -> Result<Lease, Error> {
        self.0.borrow_mut().arms.push(bytes.len());
        Ok(Lease(self))
    }
    fn observe(&mut self, cx: &mut Context<'_>, _: WakeOn) -> Status {
        let mut state = self.0.borrow_mut();
        state.waker = Some(cx.waker().clone());
        state.status
    }
    fn observe_streaming(lease: &mut Lease, cx: &mut Context<'_>, wake: WakeOn) -> Status {
        lease.0.observe(cx, wake)
    }
    fn poll_dma(lease: Lease, cx: &mut Context<'_>) -> DmaPoll<Self> {
        let (ready, error) = {
            let mut state = lease.0.0.borrow_mut();
            state.dma_polls += 1;
            state.waker = Some(cx.waker().clone());
            (state.dma_ready, state.dma_error)
        };
        if error {
            drop(lease);
            DmaPoll::Failed(Error::Dma)
        } else if ready {
            DmaPoll::Complete(lease.0)
        } else {
            DmaPoll::Pending(lease)
        }
    }
    fn try_stop(&mut self) -> Poll<Result<(), Error>> {
        let mut state = self.0.borrow_mut();
        state.stop_attempts += 1;
        if state.fifo_empty {
            Poll::Ready(Ok(()))
        } else {
            Poll::Pending
        }
    }
    fn read_ready(&mut self, _address: u8, _length: u8) -> Poll<Result<(), Error>> {
        let state = self.0.borrow();
        if !state.fifo_empty {
            return Poll::Pending;
        }
        Poll::Ready(Ok(()))
    }
    fn arm_receive(self, address: u8, bytes: &'static mut [u8]) -> Result<RxLease, Error> {
        self.0.borrow_mut().reads.push((address, bytes.len() as u8));
        Ok(RxLease(self, bytes))
    }
    fn observe_receive(lease: &mut RxLease, cx: &mut Context<'_>, wake: WakeOn) -> Status {
        lease.0.observe(cx, wake)
    }
    fn poll_receive(lease: RxLease, cx: &mut Context<'_>) -> ReceivePoll<Self> {
        let (ready, error) = {
            let mut state = lease.0.0.borrow_mut();
            state.rx_polls += 1;
            state.waker = Some(cx.waker().clone());
            (state.rx_ready, state.dma_error)
        };
        if error {
            ReceivePoll::Failed(Error::Dma)
        } else if ready {
            lease.1.copy_from_slice(&[0x31, 0x82, 0xc7, 0x59]);
            ReceivePoll::Complete((lease.0, lease.1))
        } else {
            ReceivePoll::Pending(lease)
        }
    }
}

#[test]
fn combined_read_keeps_memory_until_stop_and_never_requests_an_intermediate_stop() {
    let fixture = Fixture::default();
    let tx = queued(&fixture);
    let mut restart = Box::pin(tx.receive(0x2a, Box::leak(Box::new([0; 4]))));
    assert!(fixture.poll(&mut restart).is_pending());
    assert!(fixture.state.borrow().reads.is_empty());
    fixture.change(|state| state.fifo_empty = true);
    let receiving = ready(fixture.poll(&mut restart));
    assert_eq!(fixture.state.borrow().reads, [(0x2a, 4)]);
    assert_eq!(fixture.state.borrow().stop_attempts, 0);
    let mut drain = Box::pin(receiving.drain());
    assert!(fixture.poll(&mut drain).is_pending());
    fixture.change(|state| state.rx_ready = true);
    let received = ready(fixture.poll(&mut drain));
    let stopped = ready(fixture.poll(&mut Box::pin(received.stop())));
    assert_eq!(fixture.state.borrow().stop_attempts, 1);
    let mut wire = Box::pin(stopped.complete());
    assert!(fixture.poll(&mut wire).is_pending());
    fixture.change(|state| state.status.stop_seen = true);
    assert!(fixture.poll(&mut wire).is_pending());
    fixture.change(|state| state.status.controller_idle = true);
    let bytes = ready(fixture.poll(&mut wire));
    assert_eq!(
        fixture.state.borrow().drops,
        1,
        "backend released before RX memory is returned"
    );
    assert_eq!(bytes, &[0x31, 0x82, 0xc7, 0x59]);
    assert_eq!(fixture.state.borrow().drops, 1);
}

#[test]
fn receive_faults_and_unexpected_stop_precede_even_ready_dma() {
    for mask in 0..16 {
        for stop in [false, true] {
            let fixture = Fixture::default();
            let tx = queued(&fixture);
            fixture.change(|state| state.fifo_empty = true);
            let receiving = ready(fixture.poll(&mut Box::pin(tx.receive(0x2a, Box::leak(Box::new([0; 4]))))));
            fixture.change(|state| {
                state.status = observation(mask, stop, true);
                state.rx_ready = true;
            });
            let expected = fault(mask).or(stop.then_some(Error::UnexpectedStop));
            let mut drain = Box::pin(receiving.drain());
            match (fixture.poll(&mut drain), expected) {
                (Poll::Ready(Err(actual)), Some(expected)) => assert_eq!(actual, expected),
                (Poll::Ready(Ok(owner)), None) => drop(owner),
                _ => panic!("RX priority mismatch"),
            }
            drop(drain);
            assert_eq!(fixture.state.borrow().rx_polls, usize::from(expected.is_none()));
            assert_eq!(fixture.state.borrow().drops, 1);
        }
    }
}

#[test]
fn cancelling_repeated_start_or_receive_releases_the_same_owner() {
    for pending_rx in [false, true] {
        let fixture = Fixture::default();
        let tx = queued(&fixture);
        fixture.change(|state| state.fifo_empty = pending_rx);
        let mut restart = Box::pin(tx.receive(0x2a, Box::leak(Box::new([0; 4]))));
        if pending_rx {
            let receiving = ready(fixture.poll(&mut restart));
            let mut drain = Box::pin(receiving.drain());
            assert!(fixture.poll(&mut drain).is_pending());
            drop(drain);
        } else {
            assert!(fixture.poll(&mut restart).is_pending());
        }
        drop(restart);
        assert_eq!(fixture.state.borrow().drops, 1);
        assert_eq!(fixture.state.borrow().stop_attempts, 0);
    }
}

fn ready<T>(result: Poll<Result<T, Error>>) -> T {
    match result {
        Poll::Ready(Ok(value)) => value,
        Poll::Ready(Err(error)) => panic!("unexpected protocol error: {error:?}"),
        Poll::Pending => panic!("expected readiness"),
    }
}
fn streaming(fixture: &Fixture) -> Streaming<Mock> {
    fixture
        .owner()
        .start(0x2a)
        .unwrap_or_else(|error| panic!("{error:?}"))
        .transmit(&BYTES[..16])
        .unwrap_or_else(|error| panic!("{error:?}"))
}
fn queued(fixture: &Fixture) -> TxDmaQuiescent<Mock> {
    fixture.change(|state| state.dma_ready = true);
    ready(fixture.poll(&mut Box::pin(streaming(fixture).drain())))
}
fn stopping(fixture: &Fixture) -> Stopping<Mock> {
    fixture.change(|state| state.fifo_empty = true);
    ready(fixture.poll(&mut Box::pin(queued(fixture).stop())))
}
fn observation(mask: u8, stop_seen: bool, controller_idle: bool) -> Status {
    Status {
        pin_low_timeout: mask & 1 != 0,
        nack: mask & 2 != 0,
        arbitration_loss: mask & 4 != 0,
        fifo_error: mask & 8 != 0,
        stop_seen,
        controller_idle,
        bus_idle: false,
    }
}
fn fault(mask: u8) -> Option<Error> {
    let priority = [Error::PinLowTimeout, Error::Nack, Error::ArbitrationLoss, Error::Fifo];
    (mask != 0).then(|| priority[mask.trailing_zeros() as usize])
}

#[test]
fn every_fault_set_precedes_dma_and_unexpected_stop() {
    for mask in 0..16 {
        for stop in [false, true] {
            for idle in [false, true] {
                let fixture = Fixture::default();
                let live = streaming(&fixture);
                fixture.change(|state| {
                    state.status = observation(mask, stop, idle);
                    state.dma_ready = true;
                });
                let mut drain = Box::pin(live.drain());
                let expected = fault(mask).or(stop.then_some(Error::UnexpectedStop));
                match (fixture.poll(&mut drain), expected) {
                    (Poll::Ready(Err(actual)), Some(expected)) => assert_eq!(actual, expected),
                    (Poll::Ready(Ok(owner)), None) => drop(owner),
                    _ => panic!("fault/STOP priority mismatch: mask={mask}, stop={stop}, idle={idle}"),
                }
                drop(drain);
                assert_eq!(fixture.state.borrow().dma_polls, usize::from(expected.is_none()));
                assert_eq!(fixture.state.borrow().drops, 1);
            }
        }
    }
}

#[test]
fn every_fault_set_precedes_stop_enqueue_and_wire_completion() {
    for mask in 0..16 {
        for stop in [false, true] {
            for idle in [false, true] {
                let fixture = Fixture::default();
                let owner = queued(&fixture);
                fixture.change(|state| {
                    state.status = observation(mask, stop, idle);
                    state.fifo_empty = true;
                });
                let mut enqueue = Box::pin(owner.stop());
                let expected = fault(mask).or(stop.then_some(Error::UnexpectedStop));
                match (fixture.poll(&mut enqueue), expected) {
                    (Poll::Ready(Err(actual)), Some(expected)) => assert_eq!(actual, expected),
                    (Poll::Ready(Ok(owner)), None) => drop(owner),
                    _ => panic!("STOP enqueue mismatch: mask={mask}, stop={stop}, idle={idle}"),
                }
                drop(enqueue);
                assert_eq!(fixture.state.borrow().stop_attempts, usize::from(expected.is_none()));
                assert_eq!(fixture.state.borrow().drops, 1);

                let fixture = Fixture::default();
                let owner = stopping(&fixture);
                fixture.change(|state| state.status = observation(mask, stop, idle));
                let mut complete = Box::pin(owner.complete());
                match (fixture.poll(&mut complete), fault(mask)) {
                    (Poll::Ready(Err(actual)), Some(expected)) => assert_eq!(actual, expected),
                    (Poll::Ready(Ok(())), None) if stop && idle => assert_eq!(fixture.state.borrow().drops, 1),
                    (Poll::Pending, None) if !stop || !idle => assert_eq!(fixture.state.borrow().drops, 0),
                    _ => panic!("wire completion mismatch: mask={mask}, stop={stop}, idle={idle}"),
                }
                drop(complete);
                assert_eq!(fixture.state.borrow().drops, 1);
            }
        }
    }
}

#[test]
fn pending_retains_the_owner_and_does_not_invent_a_timeout() {
    let fixture = Fixture::default();
    let mut drain = Box::pin(streaming(&fixture).drain());
    for _ in 0..256 {
        assert!(fixture.poll(&mut drain).is_pending());
        assert_eq!(fixture.state.borrow().drops, 0);
    }
    fixture.change(|state| state.dma_ready = true);
    assert!(fixture.wakes.0.load(Ordering::Relaxed) > 0);
    let owner = ready(fixture.poll(&mut drain));
    drop(drain);
    let mut enqueue = Box::pin(owner.stop());
    assert!(fixture.poll(&mut enqueue).is_pending());
    fixture.change(|state| state.fifo_empty = true);
    let owner = ready(fixture.poll(&mut enqueue));
    drop(enqueue);
    let mut complete = Box::pin(owner.complete());
    for _ in 0..256 {
        assert!(
            fixture.poll(&mut complete).is_pending(),
            "DMA DONE is not wire completion or a timeout"
        );
    }
    fixture.change(|state| state.status = observation(0, true, true));
    ready(fixture.poll(&mut complete));
    assert_eq!(
        fixture.state.borrow().drops,
        1,
        "completion releases the backend before returning"
    );
    drop(complete);
    assert_eq!(fixture.state.borrow().drops, 1);
}

#[test]
fn wire_completion_uses_controller_idle_not_bus_idle() {
    for bus_idle in [false, true] {
        for stop_seen in [false, true] {
            for controller_idle in [false, true] {
                let fixture = Fixture::default();
                let owner = stopping(&fixture);
                fixture.change(|state| {
                    state.status = Status {
                        bus_idle,
                        ..observation(0, stop_seen, controller_idle)
                    };
                });
                let mut complete = Box::pin(owner.complete());
                if stop_seen && controller_idle {
                    ready(fixture.poll(&mut complete));
                    assert_eq!(fixture.state.borrow().drops, 1);
                } else {
                    assert!(fixture.poll(&mut complete).is_pending());
                    assert_eq!(fixture.state.borrow().drops, 0);
                }
                drop(complete);
                assert_eq!(fixture.state.borrow().drops, 1);
            }
        }
    }
}

#[test]
fn cancellation_at_each_pending_stage_and_dma_error_drop_exactly_once() {
    for phase in 0..3 {
        let fixture = Fixture::default();
        let mut pending: Pin<Box<dyn Future<Output = ()>>> = match phase {
            0 => {
                let owner = streaming(&fixture);
                Box::pin(async move {
                    let _ = owner.drain().await;
                })
            }
            1 => {
                let owner = queued(&fixture);
                Box::pin(async move {
                    let _ = owner.stop().await;
                })
            }
            _ => {
                let owner = stopping(&fixture);
                Box::pin(async move {
                    let _ = owner.complete().await;
                })
            }
        };
        assert!(fixture.poll(&mut pending).is_pending());
        assert_eq!(fixture.state.borrow().drops, 0);
        drop(pending);
        assert_eq!(fixture.state.borrow().drops, 1);
    }
    let fixture = Fixture::default();
    let mut drain = Box::pin(streaming(&fixture).drain());
    fixture.change(|state| {
        state.dma_ready = true;
        state.dma_error = true;
    });
    assert!(matches!(fixture.poll(&mut drain), Poll::Ready(Err(Error::Dma))));
    drop(drain);
    assert_eq!(fixture.state.borrow().drops, 1);
}

#[test]
fn invalid_input_never_reaches_the_corresponding_backend_operation() {
    for address in [0x00, 0x07, 0x08, 0x77, 0x78, 0xff] {
        let fixture = Fixture::default();
        let result = fixture.owner().start(address);
        let valid = (0x08..=0x77).contains(&address);
        assert_eq!(result.is_ok(), valid);
        if !valid {
            assert!(matches!(result, Err(Error::InvalidInput)));
        }
        drop(result);
        assert_eq!(fixture.state.borrow().starts.len(), usize::from(valid));
        assert_eq!(fixture.state.borrow().drops, 1);
    }
    for length in [0, 1, 32, 33] {
        let fixture = Fixture::default();
        let open = fixture.owner().start(0x2a).unwrap_or_else(|error| panic!("{error:?}"));
        let result = open.transmit(&BYTES[..length]);
        let valid = (1..=32).contains(&length);
        assert_eq!(result.is_ok(), valid);
        if !valid {
            assert!(matches!(result, Err(Error::InvalidInput)));
        }
        drop(result);
        assert_eq!(fixture.state.borrow().arms.len(), usize::from(valid));
        assert_eq!(fixture.state.borrow().drops, 1);
    }
}
