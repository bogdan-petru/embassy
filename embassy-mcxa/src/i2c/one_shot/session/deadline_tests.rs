//! Test the REAL HAL deadline wrapper with controlled time and a pending
//! backend. These are wrapper decisions, not MMIO/DMA-effect evidence.
extern crate std;

use core::pin::Pin;
use core::task::{Context, Poll, Waker};
use std::boxed::Box;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

use i2c_one_shot_core::{DmaPoll, ReceivePoll, Status, WakeOn};

use super::*;

#[derive(Default)]
struct Model {
    now: Cell<u64>,
    dma_done: Cell<bool>,
    rx_done: Cell<bool>,
    read_room: Cell<bool>,
    read_started: Cell<bool>,
    stop_room: Cell<bool>,
    wire_complete: Cell<bool>,
    preparation_cost: Cell<u64>,
    drops: Cell<usize>,
    timer_polls: Cell<usize>,
    timer_waker: RefCell<Option<Waker>>,
    backend_waker: RefCell<Option<Waker>>,
}

impl Model {
    fn advance(&self, now: u64) {
        self.now.set(now);
        let waker = self.timer_waker.borrow_mut().take();
        if let Some(waker) = waker {
            waker.wake();
        }
        let waker = self.backend_waker.borrow_mut().take();
        if let Some(waker) = waker {
            waker.wake();
        }
    }
    fn observation(&self) -> Status {
        Status {
            stop_seen: self.wire_complete.get(),
            controller_idle: true,
            ..Status::default()
        }
    }
}

struct ClockDeadline {
    model: Rc<Model>,
    expires: u64,
}

impl Future for ClockDeadline {
    type Output = ();
    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<()> {
        self.model.timer_polls.set(self.model.timer_polls.get() + 1);
        if self.model.now.get() >= self.expires {
            Poll::Ready(())
        } else {
            *self.model.timer_waker.borrow_mut() = Some(cx.waker().clone());
            Poll::Pending
        }
    }
}

struct Mock(Rc<Model>);
struct Lease(Mock);
struct RxLease(Mock, &'static mut [u8]);

impl Drop for Mock {
    fn drop(&mut self) {
        self.0.drops.set(self.0.drops.get() + 1);
    }
}

// No physical MMIO or DMA; one owned Mock is retained through every
// transition and its static input never becomes borrowed transient storage.
// This safe service models protocol/deadline decisions, not silicon effects.
impl Backend for Mock {
    type Streaming = Lease;
    type Receiving = RxLease;
    fn start(self, _address: u8) -> Result<Self, Error> {
        self.0.now.set(self.0.now.get() + self.0.preparation_cost.get());
        Ok(self)
    }
    fn arm(self, _bytes: &'static [u8]) -> Result<Lease, Error> {
        Ok(Lease(self))
    }
    fn observe(&mut self, cx: &mut Context<'_>, _wake: WakeOn) -> Status {
        *self.0.backend_waker.borrow_mut() = Some(cx.waker().clone());
        self.0.observation()
    }
    fn observe_streaming(lease: &mut Lease, cx: &mut Context<'_>, _wake: WakeOn) -> Status {
        *lease.0.0.backend_waker.borrow_mut() = Some(cx.waker().clone());
        lease.0.0.observation()
    }
    fn poll_dma(lease: Lease, cx: &mut Context<'_>) -> DmaPoll<Self> {
        *lease.0.0.backend_waker.borrow_mut() = Some(cx.waker().clone());
        if lease.0.0.dma_done.get() {
            DmaPoll::Complete(lease.0)
        } else {
            DmaPoll::Pending(lease)
        }
    }
    fn try_stop(&mut self) -> Poll<Result<(), Error>> {
        if self.0.stop_room.get() {
            Poll::Ready(Ok(()))
        } else {
            Poll::Pending
        }
    }
    fn read_ready(&mut self, _address: u8, _length: u8) -> Poll<Result<(), Error>> {
        if self.0.read_room.get() {
            Poll::Ready(Ok(()))
        } else {
            Poll::Pending
        }
    }
    fn arm_receive(self, _address: u8, bytes: &'static mut [u8]) -> Result<RxLease, Error> {
        self.0.read_started.set(true);
        Ok(RxLease(self, bytes))
    }
    fn observe_receive(lease: &mut RxLease, cx: &mut Context<'_>, wake: WakeOn) -> Status {
        lease.0.observe(cx, wake)
    }
    fn poll_receive(lease: RxLease, cx: &mut Context<'_>) -> ReceivePoll<Self> {
        *lease.0.0.backend_waker.borrow_mut() = Some(cx.waker().clone());
        if lease.0.0.rx_done.get() {
            ReceivePoll::Complete((lease.0, lease.1))
        } else {
            ReceivePoll::Pending(lease)
        }
    }
}

#[test]
fn combined_transfer_has_one_deadline_across_tx_restart_rx_stop_and_wire() {
    // Every phase receives its own exclusive static buffer, including paths
    // that consume the reference on timeout without returning it.
    static RX_BUFFERS: static_cell::ConstStaticCell<[[u8; 4]; 5]> = static_cell::ConstStaticCell::new([[0; 4]; 5]);
    for (phase, read) in RX_BUFFERS.take().iter_mut().enumerate() {
        let model = Rc::new(Model::default());
        model.dma_done.set(phase >= 1);
        model.read_room.set(phase >= 2);
        model.rx_done.set(phase >= 3);
        model.stop_room.set(phase >= 4);
        let mut operation = Box::pin(with_deadline(
            write_read_transfer(Idle::new(Mock(model.clone())), 0x2a, &BYTES, read),
            ClockDeadline {
                model: model.clone(),
                expires: 100,
            },
        ));
        assert!(poll(&mut operation).is_pending());
        assert_eq!(model.read_started.get(), phase >= 2);
        model.advance(100);
        assert!(
            matches!(poll(&mut operation), Poll::Ready(Err(Error::Timeout))),
            "COMBINED_ONE_DEADLINE phase={phase}"
        );
        assert_eq!(model.drops.get(), 1, "phase={phase}");
    }
}

static BYTES: [u8; 3] = [1, 2, 3];

fn transfer(model: &Rc<Model>) -> impl Future<Output = Result<(), Error>> + use<> {
    with_deadline(
        write_transfer(Idle::new(Mock(model.clone())), 0x2a, &BYTES),
        ClockDeadline {
            model: model.clone(),
            expires: 100,
        },
    )
}

fn poll<F: Future>(future: &mut Pin<Box<F>>) -> Poll<F::Output> {
    future.as_mut().poll(&mut Context::from_waker(Waker::noop()))
}

#[test]
fn one_deadline_is_not_restarted_between_dma_stop_and_wire_waits() {
    let model = Rc::new(Model::default());
    let mut operation = Box::pin(transfer(&model));
    assert!(poll(&mut operation).is_pending());
    model.advance(90);
    model.dma_done.set(true);
    assert!(poll(&mut operation).is_pending());
    model.advance(99);
    model.stop_room.set(true);
    assert!(poll(&mut operation).is_pending());
    model.advance(100);
    assert_eq!(poll(&mut operation), Poll::Ready(Err(Error::Timeout)), "ONE_DEADLINE");
    assert_eq!(model.drops.get(), 1, "deadline must release the owned backend");
    assert_eq!(model.timer_polls.get(), 4);
}

#[test]
fn deadline_cancels_each_pending_phase_and_includes_preparation_elapsed_time() {
    for phase in 0..4 {
        let model = Rc::new(Model::default());
        model.dma_done.set(phase > 0 && phase < 3);
        model.stop_room.set(phase == 2);
        if phase == 3 {
            model.preparation_cost.set(120);
        }
        let mut operation = Box::pin(transfer(&model));
        if phase != 3 {
            assert!(poll(&mut operation).is_pending());
            model.advance(100);
        }
        assert_eq!(poll(&mut operation), Poll::Ready(Err(Error::Timeout)));
        assert_eq!(model.drops.get(), 1);
    }
}

#[test]
fn ready_wire_completion_keeps_existing_transfer_first_deadline_tie_break() {
    let model = Rc::new(Model::default());
    model.dma_done.set(true);
    model.stop_room.set(true);
    let mut operation = Box::pin(transfer(&model));
    assert!(poll(&mut operation).is_pending());
    model.advance(100);
    model.wire_complete.set(true);
    assert_eq!(poll(&mut operation), Poll::Ready(Ok(())));
    assert_eq!(model.drops.get(), 1);
}

#[test]
fn combined_completion_returns_owned_rx_before_a_simultaneously_ready_deadline() {
    static RX: static_cell::ConstStaticCell<[u8; 4]> = static_cell::ConstStaticCell::new([9, 8, 7, 6]);
    let model = Rc::new(Model::default());
    model.dma_done.set(true);
    model.read_room.set(true);
    model.rx_done.set(true);
    model.stop_room.set(true);
    let read = RX.take();
    let original_address = read.as_ptr();
    let mut operation = Box::pin(with_deadline(
        write_read_transfer(Idle::new(Mock(model.clone())), 0x2a, &BYTES, read),
        ClockDeadline {
            model: model.clone(),
            expires: 100,
        },
    ));
    assert!(poll(&mut operation).is_pending());
    assert_eq!(model.drops.get(), 0);
    let timer_polls = model.timer_polls.get();
    model.advance(100);
    model.wire_complete.set(true);
    let Poll::Ready(Ok(returned)) = poll(&mut operation) else {
        panic!("completed RX must win the existing transfer-first tie break");
    };
    assert_eq!(returned.as_ptr(), original_address);
    assert_eq!(returned, &[9, 8, 7, 6]);
    assert_eq!(
        model.timer_polls.get(),
        timer_polls,
        "ready transfer must be polled first"
    );
    assert_eq!(model.drops.get(), 1);
}
