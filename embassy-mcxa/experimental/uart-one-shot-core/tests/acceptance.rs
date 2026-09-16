//! Protocol decisions over a safe fake. Physical DMA, MMIO, and interrupt
//! correctness require the independent HAL effect tests and two-board pilot.
#![forbid(unsafe_code)]

use std::cell::RefCell;
use std::future::Future;
use std::pin::Pin;
use std::rc::Rc;
use std::task::{Context, Poll, Waker};

use uart_one_shot_core::{Backend, DmaPoll, Error, run_write};

static MESSAGE: [u8; 16] = *b"UART pilot DMA!\n";
static BOUNDARY_BYTES: [u8; 33] = [0x5a; 33];

#[derive(Clone, Debug, PartialEq, Eq)]
enum Event {
    Arm(usize),
    DmaPoll,
    DmaQuiescent,
    WirePoll,
    Drop,
}

#[derive(Default)]
struct State {
    dma_done: bool,
    dma_error: bool,
    dma_quiescent: bool,
    tc: bool,
    arm_error: Option<Error>,
    wire_error: Option<Error>,
    events: Vec<Event>,
}

#[derive(Default)]
struct Fixture(Rc<RefCell<State>>);

impl Fixture {
    fn owner(&self) -> Fake {
        Fake(self.0.clone())
    }

    fn events(&self) -> Vec<Event> {
        self.0.borrow().events.clone()
    }

    fn drops(&self) -> usize {
        self.0
            .borrow()
            .events
            .iter()
            .filter(|event| **event == Event::Drop)
            .count()
    }
}

struct Fake(Rc<RefCell<State>>);
struct Lease {
    owner: Fake,
    _source: &'static [u8],
}

impl Drop for Fake {
    fn drop(&mut self) {
        self.0.borrow_mut().events.push(Event::Drop);
    }
}

// There is one retained Fake in either the CPU or DMA state. This implements
// only the safe service interface; no hardware safety contract is asserted.
impl Backend for Fake {
    type Streaming = Lease;

    fn arm(self, bytes: &'static [u8]) -> Result<Lease, Error> {
        let error = {
            let mut state = self.0.borrow_mut();
            state.events.push(Event::Arm(bytes.len()));
            state.arm_error
        };
        match error {
            Some(error) => Err(error),
            None => Ok(Lease {
                owner: self,
                _source: bytes,
            }),
        }
    }

    fn poll_dma(lease: Lease, _cx: &mut Context<'_>) -> DmaPoll<Self> {
        let (done, error) = {
            let mut state = lease.owner.0.borrow_mut();
            state.events.push(Event::DmaPoll);
            (state.dma_done, state.dma_error)
        };
        if error {
            DmaPoll::Failed(Error::Dma)
        } else if done {
            {
                let mut state = lease.owner.0.borrow_mut();
                state.dma_quiescent = true;
                state.events.push(Event::DmaQuiescent);
            }
            DmaPoll::Complete(lease.owner)
        } else {
            DmaPoll::Pending(lease)
        }
    }

    fn poll_wire(&mut self, _cx: &mut Context<'_>) -> Poll<Result<(), Error>> {
        let mut state = self.0.borrow_mut();
        assert!(state.dma_quiescent, "UART_NO_TC_POLL_BEFORE_DMA_QUIESCENCE");
        state.events.push(Event::WirePoll);
        if let Some(error) = state.wire_error {
            Poll::Ready(Err(error))
        } else if state.tc {
            Poll::Ready(Ok(()))
        } else {
            Poll::Pending
        }
    }
}

fn poll<F: Future>(future: &mut Pin<Box<F>>) -> Poll<F::Output> {
    future.as_mut().poll(&mut Context::from_waker(Waker::noop()))
}

#[test]
fn dma_done_is_not_transmission_complete() {
    let fixture = Fixture::default();
    let mut transfer = Box::pin(run_write(fixture.owner(), &MESSAGE));
    assert!(poll(&mut transfer).is_pending());
    fixture.0.borrow_mut().dma_done = true;
    assert!(poll(&mut transfer).is_pending(), "UART_DMA_DONE_IS_NOT_TC");
    assert!(fixture.events().contains(&Event::DmaQuiescent));
    assert_eq!(fixture.drops(), 0, "UART_PENDING_WIRE_RETAINS_OWNER");
    fixture.0.borrow_mut().tc = true;
    assert_eq!(poll(&mut transfer), Poll::Ready(Ok(())));
    assert_eq!(fixture.drops(), 1);
}

#[test]
fn transmission_complete_is_not_polled_until_dma_quiescence() {
    let fixture = Fixture::default();
    fixture.0.borrow_mut().tc = true;
    let mut transfer = Box::pin(run_write(fixture.owner(), &MESSAGE));
    for _ in 0..3 {
        assert!(poll(&mut transfer).is_pending());
        assert!(!fixture.events().contains(&Event::WirePoll));
    }
    fixture.0.borrow_mut().dma_done = true;
    assert_eq!(poll(&mut transfer), Poll::Ready(Ok(())));
    let events = fixture.events();
    let quiescent = events.iter().position(|event| *event == Event::DmaQuiescent).unwrap();
    let wire = events.iter().position(|event| *event == Event::WirePoll).unwrap();
    assert!(quiescent < wire);
}

#[test]
fn cancellation_in_each_pending_phase_drops_the_same_owner_once() {
    for dma_done in [false, true] {
        let fixture = Fixture::default();
        fixture.0.borrow_mut().dma_done = dma_done;
        let mut transfer = Box::pin(run_write(fixture.owner(), &MESSAGE));
        assert!(poll(&mut transfer).is_pending());
        assert_eq!(fixture.events().contains(&Event::WirePoll), dma_done);
        assert_eq!(fixture.drops(), 0);
        drop(transfer);
        assert_eq!(fixture.drops(), 1, "UART_CANCELLATION_RETAINS_CLEANUP");
        assert_eq!(fixture.events().last(), Some(&Event::Drop));
    }
}

#[test]
fn dropping_an_unpolled_transfer_releases_owner_without_arming() {
    let fixture = Fixture::default();
    let transfer = Box::pin(run_write(fixture.owner(), &MESSAGE));
    drop(transfer);
    assert_eq!(fixture.events(), [Event::Drop]);
}

#[test]
fn invalid_lengths_never_arm_dma() {
    for bytes in [&BOUNDARY_BYTES[..0], &BOUNDARY_BYTES[..]] {
        let fixture = Fixture::default();
        let mut transfer = Box::pin(run_write(fixture.owner(), bytes));
        assert_eq!(poll(&mut transfer), Poll::Ready(Err(Error::InvalidInput)));
        assert_eq!(fixture.events(), [Event::Drop]);
        drop(transfer);
        assert_eq!(fixture.drops(), 1);
    }
}

#[test]
fn boundary_lengths_arm_and_complete() {
    for bytes in [&BOUNDARY_BYTES[..1], &BOUNDARY_BYTES[..32]] {
        let fixture = Fixture::default();
        fixture.0.borrow_mut().dma_done = true;
        fixture.0.borrow_mut().tc = true;
        let mut transfer = Box::pin(run_write(fixture.owner(), bytes));
        assert_eq!(poll(&mut transfer), Poll::Ready(Ok(())));
        assert_eq!(fixture.events().first(), Some(&Event::Arm(bytes.len())));
        assert_eq!(fixture.drops(), 1);
    }
}

#[test]
fn arm_dma_and_wire_errors_never_report_success() {
    for stage in 0..3 {
        let fixture = Fixture::default();
        let expected = {
            let mut state = fixture.0.borrow_mut();
            state.dma_done = true;
            state.tc = true;
            match stage {
                0 => {
                    state.arm_error = Some(Error::Clock);
                    Error::Clock
                }
                1 => {
                    state.dma_error = true;
                    Error::Dma
                }
                _ => {
                    state.wire_error = Some(Error::Reset);
                    Error::Reset
                }
            }
        };
        let mut transfer = Box::pin(run_write(fixture.owner(), &MESSAGE));
        assert_eq!(poll(&mut transfer), Poll::Ready(Err(expected)));
        assert_eq!(fixture.drops(), 1);
        if stage < 2 {
            assert!(!fixture.events().contains(&Event::WirePoll));
        }
        if stage == 0 {
            assert!(!fixture.events().contains(&Event::DmaPoll));
        }
        drop(transfer);
        assert_eq!(fixture.drops(), 1);
    }
}

#[test]
fn successful_return_drops_owner_only_after_tc_observation() {
    let fixture = Fixture::default();
    fixture.0.borrow_mut().dma_done = true;
    let mut transfer = Box::pin(run_write(fixture.owner(), &MESSAGE));
    assert!(poll(&mut transfer).is_pending());
    assert_eq!(fixture.drops(), 0);
    fixture.0.borrow_mut().tc = true;
    assert_eq!(poll(&mut transfer), Poll::Ready(Ok(())));
    assert!(fixture.events().ends_with(&[Event::WirePoll, Event::Drop]));
    assert_eq!(fixture.drops(), 1);
    drop(transfer);
    assert_eq!(fixture.drops(), 1);
}
