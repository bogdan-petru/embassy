//! Proof-only L2 binding. There is no physical MMIO, DMA, interrupt or thread.
//! L3 and its opaque L2 wrappers are the real path dependencies, not stubs.
//! Quiescence and wake registration are modeled premises, not hardware proofs.
use std::cell::RefCell;
use std::sync::Arc;
use std::task::{Context, Wake, Waker};

use i2c_one_shot_interface::{Command, Device, DmaResult, Error, ReceiveResult, Status, WakeOn};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Path {
    Cpu,
    Tx,
    Rx,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Decision {
    Pending,
    Complete,
    Failed(Error),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Event {
    Empty,
    Snapshot,
    Fifo(bool),
    Command(Command),
    ArmTx(usize),
    ArmRx(u8, usize),
    Observe(Path, WakeOn),
    Poll(Path, Decision),
    Quiesce,
    Drop,
}

pub struct State {
    pub status: Status,
    pub fifo_empty: bool,
    pub reject: Option<Error>,
    pub decision: Decision,
    pub active: bool,
    pub drops: usize,
    pub completions: usize,
    pub expected_id: u8,
    pub tx_identity: Option<(*const u8, usize)>,
    pub rx_identity: Option<(*const u8, usize)>,
    events: [Event; 32],
    count: usize,
}

impl State {
    pub fn new(status: Status) -> Self {
        Self {
            status,
            fifo_empty: true,
            reject: None,
            decision: Decision::Pending,
            active: false,
            drops: 0,
            completions: 0,
            expected_id: 73,
            tx_identity: None,
            rx_identity: None,
            events: [Event::Empty; 32],
            count: 0,
        }
    }

    fn record(&mut self, event: Event) {
        assert!(self.count < self.events.len(), "MODEL_EVENT_BOUND");
        self.events[self.count] = event;
        self.count += 1;
    }

    pub fn events(&self) -> &[Event] {
        &self.events[..self.count]
    }

    pub fn clear_events(&mut self) {
        self.count = 0;
    }
}

// A distinguishable safe waker. Context substitution with Waker::noop must
// fail the forwarding assertion; this does not model actual IRQ delivery.
struct Witness;
impl Wake for Witness {
    fn wake(self: Arc<Self>) {}
}
pub fn waker() -> Waker {
    Waker::from(Arc::new(Witness))
}

pub struct Model<'a> {
    pub state: &'a RefCell<State>,
    expected_waker: &'a Waker,
    id: u8,
}

impl<'a> Model<'a> {
    pub fn new(state: &'a RefCell<State>, expected_waker: &'a Waker) -> Self {
        Self {
            state,
            expected_waker,
            id: state.borrow().expected_id,
        }
    }

    fn context(&self, cx: &Context<'_>) {
        assert!(cx.waker().will_wake(self.expected_waker), "SUPPLIED_WAKER_FORWARDED");
        assert_eq!(self.id, self.state.borrow().expected_id, "SAME_LOWER_OWNER");
    }

    fn observe_path(&self, path: Path, cx: &Context<'_>, wake: WakeOn) -> Status {
        self.context(cx);
        let mut state = self.state.borrow_mut();
        state.record(Event::Observe(path, wake));
        state.status
    }

    fn complete(&self) {
        let mut state = self.state.borrow_mut();
        // This is the Device contract premise. No hardware is being quiesced.
        assert!(state.active);
        state.active = false;
        state.completions += 1;
        state.record(Event::Quiesce);
    }
}

impl Drop for Model<'_> {
    fn drop(&mut self) {
        let mut state = self.state.borrow_mut();
        if state.active {
            state.active = false;
            state.record(Event::Quiesce);
        }
        state.drops += 1;
        state.record(Event::Drop);
    }
}

pub struct Tx<'a> {
    device: Model<'a>,
    source: &'static [u8],
}
pub struct Rx<'a> {
    device: Model<'a>,
    bytes: &'static mut [u8],
}

// SAFETY: this proof-only binding owns no hardware and starts no background
// access. One non-Clone Model moves into each lease with immutable static TX
// or exclusive static RX storage. Completion marks modeled quiescence before
// returning storage; errors/Drop close modeled activity, while forgetting
// leaks authority/storage without future access. Every safe call order is
// memory-safe. These premises are NOT validation of the real unsafe HAL.
#[allow(unsafe_code)]
unsafe impl<'a> Device for Model<'a> {
    type InFlight = Tx<'a>;
    type Receiving = Rx<'a>;

    fn snapshot(&self) -> Status {
        let mut state = self.state.borrow_mut();
        state.record(Event::Snapshot);
        state.status
    }

    fn command(&mut self, command: Command) {
        assert!(!self.state.borrow().active);
        self.state.borrow_mut().record(Event::Command(command));
    }

    fn fifo_empty(&self) -> bool {
        let mut state = self.state.borrow_mut();
        let empty = state.fifo_empty;
        state.record(Event::Fifo(empty));
        empty
    }

    fn arm(self, source: &'static [u8]) -> Result<Tx<'a>, Error> {
        let reject = {
            let mut state = self.state.borrow_mut();
            state.record(Event::ArmTx(source.len()));
            state.tx_identity = Some((source.as_ptr(), source.len()));
            state.reject
        };
        if let Some(error) = reject {
            return Err(error);
        }
        self.state.borrow_mut().active = true;
        Ok(Tx { device: self, source })
    }

    fn observe(&mut self, cx: &mut Context<'_>, wake: WakeOn) -> Status {
        self.observe_path(Path::Cpu, cx, wake)
    }

    fn observe_dma(lease: &mut Tx<'a>, cx: &mut Context<'_>, wake: WakeOn) -> Status {
        lease.device.observe_path(Path::Tx, cx, wake)
    }

    fn poll_dma(lease: Tx<'a>, cx: &mut Context<'_>) -> DmaResult<Self> {
        lease.device.context(cx);
        let decision = {
            let mut state = lease.device.state.borrow_mut();
            assert!(state.active);
            assert_eq!(state.tx_identity, Some((lease.source.as_ptr(), lease.source.len())));
            let decision = state.decision;
            state.record(Event::Poll(Path::Tx, decision));
            decision
        };
        match decision {
            Decision::Pending => DmaResult::Pending(lease),
            Decision::Complete => {
                lease.device.complete();
                DmaResult::Complete(lease.device)
            }
            Decision::Failed(error) => {
                drop(lease);
                DmaResult::Failed(error)
            }
        }
    }

    fn arm_receive(self, address: u8, bytes: &'static mut [u8]) -> Result<Rx<'a>, Error> {
        let reject = {
            let mut state = self.state.borrow_mut();
            state.record(Event::ArmRx(address, bytes.len()));
            state.rx_identity = Some((bytes.as_ptr(), bytes.len()));
            if !(8..=0x77).contains(&address) || !(1..=32).contains(&bytes.len()) {
                Some(Error::InvalidInput)
            } else if !state.fifo_empty {
                Some(Error::Busy)
            } else {
                state.reject
            }
        };
        if let Some(error) = reject {
            return Err(error);
        }
        self.state.borrow_mut().active = true;
        Ok(Rx { device: self, bytes })
    }

    fn observe_receive(lease: &mut Rx<'a>, cx: &mut Context<'_>, wake: WakeOn) -> Status {
        lease.device.observe_path(Path::Rx, cx, wake)
    }

    fn poll_receive(lease: Rx<'a>, cx: &mut Context<'_>) -> ReceiveResult<Self> {
        lease.device.context(cx);
        let decision = {
            let mut state = lease.device.state.borrow_mut();
            assert!(state.active);
            assert_eq!(state.rx_identity, Some((lease.bytes.as_ptr(), lease.bytes.len())));
            let decision = state.decision;
            state.record(Event::Poll(Path::Rx, decision));
            decision
        };
        match decision {
            Decision::Pending => ReceiveResult::Pending(lease),
            Decision::Complete => {
                lease.device.complete();
                ReceiveResult::Complete((lease.device, lease.bytes))
            }
            Decision::Failed(error) => {
                drop(lease);
                ReceiveResult::Failed(error)
            }
        }
    }
}
