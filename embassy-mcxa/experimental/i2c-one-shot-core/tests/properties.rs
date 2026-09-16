//! Sequence/model tests of the real public protocol, not a simulated MMIO driver.
//! The oracle is a small bus-service specification; it never calls `checked` or
//! `validate_transfer`. Backend logs describe calls, NOT physical DMA effects.
//! Random traces currently cover writes. RX ownership has separate deterministic
//! and effect tests: cancelling its static buffer loan cannot safely recycle that
//! buffer here, and test volume must not require leaks or forged static borrows.
//! Manual polling checks the requested wake class, not actual wake registration,
//! scheduling, lost interrupts, DMA quiescence, or cleanup after `mem::forget`.
#![forbid(unsafe_code)]

use std::cell::RefCell;
use std::future::Future;
use std::rc::Rc;
use std::task::{Context, Poll, Waker};

use i2c_one_shot_core::{Backend, DmaPoll, Error, Idle, ReceivePoll, Status, WakeOn};
use proptest::prelude::*;
use proptest::test_runner::{Config, FileFailurePersistence};

static BYTES: [u8; 33] = [0x53; 33];

fn config() -> Config {
    let mut config = Config::default();
    // Sidecar seeds are replayed on subsequent runs; commit new regressions.
    // Preserve the defaults' environment overrides, including explicit disable
    // during mutation campaigns (mutated failures must not alter replay input).
    if config.failure_persistence.is_some() {
        config.failure_persistence = Some(Box::new(FileFailurePersistence::WithSource("proptest-regressions")));
    }
    config
}

#[derive(Clone, Copy, Debug, Default)]
struct Frame {
    faults: u8,
    stop: bool,
    idle: bool,
    dma: Reply,
    command: Reply,
}

#[derive(Clone, Copy, Debug, Default)]
enum Reply {
    #[default]
    Pending,
    Complete,
    Failed(Error),
}

#[derive(Clone, Copy, Debug)]
enum Event {
    Wait,
    Dma(Reply),
    Fifo(Reply),
    Wire { stop: bool, idle: bool },
    Fault(u8),
    Snapshot(Frame),
    Cancel,
}

impl Frame {
    fn apply(&mut self, event: Event) {
        match event {
            Event::Wait | Event::Cancel => (),
            Event::Dma(reply) => self.dma = reply,
            Event::Fifo(reply) => self.command = reply,
            Event::Wire { stop, idle } => (self.stop, self.idle) = (stop, idle),
            Event::Fault(faults) => self.faults = faults,
            Event::Snapshot(frame) => *self = frame,
        }
    }

    fn observation(self) -> Status {
        Status {
            pin_low_timeout: self.faults & 1 != 0,
            nack: self.faults & 2 != 0,
            arbitration_loss: self.faults & 4 != 0,
            fifo_error: self.faults & 8 != 0,
            stop_seen: self.stop,
            controller_idle: self.idle,
            bus_idle: false,
        }
    }
}

fn error() -> impl Strategy<Value = Error> {
    prop::sample::select(vec![
        Error::InvalidInput,
        Error::Clock,
        Error::Busy,
        Error::Nack,
        Error::ArbitrationLoss,
        Error::Fifo,
        Error::PinLowTimeout,
        Error::Dma,
        Error::Timeout,
        Error::UnexpectedStop,
    ])
}

fn reply() -> impl Strategy<Value = Reply> {
    prop_oneof![4 => Just(Reply::Pending), 4 => Just(Reply::Complete), 1 => error().prop_map(Reply::Failed)]
}

fn event() -> impl Strategy<Value = Event> {
    prop_oneof![
        6 => Just(Event::Wait),
        5 => reply().prop_map(Event::Dma),
        5 => reply().prop_map(Event::Fifo),
        4 => (any::<bool>(), any::<bool>()).prop_map(|(stop, idle)| Event::Wire { stop, idle }),
        1 => (0u8..16).prop_map(Event::Fault),
        1 => (0u8..16, any::<bool>(), any::<bool>(), reply(), reply())
            .prop_map(|(faults, stop, idle, dma, command)| Event::Snapshot(Frame { faults, stop, idle, dma, command })),
        1 => Just(Event::Cancel),
    ]
}

#[derive(Clone, Debug, PartialEq, Eq)]
enum Call {
    Start(u8),
    Arm(usize),
    Observe(WakeOn),
    Dma,
    Stop,
    Drop,
}

#[derive(Default)]
struct Script {
    frame: Frame,
    start_error: Option<Error>,
    arm_error: Option<Error>,
    calls: Vec<Call>,
}

struct Device(Rc<RefCell<Script>>);
struct Lease(Device);

impl Drop for Device {
    fn drop(&mut self) {
        self.0.borrow_mut().calls.push(Call::Drop);
    }
}

impl Backend for Device {
    type Streaming = Lease;
    type Receiving = (Self, &'static mut [u8]);

    fn start(self, address: u8) -> Result<Self, Error> {
        let error = {
            let mut script = self.0.borrow_mut();
            script.calls.push(Call::Start(address));
            script.start_error
        };
        match error {
            Some(error) => Err(error),
            None => Ok(self),
        }
    }

    fn arm(self, bytes: &'static [u8]) -> Result<Lease, Error> {
        let error = {
            let mut script = self.0.borrow_mut();
            script.calls.push(Call::Arm(bytes.len()));
            script.arm_error
        };
        match error {
            Some(error) => Err(error),
            None => Ok(Lease(self)),
        }
    }

    fn observe(&mut self, _: &mut Context<'_>, wake: WakeOn) -> Status {
        let mut script = self.0.borrow_mut();
        script.calls.push(Call::Observe(wake));
        script.frame.observation()
    }

    fn observe_streaming(lease: &mut Lease, cx: &mut Context<'_>, wake: WakeOn) -> Status {
        lease.0.observe(cx, wake)
    }

    fn poll_dma(lease: Lease, _: &mut Context<'_>) -> DmaPoll<Self> {
        let reply = {
            let mut script = lease.0.0.borrow_mut();
            script.calls.push(Call::Dma);
            script.frame.dma
        };
        match reply {
            Reply::Pending => DmaPoll::Pending(lease),
            Reply::Complete => DmaPoll::Complete(lease.0),
            Reply::Failed(error) => DmaPoll::Failed(error),
        }
    }

    fn try_stop(&mut self) -> Poll<Result<(), Error>> {
        let mut script = self.0.borrow_mut();
        script.calls.push(Call::Stop);
        match script.frame.command {
            Reply::Pending => Poll::Pending,
            Reply::Complete => Poll::Ready(Ok(())),
            Reply::Failed(error) => Poll::Ready(Err(error)),
        }
    }

    fn read_ready(&mut self, _: u8, _: u8) -> Poll<Result<(), Error>> {
        panic!("write trace issued a read")
    }
    fn arm_receive(self, _: u8, _: &'static mut [u8]) -> Result<Self::Receiving, Error> {
        panic!("write trace armed RX")
    }
    fn observe_receive(_: &mut Self::Receiving, _: &mut Context<'_>, _: WakeOn) -> Status {
        panic!("write trace observed RX")
    }
    fn poll_receive(_: Self::Receiving, _: &mut Context<'_>) -> ReceivePoll<Self> {
        panic!("write trace polled RX")
    }
}

// Independent oracle: rank a set of faults, then interpret STOP by bus phase.
// Do not reuse the production classifier or its nested-if implementation.
fn expected_fault(bits: u8) -> Option<Error> {
    [
        (1, Error::PinLowTimeout),
        (2, Error::Nack),
        (4, Error::ArbitrationLoss),
        (8, Error::Fifo),
    ]
    .into_iter()
    .find_map(|(bit, error)| (bits & bit != 0).then_some(error))
}

#[derive(Clone, Copy)]
enum Phase {
    Open,
    Dma,
    EnqueueStop,
    AwaitClosed,
    Terminal,
}

struct Model {
    phase: Phase,
    calls: Vec<Call>,
}

impl Model {
    fn new() -> Self {
        Self {
            phase: Phase::Open,
            calls: Vec::new(),
        }
    }

    fn end(&mut self, result: Result<(), Error>) -> Poll<Result<(), Error>> {
        self.phase = Phase::Terminal;
        self.calls.push(Call::Drop);
        Poll::Ready(result)
    }

    fn poll(
        &mut self,
        frame: Frame,
        address: u8,
        length: usize,
        start_error: Option<Error>,
        arm_error: Option<Error>,
    ) -> Poll<Result<(), Error>> {
        // Each poll may cross all immediately ready stages. No limit or timer
        // is invented for a backend which keeps saying Pending.
        loop {
            if matches!(self.phase, Phase::Open) {
                if address < 8 || address > 119 {
                    return self.end(Err(Error::InvalidInput));
                }
                self.calls.push(Call::Start(address));
                if let Some(error) = start_error {
                    return self.end(Err(error));
                }
                if length == 0 || length > 32 {
                    return self.end(Err(Error::InvalidInput));
                }
                self.calls.push(Call::Arm(length));
                if let Some(error) = arm_error {
                    return self.end(Err(error));
                }
                self.phase = Phase::Dma;
            }
            let closing = matches!(self.phase, Phase::AwaitClosed);
            self.calls
                .push(Call::Observe(if matches!(self.phase, Phase::EnqueueStop) {
                    WakeOn::TxSpace
                } else {
                    WakeOn::Completion
                }));
            if let Some(error) =
                expected_fault(frame.faults).or((frame.stop && !closing).then_some(Error::UnexpectedStop))
            {
                return self.end(Err(error));
            }
            let (reply, next) = match self.phase {
                Phase::Dma => {
                    self.calls.push(Call::Dma);
                    (frame.dma, Phase::EnqueueStop)
                }
                Phase::EnqueueStop => {
                    self.calls.push(Call::Stop);
                    (frame.command, Phase::AwaitClosed)
                }
                Phase::AwaitClosed => {
                    return if frame.stop && frame.idle {
                        self.end(Ok(()))
                    } else {
                        Poll::Pending
                    };
                }
                _ => panic!("model polled after termination"),
            };
            match reply {
                Reply::Pending => return Poll::Pending,
                Reply::Complete => self.phase = next,
                Reply::Failed(error) => return self.end(Err(error)),
            }
        }
    }
}

fn trace(address: u8, length: usize, start_error: Option<Error>, arm_error: Option<Error>, events: &[Event]) {
    let script = Rc::new(RefCell::new(Script {
        start_error,
        arm_error,
        ..Script::default()
    }));
    let owner = Idle::new(Device(script.clone()));
    let mut transfer = Box::pin(async move {
        let queued = owner.start(address)?.transmit(&BYTES[..length])?.drain().await?;
        queued.stop().await?.complete().await?;
        Ok(())
    });
    let mut model = Model::new();
    let mut frame = Frame::default();
    let mut cx = Context::from_waker(Waker::noop());
    for (step, event) in events.iter().copied().enumerate() {
        if matches!(event, Event::Cancel) {
            break;
        }
        frame.apply(event);
        script.borrow_mut().frame = frame;
        let expected = model.poll(frame, address, length, start_error, arm_error);
        let actual = transfer.as_mut().poll(&mut cx);
        assert_eq!(actual, expected, "PROTOCOL_MODEL step={step} event={event:?}");
        assert_eq!(
            script.borrow().calls,
            model.calls,
            "PROTOCOL_CALL_ORDER step={step} event={event:?}"
        );
        if actual.is_ready() {
            drop(transfer);
            assert_eq!(script.borrow().calls, model.calls, "TERMINAL_OWNER_DROP");
            return;
        }
    }
    // End-of-trace is cancellation too, including a future never polled.
    drop(transfer);
    model.calls.push(Call::Drop);
    assert_eq!(script.borrow().calls, model.calls, "CANCEL_OWNER_DROP");
}

proptest! {
    #![proptest_config(config())]

    #[test]
    fn write_sequences_match_independent_model(events in prop::collection::vec(event(), 0..65), address in 8u8..120, length in 1usize..33) {
        trace(address, length, None, None, &events);
    }

    #[test]
    fn rejected_input_and_backend_failures_are_not_hidden(
        address in any::<u8>(), length in 0usize..34,
        start_error in prop::option::of(error()), arm_error in prop::option::of(error()),
        events in prop::collection::vec(event(), 1..17),
    ) {
        // Always poll once before generated cancellation to exercise rejection.
        let events: Vec<_> = std::iter::once(Event::Wait).chain(events).collect();
        trace(address, length, start_error, arm_error, &events);
    }

    #[test]
    fn preflight_matches_address_and_size_contract(address in any::<u8>(), length in prop_oneof![0usize..35, any::<usize>()]) {
        let expected = if address >= 8 && address <= 119 && length != 0 && length <= 32 { Ok(()) } else { Err(Error::InvalidInput) };
        prop_assert_eq!(i2c_one_shot_core::validate_transfer(address, length), expected);
    }
}

#[test]
fn all_256_classifier_inputs_preserve_facts_and_ignore_bus_idle_for_fault_priority() {
    // Seven snapshot facts plus want_stop. Bus-idle is for START admission,
    // not another fault or a substitute for controller-idle on completion.
    for faults in 0..16 {
        for stop in [false, true] {
            for idle in [false, true] {
                for want_stop in [false, true] {
                    for bus_idle in [false, true] {
                        let status = Status {
                            bus_idle,
                            ..Frame {
                                faults,
                                stop,
                                idle,
                                ..Frame::default()
                            }
                            .observation()
                        };
                        let expected = expected_fault(faults).or((stop && !want_stop).then_some(Error::UnexpectedStop));
                        assert_eq!(
                            i2c_one_shot_core::checked(status, want_stop),
                            expected.map_or(Ok(status), Err)
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn successful_and_cancelled_prefixes_are_always_exercised() {
    let events = [
        Event::Wait,
        Event::Dma(Reply::Complete),
        Event::Wait,
        Event::Fifo(Reply::Complete),
        Event::Wire {
            stop: true,
            idle: false,
        },
        Event::Wire {
            stop: false,
            idle: true,
        },
        Event::Wire { stop: true, idle: true },
    ];
    for length in 0..=events.len() {
        trace(0x2a, 4, None, None, &events[..length]);
    }
}
