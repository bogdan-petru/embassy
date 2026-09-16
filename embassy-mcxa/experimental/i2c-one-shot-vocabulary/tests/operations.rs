//! Vocabulary decisions over a mock physical binding. These tests do not
//! replace the HAL recorder tests of actual DMA/MMIO/wake ordering.
use core::task::{Context, Waker};
use std::cell::RefCell;
use std::rc::Rc;

use i2c_one_shot_interface::{Command, Cpu, Device, DmaResult, InFlight, ReceiveResult, Receiving, Status};
use i2c_one_shot_vocabulary::{Backend, DmaPoll, Error, Hardware, ReceivePoll, WakeOn};

#[derive(Default)]
struct State {
    status: Status,
    fifo_empty: bool,
    done: bool,
    dma_error: bool,
    active: bool,
    reads: usize,
    drops: usize,
    commands: Vec<Command>,
    receives: Vec<(u8, usize)>,
}

struct Fake(Rc<RefCell<State>>);
struct Lease {
    device: Fake,
    _source: &'static [u8],
}
struct RxLease {
    device: Fake,
    bytes: &'static mut [u8],
}

impl Drop for Fake {
    fn drop(&mut self) {
        let mut state = self.0.borrow_mut();
        state.active = false;
        state.drops += 1;
    }
}

// SAFETY: this integration-test binding has no physical device or DMA. One
// owned Fake and immutable static source move with every lease. All call
// orders are memory-safe; terminal returns/drop close the modeled activity.
// This is outside the forbidden vocabulary library, in the explicit mock TCB.
unsafe impl Device for Fake {
    type InFlight = Lease;
    type Receiving = RxLease;

    fn snapshot(&self) -> Status {
        let mut state = self.0.borrow_mut();
        state.reads += 1;
        state.status
    }

    fn command(&mut self, command: Command) {
        self.0.borrow_mut().commands.push(command);
    }

    fn fifo_empty(&self) -> bool {
        self.0.borrow().fifo_empty
    }

    fn arm(self, bytes: &'static [u8]) -> Result<Lease, Error> {
        self.0.borrow_mut().active = true;
        Ok(Lease {
            device: self,
            _source: bytes,
        })
    }

    fn observe(&mut self, _cx: &mut Context<'_>, _wake: WakeOn) -> Status {
        self.snapshot()
    }

    fn observe_dma(lease: &mut Lease, cx: &mut Context<'_>, wake: WakeOn) -> Status {
        lease.device.observe(cx, wake)
    }

    fn poll_dma(lease: Lease, _cx: &mut Context<'_>) -> DmaResult<Self> {
        if lease.device.0.borrow().dma_error {
            DmaResult::Failed(Error::Dma)
        } else if lease.device.0.borrow().done {
            lease.device.0.borrow_mut().active = false;
            DmaResult::Complete(lease.device)
        } else {
            DmaResult::Pending(lease)
        }
    }
    fn arm_receive(self, address: u8, bytes: &'static mut [u8]) -> Result<RxLease, Error> {
        self.0.borrow_mut().active = true;
        self.0.borrow_mut().receives.push((address, bytes.len()));
        Ok(RxLease { device: self, bytes })
    }
    fn observe_receive(lease: &mut RxLease, cx: &mut Context<'_>, wake: WakeOn) -> Status {
        lease.device.observe(cx, wake)
    }
    fn poll_receive(lease: RxLease, _cx: &mut Context<'_>) -> ReceiveResult<Self> {
        if lease.device.0.borrow().dma_error {
            ReceiveResult::Failed(Error::Dma)
        } else if lease.device.0.borrow().done {
            lease.device.0.borrow_mut().active = false;
            ReceiveResult::Complete((lease.device, lease.bytes))
        } else {
            ReceiveResult::Pending(lease)
        }
    }
}

#[test]
fn read_readiness_never_launches_commands_before_consuming_dma_arm() {
    let (hardware, state) = fixture();
    let mut hardware = hardware.start(0x2a).unwrap();
    assert!(hardware.read_ready(0x2a, 4).is_pending());
    assert_eq!(state.borrow().commands, [Command::Start(0x2a)]);
    state.borrow_mut().fifo_empty = true;
    assert!(matches!(hardware.read_ready(0x2a, 4), core::task::Poll::Ready(Ok(()))));
    assert_eq!(state.borrow().commands, [Command::Start(0x2a)]);
    assert!(state.borrow().receives.is_empty());
    let lease = hardware.arm_receive(0x2a, Box::leak(Box::new([0; 4]))).unwrap();
    assert_eq!(state.borrow().receives, [(0x2a, 4)]);
    assert_eq!(state.borrow().commands, [Command::Start(0x2a)]);
    drop(lease);
}

#[test]
fn read_invalid_input_cannot_enqueue_partial_commands() {
    for (address, length) in [(0, 4), (0x78, 4), (0x2a, 0), (0x2a, 33)] {
        let (mut hardware, state) = fixture();
        state.borrow_mut().fifo_empty = true;
        assert!(matches!(
            hardware.read_ready(address, length),
            core::task::Poll::Ready(Err(Error::InvalidInput))
        ));
        assert!(state.borrow().commands.is_empty());
    }
}

#[test]
fn receive_pending_retains_memory_and_completion_returns_it_after_lower_quiescence() {
    let (hardware, state) = fixture();
    let bytes = Box::leak(Box::new([0x39; 4]));
    let address = bytes.as_ptr();
    let lease = hardware.arm_receive(0x2a, bytes).unwrap();
    let mut cx = Context::from_waker(Waker::noop());
    let ReceivePoll::Pending(lease) = Hardware::<Fake>::poll_receive(lease, &mut cx) else {
        panic!("RX pending required")
    };
    assert!(state.borrow().active);
    state.borrow_mut().done = true;
    let ReceivePoll::Complete((hardware, bytes)) = Hardware::<Fake>::poll_receive(lease, &mut cx) else {
        panic!("RX completion required")
    };
    assert!(!state.borrow().active);
    assert_eq!(bytes.as_ptr(), address);
    assert_eq!(bytes, &[0x39; 4]);
    drop(hardware);
    assert_eq!(state.borrow().drops, 1);
}

#[test]
fn forgotten_receive_keeps_static_storage_and_lower_ownership_inaccessible() {
    // Positive anchors for isolated compiler mutations. Forgetting does NOT
    // promise cleanup: it leaks this mock owner and its static allocation.
    fn retain_rx<D: Device>(receiving: Receiving<D>) -> Receiving<D> {
        let _ = &receiving;
        receiving
    }
    fn forget_rx<D: Device>(cpu: Cpu<D>, bytes: &'static mut [u8]) {
        let receiving = cpu.arm_receive(0x2a, bytes).unwrap();
        core::mem::forget(retain_rx(receiving));
    }
    let state = Rc::new(RefCell::new(State::default()));
    forget_rx(Cpu::new(Fake(state.clone())), Box::leak(Box::new([0; 4])));
    assert!(state.borrow().active);
    assert_eq!(state.borrow().drops, 0);
}

fn fixture() -> (Hardware<Fake>, Rc<RefCell<State>>) {
    let state = Rc::new(RefCell::new(State {
        status: Status {
            bus_idle: true,
            controller_idle: true,
            ..Status::default()
        },
        ..State::default()
    }));
    (Hardware::new(Cpu::new(Fake(state.clone()))), state)
}

#[test]
fn start_requires_both_bus_and_controller_idle() {
    for controller_idle in [false, true] {
        for bus_idle in [false, true] {
            let (hardware, state) = fixture();
            state.borrow_mut().status.controller_idle = controller_idle;
            state.borrow_mut().status.bus_idle = bus_idle;
            let result = hardware.start(0x2a);
            if controller_idle && bus_idle {
                assert!(result.is_ok());
                assert_eq!(state.borrow().commands, [Command::Start(0x2a)]);
            } else {
                assert!(matches!(result, Err(Error::Busy)));
                assert!(state.borrow().commands.is_empty());
            }
        }
    }
}

#[test]
fn stop_checks_current_fifo_state_before_each_command_attempt() {
    let (mut hardware, state) = fixture();
    assert!(hardware.try_stop().is_pending());
    assert!(state.borrow().commands.is_empty());
    state.borrow_mut().fifo_empty = true;
    assert!(matches!(hardware.try_stop(), core::task::Poll::Ready(Ok(()))));
    assert_eq!(state.borrow().commands, [Command::Stop]);
}

#[test]
fn one_snapshot_preserves_all_faults_and_independent_completion_facts() {
    for bits in 0..128 {
        let (mut hardware, state) = fixture();
        state.borrow_mut().status = Status {
            pin_low_timeout: bits & 1 != 0,
            nack: bits & 2 != 0,
            arbitration_loss: bits & 4 != 0,
            fifo_error: bits & 8 != 0,
            stop_seen: bits & 16 != 0,
            controller_idle: bits & 32 != 0,
            bus_idle: bits & 64 != 0,
        };
        let observation = hardware.observe(&mut Context::from_waker(Waker::noop()), WakeOn::Completion);
        assert_eq!(state.borrow().reads, 1);
        assert_eq!(observation.pin_low_timeout, bits & 1 != 0);
        assert_eq!(observation.nack, bits & 2 != 0);
        assert_eq!(observation.arbitration_loss, bits & 4 != 0);
        assert_eq!(observation.fifo_error, bits & 8 != 0);
        assert_eq!(observation.stop_seen, bits & 16 != 0);
        assert_eq!(observation.controller_idle, bits & 32 != 0);
        assert_eq!(observation.bus_idle, bits & 64 != 0);
    }
}

#[test]
fn pending_retains_the_same_pair_and_completion_returns_cpu_authority() {
    let (hardware, state) = fixture();
    let lease = hardware.arm(b"data").unwrap();
    let mut cx = Context::from_waker(Waker::noop());
    let DmaPoll::Pending(lease) = Hardware::<Fake>::poll_dma(lease, &mut cx) else {
        panic!("pending required")
    };
    assert!(state.borrow().active);
    assert_eq!(state.borrow().drops, 0);
    state.borrow_mut().done = true;
    let DmaPoll::Complete(hardware) = Hardware::<Fake>::poll_dma(lease, &mut cx) else {
        panic!("completion required")
    };
    assert!(!state.borrow().active);
    assert_eq!(state.borrow().drops, 0);
    drop(hardware);
    assert_eq!(state.borrow().drops, 1);
}

#[test]
fn dropping_upper_lease_drops_lower_cleanup_owner_without_completion() {
    let (hardware, state) = fixture();
    let lease = hardware.arm(b"data").unwrap();
    assert!(state.borrow().active);
    drop(lease);
    assert!(!state.borrow().active);
    assert_eq!(state.borrow().drops, 1);
}

#[test]
fn dma_error_cleans_pair_without_returning_cpu_success() {
    let (hardware, state) = fixture();
    let lease = hardware.arm(b"data").unwrap();
    state.borrow_mut().dma_error = true;
    assert!(matches!(
        Hardware::<Fake>::poll_dma(lease, &mut Context::from_waker(Waker::noop())),
        DmaPoll::Failed(Error::Dma)
    ));
    assert!(!state.borrow().active);
    assert_eq!(state.borrow().drops, 1);
}

#[test]
fn opaque_capabilities_can_move_without_exposing_lower_resources() {
    // These otherwise ordinary helpers are exact positive anchors for compiler
    // mutations: extracting either raw field, or commanding InFlight, must fail.
    fn keep_cpu<D: Device>(cpu: Cpu<D>) -> Cpu<D> {
        let _ = &cpu;
        cpu
    }
    fn keep_dma<D: Device>(flight: InFlight<D>) -> InFlight<D> {
        let _ = &flight;
        flight
    }
    fn observe_owner<D: Device>(flight: &mut InFlight<D>) {
        let _ = &flight;
    }
    let state = Rc::new(RefCell::new(State::default()));
    let cpu = keep_cpu(Cpu::new(Fake(state.clone())));
    let mut flight = keep_dma(cpu.arm(b"data").unwrap());
    observe_owner(&mut flight);
    assert!(state.borrow().active);
    drop(flight);
    assert_eq!(state.borrow().drops, 1);
}
