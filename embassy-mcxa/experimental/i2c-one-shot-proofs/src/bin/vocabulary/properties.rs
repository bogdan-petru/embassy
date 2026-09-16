//! Independent expectations applied to the real L3 Hardware implementation.
#![forbid(unsafe_code)]

use std::cell::RefCell;
use std::task::{Context, Poll};

use i2c_one_shot_interface::{Command, Cpu, Error, Status, WakeOn};
use i2c_one_shot_vocabulary::{Backend, DmaPoll, Hardware, ReceivePoll};

use super::model::{Decision, Event, Model, Path, State, waker};

static SOURCE: [u8; 32] = [0xa7; 32];

pub fn start(status: Status, address: u8) {
    let state = RefCell::new(State::new(status));
    let wake = waker();
    let hardware = Hardware::new(Cpu::new(Model::new(&state, &wake)));
    let result = hardware.start(address);
    if status.bus_idle && status.controller_idle {
        let hardware = result.expect("START_IDLE_ADMISSION");
        assert_eq!(
            state.borrow().events(),
            &[Event::Snapshot, Event::Command(Command::Start(address))],
            "START_EXACT_ADDRESS"
        );
        assert_eq!(state.borrow().drops, 0);
        drop(hardware);
    } else {
        assert!(matches!(result, Err(Error::Busy)), "START_BUSY_REJECTED");
        assert_eq!(
            state.borrow().events(),
            &[Event::Snapshot, Event::Drop],
            "START_BUSY_NO_COMMAND"
        );
    }
    assert_eq!(state.borrow().drops, 1);
}

pub fn stop(fifo: [bool; 3]) {
    let state = RefCell::new(State::new(Status::default()));
    let wake = waker();
    let mut hardware = Hardware::new(Cpu::new(Model::new(&state, &wake)));
    for empty in fifo {
        state.borrow_mut().fifo_empty = empty;
        state.borrow_mut().clear_events();
        let result = hardware.try_stop();
        if empty {
            assert_eq!(result, Poll::Ready(Ok(())), "STOP_CURRENT_READY");
            assert_eq!(
                state.borrow().events(),
                &[Event::Fifo(true), Event::Command(Command::Stop)],
                "STOP_REQUIRES_FRESH_FIFO"
            );
        } else {
            assert_eq!(result, Poll::Pending, "STOP_CURRENT_NOT_READY");
            assert_eq!(
                state.borrow().events(),
                &[Event::Fifo(false)],
                "STOP_REQUIRES_FRESH_FIFO"
            );
        }
    }
    assert_eq!(state.borrow().drops, 0);
    drop(hardware);
    assert_eq!(state.borrow().drops, 1);
}

pub fn read_ready(address: u8, length: u8, empty: bool) {
    let state = RefCell::new(State::new(Status::default()));
    state.borrow_mut().fifo_empty = empty;
    let wake = waker();
    let mut hardware = Hardware::new(Cpu::new(Model::new(&state, &wake)));
    let result = hardware.read_ready(address, length);
    // Independent range oracle, not the implementation's Range::contains.
    if address < 8 || address > 119 || length == 0 || length > 32 {
        assert_eq!(result, Poll::Ready(Err(Error::InvalidInput)), "RX_INVALID_REJECTED");
        assert_eq!(state.borrow().events(), &[], "RX_INVALID_HAS_NO_EFFECTS");
    } else {
        assert_eq!(
            result,
            if empty { Poll::Ready(Ok(())) } else { Poll::Pending },
            "RX_READINESS_RESULT"
        );
        assert_eq!(state.borrow().events(), &[Event::Fifo(empty)], "RX_READINESS_IS_PURE");
    }
    assert!(!state.borrow().active);
    assert_eq!(state.borrow().drops, 0);
    drop(hardware);
}

pub fn observe(status: Status, wake_on: WakeOn, path: Path) {
    let state = RefCell::new(State::new(status));
    let wake = waker();
    let mut cx = Context::from_waker(&wake);
    let mut hardware = Hardware::new(Cpu::new(Model::new(&state, &wake)));
    let answer = match path {
        Path::Cpu => hardware.observe(&mut cx, wake_on),
        Path::Tx => {
            let mut lease = hardware.arm(&SOURCE).unwrap();
            state.borrow_mut().clear_events();
            let answer = Hardware::<Model<'_>>::observe_streaming(&mut lease, &mut cx, wake_on);
            assert_eq!(
                state.borrow().events(),
                &[Event::Observe(path, wake_on)],
                "OBSERVE_EXACT_CALL"
            );
            drop(lease);
            answer
        }
        Path::Rx => {
            let bytes = Box::leak(Box::new([0; 4]));
            let mut lease = hardware.arm_receive(0x2a, bytes).unwrap();
            state.borrow_mut().clear_events();
            let answer = Hardware::<Model<'_>>::observe_receive(&mut lease, &mut cx, wake_on);
            assert_eq!(
                state.borrow().events(),
                &[Event::Observe(path, wake_on)],
                "OBSERVE_EXACT_CALL"
            );
            drop(lease);
            answer
        }
    };
    assert_eq!(answer, status, "OBSERVE_ALL_STATUS_BITS_UNCHANGED");
    if path == Path::Cpu {
        assert_eq!(
            state.borrow().events(),
            &[Event::Observe(path, wake_on)],
            "OBSERVE_EXACT_CALL"
        );
    }
}

pub fn tx_arm(length: usize, reject: Option<Error>) {
    let state = RefCell::new(State::new(Status::default()));
    state.borrow_mut().reject = reject;
    let wake = waker();
    let source = &SOURCE[..length];
    let hardware = Hardware::new(Cpu::new(Model::new(&state, &wake)));
    let result = hardware.arm(source);
    assert_eq!(
        state.borrow().tx_identity,
        Some((source.as_ptr(), length)),
        "TX_ARM_EXACT_SOURCE"
    );
    match (reject, result) {
        (None, Ok(lease)) => {
            assert!(state.borrow().active);
            assert_eq!(state.borrow().events(), &[Event::ArmTx(length)]);
            drop(lease);
        }
        (Some(expected), Err(actual)) => {
            assert_eq!(actual, expected, "TX_ARM_EXACT_ERROR");
            assert_eq!(state.borrow().events(), &[Event::ArmTx(length), Event::Drop]);
        }
        _ => panic!("TX_ARM_OUTCOME_CHANGED"),
    }
    assert_eq!(state.borrow().drops, 1);
    assert!(!state.borrow().active);
    assert_eq!(state.borrow().completions, 0);
}

pub fn rx_arm(address: u8, length: usize, empty: bool, reject: Option<Error>) {
    let state = RefCell::new(State::new(Status::default()));
    state.borrow_mut().reject = reject;
    let wake = waker();
    let bytes = &mut Box::leak(Box::new([0x31; 4]))[..length];
    let pointer = bytes.as_ptr();
    let mut hardware = Hardware::new(Cpu::new(Model::new(&state, &wake)));
    // Readiness cannot license a later arm: L2 independently rechecks its
    // current state. This proof checks delegation, not the physical recheck.
    assert_eq!(hardware.read_ready(address, length as u8), Poll::Ready(Ok(())));
    state.borrow_mut().fifo_empty = empty;
    state.borrow_mut().clear_events();
    let result = hardware.arm_receive(address, bytes);
    assert_eq!(
        state.borrow().rx_identity,
        Some((pointer, length)),
        "RX_ARM_EXACT_BUFFER"
    );
    let expected_error = if empty { reject } else { Some(Error::Busy) };
    match (expected_error, result) {
        (None, Ok(lease)) => {
            assert!(state.borrow().active);
            assert_eq!(state.borrow().events(), &[Event::ArmRx(address, length)]);
            drop(lease);
        }
        (Some(expected), Err(actual)) => {
            assert_eq!(actual, expected, "RX_ARM_EXACT_ERROR");
            assert_eq!(state.borrow().events(), &[Event::ArmRx(address, length), Event::Drop]);
        }
        _ => panic!("RX_ARM_OUTCOME_CHANGED"),
    }
    assert_eq!(state.borrow().drops, 1);
    assert!(!state.borrow().active);
    assert_eq!(state.borrow().completions, 0);
}

pub fn tx_polls(decisions: [Decision; 3], cancel: [bool; 3]) {
    let state = RefCell::new(State::new(Status::default()));
    let wake = waker();
    let mut cx = Context::from_waker(&wake);
    let hardware = Hardware::new(Cpu::new(Model::new(&state, &wake)));
    let mut lease = hardware.arm(&SOURCE).unwrap();
    for index in 0..3 {
        state.borrow_mut().decision = decisions[index];
        state.borrow_mut().clear_events();
        match (decisions[index], Hardware::<Model<'_>>::poll_dma(lease, &mut cx)) {
            (Decision::Pending, DmaPoll::Pending(next)) => {
                assert_eq!(
                    state.borrow().events(),
                    &[Event::Poll(Path::Tx, Decision::Pending)],
                    "TX_PENDING_NO_CLEANUP"
                );
                assert!(state.borrow().active);
                assert_eq!(state.borrow().drops, 0);
                assert_eq!(state.borrow().completions, 0);
                if cancel[index] {
                    drop(next);
                    assert_eq!(state.borrow().drops, 1);
                    assert!(!state.borrow().active);
                    assert_eq!(state.borrow().completions, 0, "CANCEL_NEVER_COMPLETES");
                    return;
                }
                lease = next;
            }
            (Decision::Complete, DmaPoll::Complete(mut cpu)) => {
                assert_eq!(
                    state.borrow().events(),
                    &[Event::Poll(Path::Tx, Decision::Complete), Event::Quiesce]
                );
                assert!(!state.borrow().active);
                assert_eq!(state.borrow().drops, 0);
                assert_eq!(state.borrow().completions, 1);
                cpu.observe(&mut cx, WakeOn::Completion); // Same owned device and waker.
                drop(cpu);
                assert_eq!(state.borrow().drops, 1);
                return;
            }
            (Decision::Failed(expected), DmaPoll::Failed(actual)) => {
                assert_eq!(actual, expected, "TX_POLL_EXACT_ERROR");
                assert_eq!(
                    state.borrow().events(),
                    &[Event::Poll(Path::Tx, decisions[index]), Event::Quiesce, Event::Drop]
                );
                assert_eq!(state.borrow().drops, 1);
                assert!(!state.borrow().active);
                assert_eq!(state.borrow().completions, 0);
                return;
            }
            _ => panic!("TX_POLL_OUTCOME_CHANGED"),
        }
    }
    drop(lease); // Three Pending decisions: bounded observation, not liveness.
    assert_eq!(state.borrow().drops, 1);
    assert!(!state.borrow().active);
    assert_eq!(state.borrow().completions, 0);
}

pub fn rx_polls(decisions: [Decision; 3], cancel: [bool; 3], data: [u8; 4], length: usize) {
    let state = RefCell::new(State::new(Status::default()));
    let wake = waker();
    let mut cx = Context::from_waker(&wake);
    let bytes = &mut Box::leak(Box::new(data))[..length];
    let pointer = bytes.as_ptr();
    let hardware = Hardware::new(Cpu::new(Model::new(&state, &wake)));
    let mut lease = hardware.arm_receive(0x2a, bytes).unwrap();
    for index in 0..3 {
        state.borrow_mut().decision = decisions[index];
        state.borrow_mut().clear_events();
        match (decisions[index], Hardware::<Model<'_>>::poll_receive(lease, &mut cx)) {
            (Decision::Pending, ReceivePoll::Pending(next)) => {
                assert_eq!(
                    state.borrow().events(),
                    &[Event::Poll(Path::Rx, Decision::Pending)],
                    "RX_PENDING_NO_CLEANUP"
                );
                assert!(state.borrow().active);
                assert_eq!(state.borrow().drops, 0);
                assert_eq!(state.borrow().completions, 0);
                if cancel[index] {
                    drop(next);
                    assert_eq!(state.borrow().drops, 1);
                    assert!(!state.borrow().active);
                    assert_eq!(state.borrow().completions, 0, "CANCEL_NEVER_COMPLETES");
                    return;
                }
                lease = next;
            }
            (Decision::Complete, ReceivePoll::Complete((mut cpu, returned))) => {
                assert_eq!(
                    state.borrow().events(),
                    &[Event::Poll(Path::Rx, Decision::Complete), Event::Quiesce]
                );
                assert!(!state.borrow().active);
                assert_eq!(returned.as_ptr(), pointer, "RX_SAME_ALLOCATION");
                assert_eq!(returned, &data[..length], "RX_EXACT_CONTENTS_AND_LENGTH");
                assert_eq!(state.borrow().drops, 0);
                assert_eq!(state.borrow().completions, 1);
                cpu.observe(&mut cx, WakeOn::Completion);
                drop(cpu);
                assert_eq!(state.borrow().drops, 1);
                return;
            }
            (Decision::Failed(expected), ReceivePoll::Failed(actual)) => {
                assert_eq!(actual, expected, "RX_POLL_EXACT_ERROR");
                assert_eq!(
                    state.borrow().events(),
                    &[Event::Poll(Path::Rx, decisions[index]), Event::Quiesce, Event::Drop]
                );
                assert_eq!(state.borrow().drops, 1);
                assert!(!state.borrow().active);
                assert_eq!(state.borrow().completions, 0);
                return;
            }
            _ => panic!("RX_POLL_OUTCOME_CHANGED"),
        }
    }
    drop(lease);
    assert_eq!(state.borrow().drops, 1);
    assert!(!state.borrow().active);
    assert_eq!(state.borrow().completions, 0);
}

pub fn forget(receive: bool) {
    let state = RefCell::new(State::new(Status::default()));
    let wake = waker();
    let mut cx = Context::from_waker(&wake);
    let hardware = Hardware::new(Cpu::new(Model::new(&state, &wake)));
    if receive {
        let lease = hardware.arm_receive(0x2a, Box::leak(Box::new([0; 4]))).unwrap();
        let ReceivePoll::Pending(lease) = Hardware::<Model<'_>>::poll_receive(lease, &mut cx) else {
            panic!("pending required")
        };
        core::mem::forget(lease);
    } else {
        let lease = hardware.arm(&SOURCE).unwrap();
        let DmaPoll::Pending(lease) = Hardware::<Model<'_>>::poll_dma(lease, &mut cx) else {
            panic!("pending required")
        };
        core::mem::forget(lease);
    }
    assert!(state.borrow().active, "FORGET_IS_NOT_CLEANUP");
    assert_eq!(state.borrow().drops, 0);
    assert_eq!(state.borrow().completions, 0);
    // No returned RX reference exists here. Static-storage exclusion remains
    // a compiler-control property, not inferred from this modeled active bit.
}
