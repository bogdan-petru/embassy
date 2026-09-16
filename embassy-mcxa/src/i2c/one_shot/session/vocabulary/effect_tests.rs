//! Real vocabulary/adapter effects over registered host MMIO, not a mock protocol.

extern crate std;

use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};
use std::boxed::Box;
use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::task::{Wake, Waker};
use std::vec;

use i2c_one_shot_core::{Backend, DmaPoll, Error, Idle, Stopping, Streaming, TxDmaQuiescent, WakeOn};

use super::registers::Hardware;
use super::registers::test_support::{Event, Fixture, Injection, TestEffects};
use crate::pac::lpi2c::{Msr, MsrSdf, Ndf};

static BYTES: [u8; 3] = [0x11, 0x22, 0x33];

#[derive(Default)]
struct WakeCount(AtomicUsize);
impl Wake for WakeCount {
    fn wake(self: Arc<Self>) {
        self.wake_by_ref();
    }
    fn wake_by_ref(self: &Arc<Self>) {
        self.0.fetch_add(1, Ordering::SeqCst);
    }
}

fn poll<F: Future>(future: &mut Pin<Box<F>>, wakes: &Arc<WakeCount>) -> Poll<F::Output> {
    let waker = Waker::from(wakes.clone());
    future.as_mut().poll(&mut Context::from_waker(&waker))
}

fn ready<T>(result: Poll<Result<T, Error>>) -> T {
    match result {
        Poll::Ready(Ok(value)) => value,
        Poll::Ready(Err(error)) => panic!("unexpected transfer error: {error:?}"),
        Poll::Pending => panic!("expected an immediately ready phase"),
    }
}

fn idle(fixture: &Fixture) -> Idle<Hardware<TestEffects>> {
    Idle::new(fixture.hardware())
}

fn streaming(fixture: &Fixture) -> Streaming<Hardware<TestEffects>> {
    let open = idle(fixture)
        .start(0x29)
        .unwrap_or_else(|error| panic!("start: {error:?}"));
    open.transmit(&BYTES).unwrap_or_else(|error| panic!("arm: {error:?}"))
}

macro_rules! receive_bytes {
    ($length:expr) => {{
        // Each call site has distinct static storage and takes it once. This
        // preserves the public API's exclusive static lifetime without a heap
        // allocation or a second reference retained by the test.
        static BUFFER: static_cell::ConstStaticCell<[u8; $length]> = static_cell::ConstStaticCell::new([0; $length]);
        BUFFER.take().as_mut_slice()
    }};
}

fn queued(fixture: &Fixture) -> TxDmaQuiescent<Hardware<TestEffects>> {
    fixture.set_dma(true, false);
    ready(poll(&mut Box::pin(streaming(fixture).drain()), &Arc::default()))
}

fn stopping(fixture: &Fixture) -> Stopping<Hardware<TestEffects>> {
    ready(poll(&mut Box::pin(queued(fixture).stop()), &Arc::default()))
}

fn status(nack: bool, stop: bool, busy: bool) -> u32 {
    let mut word = Msr(0);
    if nack {
        word.set_ndf(Ndf::IntYes);
    }
    if stop {
        word.set_sdf(MsrSdf::IntYes);
    }
    if busy {
        word.set_mbf(crate::pac::lpi2c::Mbf::Busy);
    }
    word.0
}

fn cleanup() -> [Event; 14] {
    // Independent expected sequence: do not derive this from the implementation.
    [
        Event::IrqDisable,
        Event::IrqUnpend,
        Event::Write("mier", 0),
        Event::Write("mder", 0),
        Event::QuiesceEnter,
        Event::QuiesceReturn,
        Event::Write("mier", 0),
        Event::Write("mder", 0),
        Event::Read("mcr", 1),
        Event::Write("mcr", 2),
        Event::Read("mcr", 2),
        Event::Write("mcr", 0),
        Event::Read("mcr", 0),
        Event::PinsRelease,
    ]
}

fn assert_cleanup(fixture: &Fixture) {
    assert!(
        fixture.log().ends_with(&cleanup()),
        "EFFECT_QUIESCE_ORDER: {:?}",
        fixture.log()
    );
}

#[test]
fn reset_acknowledgment_precedes_release_and_pin_handoff() {
    let fixture = Fixture::new();
    let hardware = fixture.hardware();
    fixture.control_delays(&[Some(2), Some(3)]);
    drop(hardware);
    let log = fixture.log();
    assert!(
        log.ends_with(&[
            Event::Write("mcr", 2),
            Event::Read("mcr", 1),
            Event::Read("mcr", 1),
            Event::Read("mcr", 2),
            Event::Write("mcr", 0),
            Event::Read("mcr", 2),
            Event::Read("mcr", 2),
            Event::Read("mcr", 2),
            Event::Read("mcr", 0),
            Event::PinsRelease,
        ]),
        "ONE_SHOT_RESET_CONFIRMED_BEFORE_RELEASE: {log:?}"
    );
    let quiesced = log.iter().position(|event| *event == Event::QuiesceReturn).unwrap();
    let reset = log.iter().position(|event| *event == Event::Write("mcr", 2)).unwrap();
    assert!(quiesced < reset, "ONE_SHOT_QUIESCE_BEFORE_RESET");
}

#[test]
fn enabled_owner_waits_for_actual_control_readback() {
    let fixture = Fixture::new();
    fixture.control_delays(&[Some(0), Some(0), Some(3)]);
    let hardware = fixture.hardware_recorded();
    assert!(
        fixture.log().ends_with(&[
            Event::Write("mcr", 1),
            Event::Read("mcr", 0),
            Event::Read("mcr", 0),
            Event::Read("mcr", 0),
            Event::Read("mcr", 1),
        ]),
        "ONE_SHOT_ENABLE_CONFIRMED_BEFORE_AUTHORITY"
    );
    drop(hardware);
}

#[test]
fn failed_reset_never_reports_pin_handoff_even_with_a_stopped_clock() {
    for stopped_clock in [false, true] {
        let fixture = Fixture::new();
        let hardware = fixture.hardware();
        fixture.control_delays(&[Some(0), None]);
        if stopped_clock {
            fixture.stop_clock();
        }
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| drop(hardware)));
        assert!(result.is_err(), "ONE_SHOT_FAILED_RESET_WITHHOLDS_AUTHORITY");
        let log = fixture.log();
        assert!(
            !log.contains(&Event::PinsRelease),
            "ONE_SHOT_FAILED_RESET_RELEASED_PINS"
        );
        assert_eq!(
            log.last(),
            Some(&Event::Write("mcr", 2)),
            "ONE_SHOT_FAILED_RESET_ATTEMPTS_HOLD"
        );
        // A poll-count bound must work even if the clock source never advances.
        assert!(log.len() < 65_560, "ONE_SHOT_RESET_POLL_BOUND");
    }
}

#[test]
fn failed_enable_never_constructs_a_hardware_owner() {
    let fixture = Fixture::new();
    fixture.control_delays(&[Some(0), Some(0), None]);
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| fixture.hardware_recorded()));
    assert!(result.is_err(), "ONE_SHOT_FAILED_ENABLE_WITHHOLDS_AUTHORITY");
    let log = fixture.log();
    assert!(!log.iter().any(|event| matches!(event, Event::Write("mtdr", _))));
    assert_eq!(log.last(), Some(&Event::Write("mcr", 2)));
}

#[test]
fn setup_uses_the_owned_endpoint_before_enabling_requests() {
    let fixture = Fixture::new();
    let port = idle(&fixture);
    fixture.clear_log();
    let mut expected = vec![
        Event::Read("msr", 0),
        Event::Write("mtdr", 0x452),
        Event::DmaPrepare(BYTES.len()),
        Event::DmaSource,
        Event::DmaInterruptEnable,
        Event::DmaRequestEnable,
        Event::Write("mder", 1),
    ];
    expected.extend(cleanup());
    fixture.expect(expected.clone());
    let open = port.start(0x29).unwrap_or_else(|error| panic!("start: {error:?}"));
    let live = open.transmit(&BYTES).unwrap_or_else(|error| panic!("arm: {error:?}"));
    drop(live);
    fixture.assert_consumed();
    assert_eq!(fixture.log(), expected);
    assert_cleanup(&fixture);
}

#[test]
fn arm_before_start_still_owns_dma_until_drop_quiesces_it() {
    // Deliberately bypass layer 4. This is not a valid I2C transaction, but
    // layer 2 must preserve its memory/resource safety for this safe call order.
    // The fixture executes DeviceHardware, not the vocabulary's mock Device.
    let fixture = Fixture::new();
    let hardware = fixture.hardware();
    let mut expected = vec![
        Event::DmaPrepare(BYTES.len()),
        Event::DmaSource,
        Event::DmaInterruptEnable,
        Event::DmaRequestEnable,
        Event::Write("mder", 1),
    ];
    expected.extend(cleanup());
    fixture.expect(expected.clone());
    let live = hardware.arm(&BYTES).unwrap_or_else(|error| panic!("arm: {error:?}"));
    drop(live);
    fixture.assert_consumed();
    assert_eq!(fixture.log(), expected, "L2_ARM_WITHOUT_START_RETAINS_CLEANUP");
    assert_cleanup(&fixture);
}

#[test]
fn prepare_failure_before_start_never_enables_dma_requests() {
    let fixture = Fixture::new();
    let hardware = fixture.hardware();
    // Inject the HAL operation's error, not a guessed length restriction:
    // the lower DMA HAL may accept buffers larger than layer 4's 32-byte limit.
    fixture.fail_prepare(true);
    let mut expected = vec![Event::DmaPrepare(BYTES.len())];
    expected.extend(cleanup());
    fixture.expect(expected.clone());
    assert!(matches!(hardware.arm(&BYTES), Err(Error::Dma)));
    fixture.assert_consumed();
    assert_eq!(fixture.log(), expected, "L2_FAILED_PREPARE_NEVER_ARMS_REQUESTS");
    assert_cleanup(&fixture);
}

#[test]
fn completed_dma_can_rearm_before_stop_only_after_prior_quiescence() {
    // Another intentionally invalid protocol sequence. We assert only that
    // the same lower owner quiesces the first DMA before preparing the next;
    // no STOP, wire-completion or target-data claim follows from these effects.
    let fixture = Fixture::new();
    let hardware = fixture.hardware();
    let live = hardware.arm(&BYTES).unwrap_or_else(|error| panic!("arm: {error:?}"));
    fixture.set_dma(true, false);
    fixture.clear_log();
    let mut expected = vec![
        Event::DmaWait(false),
        Event::DmaError(false),
        Event::DmaDone(true),
        Event::Write("mder", 0),
        Event::QuiesceEnter,
        Event::QuiesceReturn,
        Event::DmaPrepare(BYTES.len()),
        Event::DmaSource,
        Event::DmaInterruptEnable,
        Event::DmaRequestEnable,
        Event::Write("mder", 1),
        Event::DmaWait(false),
        Event::DmaError(false),
        Event::DmaDone(false),
    ];
    expected.extend(cleanup());
    fixture.expect(expected.clone());
    let mut cx = Context::from_waker(Waker::noop());
    let DmaPoll::Complete(hardware) = Hardware::<TestEffects>::poll_dma(live, &mut cx) else {
        panic!("first DMA must return its quiescent CPU owner")
    };
    // The recorder controls sampled channel status; it does not emulate the
    // HAL's TCD setup/reset implementation. Script the new transfer as pending.
    fixture.set_dma(false, false);
    let second = hardware.arm(&BYTES).unwrap_or_else(|error| panic!("rearm: {error:?}"));
    let DmaPoll::Pending(second) = Hardware::<TestEffects>::poll_dma(second, &mut cx) else {
        panic!("second DMA must retain its own live lease")
    };
    drop(second);
    fixture.assert_consumed();
    assert_eq!(fixture.log(), expected, "L2_REARM_REQUIRES_PRIOR_QUIESCENCE");
    assert_cleanup(&fixture);
}

#[test]
fn co_latched_nack_and_dma_done_cannot_mint_success() {
    let fixture = Fixture::new();
    let live = streaming(&fixture);
    fixture.set_status(status(true, false, false));
    fixture.set_dma(true, false);
    fixture.clear_log();
    let mut drain = Box::pin(live.drain());
    assert!(matches!(
        poll(&mut drain, &Arc::default()),
        Poll::Ready(Err(Error::Nack))
    ));
    drop(drain);
    assert!(
        !fixture.log().contains(&Event::DmaDone(true)),
        "fault must be classified before DMA DONE"
    );
    assert_cleanup(&fixture);
}

#[test]
fn stop_during_dma_drain_is_not_normal_completion() {
    let fixture = Fixture::new();
    let live = streaming(&fixture);
    fixture.set_status(status(false, true, false));
    let mut drain = Box::pin(live.drain());
    assert!(matches!(
        poll(&mut drain, &Arc::default()),
        Poll::Ready(Err(Error::UnexpectedStop))
    ));
    drop(drain);
    assert_cleanup(&fixture);
}

#[test]
fn dma_success_quiesces_before_cpu_authority_returns() {
    let fixture = Fixture::new();
    let live = streaming(&fixture);
    fixture.set_dma(true, false);
    fixture.clear_log();
    let owner = ready(poll(&mut Box::pin(live.drain()), &Arc::default()));
    assert!(
        fixture.log().ends_with(&[
            Event::DmaError(false),
            Event::DmaDone(true),
            Event::Write("mder", 0),
            Event::QuiesceEnter,
            Event::QuiesceReturn,
        ]),
        "EFFECT_QUIESCE_ORDER: {:?}",
        fixture.log()
    );
    drop(owner);
    assert_cleanup(&fixture);
}

#[test]
fn dma_done_is_not_wire_completion_stop_and_idle_are_both_required() {
    let fixture = Fixture::new();
    let mut complete = Box::pin(stopping(&fixture).complete());
    let wakes = Arc::default();
    assert!(
        poll(&mut complete, &wakes).is_pending(),
        "EFFECT_WIRE_COMPLETION: DMA alone completed"
    );
    fixture.set_status(status(false, true, true));
    assert!(
        poll(&mut complete, &wakes).is_pending(),
        "EFFECT_WIRE_COMPLETION: STOP while still busy completed"
    );
    fixture.set_status(status(false, true, false));
    ready(poll(&mut complete, &wakes));
    // Completion now releases the owner before returning, not in a later finish.
    assert_cleanup(&fixture);
    drop(complete);
    assert_cleanup(&fixture);
}

#[test]
fn cancellation_at_each_pending_phase_retains_full_cleanup() {
    for phase in 0..3 {
        let fixture = Fixture::new();
        let mut pending: Pin<Box<dyn Future<Output = ()>>> = match phase {
            0 => {
                let owner = streaming(&fixture);
                Box::pin(async move {
                    let _ = owner.drain().await;
                })
            }
            1 => {
                let owner = queued(&fixture);
                fixture.set_fifo(1);
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
        let waker = Waker::from(Arc::new(WakeCount::default()));
        assert!(pending.as_mut().poll(&mut Context::from_waker(&waker)).is_pending());
        fixture.clear_log();
        drop(pending);
        assert_cleanup(&fixture);
    }
}

#[test]
fn prepare_error_and_dma_fault_also_quiesce_before_reset() {
    let fixture = Fixture::new();
    let open = idle(&fixture)
        .start(0x29)
        .unwrap_or_else(|error| panic!("start: {error:?}"));
    fixture.fail_prepare(true);
    assert!(matches!(open.transmit(&BYTES), Err(Error::Dma)));
    assert_cleanup(&fixture);
    drop(fixture);

    let fixture = Fixture::new();
    let live = streaming(&fixture);
    fixture.set_dma(true, true);
    let mut drain = Box::pin(live.drain());
    assert!(matches!(
        poll(&mut drain, &Arc::default()),
        Poll::Ready(Err(Error::Dma))
    ));
    drop(drain);
    assert_cleanup(&fixture);
}

#[test]
fn stale_i2c_and_dma_tokens_are_drained_until_registered() {
    let fixture = Fixture::new();
    let live = streaming(&fixture);
    fixture.stale_i2c_wake();
    fixture.stale_dma_wake();
    fixture.clear_log();
    let mut drain = Box::pin(live.drain());
    assert!(poll(&mut drain, &Arc::default()).is_pending());
    for pair in [
        [Event::I2cWait(true), Event::I2cWait(false)],
        [Event::DmaWait(true), Event::DmaWait(false)],
    ] {
        assert!(
            fixture.log().windows(2).any(|window| window == pair),
            "EFFECT_WAKE_LIVE: stale token not drained"
        );
    }
    drop(drain);
    assert_cleanup(&fixture);
}

#[test]
fn i2c_observe_event_boundaries_are_ready_or_have_a_live_wake() {
    // WaitCell registration, MIER write, NVIC enable, then status read. The
    // last injection may follow the sampled read: Pending must then be woken.
    for boundary in 0..=4 {
        let fixture = Fixture::new();
        let mut hardware = fixture.hardware();
        let wakes = Arc::new(WakeCount::default());
        let waker = Waker::from(wakes.clone());
        fixture.clear_log();
        let event = Injection::Status(status(true, false, false));
        if boundary == 0 {
            fixture.fire(event);
        } else {
            fixture.inject_after(boundary, event);
        }
        let observed = hardware.observe(&mut Context::from_waker(&waker), WakeOn::Completion);
        if !observed.nack {
            assert!(
                wakes.0.load(Ordering::SeqCst) > 0,
                "EFFECT_WAKE_LIVE: I2C boundary {boundary}: {:?}",
                fixture.log()
            );
            assert!(
                hardware
                    .observe(&mut Context::from_waker(&waker), WakeOn::Completion)
                    .nack
            );
        }
        drop(hardware);
        assert_cleanup(&fixture);
    }
}

#[test]
fn dma_completion_boundaries_are_ready_or_have_a_live_wake() {
    // Four observe effects precede DMA registration/error/DONE reads.
    for boundary in [0, 5, 6, 7] {
        let fixture = Fixture::new();
        let live = streaming(&fixture);
        fixture.clear_log();
        if boundary == 0 {
            fixture.fire(Injection::DmaDone);
        } else {
            fixture.inject_after(boundary, Injection::DmaDone);
        }
        let wakes = Arc::new(WakeCount::default());
        let mut drain = Box::pin(live.drain());
        let result = match poll(&mut drain, &wakes) {
            Poll::Pending => {
                assert!(
                    wakes.0.load(Ordering::SeqCst) > 0,
                    "EFFECT_WAKE_LIVE: DMA boundary {boundary}: {:?}",
                    fixture.log()
                );
                poll(&mut drain, &wakes)
            }
            ready => ready,
        };
        let owner = ready(result);
        drop(drain);
        drop(owner);
        assert_cleanup(&fixture);
    }
}

#[test]
fn rx_prearm_enables_dma_before_the_fixed_read_command_pair() {
    let fixture = Fixture::new();
    let live = streaming(&fixture);
    fixture.set_dma(true, false);
    let quiescent = ready(poll(&mut Box::pin(live.drain()), &Arc::default()));

    fixture.clear_log();
    let receiving = ready(poll(
        &mut Box::pin(quiescent.receive(0x29, receive_bytes!(3))),
        &Arc::default(),
    ));
    let log = fixture.log();
    assert!(
        log.ends_with(&[
            Event::DmaReceivePrepare(3),
            Event::DmaReceiveSource,
            Event::DmaInterruptEnable,
            Event::DmaRequestEnable,
            Event::Write("mder", 2),
            Event::Write("mtdr", 0x453),
            Event::Write("mtdr", 0x102),
        ]),
        "RX_PREARM_ORDER: must prepare the paired MRDR/LPI2C0Rx channel, then RDDE, then fixed START-read/RECEIVE: {:?}",
        log
    );
    let mtdr_writes: std::vec::Vec<u32> = log
        .iter()
        .filter_map(|event| match event {
            Event::Write("mtdr", word) => Some(*word),
            _ => None,
        })
        .collect();
    assert_eq!(
        mtdr_writes,
        vec![0x453, 0x102],
        "RX_PREARM_ORDER: no command may escape before the generated pre-arm queue"
    );
    let first_mtdr = log
        .iter()
        .position(|event| matches!(event, Event::Write("mtdr", _)))
        .expect("RX_PREARM_ORDER: generated queue must contain START-read");
    for required in [
        Event::DmaReceivePrepare(3),
        Event::DmaReceiveSource,
        Event::DmaInterruptEnable,
        Event::DmaRequestEnable,
        Event::Write("mder", 2),
    ] {
        let position = log
            .iter()
            .position(|event| event == &required)
            .expect("RX_PREARM_ORDER: missing pre-arm effect");
        assert!(
            position < first_mtdr,
            "RX_PREARM_ORDER: {required:?} must precede START-read: {log:?}"
        );
    }

    // A pending RX future has no CPU FIFO or buffer authority. Dropping it
    // executes the same request-off/quiesce/fence path before reset/release.
    drop(receiving);
    assert_cleanup(&fixture);
}

#[test]
fn rx_completion_returns_storage_only_after_quiescence_and_stop() {
    let fixture = Fixture::new();
    let live = streaming(&fixture);
    fixture.set_dma(true, false);
    let quiescent = ready(poll(&mut Box::pin(live.drain()), &Arc::default()));
    let receiving = ready(poll(
        &mut Box::pin(quiescent.receive(0x29, receive_bytes!(3))),
        &Arc::default(),
    ));

    fixture.set_dma(true, false);
    fixture.clear_log();
    let received = ready(poll(&mut Box::pin(receiving.drain()), &Arc::default()));
    assert!(
        fixture.log().ends_with(&[
            Event::DmaError(false),
            Event::DmaDone(true),
            Event::Write("mder", 0),
            Event::QuiesceEnter,
            Event::QuiesceReturn,
        ]),
        "RX_BUFFER_QUIESCENCE: buffer cannot leave DMA ownership before RDDE-off and quiescence: {:?}",
        fixture.log()
    );

    let stopping = ready(poll(&mut Box::pin(received.stop()), &Arc::default()));
    fixture.set_status(status(false, true, false));
    let bytes = ready(poll(&mut Box::pin(stopping.complete()), &Arc::default()));
    assert_eq!(bytes.len(), 3);
    assert_cleanup(&fixture);
}

#[test]
fn rx_prepare_failure_consumes_storage_without_arming_rdde() {
    let fixture = Fixture::new();
    let live = streaming(&fixture);
    fixture.set_dma(true, false);
    let quiescent = ready(poll(&mut Box::pin(live.drain()), &Arc::default()));

    fixture.fail_prepare(true);
    fixture.clear_log();
    assert!(matches!(
        poll(
            &mut Box::pin(quiescent.receive(0x29, receive_bytes!(3))),
            &Arc::default(),
        ),
        Poll::Ready(Err(Error::Dma))
    ));
    assert!(
        fixture.log().contains(&Event::DmaReceivePrepare(3))
            && !fixture.log().contains(&Event::DmaReceiveSource)
            && !fixture.log().contains(&Event::Write("mder", 2))
            && !fixture
                .log()
                .iter()
                .any(|event| matches!(event, Event::Write("mtdr", _))),
        "failed RX setup must not select a request, arm RDDE, or queue a read command: {:?}",
        fixture.log()
    );
    assert_cleanup(&fixture);
}

#[test]
fn rx_pending_drop_quiesces_before_forgetting_the_buffer() {
    let fixture = Fixture::new();
    let live = streaming(&fixture);
    fixture.set_dma(true, false);
    let quiescent = ready(poll(&mut Box::pin(live.drain()), &Arc::default()));
    let receiving = ready(poll(
        &mut Box::pin(quiescent.receive(0x29, receive_bytes!(3))),
        &Arc::default(),
    ));

    fixture.set_dma(false, false);
    let mut drain = Box::pin(receiving.drain());
    assert!(
        poll(&mut drain, &Arc::default()).is_pending(),
        "RX must retain a live pending lease"
    );
    fixture.clear_log();
    drop(drain);
    assert_cleanup(&fixture);
}

#[test]
fn rx_dma_error_wins_over_done_and_never_returns_the_buffer() {
    let fixture = Fixture::new();
    let live = streaming(&fixture);
    fixture.set_dma(true, false);
    let quiescent = ready(poll(&mut Box::pin(live.drain()), &Arc::default()));
    let receiving = ready(poll(
        &mut Box::pin(quiescent.receive(0x29, receive_bytes!(3))),
        &Arc::default(),
    ));

    fixture.set_dma(true, true);
    fixture.clear_log();
    let mut drain = Box::pin(receiving.drain());
    assert!(matches!(
        poll(&mut drain, &Arc::default()),
        Poll::Ready(Err(Error::Dma))
    ));
    drop(drain);
    assert!(
        !fixture.log().contains(&Event::DmaDone(true)),
        "RX DMA fault must not manufacture a completed buffer: {:?}",
        fixture.log()
    );
    assert_cleanup(&fixture);
}

#[test]
fn rx_prearm_validates_inputs_and_rechecks_fifo_before_effects() {
    for (address, bytes) in [
        (0x07, receive_bytes!(3)),
        (0x29, receive_bytes!(0)),
        (0x29, receive_bytes!(33)),
    ] {
        let fixture = Fixture::new();
        let hardware = fixture.hardware();
        fixture.clear_log();
        let expected = cleanup().to_vec();
        fixture.expect(expected.clone());
        assert!(matches!(hardware.arm_receive(address, bytes), Err(Error::InvalidInput)));
        fixture.assert_consumed();
        assert_eq!(fixture.log(), expected, "invalid RX input must not reach an effect");
    }

    let fixture = Fixture::new();
    let hardware = fixture.hardware();
    fixture.set_fifo(1);
    fixture.clear_log();
    let mut expected = vec![Event::Read("mfsr", 1)];
    expected.extend(cleanup());
    fixture.expect(expected.clone());
    assert!(matches!(
        hardware.arm_receive(0x29, receive_bytes!(3)),
        Err(Error::Busy)
    ));
    fixture.assert_consumed();
    assert_eq!(
        fixture.log(),
        expected,
        "busy RX FIFO must reject before DMA effects or commands"
    );
}
