//! Host-only effect recorder. Real adapter methods and WaitCells run unchanged.
//! The model knows addresses/access direction and NVIC eligibility, not silicon
//! timing, bus behavior, or the internals of the DMA HAL's quiescence operation.

extern crate std;

use std::boxed::Box;
use std::cell::{RefCell, UnsafeCell};
use std::collections::VecDeque;
use std::rc::Rc;
use std::vec::Vec;

use i2c_one_shot_core::Backend;
use i2c_one_shot_interface::Cpu;
use maitake_sync::WaitCell;

use super::effects::{Effects, wake_interrupt};
use super::endpoint::{ErasedEndpointPair, ErasedRxEndpoint, ErasedTxEndpoint};
use super::{DeviceHardware, Error, Hardware, mmio};

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Event {
    Read(&'static str, u32),
    Write(&'static str, u32),
    DmaPrepare(usize),
    DmaSource,
    DmaReceivePrepare(usize),
    DmaReceiveSource,
    DmaInterruptEnable,
    DmaRequestEnable,
    DmaError(bool),
    DmaDone(bool),
    QuiesceEnter,
    QuiesceReturn,
    IrqDisable,
    IrqUnpend,
    IrqEnable,
    I2cWait(bool),
    DmaWait(bool),
    PinsRelease,
}

#[derive(Clone, Copy, Debug)]
pub enum Injection {
    Status(u32),
    DmaDone,
}

struct Recorder {
    // Box keeps the live host mapping stable even when the recorder is moved.
    memory: Box<[UnsafeCell<u32>]>,
    log: Vec<Event>,
    script: Option<VecDeque<Event>>,
    injection: Option<(usize, Injection)>,
    steps: usize,
    irq_enabled: bool,
    irq_pending: bool,
    dma_done: bool,
    dma_error: bool,
    fail_prepare: bool,
    now_us: u64,
    clock_step: u64,
    // Per MCR write: Some(n) delays acceptance by n reads; None never accepts.
    control_delays: VecDeque<Option<usize>>,
    control_pending: Option<(u32, Option<usize>)>,
    i2c: Rc<WaitCell>,
    dma: Rc<WaitCell>,
}

impl Recorder {
    fn base(&self) -> *mut u8 {
        // Derive all MMIO handles from shared UnsafeCell storage. as_mut_ptr()
        // would first borrow the slice exclusively; simulated device writes
        // through word()/set_word() would then invalidate that derived pointer.
        // UnsafeCell permits those mutations without an exclusive slice borrow.
        self.memory.as_ptr().cast_mut().cast::<u8>()
    }

    fn word(&self, name: &str) -> u32 {
        // SAFETY: test events execute on this thread. UnsafeCell models device
        // mutation without creating shared references to mutable register data.
        unsafe { *self.memory[index(name)].get() }
    }
    fn set_word(&self, name: &str, value: u32) {
        // SAFETY: same single-threaded, live fixture contract as word().
        unsafe {
            *self.memory[index(name)].get() = value;
        }
    }
}

std::thread_local! {
    static ACTIVE: RefCell<Option<Rc<RefCell<Recorder>>>> = const { RefCell::new(None) };
}

fn active() -> Rc<RefCell<Recorder>> {
    ACTIVE.with(|slot| {
        slot.borrow()
            .as_ref()
            .expect("MMIO outside registered live fixture")
            .clone()
    })
}

fn index(name: &str) -> usize {
    mmio::REGISTER_ACCESS
        .iter()
        .find(|entry| entry.0 == name)
        .expect("known register")
        .1
        / 4
}

fn deliver() {
    let state = active();
    let wait = {
        let mut state = state.borrow_mut();
        if !state.irq_enabled || !state.irq_pending {
            return;
        }
        state.irq_pending = false;
        state.i2c.clone()
    };
    // Re-entrant logging MUST happen outside every RefCell borrow. Execute
    // the production wake-only ISR body, including its NVIC masking effect.
    wake_interrupt(|| record(Event::IrqDisable), &wait);
}

fn inject(event: Injection) {
    let state = active();
    let dma_wake = {
        let mut state = state.borrow_mut();
        match event {
            Injection::Status(value) => {
                state.set_word("msr", value);
                if value & state.word("mier") != 0 {
                    state.irq_pending = true;
                }
                None
            }
            Injection::DmaDone => {
                state.dma_done = true;
                Some(state.dma.clone())
            }
        }
    };
    if let Some(wait) = dma_wake {
        wait.wake();
    }
    deliver();
}

fn record(event: Event) {
    let state = active();
    let injection = {
        let mut state = state.borrow_mut();
        if let Some(script) = &mut state.script {
            assert_eq!(script.pop_front().as_ref(), Some(&event), "EFFECT_SCRIPT_ORDER");
        }
        match event {
            Event::IrqDisable => state.irq_enabled = false,
            Event::IrqUnpend => state.irq_pending = false,
            Event::IrqEnable => state.irq_enabled = true,
            _ => {}
        }
        // I2C events are level-latched; enabling an interrupt for an existing
        // status must pend it too. This is not an unconditional ISR callback.
        if state.word("msr") & state.word("mier") != 0 {
            state.irq_pending = true;
        }
        state.log.push(event);
        state.steps += 1;
        if state.injection.is_some_and(|(step, _)| step == state.steps) {
            state.injection.take().map(|(_, event)| event)
        } else {
            None
        }
    };
    if let Some(event) = injection {
        inject(event);
    }
    deliver();
}

fn validate_access(state: &Recorder, address: usize, width: usize, write: bool) -> &'static str {
    assert_eq!(width, 4, "MMIO_WIDTH");
    let base = state.base() as usize;
    let offset = address.checked_sub(base).expect("MMIO_ADDRESS");
    assert!(offset < mmio::REGION_LEN && offset % 4 == 0, "MMIO_ADDRESS");
    let entry = mmio::REGISTER_ACCESS
        .iter()
        .find(|entry| entry.1 == offset)
        .expect("MMIO_PADDING");
    assert!(if write { entry.3 } else { entry.2 }, "MMIO_DIRECTION");
    entry.0
}

struct RecordedMmio;

macro_rules! reject_width {
    ($read:ident, $write:ident, $word:ty) => {
        unsafe fn $read(_src: *const $word) -> $word {
            panic!("MMIO_WIDTH")
        }
        unsafe fn $write(_dst: *mut $word, _value: $word) {
            panic!("MMIO_WIDTH")
        }
    };
}

impl safe_mmio::MmioOps for RecordedMmio {
    reject_width!(read_u8, write_u8, u8);
    reject_width!(read_u16, write_u16, u16);
    reject_width!(read_u64, write_u64, u64);

    unsafe fn read_u32(src: *const u32) -> u32 {
        let state = active();
        let (name, value) = {
            let mut state = state.borrow_mut();
            let name = validate_access(&state, src as usize, 4, false);
            if name == "mcr" {
                if let Some((word, delay)) = state.control_pending.take() {
                    match delay {
                        Some(0) => state.set_word("mcr", word),
                        Some(n) => state.control_pending = Some((word, Some(n - 1))),
                        None => state.control_pending = Some((word, None)),
                    }
                }
            }
            // A script may supply the next device read. Unexpected direction,
            // register, value or order is still checked by record below.
            let value = match state.script.as_ref().and_then(|script| script.front()) {
                Some(Event::Read(expected, value)) if *expected == name => *value,
                _ => {
                    // SAFETY: checked aligned u32 inside this still-owned host
                    // allocation. No unknown address reaches volatile access.
                    unsafe { src.read_volatile() }
                }
            };
            // MRDR pops a receive-FIFO entry. The recorder has one modeled
            // entry, so consuming it makes a second read observably empty;
            // MSR/MFSR deliberately retain their backing values instead.
            if name == "mrdr" {
                state.set_word("mrdr", 0);
            }
            (name, value)
        };
        record(Event::Read(name, value));
        value
    }

    unsafe fn write_u32(dst: *mut u32, value: u32) {
        let state = active();
        let name = {
            let mut state = state.borrow_mut();
            let name = validate_access(&state, dst as usize, 4, true);
            if name == "mcr" {
                let delay = state.control_delays.pop_front().unwrap_or(Some(0));
                state.control_pending = Some((value, delay));
            } else {
                // SAFETY: validated live fixture mapping; never physical MMIO.
                unsafe {
                    dst.write_volatile(value);
                }
            }
            name
        };
        record(Event::Write(name, value));
    }
}

// Exactly one provider in the HAL library-test executable. The fork's
// integration/doctest executables own independent fixture registrations.
safe_mmio::set_mmio_ops!(RecordedMmio);

pub struct Fixture {
    state: Rc<RefCell<Recorder>>,
}

impl Fixture {
    pub fn new() -> Self {
        let state = Rc::new(RefCell::new(Recorder {
            memory: (0..mmio::REGION_LEN / 4).map(|_| UnsafeCell::new(0)).collect(),
            log: Vec::new(),
            script: None,
            injection: None,
            steps: 0,
            irq_enabled: false,
            irq_pending: false,
            dma_done: false,
            dma_error: false,
            fail_prepare: false,
            now_us: 0,
            clock_step: 1,
            control_delays: VecDeque::new(),
            control_pending: None,
            i2c: Rc::new(WaitCell::new()),
            dma: Rc::new(WaitCell::new()),
        }));
        ACTIVE.with(|slot| {
            let mut slot = slot.borrow_mut();
            assert!(slot.is_none(), "nested MMIO fixture");
            *slot = Some(state.clone());
        });
        Self { state }
    }

    pub fn hardware(&self) -> Hardware<TestEffects> {
        let hardware = self.hardware_recorded();
        self.clear_log();
        hardware
    }

    pub fn hardware_recorded(&self) -> Hardware<TestEffects> {
        // Check before deriving another pointer into an already-owned region.
        assert_eq!(Rc::strong_count(&self.state), 2, "fixture already owned");
        let (base, i2c, dma) = {
            let state = self.state.borrow();
            (state.base(), state.i2c.clone(), state.dma.clone())
        };
        // SAFETY: registered aligned live RAM, no concurrent
        // CPU accessor. TestEffects retains the allocation through shutdown.
        let registers = unsafe { mmio::Lpi2cRegisters::<mmio::Unconfirmed>::new(base) };
        let effects = TestEffects {
            state: self.state.clone(),
            i2c,
            dma,
        };
        let registers = registers
            .reset_confirmed(|| effects.now_us())
            .expect("fixture reset confirmation");
        let registers = registers
            .enable_confirmed(|| effects.now_us())
            .expect("fixture enable confirmation");
        // SAFETY: this fixture owns both fixed FIFO addresses through
        // TestEffects. The opaque pair cannot be split or repointed by tests.
        let endpoints = unsafe {
            ErasedEndpointPair::from_test_addresses(
                base.wrapping_add(index("mtdr") * 4),
                base.wrapping_add(index("mrdr") * 4).cast_const(),
            )
        };
        let device = DeviceHardware::from_owned(registers, endpoints, effects);
        Hardware::new(Cpu::new(device))
    }

    // Keep direct-map property checks on the actual generated owner and the
    // same registered recorder as the adapter tests. The owner cannot escape
    // this callback, so it cannot coexist with a Hardware owner of this RAM.
    fn with_direct_enabled_registers<R>(&self, f: impl FnOnce(&mut mmio::Lpi2cRegisters<mmio::Enabled>) -> R) -> R {
        assert_eq!(Rc::strong_count(&self.state), 2, "fixture already owned");
        // Retain the same ownership claim as TestEffects through construction,
        // callback and unwinding. A pre-check alone would allow re-entrant
        // hardware()/direct acquisition while these shared handles still live.
        let _owner_guard = self.state.clone();
        let base = self.state.borrow().base();
        // SAFETY: this callback is the sole direct owner of the live fixture
        // mapping, and the registered recorder remains installed throughout.
        let registers = unsafe { mmio::Lpi2cRegisters::<mmio::Unconfirmed>::new(base) };
        let registers = registers
            .reset_confirmed(|| self.now_us())
            .expect("fixture reset confirmation");
        let mut registers = registers
            .enable_confirmed(|| self.now_us())
            .expect("fixture enable confirmation");
        self.clear_log();
        f(&mut registers)
    }

    pub fn control_delays(&self, delays: &[Option<usize>]) {
        self.state.borrow_mut().control_delays = delays.iter().copied().collect();
    }

    pub fn stop_clock(&self) {
        self.state.borrow_mut().clock_step = 0;
    }

    pub fn set_status(&self, value: u32) {
        self.state.borrow().set_word("msr", value);
    }
    pub fn set_fifo(&self, value: u32) {
        self.state.borrow().set_word("mfsr", value);
    }
    pub fn set_receive(&self, value: u32) {
        self.state.borrow().set_word("mrdr", value);
    }
    pub fn set_dma(&self, done: bool, error: bool) {
        let mut state = self.state.borrow_mut();
        state.dma_done = done;
        state.dma_error = error;
    }
    pub fn fail_prepare(&self, fail: bool) {
        self.state.borrow_mut().fail_prepare = fail;
    }
    pub fn stale_i2c_wake(&self) {
        let wait = self.state.borrow().i2c.clone();
        wait.wake();
    }
    pub fn stale_dma_wake(&self) {
        let wait = self.state.borrow().dma.clone();
        wait.wake();
    }
    pub fn fire(&self, event: Injection) {
        inject(event);
    }
    pub fn inject_after(&self, steps: usize, event: Injection) {
        assert!(steps > 0);
        let mut state = self.state.borrow_mut();
        assert!(state.injection.is_none(), "schedule already installed");
        state.injection = Some((state.steps + steps, event));
    }
    pub fn clear_log(&self) {
        self.state.borrow_mut().log.clear();
    }
    pub fn log(&self) -> Vec<Event> {
        self.state.borrow().log.clone()
    }
    pub fn expect(&self, script: Vec<Event>) {
        let mut state = self.state.borrow_mut();
        assert!(state.script.is_none(), "script already installed");
        state.script = Some(script.into());
    }
    pub fn assert_consumed(&self) {
        let state = self.state.borrow();
        assert!(
            state.script.as_ref().is_none_or(VecDeque::is_empty),
            "EFFECT_SCRIPT_REMAINS"
        );
        assert!(state.injection.is_none(), "EFFECT_INJECTION_NOT_REACHED");
    }

    fn now_us(&self) -> u64 {
        let mut state = self.state.borrow_mut();
        let now = state.now_us;
        state.now_us = now.saturating_add(state.clock_step);
        now
    }
}

impl Drop for Fixture {
    fn drop(&mut self) {
        // Restore TLS even while unwinding. The real Hardware must have been
        // dropped first; its effects hold this memory alive until then.
        let prior = ACTIVE.with(|slot| slot.borrow_mut().take());
        drop(prior);
        if !std::thread::panicking() {
            self.assert_consumed();
            assert_eq!(Rc::strong_count(&self.state), 1, "fixture dropped before hardware");
        }
    }
}

pub struct TestEffects {
    state: Rc<RefCell<Recorder>>,
    i2c: Rc<WaitCell>,
    dma: Rc<WaitCell>,
}

impl Effects for TestEffects {
    fn i2c_wait(&self) -> &WaitCell {
        &self.i2c
    }
    fn dma_wait(&self) -> &WaitCell {
        &self.dma
    }
    fn irq_disable(&self) {
        record(Event::IrqDisable);
    }
    fn irq_unpend(&self) {
        record(Event::IrqUnpend);
    }
    fn irq_enable(&self) {
        record(Event::IrqEnable);
    }
    fn prepare(&mut self, bytes: &'static [u8], endpoint: &ErasedTxEndpoint) -> Result<(), Error> {
        {
            let state = self.state.borrow();
            assert_eq!(
                validate_access(&state, endpoint.address() as usize, 4, true),
                "mtdr",
                "DMA_ENDPOINT"
            );
        }
        record(Event::DmaPrepare(bytes.len()));
        if self.state.borrow().fail_prepare {
            Err(Error::Dma)
        } else {
            Ok(())
        }
    }
    fn select_request(&self, endpoint: &ErasedTxEndpoint) {
        assert_eq!(endpoint.request(), crate::dma::DmaRequest::Lpi2C0Tx, "DMA_REQUEST_PAIR");
        record(Event::DmaSource);
    }
    fn prepare_receive(&mut self, bytes: &mut [u8], endpoint: &ErasedRxEndpoint) -> Result<(), Error> {
        {
            let state = self.state.borrow();
            assert_eq!(
                validate_access(&state, endpoint.address() as usize, 4, false),
                "mrdr",
                "DMA_RX_ENDPOINT"
            );
        }
        record(Event::DmaReceivePrepare(bytes.len()));
        if self.state.borrow().fail_prepare {
            Err(Error::Dma)
        } else {
            Ok(())
        }
    }
    fn select_receive_request(&self, endpoint: &ErasedRxEndpoint) {
        assert_eq!(
            endpoint.request(),
            crate::dma::DmaRequest::Lpi2C0Rx,
            "DMA_RX_REQUEST_PAIR"
        );
        record(Event::DmaReceiveSource);
    }
    fn dma_interrupt_enable(&self) {
        record(Event::DmaInterruptEnable);
    }
    fn dma_request_enable(&self) {
        record(Event::DmaRequestEnable);
    }
    fn dma_error(&self) -> bool {
        let error = self.state.borrow().dma_error;
        record(Event::DmaError(error));
        error
    }
    fn dma_done(&self) -> bool {
        let done = self.state.borrow().dma_done;
        record(Event::DmaDone(done));
        done
    }
    fn quiesce(&self) {
        record(Event::QuiesceEnter);
        record(Event::QuiesceReturn);
    }
    fn release_pins(&mut self) {
        record(Event::PinsRelease);
    }
    fn now_us(&self) -> u64 {
        let mut state = self.state.borrow_mut();
        let now = state.now_us;
        state.now_us = now.saturating_add(state.clock_step);
        now
    }
    fn i2c_wait_observed(&self, ready: bool) {
        record(Event::I2cWait(ready));
    }
    fn dma_wait_observed(&self, ready: bool) {
        record(Event::DmaWait(ready));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn direct_map_callback_excludes_nested_owners_and_releases_on_unwind() {
        use std::panic::{AssertUnwindSafe, catch_unwind};
        let fixture = Fixture::new();
        fixture.with_direct_enabled_registers(|_| {
            assert!(catch_unwind(AssertUnwindSafe(|| fixture.hardware())).is_err());
            assert!(catch_unwind(AssertUnwindSafe(|| fixture.with_direct_enabled_registers(|_| ()))).is_err());
        });
        assert!(
            catch_unwind(AssertUnwindSafe(|| {
                fixture.with_direct_enabled_registers(|_| panic!("deliberate callback unwind"));
            }))
            .is_err()
        );
        // The RAII claim is gone; a fresh owner can now safely be acquired.
        drop(fixture.hardware());
    }

    #[test]
    fn retained_owner_size_is_measured_not_assumed_free() {
        let size = core::mem::size_of::<mmio::Lpi2cRegisters<mmio::Enabled>>();
        std::println!("one-shot register owner: {size} bytes; baseline one pointer");
        assert_eq!(size, 3 * core::mem::size_of::<usize>());
    }

    #[test]
    fn direct_map_retains_pure_status_and_consumes_receive_data() {
        let fixture = Fixture::new();
        fixture.set_status(0x1122_3344);
        fixture.set_fifo(0x5566_7788);
        fixture.set_receive(0x99aa_bbcc);

        let observed = fixture.with_direct_enabled_registers(|registers| {
            (
                registers.read_msr(),
                registers.read_msr(),
                registers.read_mfsr(),
                registers.read_mfsr(),
                registers.read_mrdr(),
                registers.read_mrdr(),
            )
        });

        assert_eq!(
            observed,
            (0x1122_3344, 0x1122_3344, 0x5566_7788, 0x5566_7788, 0x99aa_bbcc, 0,)
        );
        assert_eq!(fixture.state.borrow().word("msr"), 0x1122_3344);
        assert_eq!(fixture.state.borrow().word("mfsr"), 0x5566_7788);
        assert_eq!(fixture.state.borrow().word("mrdr"), 0);
        assert_eq!(
            fixture.log(),
            std::vec![
                Event::Read("msr", 0x1122_3344),
                Event::Read("msr", 0x1122_3344),
                Event::Read("mfsr", 0x5566_7788),
                Event::Read("mfsr", 0x5566_7788),
                Event::Read("mrdr", 0x99aa_bbcc),
                Event::Read("mrdr", 0),
            ]
        );
    }

    #[test]
    fn scripted_mmio_read_and_cleanup_execute_real_adapter() {
        let fixture = Fixture::new();
        let mut hardware = fixture.hardware();
        // Backing MFSR is zero, but the device script reports a nonempty FIFO.
        // This exercises both linker dispatch and scripted reads, not just RAM.
        fixture.expect(std::vec![
            Event::Read("mfsr", 1),
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
        ]);
        assert!(hardware.try_stop().is_pending());
        drop(hardware);
        fixture.assert_consumed();
    }

    #[test]
    fn scripts_fail_closed_and_fixture_recovers_from_unwind() {
        assert!(
            std::panic::catch_unwind(|| {
                let fixture = Fixture::new();
                fixture.expect(std::vec![Event::DmaSource]);
                record(Event::DmaInterruptEnable);
            })
            .is_err()
        );
        assert!(
            std::panic::catch_unwind(|| {
                let fixture = Fixture::new();
                fixture.expect(std::vec![Event::DmaSource]);
                fixture.assert_consumed();
            })
            .is_err()
        );
        let fixture = Fixture::new();
        assert!(std::panic::catch_unwind(Fixture::new).is_err());
        drop(fixture.hardware());
    }

    #[test]
    fn independent_threads_have_independent_live_regions() {
        let workers: Vec<_> = (0..4)
            .map(|_| {
                std::thread::spawn(|| {
                    let fixture = Fixture::new();
                    let hardware = fixture.hardware();
                    let hardware = hardware
                        .start(0x2a)
                        .unwrap_or_else(|error| panic!("start failed: {error:?}"));
                    drop(hardware);
                    assert_eq!(fixture.log()[0], Event::Read("msr", 0));
                })
            })
            .collect();
        for worker in workers {
            worker.join().unwrap();
        }
        assert!(ACTIVE.with(|slot| slot.borrow().is_none()));
    }

    #[test]
    fn recorder_rejects_unknown_width_address_and_direction() {
        let fixture = Fixture::new();
        let state = fixture.state.borrow();
        let base = state.memory.as_ptr() as usize;
        for (address, width, write) in [
            (base, 4, false),
            (base + index("msr") * 4, 2, false),
            (base + index("msr") * 4, 4, true),
            (base + index("mtdr") * 4, 4, false),
            (base + mmio::REGION_LEN, 4, false),
            (base + 1, 4, false),
        ] {
            assert!(
                std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| validate_access(
                    &state, address, width, write
                )))
                .is_err()
            );
        }
    }
}
