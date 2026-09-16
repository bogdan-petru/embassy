//! MCXA integration for the short I2C service, separate from IP sequencing.
//! This concrete port retains clock/power, pins, paired FIFO requests, channel
//! and interrupt authority. It does not claim a second SoC is already supported.

use core::marker::PhantomData;

use super::effects;
use super::effects::Effects;
use super::endpoint::{ErasedEndpointPair, ErasedRxEndpoint, ErasedTxEndpoint, TxEndpoint};
use super::{Registers, mmio, release_sequence};
use crate::Peri;
use crate::clocks::periph_helpers::{Div4, Lpi2cClockSel, Lpi2cConfig};
use crate::clocks::{PoweredClock, WakeGuard, enable_and_reset};
use crate::dma::{Channel, DmaChannel, TransferOptions};
use crate::gpio::{AnyPin, SealedPin};
use crate::i2c::{Info, Instance, SclPin, SdaPin};
use crate::interrupt::typelevel::{Binding, Handler, Interrupt};
use i2c_one_shot_interface::Error;
use maitake_sync::WaitCell;

/// MCXA clock selection; the instance and effective frequency are derived from
/// the owned singleton and initialized clock tree. This unchanged timing profile
/// accepts ONLY 12 MHz and runs at 100 kHz. Other frequencies fail before START.
#[derive(Clone, Copy, Debug)]
pub struct ClockConfig {
    pub source: Lpi2cClockSel,
    pub div: Div4,
}

impl Default for ClockConfig {
    fn default() -> Self {
        Self {
            source: Lpi2cClockSel::FroLfDiv,
            div: Div4::no_div(),
        }
    }
}

/// Bind this handler to the chosen LPI2C vector with `bind_interrupts!`.
pub struct InterruptHandler<T: Instance>(PhantomData<T>);

impl<T: Instance> Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        // SAFETY INVARIANT: this ISR must not write, pop a FIFO, acquire an
        // exclusive accessor, or change clocks. Retained shared MSR/MFSR
        // handles permit pure reads without per-read unique acquisition, but
        // public PAC writes/pops still bypass ownership. A bounded AST policy
        // pins this wake path and its local callees; visibility/forbid alone
        // cannot enforce it. This is not compiler sealing of the whole HAL.
        // One-shot wake without a second MMIO owner. The task re-arms NVIC
        // after registering its waker, then observes the level-latched status.
        effects::wake_interrupt(T::Interrupt::disable, T::info().wait_cell());
    }
}

/// An owned, non-Clone MCXA binding consumed by `I2c::new`. There are no public
/// register/channel accessors or constructors from unrelated raw addresses.
/// Dropping an unused port releases its resources; forgetting retains them.
pub struct McxaI2cPort {
    pub(super) registers: mmio::Lpi2cRegisters<mmio::Unconfirmed>,
    pub(super) endpoints: ErasedEndpointPair,
    pub(super) effects: RealEffects,
}

/// A lifetime/power guard, not a physical clock-frequency lock. WakeGuard
/// inhibits deep sleep; stable root clocks rely on the HAL initialization
/// policy. Raw PAC writes, foreign code and another core can bypass it.
struct ClockLease {
    hz: u32,
    _wake: Option<WakeGuard>,
}

impl ClockLease {
    fn release(&mut self) {
        // Preserve the original one-shot's post-shutdown clock policy: the
        // peripheral gate stays enabled. This also permits read-only register
        // audits after Drop without re-enabling/resetting and hiding a defect.
        // Only the deep-sleep inhibition is released, after DMA quiescence.
        drop(self._wake.take());
    }
}

impl Drop for ClockLease {
    fn drop(&mut self) {
        self.release();
    }
}

fn validate_clock(hz: u32) -> Result<(), Error> {
    if hz == 12_000_000 { Ok(()) } else { Err(Error::Clock) }
}

impl McxaI2cPort {
    /// Acquire the normal Embassy resources and keep the IRQ disabled until the
    /// IP adapter confirms reset, installs timing, and confirms enable. Pins are
    /// muxed during acquisition, while the reset local engine is still disabled.
    pub fn acquire<T: Instance, C: Channel>(
        peripheral: Peri<'static, T>,
        scl: Peri<'static, impl SclPin<T>>,
        sda: Peri<'static, impl SdaPin<T>>,
        dma: Peri<'static, C>,
        irqs: impl Binding<T::Interrupt, InterruptHandler<T>>
        + Binding<C::Interrupt, crate::dma::InterruptHandler<C>>
        + Copy,
        config: ClockConfig,
    ) -> Result<Self, Error> {
        let config = Lpi2cConfig {
            power: PoweredClock::NormalEnabledDeepSleepDisabled,
            source: config.source,
            div: config.div,
            instance: T::CLOCK_INSTANCE,
        };
        let hz = crate::clocks::with_clocks(|clocks| config.functional_clock_hz(clocks))
            .ok_or(Error::Clock)?
            .map_err(|_| Error::Clock)?;
        validate_clock(hz)?;
        T::Interrupt::disable();
        let dma = DmaChannel::new(dma, irqs);
        // SAFETY: the consumed singleton excludes another safe owner; the
        // required wake-only interrupt is disabled and cannot access MMIO.
        let enabled = unsafe { enable_and_reset::<T>(&config) }.map_err(|_| Error::Clock)?;
        let clock = ClockLease {
            hz: enabled.freq,
            _wake: enabled.wake_guard,
        };
        validate_clock(clock.hz)?;
        // SAFETY: exclusive live mapping, reset issued by enable_and_reset.
        // This same consumed instance derives both directional endpoints.
        let (Registers { owned, .. }, endpoint) = unsafe { TxEndpoint::<T>::acquire(peripheral) };
        dma.quiesce();
        scl.mux();
        sda.mux();
        Ok(Self {
            registers: owned,
            endpoints: endpoint.erase(),
            effects: RealEffects {
                info: T::info(),
                dma,
                disable_irq: T::Interrupt::disable,
                unpend_irq: T::Interrupt::unpend,
                // SAFETY: acquire requires the exact instance's IRQ binding.
                enable_irq: || unsafe { T::Interrupt::enable() },
                scl: scl.into(),
                sda: sda.into(),
                clock,
            },
        })
    }

    /// Derived effective clock; no clock reconfiguration authority escapes.
    pub fn clock_hz(&self) -> u32 {
        self.effects.clock.hz
    }
}

pub(in crate::i2c::one_shot) struct RealEffects {
    info: &'static Info,
    dma: DmaChannel<'static>,
    disable_irq: fn(),
    unpend_irq: fn(),
    enable_irq: fn(),
    scl: Peri<'static, AnyPin>,
    sda: Peri<'static, AnyPin>,
    clock: ClockLease,
}

impl Effects for RealEffects {
    fn i2c_wait(&self) -> &WaitCell {
        self.info.wait_cell()
    }
    fn dma_wait(&self) -> &WaitCell {
        self.dma.wait_cell()
    }
    fn irq_disable(&self) {
        (self.disable_irq)();
    }
    fn irq_unpend(&self) {
        (self.unpend_irq)();
    }
    fn irq_enable(&self) {
        (self.enable_irq)();
    }
    fn now_us(&self) -> u64 {
        embassy_time::Instant::now().as_micros()
    }
    fn prepare(&mut self, bytes: &'static [u8], endpoint: &ErasedTxEndpoint) -> Result<(), Error> {
        // SAFETY: the enclosing private Hardware owns this endpoint/channel;
        // the immutable static source remains valid even if its future leaks.
        unsafe {
            self.dma
                .setup_write_to_peripheral(bytes, endpoint.address(), false, TransferOptions::COMPLETE_INTERRUPT)
                .map_err(|_| Error::Dma)
        }
    }
    fn select_request(&self, endpoint: &ErasedTxEndpoint) {
        // SAFETY: this request is inseparable from the same capability's byte
        // destination used to prepare the privately owned channel.
        unsafe {
            self.dma.set_request_source(endpoint.request());
        }
    }
    fn prepare_receive(&mut self, bytes: &mut [u8], endpoint: &ErasedRxEndpoint) -> Result<(), Error> {
        // SAFETY: the enclosing private ReceivingLease retains the exclusive
        // static destination until its RDDE-off -> quiesce -> fence exit. The
        // fixed endpoint is the paired MRDR byte source, never an arbitrary
        // caller-provided peripheral address.
        unsafe {
            self.dma
                .setup_read_from_peripheral(endpoint.address(), bytes, false, TransferOptions::COMPLETE_INTERRUPT)
                .map_err(|_| Error::Dma)
        }
    }
    fn select_receive_request(&self, endpoint: &ErasedRxEndpoint) {
        // SAFETY: the endpoint/request pair came from the same sealed LPI2C
        // singleton and is moved with the one owned DMA channel.
        unsafe {
            self.dma.set_request_source(endpoint.request());
        }
    }
    fn dma_interrupt_enable(&self) {
        self.dma.enable_interrupt();
    }
    fn dma_request_enable(&self) {
        // SAFETY: the adapter has prepared the retained static source first.
        unsafe {
            self.dma.enable_request();
        }
    }
    fn dma_error(&self) -> bool {
        self.dma.tcd().ch_es().read().err()
    }
    fn dma_done(&self) -> bool {
        self.dma.is_done()
    }
    fn quiesce(&self) {
        self.dma.quiesce();
    }
    fn release_pins(&mut self) {
        self.scl.set_as_disabled();
        self.sda.set_as_disabled();
    }
}

impl Drop for RealEffects {
    fn drop(&mut self) {
        // The adapter's normal/error/cancellation exits first turn requests off
        // and confirm abrupt reset. This final resource release also handles an
        // unused port. Quiescence is synchronous, not preempted by the deadline.
        release_sequence::release(self);
    }
}

impl release_sequence::ReleaseIo for RealEffects {
    fn disable_irq(&mut self) {
        self.irq_disable();
    }
    fn quiesce_dma(&mut self) {
        self.quiesce();
    }
    fn unpend_irq(&mut self) {
        self.irq_unpend();
    }
    fn disable_pins(&mut self) {
        self.release_pins();
    }
    fn release_clock_power(&mut self) {
        self.clock.release();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn port_rejects_other_clocks_without_changing_the_existing_timing_profile() {
        assert_eq!(validate_clock(12_000_000), Ok(()));
        for hz in [0, 6_000_000, 11_999_999, 12_000_001, 24_000_000, u32::MAX] {
            assert_eq!(validate_clock(hz), Err(Error::Clock));
        }
    }
}
