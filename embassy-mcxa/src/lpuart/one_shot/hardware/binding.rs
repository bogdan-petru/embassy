//! MCXA integration, deliberately separate from the LPUART transfer algorithm.
//! This is the first concrete port, not a claim of matching register semantics on other SoCs.
//! Clock selection, pin mux, endpoint/request pairing and IRQ/DMA HAL calls live
//! here; the protocol/vocabulary crates depend on none of these MCXA types.

use core::marker::PhantomData;
use core::task::Context;

use uart_one_shot_interface::Error;

use super::mmio;
use super::release_sequence::{self, ReleaseIo};
use crate::Peri;
use crate::clocks::periph_helpers::{Div4, LpuartClockSel, LpuartConfig};
use crate::clocks::{PoweredClock, WakeGuard, enable_and_reset};
use crate::dma::{Channel, DmaChannel, DmaRequest, TransferOptions};
use crate::gpio::{AnyPin, SealedPin};
use crate::interrupt::typelevel::{Binding, Handler, Interrupt};
use crate::lpuart::{Instance, TxPin};

/// Application-selected MCXA clock policy. Instance and frequency are derived
/// from the consumed singleton and initialized HAL clock tree, never supplied
/// as an unrelated integer. This finite TX port always inhibits deep sleep.
#[derive(Clone, Copy, Debug)]
pub struct ClockConfig {
    pub source: LpuartClockSel,
    pub div: Div4,
}

impl Default for ClockConfig {
    fn default() -> Self {
        Self {
            source: LpuartClockSel::FroLfDiv,
            div: Div4::no_div(),
        }
    }
}

/// Bind this wake-only handler AND the selected eDMA channel's handler.
pub struct InterruptHandler<T: Instance>(PhantomData<T>);

impl<T: Instance> Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        // Wake-only: no PAC reads, FIFO pops, writes or second MMIO owner.
        T::Interrupt::disable();
        T::state().tx_waker.wake();
    }
}

/// Owned MCXA port capability. There is no raw mapping/channel/clock accessor,
/// Clone, or public constructor from loose addresses and request numbers.
/// `Uart::new` consumes this bundle; forgetting its future retains the bundle.
pub struct McxaUartPort {
    pub(super) registers: mmio::UartRegisters<mmio::Unconfirmed>,
    pub(super) resources: Resources,
}

/// This is a lifetime guard within the supported HAL ownership model, NOT a
/// physical frequency lock. WakeGuard only inhibits deep sleep. HAL root-clock
/// initialization is once-only; PAC writes/foreign cores can bypass that rule.
struct ClockLease {
    hz: u32,
    disable: Option<fn()>,
    _wake: Option<WakeGuard>,
}

impl Drop for ClockLease {
    fn drop(&mut self) {
        // Resources::drop drains DMA before this field is dropped. The power
        // guard is still alive while the uniquely owned UART gate is disabled.
        self.release();
    }
}

impl ClockLease {
    fn release(&mut self) {
        if let Some(disable) = self.disable.take() {
            disable();
        }
    }
}

pub(super) struct Resources {
    dma: DmaChannel<'static>,
    destination: *mut u8,
    request: DmaRequest,
    wait: &'static maitake_sync::WaitCell,
    disable_irq: fn(),
    unpend_irq: fn(),
    enable_irq: fn(),
    pin: Peri<'static, AnyPin>,
    clock: ClockLease,
}

impl McxaUartPort {
    /// Acquire and clock the selected instance. UART timing/enable is the IP
    /// adapter's next, confirmed operation. A dropped unused port releases its
    /// pin/gate/power guard without ever enabling a DMA request.
    pub fn acquire<T: Instance, C: Channel>(
        peripheral: Peri<'static, T>,
        tx: Peri<'static, impl TxPin<T>>,
        dma: Peri<'static, C>,
        irqs: impl Binding<T::Interrupt, InterruptHandler<T>>
        + Binding<C::Interrupt, crate::dma::InterruptHandler<C>>
        + Copy,
        config: ClockConfig,
    ) -> Result<Self, Error> {
        T::Interrupt::disable();
        let dma = DmaChannel::new(dma, irqs);
        let config = LpuartConfig {
            power: PoweredClock::NormalEnabledDeepSleepDisabled,
            source: config.source,
            div: config.div,
            instance: T::CLOCK_INSTANCE,
        };
        // SAFETY: the consumed singleton excludes another safe HAL owner;
        // this UART's correctly bound wake-only interrupt is disabled above.
        let enabled = unsafe { enable_and_reset::<T>(&config) }.map_err(|_| Error::Clock)?;
        let clock = ClockLease {
            hz: enabled.freq,
            // SAFETY: the private lease drops only after owned DMA quiescence.
            disable: Some(|| unsafe { crate::clocks::disable::<T>() }),
            _wake: enabled.wake_guard,
        };
        if clock.hz == 0 {
            return Err(Error::Clock);
        }
        let pac = T::info().regs;
        mmio::check_layout(pac);
        // SAFETY: clocked exclusive mapping, bound wake-only ISR disabled,
        // consumed instance also selects the matching DATA/request pair.
        let registers = unsafe { mmio::UartRegisters::new(pac.as_ptr().cast()) };
        tx.as_tx();
        dma.quiesce();
        T::Interrupt::unpend();
        // Erase the zero-sized singleton brand only here, after deriving ALL
        // paired resources. It is spent, never returned or reconstructed.
        drop(peripheral);
        Ok(Self {
            registers,
            resources: Resources {
                dma,
                destination: pac.data().as_ptr().cast(),
                request: T::TX_DMA_REQUEST,
                wait: &T::state().tx_waker,
                disable_irq: T::Interrupt::disable,
                unpend_irq: T::Interrupt::unpend,
                // SAFETY: acquire requires this instance's handler binding.
                enable_irq: || unsafe { T::Interrupt::enable() },
                pin: tx.into(),
                clock,
            },
        })
    }

    /// Effective peripheral frequency, after the selected divider. Reading it
    /// does not expose the guard or permit clock reconfiguration.
    pub fn clock_hz(&self) -> u32 {
        self.resources.clock_hz()
    }
}

impl Resources {
    pub(super) fn clock_hz(&self) -> u32 {
        self.clock.hz
    }

    pub(super) fn quiesce(&self) -> bool {
        self.dma.quiesce()
    }

    pub(super) fn prepare(&self, bytes: &'static [u8]) -> Result<(), Error> {
        self.dma.quiesce();
        // SAFETY: immutable static source outlives forgotten owners. Mapping
        // and request were inseparably derived from one singleton. No request
        // is enabled yet; the CPU->DMA owner transition follows preparation.
        unsafe {
            self.dma.set_request_source(self.request);
            self.dma
                .setup_write_to_peripheral(bytes, self.destination, false, TransferOptions::COMPLETE_INTERRUPT)
        }
        .map_err(|_| Error::Dma)
    }

    pub(super) fn start_dma(&self) {
        self.dma.enable_interrupt();
        // SAFETY: only the private adapter calls this, after prepare succeeded
        // and the complete StreamingLease retained these resources and source.
        unsafe { self.dma.enable_request() };
    }

    pub(super) fn register_dma_waker(&self, cx: &mut Context<'_>) {
        while self.dma.wait_cell().poll_wait(cx).is_ready() {}
    }

    pub(super) fn dma_error(&self) -> bool {
        self.dma.tcd().ch_es().read().err()
    }

    pub(super) fn dma_done(&self) -> bool {
        self.dma.is_done()
    }

    pub(super) fn register_uart_waker(&self, cx: &mut Context<'_>) {
        while self.wait.poll_wait(cx).is_ready() {}
    }

    pub(super) fn enable_uart_irq(&self) {
        (self.enable_irq)();
    }

    pub(super) fn disable_uart_irq(&self) {
        (self.disable_irq)();
    }

    pub(super) fn unpend_uart_irq(&self) {
        (self.unpend_irq)();
    }
}

impl Drop for Resources {
    fn drop(&mut self) {
        // Last-resort lifetime ordering, including constructor errors. Normal
        // adapter paths already shut peripheral requests off and reset first.
        // Quiesce cannot be preempted by the async transfer's deadline.
        release_sequence::release(self);
        // Clock gate is now off; subsequent field destruction releases the
        // channel and power guard. ClockLease's fallback is already defused.
    }
}

impl ReleaseIo for Resources {
    fn disable_irq(&mut self) {
        self.disable_uart_irq();
    }
    fn quiesce_dma(&mut self) {
        self.dma.quiesce();
    }
    fn unpend_irq(&mut self) {
        self.unpend_uart_irq();
    }
    fn disable_pin(&mut self) {
        self.pin.set_as_disabled();
    }
    fn release_clock(&mut self) {
        self.clock.release();
    }
}
