//! Exercise every explicitly bound eDMA vector without an I2C interrupt.
//!
//! Each memory copy must complete, preserve its data, and invoke only its own
//! channel's ISR. A second handler in the binding records actual IRQ entry, so
//! merely observing a level-latched DONE bit cannot masquerade as IRQ coverage.
//! Exits through semihosting after all eight channels pass.

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};

use embassy_executor::Spawner;
use embassy_mcxa::dma::{Channel, DmaChannel, InterruptHandler, TransferOptions};
use embassy_mcxa::interrupt::typelevel::{Binding, Handler, Interrupt};
use embassy_mcxa::{Peri, bind_interrupts, peripherals::*};
use embassy_time::{Duration, Timer, WithTimeout};
use {defmt_rtt as _, embassy_mcxa as hal, panic_probe as _};

static SEEN: AtomicU32 = AtomicU32::new(0);

struct Record<const INDEX: usize>;

impl<I: Interrupt, const INDEX: usize> Handler<I> for Record<INDEX> {
    unsafe fn on_interrupt() {
        SEEN.fetch_or(1 << INDEX, Ordering::Relaxed);
    }
}

bind_interrupts!(struct Irqs {
    DMA0_CH0 => InterruptHandler<DMA0_CH0>, Record<0>;
    DMA0_CH1 => InterruptHandler<DMA0_CH1>, Record<1>;
    DMA0_CH2 => InterruptHandler<DMA0_CH2>, Record<2>;
    DMA0_CH3 => InterruptHandler<DMA0_CH3>, Record<3>;
    DMA0_CH4 => InterruptHandler<DMA0_CH4>, Record<4>;
    DMA0_CH5 => InterruptHandler<DMA0_CH5>, Record<5>;
    DMA0_CH6 => InterruptHandler<DMA0_CH6>, Record<6>;
    DMA0_CH7 => InterruptHandler<DMA0_CH7>, Record<7>;
});

async fn exercise<C: Channel>(peri: Peri<'_, C>, irq: impl Binding<C::Interrupt, InterruptHandler<C>>) {
    let mut channel = DmaChannel::new(peri, irq);
    let mut src = [0u8; 257];
    let mut dst = [0u8; 257];
    for (i, byte) in src.iter_mut().enumerate() {
        *byte = (i as u8).wrapping_mul(31).wrapping_add(C::INDEX as u8);
    }

    for _ in 0..4 {
        dst.fill(0);
        channel
            .mem_to_mem(&src, &mut dst, TransferOptions::COMPLETE_INTERRUPT)
            .unwrap()
            .with_timeout(Duration::from_millis(100))
            .await
            .expect("DMA completion timeout")
            .expect("DMA transfer failed");
        assert_eq!(src, dst);
        // A fast copy can finish before its future's first poll. Give its IRQ
        // a bounded opportunity to run, then require independent ISR evidence.
        Timer::after_millis(1).await;
        assert_eq!(SEEN.swap(0, Ordering::Relaxed), 1 << C::INDEX);
    }
    defmt::info!("DMA0_CH{}: data + bound interrupt PASS", C::INDEX);
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = hal::config::Config::default();
    config.clock_cfg.sirc.fro_lf_div = hal::clocks::config::Div8::from_divisor(1);
    let p = hal::init(config);
    exercise(p.DMA0_CH0, Irqs).await;
    exercise(p.DMA0_CH1, Irqs).await;
    exercise(p.DMA0_CH2, Irqs).await;
    exercise(p.DMA0_CH3, Irqs).await;
    exercise(p.DMA0_CH4, Irqs).await;
    exercise(p.DMA0_CH5, Irqs).await;
    exercise(p.DMA0_CH6, Irqs).await;
    exercise(p.DMA0_CH7, Irqs).await;
    defmt::info!("DMA interrupt bindings: ALL PASS (8 channels, 32 transfers)");
    Timer::after_millis(100).await;
    cortex_m_semihosting::debug::exit(cortex_m_semihosting::debug::EXIT_SUCCESS);
    unreachable!();
}
