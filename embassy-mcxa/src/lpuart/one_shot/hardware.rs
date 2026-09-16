//! Layer 2: selected LPUART IP behavior over an owned SoC resource binding.
//! No clock-source, pin-mux, channel selection or interrupt-number policy here.
//! The upper protocol and vocabulary remain separate PAC-free shared crates.

mod binding;
mod mmio;
mod release_sequence;
mod reset_sequence;

use core::task::{Context, Poll};

use uart_one_shot_interface::{Device, DmaResult, Error};

use binding::Resources;
pub use binding::{ClockConfig, InterruptHandler, McxaUartPort};
use reset_sequence::BaudConfig;

/// CPU authority. The instance, register mapping, endpoint/request pairing and
/// channel are private and move together; upper layers cannot extract them.
pub(super) struct DeviceHardware {
    parts: Option<CpuParts>,
}

struct CpuParts {
    registers: mmio::UartRegisters<mmio::Enabled>,
    resources: Resources,
    timing: BaudConfig,
}

/// DMA authority retains immutable static source storage even if forgotten.
/// No CPU accessor, mutable buffer, channel or early-completion token escapes.
pub(super) struct StreamingLease {
    parts: Option<DmaParts>,
}

struct DmaParts {
    registers: mmio::UartRegisters<mmio::Streaming>,
    resources: Resources,
    timing: BaudConfig,
    _source: &'static [u8],
}

impl DeviceHardware {
    pub(super) fn new(port: McxaUartPort) -> Result<Self, Error> {
        let McxaUartPort { registers, resources } = port;
        let timing = BaudConfig::new(resources.clock_hz())?;
        let registers = registers.reset_confirmed(now_us)?.configure_confirmed(timing, now_us)?;
        Ok(Self {
            parts: Some(CpuParts {
                registers,
                resources,
                timing,
            }),
        })
    }
}

fn now_us() -> u64 {
    embassy_time::Instant::now().as_micros()
}

// SAFETY: immutable static source and the uniquely owned channel/endpoint stay
// together. Pending retains them; every completion/error/drop quiesces the
// channel before CPU authority or resources return. COMPLETE means FIFO-fed
// and quiescent, NOT wire complete; only fresh STAT.TC grants that observation.
unsafe impl Device for DeviceHardware {
    type InFlight = StreamingLease;

    fn arm(mut self, bytes: &'static [u8]) -> Result<StreamingLease, Error> {
        if !(1..=32).contains(&bytes.len()) {
            return Err(Error::InvalidInput);
        }
        let parts = self.parts.as_mut().expect("CPU owner exists until consumed");
        parts.resources.prepare(bytes)?;
        let CpuParts {
            registers,
            resources,
            timing,
        } = self.parts.take().expect("owned CPU pair");
        // into_arm_dma enables peripheral requests while channel ERQ is still
        // off. The complete lease owns the static source before ERQ is enabled;
        // there is no await or source-memory access in that construction gap.
        let mut lease = StreamingLease {
            parts: Some(DmaParts {
                registers: registers.into_arm_dma(timing),
                resources,
                timing,
                _source: bytes,
            }),
        };
        let parts = lease.parts.as_mut().expect("new DMA pair");
        parts.resources.start_dma();
        Ok(lease)
    }

    fn poll_dma(mut lease: StreamingLease, cx: &mut Context<'_>) -> DmaResult<Self> {
        let parts = lease.parts.as_mut().expect("DMA pair exists until consumed");
        // A stale WOKEN token returns Ready WITHOUT registering this waker.
        // Drain tokens until registration sticks, then inspect level state.
        parts.resources.register_dma_waker(cx);
        if parts.resources.dma_error() {
            return DmaResult::Failed(Error::Dma); // Drop shuts requests down first.
        }
        if !parts.resources.dma_done() {
            return DmaResult::Pending(lease);
        }
        let DmaParts {
            registers,
            resources,
            timing,
            ..
        } = lease.parts.take().expect("owned DMA pair");
        match registers.disable_dma_and_restore(timing, || resources.quiesce(), now_us) {
            Ok(registers) => {
                // Recheck after the last active minor loop has drained: a
                // fault racing the first snapshot must not become success.
                let failed = resources.dma_error();
                let device = Self {
                    parts: Some(CpuParts {
                        registers,
                        resources,
                        timing,
                    }),
                };
                if failed {
                    drop(device);
                    DmaResult::Failed(Error::Dma)
                } else {
                    DmaResult::Complete(device)
                }
            }
            Err(error) => DmaResult::Failed(error),
        }
    }

    fn poll_wire(&mut self, cx: &mut Context<'_>) -> Poll<Result<(), Error>> {
        let parts = self.parts.as_mut().expect("CPU owner exists until consumed");
        parts.resources.register_uart_waker(cx);
        // TC is level-latched and sampled AFTER waker registration/IRQ arm.
        // DATA fed by DMA is not proof that the shift register is empty.
        if parts.registers.wire_complete(|| parts.resources.enable_uart_irq()) {
            parts.resources.disable_uart_irq();
            parts.registers.disable_wire_interrupt();
            Poll::Ready(Ok(()))
        } else {
            Poll::Pending
        }
    }
}

impl Drop for DeviceHardware {
    fn drop(&mut self) {
        if let Some(CpuParts {
            registers, resources, ..
        }) = self.parts.take()
        {
            resources.disable_uart_irq();
            // CPU state has no live DMA; defensive quiescence precedes reset.
            resources.quiesce();
            registers.reset_confirmed(now_us).expect("UART reset contract failed");
            resources.unpend_uart_irq();
        }
    }
}

impl Drop for StreamingLease {
    fn drop(&mut self) {
        if let Some(DmaParts {
            registers,
            resources,
            timing,
            ..
        }) = self.parts.take()
        {
            resources.disable_uart_irq();
            // Disable peripheral requests BEFORE stopping/draining the channel,
            // then reset the UART. Cancellation never manufactures completion.
            let registers = registers
                .disable_dma_and_restore(timing, || resources.quiesce(), now_us)
                .expect("UART DMA shutdown contract failed");
            registers.reset_confirmed(now_us).expect("UART reset contract failed");
            resources.unpend_uart_irq();
        }
    }
}

// Run in the plain UART host binary. The I2C custom-MMIO binary deliberately
// rejects any access outside its registered I2C fixture; do not weaken it to
// accept these separate UART host cells. CI runs both feature paths.
#[cfg(all(test, not(feature = "one-shot-host-tests")))]
mod tests {
    use super::*;

    #[test]
    fn real_mmio_arm_and_restore_keep_the_selected_clock_configuration() {
        for hz in [6_000_000, 12_000_000, 24_000_000] {
            let mut memory = [0_u32; 12];
            let base = memory.as_mut_ptr();
            let timing = BaudConfig::new(hz).unwrap();
            // SAFETY: aligned live host storage, exclusively owned for this
            // test; no physical DMA or device is accessed. All inspection uses
            // this same raw provenance after field-sized handles expire.
            let registers = unsafe { mmio::UartRegisters::new(base.cast()) };
            let registers = registers
                .reset_confirmed(|| 0)
                .unwrap()
                .configure_confirmed(timing, || 0)
                .unwrap();
            // Independent BAUD offset, not copied from the generated layout.
            let baud = unsafe { base.add(0x10 / 4) };
            assert_eq!(unsafe { baud.read() }, timing.word(false));
            let registers = registers.into_arm_dma(timing);
            assert_eq!(
                unsafe { baud.read() },
                timing.word(true),
                "UART_ARM_PRESERVES_PORT_TIMING"
            );
            let mut quiesced = false;
            let registers = registers
                .disable_dma_and_restore(
                    timing,
                    || {
                        assert_eq!(
                            unsafe { baud.read() },
                            timing.word(false),
                            "UART_REQUEST_OFF_BEFORE_QUIESCE"
                        );
                        quiesced = true;
                        true
                    },
                    || 0,
                )
                .unwrap();
            assert!(quiesced);
            assert_eq!(
                unsafe { baud.read() },
                timing.word(false),
                "UART_RESTORE_PRESERVES_PORT_TIMING"
            );
            // Plain RAM does not emulate GLOBAL.RST clearing CTRL.TE. Reset
            // side effects/readback delays are exercised by RegisterIo tests.
            drop(registers);
        }
    }
}
