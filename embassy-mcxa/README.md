# Embassy NXP MCX-A MCUs HAL

A Hardware Abstraction Layer (HAL) for the NXP MCX-A family of
microcontrollers using the Embassy async framework. This HAL provides
safe, idiomatic Rust interfaces for GPIO, UART, and OSTIMER
peripherals.

## Explicit eDMA interrupt bindings

DMA interrupts are application-owned, just like peripheral interrupts. The HAL
no longer installs all eight DMA vectors implicitly. Bind each channel you use:

```ignore
use embassy_mcxa::{bind_interrupts, dma, i2c::controller, peripherals::*};

bind_interrupts!(struct Irqs {
    LPI2C3 => controller::InterruptHandler<LPI2C3>;
    DMA0_CH0 => dma::InterruptHandler<DMA0_CH0>;
    DMA0_CH1 => dma::InterruptHandler<DMA0_CH1>;
});

// MCXA577, after obtaining p from embassy_mcxa::init(...):
let i2c = controller::I2c::new_async_with_dma(
    p.LPI2C3, p.P3_21, p.P3_20,
    p.DMA0_CH0, p.DMA0_CH1, Irqs,
    controller::Config::default(),
)?;
```

For a target, use `i2c::target::InterruptHandler<LPI2C3>` instead; the two DMA
bindings are identical. I2C still needs its own interrupt for address matches,
STOP/repeated-START, and faults. DMA interrupts report buffer-transfer progress,
not completion of an I2C bus transaction.

Migration from the implicit-vector API:

- Standalone DMA: `DmaChannel::new(channel)` becomes
  `DmaChannel::new(channel, Irqs)`.
- I2C, I3C, SPI and FlexSPI: extend the existing `Irqs` with both channel
  handlers; their constructors check all three bindings.
- LPUART DMA: bind the channel(s) and pass `Irqs` immediately before `config`.
- Runtime channel selection: construct a bound `DmaChannel` before erasing or
  selecting channels. `AnyChannel` no longer implements `Channel`, because an
  erased raw token cannot prove which interrupt handler it requires.

Missing bindings, wrong-channel bindings, and mismatched vector/handler pairs
are compile errors. This checks wiring, not transfer timing or electrical bus
behavior; those still need hardware tests. Existing internal LPUART/I3C
completion hooks run after the common DMA acknowledgement/wake in the bound
handler, preserving queue-driver behavior.

The constructor's positive and compile-fail regression tests run on the host:

```sh
cargo test --manifest-path embassy-mcxa/Cargo.toml --no-default-features \
  --features test,mcxa5xx --doc -- "dma::DmaChannel<'a>::new"
```

CI runs these tests and the pure I2C unit tests for both MCXA families before
the optional hardware-test gate.

On an MCXA577 board, `examples/mcxa5xx/src/bin/dma-interrupt-bindings.rs`
checks 32 memory copies across all eight channels. A second bound handler
records actual IRQ entry independently of the DMA completion status. Run this
in addition to the two-board I2C suites when changing interrupt dispatch.
