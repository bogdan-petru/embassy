# One small UART service, a repeatable porting model

The reference service is deliberately narrow: send **1 through 32 immutable
static bytes** using async DMA, at nominal 115200 baud, 8N1. Success requires
DMA quiescence followed by UART transmission complete, not merely an empty DMA
descriptor. It says nothing about whether the receiving device accepted or
acted on the bytes. There is no RX-DMA implementation, chunking, retry engine,
or change to the production UART driver in this example.

The goal for a new SoC is: **keep the service and example body; supply the
hardware binding; qualify the same obligations.** MCXA is the first reference
binding. No second-SoC binding or cross-family hardware result exists here yet.

[I2C now applies the same porting recipe](I2C-PORTING.md) while retaining its
own repeated-START/STOP, exclusive-RX and post-Drop clock-gate requirements.
UART remains the first reference; shared structure does not imply identical
peripheral behavior or resource-release policy.

The [2026-09-15 validation record](PORTING-VALIDATION.md) records the passing
host/compiler/build checks, selected Miri test, both MCXA board directions and
the divided-clock hardware run, together with their limits and local evidence.

## Read the code in this order

| Read | What belongs there |
| --- | --- |
| [Board sender](../src/bin/uart-one-shot-sender.rs) | Choose clocks, instance, pins and IRQ/channel bindings; construct the port |
| [Shared body](../../nxp-shared/src/uart.rs) | Send exactly one common payload; independently check received bytes/errors/time limits |
| [Public service](../../../embassy-mcxa/src/lpuart/one_shot.rs) | Consume the port and apply one deadline to DMA plus wire completion |
| [L4 core](../../../embassy-mcxa/experimental/uart-one-shot-core/src/lib.rs) | `Idle -> Streaming -> DmaQuiescent -> WireComplete`; no PAC, HAL or timer dependency |
| [L3 vocabulary](../../../embassy-mcxa/experimental/uart-one-shot-vocabulary/src/lib.rs) | Safe named operations over opaque L2 capabilities; forbids unsafe |
| [L2 interface](../../../embassy-mcxa/experimental/uart-one-shot-interface/src/lib.rs) | The unsafe physical `Device` implementation contract and owned capability boundary |
| [L2 IP adapter](../../../embassy-mcxa/src/lpuart/one_shot/hardware.rs) | Selected LPUART register behavior, DMA-arm/restoration sequencing and leases |
| [MCXA binding](../../../embassy-mcxa/src/lpuart/one_shot/hardware/binding.rs) | Clock/power lifetime, pin mux, matching endpoint/request, channel and IRQ integration |

These are still four safety responsibilities, not eight layers. L1 is the
locked PAC; L2 contains the trusted physical implementation; L3/L4 forbid unsafe.
Board construction and shared example logic are application code above them.

The ownership handoff is small (after the board initializes its clock tree and
declares `Irqs` with both UART and DMA handlers):

```rust,ignore
let port = McxaUartPort::acquire(
    p.LPUART5, p.P0_25, p.DMA0_CH0, Irqs, ClockConfig::default(),
)?;
let uart = Uart::new(port)?;
nxp_shared_examples::uart::send(|bytes| uart.write(bytes)).await?;
```

`McxaUartPort` derives the frequency, register mapping and DMA request from the
consumed HAL resources. The caller does not supply loose addresses, a request
number, or a purported clock-frequency integer. The private resources move
through CPU and streaming states together. Clock release happens after DMA
quiescence; a forgotten owner retains its resources and static source.

`ClockLease` is a guard within this HAL's ownership model, **not a physical
frequency lock**. Its wake guard inhibits deep sleep. PAC writes, foreign code
or another core could still bypass clock-initialization policy. Lifetime typing
does not make those external actors obey the contract.

## What is reusable today, and what still needs extraction

The L3/L4 crates and the shared example body already avoid SoC dependencies.
The MCXA-specific clock/pin/channel selection has moved out of the IP algorithm.
However, `DeviceHardware::new` still takes **concrete `McxaUartPort`**, and its
register layout/value types select the MCXA LPUART PAC. This is a useful first
boundary, not a fully generic L2 crate.

For another SoC, first establish which IP revision and register semantics actually
match. Then extract the needed owned-resource interface between the IP adapter
and the binding, implementing it for MCXA and the new SoC. Do not merely replace a
PAC path or add a crate feature and call the port complete. In particular,
Flexcomm selection, clock/power behavior, IRQ routing, DMA reachability and
cache maintenance belong in that SoC integration. Different register semantics
may need a different IP adapter while retaining the same L3/L4 service.

## Add another port: the checklist

1. Match the service to the new IP: short TX, register access semantics, DMA
   endpoint width, reset handshakes and the actual transmit-complete condition.
   Select the reviewed PAC/version; do not infer compatibility from `LPUART`.
2. Implement the owned SoC binding. Retain clock/power, pin, endpoint/request,
   channel and IRQ authority through setup, pending, cleanup and forgetting.
3. Add a `Port` value to the [existing UART Rust contract](../../../embassy-mcxa/tools/driver-contracts/src/contracts/uart.rs).
   Supply provenance, explicit capabilities/limits and all nine obligations.
   Missing behavior must be rejected by the binding, not silently defaulted.
4. Keep the shared core/vocabulary and example payload/checker unchanged where
   their service still applies. Supply the board's construction and nonblocking
   receive/time callbacks. The receiver currently uses the existing RX-only
   driver, not a second new DMA driver.
5. Run the independent compiler, protocol, effect and board checks. Record the
   tested source/build, board/IP revisions, clock/memory configuration and actual
   outcomes. A valid contract report inventories evidence; it is not a pass.

The [contract tool](../../../embassy-mcxa/tools/driver-contracts/README.md) rejects
missing answers and unresolved sources. It does not generate the tests' expected
answers from the implementation under test.

| Port obligation | Evidence and remaining boundary |
| --- | --- |
| Clock/power lifetime | Owned lease plus real release-order test; root-clock stability remains a system assumption |
| Pins/interrupts | Typed matching bindings; board mapping and wake behavior still require review/qualification |
| DMA pairing | Endpoint/request derived from one instance; verify the SoC routing |
| DMA completion | DONE/error decisions and request-off/quiesce ordering; not a proof of eDMA internals |
| Wire completion | Core rejects DMA-only completion; TC meaning and return-to-pin timing need hardware evidence |
| Reset/shutdown | Scripted effects test real readback sequencing; hardware must accept and progress |
| Buffer lifetime | Static immutable source survives forgetting; cancellation never manufactures success |
| DMA reachability | Static lifetime is not bus-master accessibility; qualify the image's memory regions |
| Cache coherency | No cache maintenance in this pilot; a new cached/noncoherent port must provide it or reject that memory profile |

The new [release helper](../../../embassy-mcxa/src/lpuart/one_shot/hardware/release_sequence.rs)
tests the actual order of calls: IRQ off, DMA quiesce, unpend, pin off, clock
release. The [host-memory MMIO test](../../../embassy-mcxa/src/lpuart/one_shot/hardware.rs)
checks that real generated accesses preserve the selected baud configuration
through DMA arm/restore and disable requests before the quiesce callback. These
complement the scripted reset tests; none executes physical eDMA or proves the
HAL's quiescence internals. A receiver matching all bytes does not independently
prove that the sender returned after the final stop bit.

## Small, reproducible checks

From the repository root:

```text
cargo run --locked --manifest-path embassy-mcxa/tools/driver-contracts/Cargo.toml -- report uart
cargo run --locked --manifest-path embassy-mcxa/tools/driver-contracts/Cargo.toml -- check uart .
cargo test --locked --manifest-path embassy-mcxa/tools/driver-contracts/Cargo.toml
cargo test --locked --manifest-path embassy-mcxa/experimental/uart-one-shot-core/Cargo.toml
cargo test --manifest-path examples/nxp-shared/Cargo.toml
python embassy-mcxa/tools/gen_uart_one_shot_mmio.py --check
```

For linked firmware, use the current build flags/target-directory and two-board
runner described in [UART-PILOT.md](UART-PILOT.md). As a second clock-policy
candidate, add the **`uart-port-clock-div2`** example feature when building the
same sender/receiver bins. The sender divides the initialized 12 MHz source by
two, reports its actual 6 MHz peripheral clock, and computes UART timing for the
same nominal **115200 baud**. The receiver remains unchanged. Restore the
ordinary `uart-one-shot` build for the baseline. These are separate hardware
runs of the same shared body, not a baud-rate change or another MCU port.

The timing solver rejects zero/unrepresentable clocks and settings beyond its
3% divider-error limit. That numerical limit does not establish oscillator
accuracy, peer tolerance or board signal integrity. Build success does not
qualify the divided-clock candidate; record its independent hardware result.

Types constrain permitted operations. Effect tests check their sequencing.
Hardware tests challenge the assumptions underneath both. None replaces the others.
