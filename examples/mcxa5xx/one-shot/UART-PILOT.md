# UART one-shot pilot

This second peripheral uses the same small contract structure as one-shot I2C:
an owned physical adapter, safe named operations, and a consuming transaction.
The pilot sends one static buffer through DMA at 115200 baud, 8 data bits, no
parity, one stop bit. A successful return requires both DMA quiescence and
LPUART `STAT.TC` (transmission complete). DMA DONE alone is insufficient.

The public operation is:

```rust,ignore
let port = McxaUartPort::acquire(p.LPUART5, p.P0_25, p.DMA0_CH0, Irqs, ClockConfig::default())?;
let uart = Uart::new(port)?;
uart.write(&MESSAGE).await?;
```

The current [porting walkthrough](PORTING.md) explains the owned MCXA binding,
shared example body, clock-policy probe and the work still needed for a second
SoC. Historical results below remain records of their original builds.

The buffer length is 1 through 32 bytes. `write` consumes the UART owner. The
HAL applies one 100 ms deadline to the operation. Cancellation and errors retain
cleanup responsibility. Forgetting the future does not return its peripheral
or DMA channel; the immutable static source remains valid even if the owner is
retained indefinitely.

## Contract format

I2C and UART now share the [driver-contract tool and generation
recipe](../../../embassy-mcxa/tools/driver-contracts/README.md). The maintained
policy inputs are the Rust [UART contract](../../../embassy-mcxa/tools/driver-contracts/src/contracts/uart.rs)
and [I2C contract](../../../embassy-mcxa/tools/driver-contracts/src/contracts/i2c.rs).
JSON and Markdown reports are exported views of those inputs.

Every operation supplies eight obligations. This table summarizes the UART
contract; it is not a second manifest to edit independently.

| Obligation | TX DMA | Complete on the wire |
| --- | --- | --- |
| `owns` | UART, matched endpoint/request/channel, static immutable source | The same quiescent CPU owner |
| `effect` | Program DMA and enable peripheral/channel requests | Arm TC interrupt and observe `STAT.TC` |
| `pending` | Retain the streaming lease and all resources | Retain the CPU owner |
| `wake` | Register DMA waiter and recheck status; see error-only limitation below | Register UART waiter, arm TCIE/NVIC, then recheck TC |
| `completion` | DONE without error, peripheral requests disabled, channel quiescent | TC observed after successful DMA quiescence |
| `error` | Quiesce before cleanup; return no completion | Overall deadline and cleanup remain obligatory |
| `cancel` | Disable requests, quiesce, then reset/release | Reset/release without reporting success |
| `forget` | Keep resources retained; static source stays valid | Return no completion and retain the UART |

The contract also names register states, permitted access methods, reviewed
hook anchors, and evidence sources. The [shared Abacus backend](ABACUS-INTEGRATION.md) expands the Rust register
declarations into field-sized safe-MMIO access methods for both peripherals.
The declarations are checked against the independent Rust contracts and PAC.
These structural checks do not establish the truth of hardware observations.

Reset confirmation, DMA request shutdown/quiescence, ownership restoration,
and the async transaction remain reviewed handwritten code. A host fake can
establish ordering between calls; it cannot establish that the physical
adapter tells the truth.

## Two-board example

Use two MCXA577 boards and confirm this wiring before starting the sender:

| Sender | Receiver |
| --- | --- |
| P0_25 / LPUART5 TX | P0_24 / LPUART5 RX |
| GND | GND |

With both crossed TX/RX pairs connected, swap board roles to test the opposite
direction. Each run still has only one sender and an RX-only receiver; this is
not a simultaneous full-duplex test. The examples use the LPUART5 wiring
confirmed on 2026-09-15, replacing the original LPUART1 pin suggestion.

The receiver is RX-only. Both examples enable the 12 MHz SIRC source with no
FroLfDiv division. The payload is the exact 16 bytes `UART pilot DMA!\n`.

Build from `examples/mcxa5xx`:

```powershell
cargo build --release --features uart-one-shot --bin uart-one-shot-sender --bin uart-one-shot-receiver
```

Use the [two-board runner](run-uart.ps1). It preloads both images before startup,
then starts the sender first. The sender constructs its TX owner **before**
announcing its ten-second quiet window, so the receiver starts against a driven
idle line. The runner then starts the receiver and requires READY within a
conservative nine-second budget measured from before sender launch (including
probe/RTT delays). Both fresh runs verify the preloaded images and reset the
boards. The receiver allows 20 seconds from READY for the complete payload and
then checks for extra bytes over 20 ms.

```powershell
pwsh -NoProfile -File one-shot/run-uart.ps1 -SenderProbe '1fc9:0143:ZS1PGS20MCJBC' -ReceiverProbe '1fc9:0143:0YHQFRBFS4FIG'
# The other crossed wire, using the same two binaries:
pwsh -NoProfile -File one-shot/run-uart.ps1 -SenderProbe '1fc9:0143:0YHQFRBFS4FIG' -ReceiverProbe '1fc9:0143:ZS1PGS20MCJBC'
```

Select probe IDs from `probe-rs list` for a different pair of boards. The runner
requires distinct probes, both zero process exits and exact INFO verdicts. It
never retries a failed transfer into a pass. Logs, ELF/source hashes, source HEAD
and dirty status, probe identities, readiness time and exits are retained under
`target/uart-one-shot-hil/<run-id>/`. Cleanup failures also make the run fail.

Acceptance requires both independent verdicts:

- `UART PILOT SENDER PASS`: the consuming write returned after DMA quiescence
  and transmission complete.
- `UART PILOT RECEIVER PASS`: exactly the expected 16 bytes arrived and no extra
  byte arrived during the following 20 ms.

A panic, sender timeout, receiver timeout, framing error, wrong byte, extra byte,
or missing verdict is a failed run. Preserve both RTT logs, the tested commit,
board/probe identities, and build command. Passing the receiver alone does not
prove the sender waited for TC before returning.

## Host acceptance

The core acceptance tests use an allocation-managed, safe fake with a single
retained owner. They cover:

- DMA DONE while TC is absent remains pending.
- TC cannot be polled before DMA quiescence returns the CPU owner.
- Cancellation while DMA or wire completion is pending drops the owner once.
- Invalid lengths and arm/DMA/wire errors do not produce success.
- Success releases the owner only after TC.

Run from the repository root:

```powershell
cargo test --manifest-path embassy-mcxa/experimental/uart-one-shot-core/Cargo.toml
```

These tests provide protocol and ownership evidence. The HAL effect suite must
separately verify MMIO access, interrupt registration, and quiescence ordering;
the two-board example provides the wire-level evidence. Miri checks executed
host Rust memory accesses, not physical DMA activity or register semantics.

## Current limits

- The 100 ms deadline is cooperative. The reused DMA `quiesce` primitive spins
  until the active minor loop drains and has no independent timeout. Cleanup
  therefore assumes hardware makes progress and is not bounded by 100 ms.
- A DMA error without a completion interrupt may be observed only when the
  deadline next polls the transfer. An immediate error-only wake is not claimed.
- The UART ISR was source-reviewed to disable its NVIC line and wake the waiter
  without reading/writing UART registers. That exact wake-only body is not
  enforced by an AST check.
- UART reset/readback loops have time and poll-count bounds. Failure during
  cleanup panics without returning a usable owner; the fallback reset write
  does not prove unresponsive hardware accepted it.
- The four HAL effect tests exercise the real reset/request-off/TC helper over
  scripted effects. They do not execute physical DMA, exhaust interrupt
  schedules, or run the whole generated MMIO owner under Miri.

## Current evidence

Local working-tree validation on 2026-09-14, based on `fa859a811` plus the
uncommitted contract/UART changes. These outcomes do not describe a pushed ref.

| Check | Recorded outcome |
| --- | --- |
| Shared contract and boundary policy | 13 host tests passed, including resolved host/ARM dependency graphs and negative controls |
| Shared generator | 12 independent policy tests passed; both PAC checks passed; I2C generated MMIO byte-identical |
| Core behavioral acceptance | Eight tests passed on Rust 1.92 / `x86_64-pc-windows-msvc` |
| Compiler contract | Six cases passed under `-Dwarnings`: one valid typed sequence, five exact diagnostic/span rejections |
| HAL effect helpers | Four tests passed for each of `mcxa2xx` and `mcxa5xx` |
| HAL regression suite | 63 tests passed on each MCU family, including the four UART helper tests |
| Existing I2C core suite | All 29 tests passed; production I2C transfer source unchanged |
| ARM compile checks | Both MCU families passed, with and without `defmt`, under `-Dwarnings` |
| Linked sender/receiver firmware | Both linked with `flip-link`, `link.x`, and `defmt.x`; ARM ELF vector/text sections verified |
| Firmware feature isolation | I2C and UART firmware graphs both exclude `custom-mmio` |
| Formatting / whitespace | Rustfmt and diff checks passed; stable rustfmt emitted existing nightly-option warnings |
| UART Miri | Not yet run |
| UART two-board hardware test | Initial 2026-09-14 record: not run; see the 2026-09-15 LPUART5 qualification below |

The compiler integration uses isolated Cargo checks and never executes its
negative fixtures. It also detected a deliberate `--cap-lints=allow` change:
the test failed because the forbidden-unsafe control unexpectedly compiled.
Compiler checks run natively, separately from Miri.

The new checks are wired into `ci.sh`; the full repository CI script and the
existing committed-baseline mutation campaign were not rerun on this dirty
working tree. The new UART compiler cases use isolated temporary packages and
do not need to mutate or commit the checkout.

For the recorded linked build, run from `examples/mcxa5xx` (explicit flags
matter because `RUSTFLAGS` replaces the `.cargo/config.toml` target flags):

```powershell
$env:RUSTFLAGS = '-Dwarnings -C linker=flip-link -C link-arg=-Tlink.x -C link-arg=-Tdefmt.x -C link-arg=--nmagic'
cargo +1.92 build --locked --release --target thumbv8m.main-none-eabihf --features uart-one-shot --bin uart-one-shot-sender --bin uart-one-shot-receiver --target-dir target/uart-contract-pilot
Remove-Item Env:RUSTFLAGS
```

For host checks, use `RUSTFLAGS=-Dwarnings` without firmware linker arguments.
The contract tool's [README](../../../embassy-mcxa/tools/driver-contracts/README.md)
lists generation and policy commands; the exact family/feature matrix is in
[`ci.sh`](../../../ci.sh). Test expectations are maintained independently of
the contracts being checked.

The core acceptance command is:

```powershell
cargo +1.92 test --manifest-path embassy-mcxa/experimental/uart-one-shot-core/Cargo.toml --target x86_64-pc-windows-msvc --target-dir examples/mcxa5xx/target/uart-contract-pilot
```

## LPUART5 hardware qualification — 2026-09-15

The user confirmed both crossed pairs: P0_25 (TX) to the other board's P0_24
(RX), in both directions. Pinned MCXA577 metadata maps both pins to ALT8,
LPUART5 IRQ 95 and TX DMA request 103. The examples and IRQ binding now select
LPUART5; the generic trusted backend, generated map and protocol did not change.

The first run failed honestly: the receiver reported a UART error immediately
after READY, before the sender's ten-second wait expired. The error class was
not captured by that original generic panic, so it is not evidence of a specific
framing/noise fault. Moving UART construction before the quiet-window marker
established the TX idle level before RX startup; both subsequent directions
passed without suppressing or retrying any receive error. The receiver now logs
the exact error variant on failure. This changes startup, not DMA transfer logic.

| Sender serial | Receiver serial | READY after sender launch | Endpoint exits | Result |
| --- | --- | --- | --- | --- |
| ZS1PGS20MCJBC | 0YHQFRBFS4FIG | 4,586 ms | 0 / 0 | PASS |
| 0YHQFRBFS4FIG | ZS1PGS20MCJBC | 4,675 ms | 0 / 0 | PASS |

Each pass includes the real sender's DMA-quiescent/TC completion and the
independent RX check of `UART pilot DMA!\n` (16 exact bytes), followed by a
20 ms no-extra-byte observation. These are two sequential one-way TX-DMA tests,
not simultaneous full duplex, RX DMA, cancellation, fault injection or a soak.

Evidence retained, including the failed attempt:

- [Initial failed run](../target/uart-one-shot-hil/efcb6f1840344bbc8df2ecbc3b46f8ec/result.json).
- [First passing direction](../target/uart-one-shot-hil/5fa28cf84dc64977bc8dc51128c7e54a/result.json).
- [Reverse passing direction](../target/uart-one-shot-hil/be9f1cb18be3416b8909ed4b60573a31/result.json).

Both passing runs used the same linked ELF pair: sender SHA-256
`BA5237288A698938ED0ADDFA5895468CC5F2A4738D38D98D51F0693130751C08`,
receiver `A779FD4EAFE07A4CBE18DFA8A6DD386E6B0E1A6080A37344D6A2C9FCA3A374A8`.
Build: Rust 1.92, release, `uart-one-shot`, `DEFMT_LOG=info`, the explicit
warnings-denied linker flags above. Probe-rs 0.31.0 (`cc913fa2`), SWD 4000 kHz.
The tests are on `fa859a811` plus uncommitted source; result files identify the
individual Rust source hashes. Nothing was committed or pushed for this run.
