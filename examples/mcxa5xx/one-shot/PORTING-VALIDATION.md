# UART reference-port validation — 2026-09-15

This records the first milestone in [PORTING.md](PORTING.md): separate MCXA
resource acquisition from the small UART TX-DMA algorithm and use shared
board-independent example bodies. This is not a completed cross-family port.

## Tested source and scope

The tests used local changes on top of
`8d9c2018b342d2f90e8b30c74d45c3bdd2f08568`, not a clean committed revision.
Each hardware result below records the source HEAD, dirty status, selected
source SHA-256 hashes, ELF hashes, probe version and process exits. Those source
hashes are a selected inventory, not a hash of every build input.

Production `src/i2c/`, `src/dma.rs` and `src/lpuart/mod.rs` are unchanged. The
experimental UART L3/L4 crates still forbid unsafe. The L2 adapter still accepts
concrete `McxaUartPort`; a second SoC binding and the corresponding owned-resource
interface remain future work.

The public port owns the clock/power guard, typed pin, DMA channel and matching
interrupt integration. Tests cover timing preservation through DMA arm/restore
and the actual release helper's call order. Root-clock stability, physical DMA
reachability and cache coherency remain explicit system obligations.

## Host and build results

All listed checks passed. Rust 1.92 was used except for Miri. Host target:
`x86_64-pc-windows-msvc`; firmware target: `thumbv8m.main-none-eabihf`.
Rust build/test checks used `RUSTFLAGS=-Dwarnings`.

| Check | Result and scope |
| --- | --- |
| HAL plain host library, `test,uart-one-shot` | 27 tests per chip, both `mcxa2xx` and `mcxa5xx` |
| HAL custom-MMIO host library, `one-shot-host-tests,uart-one-shot` | 68 tests per chip, both families |
| UART core | 8 acceptance tests; existing 6 compiler cases and new 4 port compiler cases passed |
| Shared example bodies | 10 tests, including missing/wrong/extra data, errors, deadline and pending cancellation |
| Driver-contract tool | 20 tests: 6 boundary, 7 contract, 7 port checks |
| Generator unit tests | 12 tests |
| Generated maps | UART 7-register and I2C 9-register `--check` passed; maps unchanged |
| HAL ARM checks | Both chip families, with and without `defmt` |
| Linked firmware | Sender and receiver, default and divided-clock feature configurations |
| Firmware dependency gate | `safe-mmio v0.3.1` without `custom-mmio` on the UART firmware graph |
| Formatting and patch hygiene | Rustfmt check and `git diff --check` passed |

The plain UART MMIO fixture is intentionally excluded from the separate I2C
custom-MMIO recorder binary: that recorder rejects unknown register fixtures.
Both binaries execute in CI; its fail-closed policy was not relaxed.

The new compiler cases check the actual HAL API without linking or executing
firmware: construction succeeds, cloning the port fails with E0599, using it
twice fails with E0382, and accessing its private resources fails with E0616.
The runner checks the diagnostic and primary source span, not merely a nonzero
compiler exit. The committed-baseline I2C mutation sweep was not rerun for these
uncommitted UART changes; it is not included in the results above.

Miri executed **one selected real-MMIO test**, not the whole HAL suite, using
`1.95.0-nightly (f889772d6 2026-02-05)` and its default borrow model:

```powershell
$env:RUSTFLAGS = '-Dwarnings'
cargo +nightly miri test --locked --manifest-path embassy-mcxa/Cargo.toml `
  --target x86_64-pc-windows-msvc --no-default-features `
  --features mcxa5xx,test,uart-one-shot --lib `
  --target-dir examples/mcxa5xx/target/uart-contract-pilot-miri `
  lpuart::one_shot::hardware::tests::real_mmio_arm_and_restore_keep_the_selected_clock_configuration `
  -- --exact
```

This passed over host memory at 6, 12 and 24 MHz configurations. It checks the
generated accessor/pointer path and DMA-bit/timing preservation, not physical
DMA transactions, clock behavior or GLOBAL reset side effects. The separate
scripted reset tests cover the reset sequence. Stable rustfmt emitted only the
repository's usual warnings for nightly-only formatting options.

The relevant commands are in [ci.sh](../../../ci.sh). The complete repository
CI script and its unrelated board-farm matrix were not run locally.

## Connected-board results

Two MCXA577 boards, crossed LPUART5 pins (`P0_25` TX to the other board's `P0_24`
RX), common ground. Probe A: `1fc9:0143:ZS1PGS20MCJBC`; probe B:
`1fc9:0143:0YHQFRBFS4FIG`. Probe-rs `0.31.0 (git commit: cc913fa2)`, SWD 4000 kHz.

Every run required sender completion, exact reception of the same static
16-byte `UART pilot DMA!\n` payload, and no extra byte during a 20 ms observation
window. Both processes exited 0. Directions were tested sequentially, not as
simultaneous full-duplex traffic. The receiver uses the existing RX-only driver,
not RX DMA.

| Sender clock / direction | Receiver readiness after sender launch | Result record |
| --- | --- | --- |
| Default 12 MHz, A → B | 4640 ms | [PASS](../target/uart-one-shot-hil/5acaa5c625154e59b8bab08208da6d80/result.json) |
| Default 12 MHz, B → A | 4315 ms | [PASS](../target/uart-one-shot-hil/e1853ba950714c52995f1ee2dd7f6449/result.json) |
| Divided 6 MHz, A → B | 4654 ms | [PASS](../target/uart-one-shot-hil/042e4538858f48e7a48c6283ba550aa9/result.json) |
| Restored default 12 MHz, A → B | 4294 ms | [PASS](../target/uart-one-shot-hil/b177d661bea34a96b11f782d8d31d19d/result.json) |

The divided-clock build selects `uart-port-clock-div2`, keeping the shared
payload, receiver and nominal 115200-baud request unchanged. The sender logged
6000000 Hz instead of 12000000 Hz. This is the HAL-derived frequency, not an
independently calibrated clock measurement. Both clocks select OSR=26; SBR is 2
at 6 MHz and 4 at 12 MHz. Their nominal generated baud rate is approximately
115384.6 baud.

The default sender ELF SHA-256, including the restored build, was
`D31ABCCCEF4D40A1C7263EA3CA1BBD5A6D7B3E4817B6A42D04549D736FA17186`;
the default receiver was
`B27579F4BB915557585C059B499169F8868AF9CA22E77349B192E87485E0FDB3`.
The divided-clock sender was
`4DDBBAABC8549CE035CD9F259C50914170DA69E2402BFB781B723640DA85631F`;
receiver
`CDE6A8B6D4450D04984A2504D519F3F5EB88B882C347D19DC82F0A890D57294D`.

Evidence directories contain stdout/stderr, preparation logs, source status and
`result.json`; they are local ignored build evidence, not committed artifacts.
The default 12 MHz example is restored on the boards.

## What these results do not establish

- No other-SoC, cached-memory or alternative DMA-controller port was tested.
- No new UART hardware cancellation/fault-injection qualification was performed.
- The Saleae probes were on I2C, not UART. There is no new UART waveform capture
  or independent measurement of final-stop-bit-to-return timing. The sender
  observes TC; matching receiver bytes alone cannot prove that return boundary.
- Miri and host effect tests do not execute the physical eDMA implementation.
  The outer software deadline does not preempt synchronous DMA quiescence.
- The port contract report checks completeness and source references. It does
  not turn an assumption or a test plan into hardware evidence.

Types constrain permitted operations. Effect tests check their sequencing.
Hardware tests challenge the assumptions underneath both. None replaces the others.
