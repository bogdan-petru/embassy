# Abacus generation with safe-MMIO

The I2C and UART one-shot drivers now use the same Abacus procedural macro.
Their register declarations are Rust inputs, expanded by Cargo during compilation:

- I2C: `embassy-mcxa/src/i2c/one_shot/session/vocabulary/registers/mmio.rs`
- UART: `embassy-mcxa/src/lpuart/one_shot/hardware/mmio.rs`
- Shared backend: `embassy-mcxa/experimental/abacus-registers/abacus-registers-dsl/src/safe_mmio_backend.rs`

This is a **local adaptation of Abacus**, not an upstream-supported safe-MMIO
release. Its pinned upstream provenance remains in the fork's EXPERIMENT.md.
The normal Abacus dependency has no Tock runtime feature enabled. The separate
production I2C driver still uses its existing Tock layout; this change does not
replace that driver or remove its dependencies.

## What a new IP supplies

Write a register declaration and its allowed states, not a new generator.
For example, a timing field annotated `RegAttributes([Off], WriteOnly)` gets a
write method only on an Off owner. A clock peripheral can define its own states
without declaring any DMA, transfer, or reset operation.

The backend presently supports flat states and u32 fields with byte padding.
That is a concrete restriction, not a claim to support every peripheral.
Register layout/direction are checked against the pinned PAC; read-side effects
and the truth of state transitions still require hardware review.

The two one-shot maps use `mmio_initial_state(Unconfirmed)`: only that owner can
be acquired, and it grants no ordinary operations. `mmio_private` explicitly
reserves registers for reviewed companion code rather than generating setters.
Those companions still perform observed reset/enable confirmation and DMA arm
and restoration. No new generic cast grants Enabled or ends Streaming.

Retained ReadPure handles move with the owner. CPU writes and destructive FIFO
reads borrow only their own field. CPU-writable UART status is not shared.
The owner stays non-Copy, non-Clone, non-Send and non-Sync.

## Removed and retained tooling

Removed: `gen_one_shot_mmio.py`, `gen_uart_one_shot_mmio.py`, their shared
`driver_contract.py` emitter, and their three generator-specific test files.
Abacus now generates register access at compile time; there is no checked-in
expanded accessor output to regenerate.

The Rust operation contracts remain an independent review/test specification.
`driver-contracts/tests/abacus_surface.rs` compares both declarations against
that specification. It checks code; it does not generate driver code.
Existing effect tests exercise the actual expanded accessors; compiler and
mutation checks remain distinct from those runtime tests.

The unrelated full-driver `gen_lpi2c_regs.py` remains, as do Python verification
tools such as the firmware-feature check and mutation runner. Removing a
one-shot emitter does not justify deleting checks or changing the production HAL.

## Validation

Integration validation is recorded below as runs complete. Historical results
in DIRECT-SAFE-MMIO.md and PROGRESS.md describe earlier implementations, not this
replacement. No hardware claim follows merely from successful macro expansion.

### Local integration validation, 2026-09-15

Source baseline: `508e63cf8` (local; not pushed). This also retains the previously
published L3 proof files; those proofs are not new Abacus-specific results.

| Check | Result |
| --- | --- |
| Abacus DSL unit tests | 53 passed |
| Default Abacus backend | 7 executing tests + 17 doctests passed |
| Optional legacy compatibility build | 7 executing tests + 14 doctests passed; not enabled by either driver |
| Independent I2C/UART declaration contracts | 2 passed |
| HAL host effect tests, both MCU families, warnings denied | 79 per family passed |
| Plain-MMIO host tests, both MCU families, warnings denied | 44 per family passed |
| Protocol/source-policy suite | 31 passed |
| Mutation-runner self-tests | 20 passed |
| Firmware feature gate | safe-MMIO 0.3.1, no custom-MMIO instrumentation in normal/build graph |
| ARM I2C write/write-read and UART sender | Warning-denied build passed; linked board images built with the full board linker flags |
| Miri I2C, strict provenance, Stacked Borrows | 69 passed, 0 filtered, exit 0; 799 seconds |
| Miri UART, strict provenance, Stacked Borrows | 15 passed, 23 non-UART tests filtered, exit 0; 552 seconds |
| Final I2C PAC-layout Miri check | 1 passed after adding the compile-only PAC direction checks |
| Isolated mutation campaign | 35/35 passed, exit 0, on the committed integration baseline |

The two standalone Abacus test lockfiles are tracked in the validation follow-up
so the documented `--locked` commands work on a fresh checkout, rather than
depending on ignored local files. These do not alter the HAL's resolved graph.

The I2C Miri run began before formatting and the addition of compile-only PAC
direction checks; neither changed the emitted pointer/access semantics. The
final layout test was additionally rerun. No alias checks, leak checks, reset
bounds or failing tests were disabled. UART's first concurrent invocation hit a
Windows proc-macro DLL lock; the completed run used a separate build directory.

Hardware results on the connected pair:

- Write: exact 16 bytes and observed STOP.
- Combined write/read: exact four bytes, repeated START and no intermediate STOP,
  plus receive DMA quiescence before return.
- NACK: exact error and DMA inactive/requests off/MEN off/IRQ disabled.
- Cancellation after DMA arm: the same local cleanup assertions pass.
- Short RX stall: resumes with exact bytes and STOP.
- Long RX stall: exact Timeout and local cleanup; not proof of peer/bus recovery.
- UART: exact bytes, DMA quiescent and final transmitter completion, both directions.
- Normal I2C pair restored and passed after the checks.

The first board invocation rejected an image built without the defmt linker
script before driver execution; it is not a pass. The successful images used
the board's flip-link/link.x/defmt.x/nmagic settings plus warnings denied.

Board records are retained under `examples/mcxa5xx/target/`:
`i2c-one-shot-hil/93664c48c80c4b9e8a6c96dfda570438` (combined),
`i2c-one-shot-hil/cfd9ca642d3242a9beceec37aa10c804` (write),
`i2c-one-shot-rxstall/f84e34481a4c4155bc3b5724c6eda398`,
`uart-one-shot-hil/b71116e8d13b4feaa4221cd21d7fdcd9` and
`uart-one-shot-hil/4ed607007cad4afdbc18607277413b8f`,
and `i2c-one-shot-hil/2cfa974ccfdb42ce8c6293c5558e38b5` (final restoration).
NACK/cancellation output and host/Miri command results are in the task execution
record. Mutation logs and diffs are in
`C:/temp/abacus-integration-mutations-20260915`.
