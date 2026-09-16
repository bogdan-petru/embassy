# Kani checks of protocol decisions and the real L3 vocabulary

## Layer-three contract preservation

The proof-only `vocabulary` executable calls the production
`i2c_one_shot_vocabulary::Hardware<Model>` and the real `Cpu`, `InFlight`,
`Receiving` and `Step` implementations. It does **not** substitute a fake
`Backend` above L3. A small explicitly modeled `Device` supplies L2 outcomes.
No production driver, firmware dependency or unsafe allowance is changed.

The existing library still has `forbid(unsafe_code)`. The separate proof binary
uses `deny(unsafe_code)` with one narrow allowance on its model's `unsafe impl
Device`; its property helpers, harnesses and native tests forbid unsafe. This
test-only implementation is a declared trust boundary, not a second physical
HAL and not code linked into firmware.

| New harness | What is checked against real L3 |
| --- | --- |
| `start_admission_preserves_address` | One snapshot; START with the unchanged address iff both bus and controller are idle; otherwise exact Busy and owner release |
| `stop_uses_current_fifo_state` | Three independently selected FIFO observations; each attempt reads the current state before issuing STOP or returning Pending |
| `read_readiness_is_pure` | Every u8 address/length and FIFO boolean; exact invalid/pending/ready decision, without starting traffic or DMA |
| `observations_preserve_status_and_wake` | Every seven-bit Status and both WakeOn choices on CPU/TX/RX paths; exact forwarding and the supplied distinguishable waker |
| `tx_arm_preserves_input_and_error` | Source identity/length for 1..=32 bytes; exact success or any of the ten current Error variants |
| `rx_arm_preserves_input_and_error` | Original exclusive buffer/address for 1..=4 bytes; exact L2 result even if FIFO readiness changes after read_ready |
| `tx_poll_preserves_owner_and_outcome` | Up to three arbitrary Pending/Complete/Failed decisions and cancellation after any Pending; exact outcome/error and lower-owner cleanup behavior |
| `rx_poll_preserves_owner_and_outcome` | Same bound, plus the original allocation, slice length and symbolic bytes are returned only in the lower Complete case |
| `forgetting_pending_owners_does_not_complete` | Forgetting a Pending TX/RX owner does not invoke modeled cleanup or manufacture completion |

### Bounds and assumptions

The proof functions have no `kani::assume`, stubs, safety-only mode or disabled
safety checks. Each has an explicit unwind limit of 40, with unwinding checks
enabled; this covers the bounded three-poll traces and support-library loops.
RX arm/poll buffer lengths are 1..=4, not every length supported by the API.
Read-readiness validation separately covers all 256-by-256 address/length
inputs and both FIFO states. Native tests execute the same property helpers;
they are regression tests, not a substitute for symbolic verification.

The modeled lower binding promises snapshot coherence, actual DMA quiescence
before Complete, cleanup on ordinary Drop, and wake registration. These are
**assumptions about L2**, not new proofs of physical DMA, interrupt delivery,
no-lost-wake behavior, fences, MMIO or electrical timing. A distinct Arc-backed
waker lets the model detect context substitution; it does not simulate IRQs.
Kani sequentializes its Arc atomics, so this is single-threaded forwarding
evidence, not concurrency or interrupt-scheduling verification.

The modeled controller ID is fixed. This is not a universal cross-instance
substitution proof: the real non-Clone wrappers and moved-result checks are
what preserve the existing lower owner. The symbolic RX bytes stand for any
completed payload; the model does not establish that physical DMA supplied
those bytes. Static RX storage is allocated solely
for the tests and deliberately not recovered after cancellation/error/forget.
Compiler rejection tests, rather than an `active` flag in a model, establish
that a forgotten receive buffer cannot be accessed again through safe Rust.

START's L3 rule is only bus/controller admission and address forwarding. Address
preflight and fault priority remain L4 responsibilities. L2 still owns the fresh
RX FIFO check and DMA-arm-before-START ordering. The proofs do not promote
`read_ready` into a durable permission or prove an unbounded eventual completion.

### Reproduce without an empty or filtered run

On this Windows machine with the existing WSL installation, from the repository
root:

```powershell
pwsh -NoProfile -File embassy-mcxa/experimental/i2c-one-shot-proofs/run-kani.ps1 -SelfTest
pwsh -NoProfile -File embassy-mcxa/experimental/i2c-one-shot-proofs/run-kani.ps1
pwsh -NoProfile -File embassy-mcxa/experimental/i2c-one-shot-proofs/run-l3-controls.ps1 -SelfTest
pwsh -NoProfile -File embassy-mcxa/experimental/i2c-one-shot-proofs/run-l3-controls.ps1
cargo test --locked --manifest-path embassy-mcxa/experimental/i2c-one-shot-proofs/Cargo.toml --bin vocabulary
```

The runner selects the whole library and whole vocabulary binary separately,
requires exactly the named 3 + 9 successful harnesses, and records commands,
tool versions, source and lock hashes, timeouts and full logs. A changed source,
missing harness, failed proof or lock drift rejects the result. Ordinary
`cargo check` excludes `cfg(kani)` and is not accepted as proof execution.
`ci.sh` runs the native property tests; the WSL solver runner is a separate
explicit step, not claimed to run automatically in the repository's full CI.

The separate controls runner copies the four packages into a fresh isolated
fixture for each mutation, verifies its unmodified baseline, then changes one
real L3 operation. It requires an assertion failure at the expected property
source location; compilation errors, unwinding failures and timeouts are not
accepted as successful controls. It keeps the copies, diffs, logs and hashes,
without modifying or restoring files in the checkout. These intentionally
filtered controls do not replace the unfiltered 3 + 9 suite.

### Current validation — 2026-09-15

The complete, unfiltered runner passed: **3 L4 + 9 L3 harnesses verified,
zero failures**. Evidence is in
`C:/temp/i2c-l3-proofs-20260915/kani-final/`; its result confirms unchanged
source and all four package lockfiles during verification. The new proof
package's two direct local dependencies were deliberately resolved before
that baseline. Tool versions are Kani 0.67.0, CBMC 6.8.0 and CaDiCaL 2.0.0,
using Kani's compiler and the 64-bit Linux verification target under WSL.

The L3 log contains 9,165 checks, zero failures, and 42 unreachable checks.
All 97 assertion records in `properties::*` report SUCCESS, not UNREACHABLE.
The unreachable checks are support-library guards and completion/drop paths
deliberately not taken by the forget harness. Unsupported-construct guards
also succeeded; no reachable unsupported operation was accepted.

The first full solver run also reported 3 + 9 successes, but the wrapper
rejected the optional `(N unreachable)` suffix in Kani's summary. That rejected
record remains in `kani-initial/`. The parser was corrected, tested against
22 positive/negative fixtures and both real logs, and the **entire suite was
rerun** to produce `kani-final/`. It still rejects empty, missing, duplicate,
failed, timed-out, all-unreachable or changed-source runs.

Native warning-denied checks pass: seven new property tests, eleven existing
vocabulary tests, three interface tests and all 31 core/policy/property tests.
The native property test command is now in `ci.sh`; the full repository CI
matrix was not run. Production driver code is unchanged, so boards were not
reflashed for this proof-only change. The prior hardware evidence remains
tied to its original source record.

All four real-source negative controls passed, each with a successful baseline
followed by exactly the intended property failure(s):

| Deliberate change to real L3 | Failed property |
| --- | --- |
| Remove STOP's current FIFO check | `STOP_REQUIRES_FRESH_FIFO`, `STOP_CURRENT_NOT_READY` |
| Replace the requested wake condition with Completion | `OBSERVE_EXACT_CALL` |
| Clear the forwarded NACK bit | `OBSERVE_ALL_STATUS_BITS_UNCHANGED` |
| Return the received buffer one byte shorter | `RX_EXACT_CONTENTS_AND_LENGTH` |

Evidence is in `C:/temp/i2c-l3-proofs-20260915/controls-first/`. All four baseline
processes exited 0; all four mutants exited 1 due to those assertions, not a
compiler, unwinding or timeout failure. The final result is accepted, with
unchanged checkout sources and unchanged lockfiles in every fixture. The
control parser's 17 self-test fixtures also pass. A final hardening requires
exact process exits (0 for the baseline, 1 for the mutant) and detailed check
counts matching the summary. All eight saved logs pass the hardened parser;
that replay is recorded in `C:/temp/i2c-l3-proofs-20260915/controls-parser-final.json`.
The original solver logs remain unchanged. These checks demonstrate
that selected proofs detect regressions in L3, not merely that the modeled L2
can satisfy its own contract; they are not exhaustive mutation coverage.

The older three-harness results below remain historical rather than current
totals.

## Earlier three-harness shared-status validation

On 2026-09-15, all three harnesses were rerun against the uncommitted
representation-simplification sources and passed: **3 verified, 0 failures,
3 total**, exit 0. The existing WSL installation and cache were reused; no
assumptions, stubs, harness filters or disabled safety checks were added.
The harness now constructs the common seven-boolean `Status`, including an
unconstrained `bus_idle`. Fault classification therefore covers all 256
snapshot/`want_stop` combinations, and completion classification covers all
128 snapshots. The input-size harness is unchanged in meaning. This remains
classification evidence, not a proof of the async completion wait.

The complete successful log is
`C:/temp/i2c-representations-20260915/portable-kani-retry.log`.
`portable-kani.log` records an earlier shell-launch error before any harness
ran; the retry used `wsl --exec` with an explicit Linux PATH. Nothing was
installed or reconfigured. Reproduce from PowerShell without nested-shell
PATH quoting:

```powershell
wsl --distribution Ubuntu --user kani --cd /home/kani --exec /usr/bin/env `
  -u CARGO_ENCODED_RUSTFLAGS -u RUSTFLAGS -u RUSTDOCFLAGS `
  -u CARGO_BUILD_TARGET -u CARGO_TARGET_DIR `
  PATH=/home/kani/.cargo/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin `
  RUSTUP_TOOLCHAIN=stable /home/kani/.cargo/bin/cargo kani `
  --manifest-path /mnt/c/Users/forfr/Documents/Codex/2026-08-19/i2c-pac-safe-wrapper/work/embassy/embassy-mcxa/experimental/i2c-one-shot-proofs/Cargo.toml `
  --target-dir /home/kani/target/i2c-one-shot-b984630ed --output-format regular
```

## Historical six-boolean revision

**Historical validation: all three harnesses passed** at commit
`b984630ed808bd6b1e7a69f43d9b988f080d5ace`, on 2026-09-14 PDT
(2026-09-15 UTC). No driver or harness source needed changing to run them.

Environment: Ubuntu 26.04.1 under WSL2, dedicated unprivileged user `kani`,
Kani 0.67.0, CBMC 6.8.0, CaDiCaL 2.0.0. Verification uses
`x86_64-unknown-linux-gnu` (64-bit pointers/usize) and Kani's bundled
Rust 1.93.0-nightly (`nightly-2025-11-21`). Rust 1.98.1 stable dispatches
the cargo plugin; it is not the compiler used for the proof.

| Harness | Property established |
| --- | --- |
| `fault_priority_for_every_snapshot` | Exact fault priority and unchanged success observation for all 128 combinations of the six status booleans and `want_stop` |
| `completion_observation_never_swallows_a_fault` | With `want_stop=true`, status classification succeeds exactly when no fault is set, over all 64 snapshots |
| `all_input_sizes_are_rejected_or_fit_one_receive_command` | Preflight accepts exactly addresses 0x08..=0x77 and lengths 1..=32, over all u8 addresses and host-usize lengths; accepted lengths round-trip through the harness's count-minus-one calculation |

A deliberately false assertion over an unconstrained byte separately failed
with the named assertion and exit 1. That installation control verifies a real
solver rejection, not merely successful compilation. The actual package run
exited zero and reported **3 successful harnesses, 0 failures, 3 total**.
Default safety, overflow, unwinding, and assertion-reachability checks stayed
enabled. No assumptions, stubs, harness filters, or safety-only flags were used.
All four package lockfile hashes were unchanged.

### Reproduce the historical command on the selected checkout

On this machine, open the installed Linux environment:

```powershell
wsl --distribution Ubuntu --user kani
```

Then run from that Linux shell (the path is this checkout):

```sh
export PATH=/home/kani/.cargo/bin:$PATH
export RUSTUP_TOOLCHAIN=stable
unset CARGO_ENCODED_RUSTFLAGS RUSTFLAGS RUSTDOCFLAGS CARGO_BUILD_TARGET CARGO_TARGET_DIR
cd /home/kani
repo=/mnt/c/Users/forfr/Documents/Codex/2026-08-19/i2c-pac-safe-wrapper/work/embassy
cargo kani --manifest-path "$repo/embassy-mcxa/experimental/i2c-one-shot-proofs/Cargo.toml" \
  --target-dir /home/kani/target/i2c-one-shot-b984630ed --output-format regular
```

The stable override selects the installed dispatcher without downloading all
cross-target components from the repository's Rust 1.92 toolchain file. Kani
then selects its own verifier compiler. These results are not a proof of the
Rust 1.92 ARM binary.

For a fresh supported Linux environment, install Rust via rustup, then
`cargo install --locked kani-verifier --version 0.67.0` and
`cargo kani setup`. The measured installation used the official installer.

Full logs, source hashes, lockfile hashes, command, versions, negative control
and summary are preserved in
`examples/mcxa5xx/target/kani-b984630ed/` relative to the repository.
See [checkpoint 13](../../../examples/mcxa5xx/one-shot/PROGRESS.md).
An ordinary `cargo check` still disables `cfg(kani)`; it neither checks the
harness bodies nor establishes a proof. A filtered or empty Kani run also
does not reproduce this three-harness result.

## Scope and limits of the historical L4 harnesses

This separate package calls the real core `checked` and
`validate_transfer` functions through a local path dependency. Firmware does
not depend on this package, and the core gains no Kani-specific implementation,
dependency or unsafe-code allowance.

The completion harness tests classification, **not** the asynchronous
`Stopping::complete` wait or its STOP-plus-idle condition. The size harness
does not assert the precise error variant for rejected inputs, nor execute
the real DMA setup or RECEIVE register encoding: its count calculation lives
in the harness. The priority oracle checks the stated policy; passing does
not independently establish that the policy matches the chip specification.

This is source-level evidence on the 64-bit Linux verification target, not an
ARM binary proof. The checked functions have no target branches or pointer
operations and their accepted arithmetic fits either width, but that source
argument is distinct from running an ARM model.

These proofs do **not** establish async scheduling, interrupt delivery, DMA
quiescence or physical memory transactions, the unsafe Device implementation,
reset timing, or electrical I2C behavior. Compiler rejection checks,
real-adapter effect tests and connected-board tests remain separate evidence.
The separate [35-case mutation and board record](../../../examples/mcxa5xx/one-shot/REPRESENTATION-SIMPLIFICATION.md)
is not replaced by these proofs. The new L3 source controls likewise have their
own narrower scope.

References: [Kani installation](https://model-checking.github.io/kani/install-guide.html),
[supported Rust features](https://model-checking.github.io/kani/rust-feature-support.html).
