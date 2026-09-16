# One-shot controller validation — 2026-09-12

This is a chronological record. Statements that no commit was made describe
the validation sessions below, before the checkpoint containing this file.

Implemented alongside the existing I2C controller on
`experiment/i2c-pac-safe-wrapper`, base commit `a897f9418`.
No existing controller/target/DMA implementation was changed. Integration is
an opt-in feature, a new module, and a private per-instance register factory.
No commit or push was performed.

## Review follow-up — 2026-09-12

No transfer algorithm changed. Added explicit ISR/PAC-aliasing and private
Option teardown invariant comments, linked the narrowed Abacus experiment
back to the framework plan, and made the unused upstream runtime opt-in.
AbacusCell/AbacusStaticRef now compile only with the non-default legacy-tock
feature. The original Tock macro and crate re-exports are opt-in too;
the shared DSL generator source remains compiled and preserved.
The HAL dependency explicitly uses default-features=false.

Validation after this cleanup:

- DSL unit tests: 48/48.
- Default backend: 3 host tests and 15 doctests pass (11 original plus four
  independent compile-fail checks for absent legacy exports).
- legacy-tock enabled: 3 host tests and 12 doctests pass (11 original plus
  a positive compatibility check).
- Both backend feature matrices: all-target checks and docs builds pass with
  RUSTFLAGS/RUSTDOCFLAGS=-Dwarnings.
- I2C host tests: 11/11 on each chip family; public doctests 3/3.
- ARM checks: mcxa2xx/mcxa5xx, defmt on/off, plus mcxa5xx with one-shot
  disabled, all pass with warnings denied.
- All three example firmware binaries rebuilt with warnings denied.
- Default HAL dependency graph does not enable legacy-tock.
- Both PAC map generators, formatting, and diff whitespace checks pass.

The dependency change required refreshing ignored HAL/example Cargo.lock
files offline before repeating locked checks. Only two empty old-name
directory trees were removed, without deleting files; the generator cache
is now ignored.

At that checkpoint the four driver files were 587 lines (the increase was invariant comments).
Vendored Rust was 2,991 lines including tests, with the old runtime then separated; see PROGRESS.md checkpoint 6 for current sizes.
The earlier size counts below are historical measurements.

Hardware was not reflashed or rerun for this feature-gating/documentation
cleanup. The two-board/NACK/cancellation results below remain the last
hardware results; the freshly rebuilt binaries were not exercised on boards.
No commit or push was performed.

## Abacus + safe-mmio integration — 2026-09-12

The controller's Layer 1 now uses the local Abacus backend with safe-mmio
0.3.1. All nine mapped LPI2C accesses, including clock timing setup, use that
backend. The PAC provides address calculation and bitfield values only.
The receiving peer remains the existing production DMA target.

Hardware, same two FRDM-MCXA577 boards and wiring described below:

- First normal run: exact 16-byte receive and STOP, both exits 0.
  Log ID `5cb8b2074454415e81925a5c417da7cf`.
- NACK cleanup rerun: exactly Nack at unserved 0x2b; shutdown assertions pass,
  firmware exit 0.
- Cancellation cleanup rerun: poll to Pending with TDDE armed, then drop;
  shutdown assertions pass, firmware exit 0.
- Both cleanup checks assert ERQ off, ACTIVE clear, TDDE off, MEN off, and
  LPI2C NVIC disabled. These are targeted checks, not a cancellation race sweep.
- Final normal rebuild/run after cleanup checks: both exact verdicts and
  process exits 0. Log ID `cbaa6edae39243d986ee91493fc3c0b3`.
  This leaves the normal controller/target pair installed.

Normal-run logs are under `../target/i2c-one-shot-hil/`. Final tails:

```text
ONE-SHOT CONTROLLER PASS: 16 bytes, DMA finished, STOP observed
ONE-SHOT TARGET PASS: exact 16 bytes and STOP
ONE-SHOT TWO-BOARD PASS
```

Final flashed ELF SHA-256:

```text
controller A0AB2D61897D9642D194B81AEC5B639B3B973520D1F181D6DA455155F10E8853
target     B8F41C4D0B48BCE7FCD26C8726F444DACE028E5AF24FA1E63F2B8A60EA86F594
```

Static/host validation:

- Warning-denied ARM library checks: mcxa2xx/mcxa5xx, defmt on/off, all pass.
- Warning-denied mcxa5xx with one-shot disabled passes.
- All three one-shot firmware bins built with warnings denied.
- Library host tests: 11/11 per chip family; public one-shot doctests: 3/3.
- Abacus parser/generator/backend unit tests: 48/48.
- Real generated-MMIO host tests: 3/3, checking actual words/offsets/state writes.
- Backend doctests: 11/11, including one legal case and ten compile-fail cases.
  These test the same generated fixture; they are not mutations of the complete
  I2C driver and are separate from the historical source mutations below.
- New map generator: 9 registers match pinned PAC. Existing production map:
  17 registers match. Both resolvers use locked Cargo metadata.
- Rustfmt and git diff whitespace checks pass. Stable rustfmt still emits
  the existing warnings about nightly-only formatting settings.

Four driver source files total 580 lines; new generated map 96. The separate
local Abacus tree contains 2,976 Rust lines including its tests; 388 are the
new backend and its tests. These counts exclude documentation/generator/shared
HALs and include comments and blanks. No production driver migration, commit,
or push was performed.

This experiment demonstrates working integration, not complete hardware
verification. No new bus-clear, stuck-line, long-transfer, receive operation,
or MCXA2xx hardware validation was attempted. Abacus currently generates only
Off/Enabled/ResetHeld register-access states; DMA ownership and async protocol
completion remain responsibilities of the reviewed adapter/vocabulary.
See [scope and tradeoffs](ABACUS-SAFE-MMIO.md).

## Rename revalidation — 2026-09-12 (historical)

The module is now `i2c::one_shot`, enabled by `i2c-one-shot`. Binary names,
runner paths, diagnostics, build-time check selector, and documentation were
renamed together. Driver behavior was not changed by this rename.

- All three renamed binaries built with `-Dwarnings`.
- Library host tests: 11/11 on mcxa5xx and 11/11 on mcxa2xx.
- Public doctests: one positive example and two compile-fail cases passed.
- ARM library checks: mcxa2xx/mcxa5xx, defmt on/off, `-Dwarnings`, all passed.
- Generated PAC/Tock map: 17 registers match. Rustfmt and diff checks passed
  with only the existing stable-rustfmt nightly-option warnings.
- The renamed normal pair ran on the two boards below: both exact verdicts
  and both process exit codes passed. Log ID
  `5a855ec16c944d89ae58e621cb9c298c`, under `../target/i2c-one-shot-hil/`.

Tails from the rename validation:

```text
ONE-SHOT CONTROLLER PASS: 16 bytes, DMA finished, STOP observed
ONE-SHOT TARGET PASS: exact 16 bytes and STOP
ONE-SHOT TWO-BOARD PASS
```

ELF SHA-256 at the rename validation:

```text
controller 0B6B19872E538D7053075473C4D863EEA7EBC2A3CD3F72A0981A9D343FC7513D
target     9681AA3AA36723EBBCB3777FDE91D0BF3A827A5704306C5FA8287D7E96C70A71
```

The internal source mutations and the separate NACK/cancellation hardware
checks below are historical results from 2026-09-11; they were not rerun for
the rename. The renamed cleanup firmware was rebuilt, not executed.

## Original implementation validation — 2026-09-11

## Size

Line counts include comments, blank lines, and embedded host/doc tests:

| File | Lines |
| --- | ---: |
| Sending application | 33 |
| Receiving test-peer application | 37 |
| Layer 4: public API | 125 |
| Layer 3: session | 42 |
| Layer 2: vocabulary | 159 |
| Layer 1: hardware boundary | 241 |

The four new driver files total 567 lines (432 nonblank, non-line-comment
lines). These counts do not include the reused generated register map, eDMA/
clock/pin HAL, executor, or the existing target driver. They also exclude this
walkthrough, the host runner, and separate cleanup-check firmware.

## Hardware

Two FRDM-MCXA577 boards, SDA P3_20, SCL P3_21, common ground, existing 3.3 V
pull-ups. Controller: ZS1PGS20MCJBC; target: 0YHQFRBFS4FIG.
One 100 kHz, 16-byte async/DMA write to 0x2a, with exact receiver verification.

- Initial complete run: both firmware verdicts and both exit codes passed.
  Log ID `85c98d8c083a440aadc86a2a2b45254e`.
- Final warning-denied build: both firmware verdicts and both exit codes passed.
  Log ID `9e331991d7dd4ea5a2dd2d100be86059`.
- These two log IDs refer to the original pre-rename artifact directory,
  not the new directory used by the revalidation above.

Original flashed ELF SHA-256 (before the rename):

```text
controller 83BC1F3AA777464057F33B11D67B226DE7351E1AF25B87119EF4997F8BE87580
target     3828ED190D9ED05D70456210353C7983CEFA27F4B30AE3C244E43091B7CE8A42
```

Two separate hardware cleanup checks also passed:

1. Write to unserved 0x2b: exactly `Error::Nack`, not an accepted generic failure.
2. Poll a write to 0x2a once, assert Pending and TDDE still armed, then drop
   the future. This is cancellation after arming DMA, not an unpolled future.

Both then inspected hardware and asserted ERQ off, ACTIVE clear, TDDE off,
MEN off, and the LPI2C NVIC interrupt disabled. The board was reset and the
normal pair rerun afterwards. These are targeted checks, not a cancellation
race sweep or proof of every possible fault/await interleaving.

To reproduce a cleanup check, from `examples/mcxa5xx`, after the receiver
firmware is installed:

```powershell
$env:I2C_ONE_SHOT_CHECK = 'nack' # Or 'cancel'.
cargo build --locked --release --features i2c-one-shot --bin i2c-one-shot-cleanup-check
probe-rs reset --chip MCXA577 --probe '1fc9:0143:YOUR_TARGET_SERIAL'
probe-rs run --chip MCXA577 --probe '1fc9:0143:YOUR_CONTROLLER_SERIAL' --speed 4000 --scan-region ram target/thumbv8m.main-none-eabihf/release/i2c-one-shot-cleanup-check
Remove-Item Env:I2C_ONE_SHOT_CHECK
```

Use only the two-board rig with no device at 0x2b. Reset both boards after
cancellation: the check asserts local shutdown, NOT peer/bus recovery.
The cleanup binary rejects a missing or unknown build-time case.

## Compiler and host checks

- 11/11 library host tests passed on both mcxa5xx and mcxa2xx: eight existing
  tests plus three one-shot checks (input bounds, fault priority, STOP+idle).
- One-shot doctests: one positive example compiled; both intended failures
  (reuse a consumed driver, pass stack storage) were rejected.
- Six mutations of actual driver source were separately compiled and reverted.
  Each failed for its expected diagnostic:
  - STOP on Streaming: E0599, no method `stop`.
  - STOP twice on Queued: E0382, moved value.
  - Construct `Started(())` in the vocabulary: E0423, private tuple fields.
  - Write the read-only receive register: E0599, no `set` on ReadOnly.
  - Write the receive register through the PAC instead (`pac.mrdr().write_value`):
    E0599, `Reg<Mrdr, R>` does not satisfy `Write`. The PAC enforces direction
    independently of the Tock map.
  - Reach `Port.hardware` from the session: E0616, private field.
- The unmutated warning-denied build passed before and after those experiments.
  Unlike the public doctests, these six mutation experiments are a recorded
  validation exercise, not currently an automated CI compile-fail suite.
- Warning-denied ARM library checks passed with one-shot enabled for mcxa2xx
  and mcxa5xx, each with defmt on/off and default features disabled.
- A normal mcxa5xx library check with one-shot disabled passed.
- All three one-shot binaries built with warnings denied. The normal
  controller/target pair from that build was the final hardware run above.
- Generated PAC/Tock layout check passed: 17 registers.
- Rust formatting and tracked diff whitespace checks passed. Stable rustfmt
  only warned about the repository's two nightly-only formatting options.

Hardware checks were MCXA577-only. No MCXA2xx board, different clock, read
operation, long transfer, stuck-line watchdog, bus-clear, or multi-controller
scenario was validated. The full `controller::I2c` remains the option
for those wider requirements.
