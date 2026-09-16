# Shared mechanism, explicit hardware recipes

This follow-on to [the DMA/deadline cleanup](SIMPLIFICATION.md) reduces repeated
handwritten control-adapter work across I2C and UART. It does not merge their
protocols, change the four layers, or claim a smaller complete implementation.
The core/vocabulary/interface crates and transfer algorithms are unchanged.

## What a driver author reads and supplies

1. **Describe the control fields.** The existing Rust contract now explicitly
   maps each recipe's `Register` variant to a PAC field and its read/write mode.
   Seven fields are eligible across the two IPs; FIFO data and retained shared
   status handles are not. Eligibility is reviewed semantic input, not something
   an SVD's access-direction bits can establish.
2. **Read the hardware recipe.** I2C retains its controller/target mode handling,
   configuration preservation and bound `Disabled` evidence. UART retains its
   distinct reset, timing, transmitter-enable and request-disable operations.
   Successful readback still precedes usable state reconstruction; failure still
   attempts the existing reset-held fallback without claiming silicon accepted it.
3. **Reuse bounded confirmation.** `embassy-mcxa/src/register_wait.rs` owns the
   fixed 1,000-us / 65,536-poll mechanism. It does not grant hardware authority,
   choose registers, perform reset writes, or quiesce DMA.

The generator supplies only a private `ControlIo` inside each existing `mmio`
module. It creates temporary, exclusive, field-sized handles; it adds no public
register operation, arbitrary state conversion, constructor, or cleanup token.
I2C's retained MSR/MFSR handles and UART's exclusive STAT access are unchanged.
The shorter handwritten `control_boundary.rs` files now both forbid unsafe code.
The generated MMIO implementation remains part of trusted L2; L3/L4 still forbid
unsafe and retain their separate dependency boundaries.

## Preserve timing differences rather than erase them

The two previous loops differed. This refactor makes that choice explicit:

| Policy | Existing consumer | Matching readback | Failed readback |
| --- | --- | --- | --- |
| `SaturatingDeadline` | I2C | Read clock after observation; accept equality, reject after deadline | Stop at saturated absolute deadline; spin only while pending |
| `ObservationFirstElapsed` | UART | Succeeds before another clock read | Stop when wrapping elapsed time reaches the bound; no spin hint |

Both stop after at most 65,536 observations if time stops. UART's behavior is
not a promise that a successful observation was timestamped inside 1 ms. There
is deliberately no timeout-policy correction hidden inside a reuse refactor.
The helper and recipe tests independently pin these choices, including deadline
equality, late matches, clock saturation/wrap, and the last permitted poll.

## Size: less repeated handwork, not less total source

Counts below compare these unchanged-before-this-step files to snapshot
`84b2e3f98`. Code means nonblank lines not beginning with `//`; braces and
attributes count. Neither file contains tests.

| Handwritten control recipe/boundary | Before code / physical | After code / physical |
| --- | ---: | ---: |
| I2C | 76 / 99 | 47 / 61 |
| UART | 142 / 166 | 103 / 119 |
| Combined | 218 / 265 | 150 / 180 |

Those two files lose 68 code lines (31%). This is **not** a 31% driver-size
reduction. The shared wait helper adds 46 production code lines (58 physical),
the generated adapters add output, and the contract/schema/emitter and independent
tests add host-side code. Total source grows in this step. The benefit being
tested is a smaller per-IP handwritten MMIO audit surface and a reusable third-IP
recipe, not a claim of fewer aggregate lines or a measured binary-size saving.
The earlier 1,659-line I2C measurement in `SIMPLIFICATION.md` is the preceding
stage, not a current count after adding this shared infrastructure.

## Evidence and remaining scope

The generator and schema have independent expected field inventories and negative
controls for access widening, retained-handle overlap, unmapped fields and malformed
input. UART also exercises the actual generated adapter over host memory with
literal expected offsets, write isolation, rejected STAT writes and clock forwarding.
Existing real I2C adapter tests continue to exercise reset effects beneath the recipe.

| Completed check | Result |
| --- | --- |
| Final native I2C recorder link unit | 77/77 per family, MCXA2xx and MCXA5xx, with UART recipe tests enabled |
| Final native plain-MMIO link unit | 37/37 per family, including all four new UART adapter tests |
| Portable I2C vocabulary / core | 11 / 30 tests pass, including policy and compiler controls |
| Portable UART core | 10 tests pass |
| Driver contracts | 26 tests pass, including six new control-I/O metadata checks |
| Generator controls | 17 tests pass; both generated maps match the pinned PAC |
| ARM library matrix | Eight I2C/UART, MCU-family and defmt combinations pass |
| Targeted strict mutations | 12/12: ten compiler controls and two reset-effect controls |
| Miri, MCXA5xx I2C recorder/helper suite | 68/68, no filters or ignored tests, exit zero |
| Miri, MCXA5xx UART plain-MMIO adapter | Four selected adapter tests pass, exit zero; other plain-suite tests not selected |
| Watchdog 120 / 140 ms | Exactly PinLowTimeout at 120,308 / 140,276 us; local reset checks pass |
| Watchdog 200 ms control | Exact 16 bytes and STOP at 151,940 us |
| UART DMA | Exact 16 bytes, DMA quiescence and transmit complete; no extras for 20 ms |
| I2C combined write/read | Exact four bytes, positively observed repeated START and no intermediate STOP |
| Final board restoration | Ordinary I2C write pair passes |

Native checks used Rust 1.92, `--locked`, and warnings-denied flags. Miri used
Rust 1.95.0-nightly (`f889772d6500faebcac5bb70fa44b5e6581c38cd`, 2026-02-05),
strict provenance and default Stacked Borrows with leak checking enabled. The
I2C suite retained the full frozen-clock bounds and reported 799.54 seconds;
the four selected UART adapter tests reported 0.72 seconds. These are host-memory
checks, not an interpretation of physical DMA or electrical timing.

The first combined recorder run exposed a **test-link-unit conflict**: two new
UART raw-memory fixtures were rejected as unregistered addresses by the deliberately
fail-closed I2C recorder (79/81). No register mapping failed in the plain-MMIO run.
Like the existing UART raw-memory test, the four new fixtures now compile only
in the plain-MMIO test unit; the generator test pins that selection. They are
executed there, not discarded, and the I2C recorder's address checks are unchanged.
The initial combined Miri attempt was interrupted after this diagnosis, not counted
as a pass. Separate I2C-recorder and UART plain-MMIO Miri runs qualify the two seams.

From the repository root, using the installed Miri nightly and its Linux target:

```powershell
$env:MIRIFLAGS = '-Zmiri-strict-provenance'
cargo +nightly miri test --locked --manifest-path embassy-mcxa/Cargo.toml --lib --no-default-features --features mcxa5xx,one-shot-host-tests --target x86_64-unknown-linux-gnu -- --test-threads=1
cargo +nightly miri test --locked --manifest-path embassy-mcxa/Cargo.toml --lib --no-default-features --features mcxa5xx,test,uart-one-shot --target x86_64-unknown-linux-gnu -- control_io_tests --test-threads=1
```

The actual reruns reused `examples/mcxa5xx/target/miri-b984630ed/build` via
`--target-dir`; logs are `miri-i2c.log` and `miri-uart.log` in the directory below.

The twelve selected mutations ran against clean **test-only** commit
`5a0439f1559d28db0331d2f3c84735514d5fe2ba`, tree
`a42d92775c83045d876dec35a68658730cd614db`. Patch SHA-256:
`b521712497af7401d194a7da61e8f6f2d5adf9583187f8cf3d54d16189e2afeb`.
All positives passed and mutants failed their exact specified compiler diagnostic
or named effect assertion. The 35-case manifest/strict matcher was not weakened;
only these twelve relevant cases were rerun for this step. Before/after provenance
checked 2,820 files, permitting only Git newline conversion for source; all twelve
lockfiles remained byte-identical. Only this note changed after fixture freeze.
Results and lock/diagnostic/mutation records remain at
`C:/temp/shared-confirmation-mutations-001/results`; its 24 disposable build caches
were removed, not the reports. The user's branch/index/history were not amended.

Local logs, including the initial fixture failure and final native passes, live at
`C:/temp/i2c-shared-confirmation-001`. Board records are
[watchdog](../target/i2c-one-shot-watchdog/38b58e7ff1df4498aa30d71a26107686/verdict.json),
[UART](../target/uart-one-shot-hil/e5408dd955ca49f39c5aa7a31dfc7b5e/result.json),
[combined I2C](../target/i2c-one-shot-hil/1df33542938f4d58ad8e69ab01a045ff/result.json),
and [normal pair restored](../target/i2c-one-shot-hil/6de1a21aeb0a4908b4c7a0fadf178198/result.json).
Runner hashes now explicitly include the shared wait helper and, for I2C, the
full driver's reset sequence. Only test-module selection changed after the board
runs; no firmware behavior or hardware acceptance was changed to obtain PASS.
The watchdog uses `Recovery none`: local controller reset and observed peer release
are qualified, not a general bus-clear guarantee or continuous waveform measurement.

No new Kani proof, second-platform port, automatic bus clear, or generation of reset
semantics is claimed. The user snapshot remains one commit with local changes;
test-only fixture commits do not amend it and nothing is pushed.

Next candidates remain separate changes: generated transition shells require
explicit authority/evidence metadata; policy-engine consolidation must preserve
both negative-control suites; merging L3/L4 crates would need a replacement for
the lost dependency boundary. None is smuggled into this step.

Types constrain permitted operations. Effect tests check their sequencing.
Hardware tests challenge the assumptions underneath both. None replaces the others.
