# Miri: host memory validation

## What changed

The first Miri run against `b984630ed` stopped in the host recorder, not in
physical MMIO. The fixture obtained its mapping through the boxed slice's
`as_mut_ptr()`, which introduced an exclusive slice borrow. A simulated device
write through a separate `UnsafeCell` access invalidated that pointer; the next
volatile read during reset confirmation was rejected by both Stacked Borrows
and Tree Borrows.

`Recorder::base()` now derives the mapping through shared `UnsafeCell` storage:
`self.memory.as_ptr().cast_mut().cast::<u8>()`. The allocation remains owned and
the existing single-owner guards remain intact. The important change is avoiding
the exclusive slice borrow, not hiding an access from the interpreter.

The RX effect tests also use distinct `ConstStaticCell` buffers instead of
`Box::leak`. The five deadline phases receive five disjoint static slices.
This preserves the actual exclusive `'static` API contract, including when
cancellation consumes the reference, without a deliberately leaked heap
allocation. Each effect-test macro expansion may be taken once per process;
a future loop needs distinct buffers, as the deadline test already provides.

Only these host-test files changed:

- `embassy-mcxa/src/i2c/one_shot/session/vocabulary/registers/test_support.rs`
- `embassy-mcxa/src/i2c/one_shot/session/vocabulary/effect_tests.rs`
- `embassy-mcxa/src/i2c/one_shot/session/deadline_tests.rs`

No production transfer, MMIO, DMA, interrupt, reset, or cancellation code changed.
No dependency changes, test skips, `cfg(miri)` exceptions, reduced poll bounds,
or disabled alias/leak checks were introduced. All original test assertions and
the total of 59 HAL library tests remain.

## Validation, 2026-09-14

Toolchain: installed nightly manifest `2026-02-06`, rustc
`1.95.0-nightly (f889772d6 2026-02-05)`, Miri
`0.1.0 (f889772d65 2026-02-05)`; full compiler commit
`f889772d6500faebcac5bb70fa44b5e6581c38cd`.
Windows hosted the interpreter; `x86_64-unknown-linux-gnu` was the interpreted
target. WSL was not used. The repository's Rust 1.92 pin remains unchanged.

| Run | Result |
| --- | --- |
| MCXA5xx, Stacked Borrows, strict provenance | 59/59, exit 0 |
| MCXA2xx, Stacked Borrows, strict provenance | 59/59, exit 0 |
| MCXA5xx, Tree Borrows, strict provenance | Incomplete: stopped after the 20-minute wall-clock limit |
| MCXA2xx, Tree Borrows, strict provenance | Incomplete: stopped after the 20-minute wall-clock limit |
| Recorder-only smoke tests, both borrow models, MCXA5xx | 7/7 each |
| Native Rust 1.92 HAL suites, `RUSTFLAGS=-Dwarnings` | 59/59 per MCU family |
| Native core protocol/source/IRQ policy tests, `RUSTFLAGS=-Dwarnings` | 24/24 |
| Formatting and `git diff --check` | Pass; stable rustfmt notes nightly-only options |

The full Stacked Borrows runs reached all 59 tests and exited zero with leak
checking enabled. The optional full Tree Borrows runs were still executing
`failed_reset_never_reports_pin_handoff_even_with_a_stopped_clock` when stopped
at approximately 1,233 and 1,223 seconds respectively. Neither emitted a UB
diagnostic before termination, but neither completed the suite or exit-time
leak checks. Their nonzero termination exits and `*.limit.log` records must not
be treated as passes. The seven-test Tree Borrows recorder smoke run is a
separate, completed, filtered result; it does not substitute for the full suite.

Separate interpreter controls accepted valid raw-pointer volatile accesses and
rejected intentional competing-alias and use-after-free accesses, under both
borrow models. Those four rejections were runtime UB diagnostics, not compiler
or linker failures. The invalid controls ran only under Miri.

Ignored logs, exit records, runner, baseline failures, and controls are retained
under `examples/mcxa5xx/target/miri-b984630ed/`. The `baseline-*` logs describe
the unmodified source; `fixed-*` describe the host-fixture fixes. This record is
not a claim that the isolated compiler-mutation suite has been rerun.

## Reproduce

Use the toolchain above. The local evidence runner checks its full compiler
commit before invoking the installed `nightly` alias. From the repository root,
with unrelated `RUSTFLAGS`, `CARGO_ENCODED_RUSTFLAGS`, and build-target overrides
unset:

```powershell
$env:MIRIFLAGS = '-Zmiri-strict-provenance'
cargo +nightly miri test --manifest-path embassy-mcxa/Cargo.toml --locked --lib `
  --no-default-features --features mcxa5xx,one-shot-host-tests `
  --target x86_64-unknown-linux-gnu -- --test-threads=1
```

Repeat for `mcxa2xx`. To repeat with Tree Borrows, set
`MIRIFLAGS` to `-Zmiri-strict-provenance -Zmiri-tree-borrows`.
There is no test filter. Each invocation must reach a 59-pass summary **and**
exit zero: a test summary alone does not rule out an exit-time leak diagnostic.
The stopped-clock recovery tests execute their real 65,536-poll bound and are
substantially slower in the interpreter than natively.

## What this does not establish

Miri checks Rust memory behavior along the host paths actually executed, under
its experimental borrow models. It does not automatically enforce every
`safe-mmio` constructor contract merely because a raw-pointer wrapper is named
`UniqueMmioPointer`. Some scripted reads and control-register writes use the
recorder model without dereferencing the supplied pointer. The DMA seam records
HAL calls rather than performing hardware memory transactions.

Consequently this is not a proof of physical DMA quiescence, device timing,
electrical behavior, all interrupt schedules, or general driver soundness.
The Kani decision proofs, effect-order assertions, compiler rejection controls,
and board evidence remain separate claims. No hardware rerun is required to
exercise these host-only fixture changes, and none was performed for this fix.
