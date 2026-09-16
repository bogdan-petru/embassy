# I2C port-binding refactor — validation, 2026-09-15

The implementation is described in [I2C-PORTING.md](I2C-PORTING.md). This run
uses uncommitted changes on `8d9c2018b342d2f90e8b30c74d45c3bdd2f08568`, alongside
the preceding uncommitted UART port work. Nothing was committed or pushed.

## Preserved behavior and the reviewed boundary

- L3 vocabulary and L4 protocol **library sources are unchanged**, SoC-free,
  and still forbid unsafe. Policy/port compiler tests were updated separately.
- The production controller, target, reset and eDMA sources are unchanged.
- Same 12 MHz functional clock, 100 kHz bus, 1..=32 bytes per phase, one
  consuming write or combined write/read, and one 100 ms cooperative deadline.
- Same confirmed reset/enable, directional endpoint/request pairs, DMA arm and
  quiescence, exclusive static RX ownership, cancellation and forgotten-future
  constraints. No chunking, retry, bus-clear or standalone-read path was added.
- Acquisition now muxes pins while the reset engine is disabled, before the
  IP adapter installs timing. Pin/DMA/interrupt/clock integration belongs to the
  concrete `McxaI2cPort`, not the transfer algorithm.
- The final resource guard quiesces DMA before releasing its WakeGuard. Unlike
  UART, I2C keeps the peripheral clock gate enabled after Drop, as before.
  An initial draft copied UART's gate-off policy; review caught that it would
  make the existing passive MCR/MDER audits unreliable. That draft was removed
  before board testing; the audits were not weakened or preceded by repairs.

An independent read-only comparison checked all four original example bodies
against the shared bodies and MCXA peer adapter: no payload, address, outcome
check, error propagation or ownership assertion was lost. This is a reference
binding, not a completed generic L2 crate or a cross-family hardware port.

## Native, compiler and build results

All listed checks passed using Rust 1.92 and `RUSTFLAGS=-Dwarnings` where
applicable. Host target: `x86_64-pc-windows-msvc`. ARM target:
`thumbv8m.main-none-eabihf`.

| Check | Completed result |
| --- | --- |
| HAL library, `one-shot-host-tests,uart-one-shot` | 70/70 per family, MCXA2xx and MCXA5xx |
| Core decision tests | 8 protocol tests and 5 property/exhaustive tests |
| L3 vocabulary | 11 operation/ownership tests |
| Core boundary policies | 11 unsafe/dependency policy tests and 5 wake-only IRQ policy tests |
| New owned-port compiler controls | Positive construction; E0599 for clone, E0382 for reuse, E0616 for private-resource extraction; exact diagnostic/span checks |
| HAL I2C doctests | 1 positive and 4 compile-fail snippets per family, without custom-MMIO |
| Shared example crate | 12 I2C and 10 UART tests; also checked as ARM `no_std` |
| Driver contracts | 20 tests; I2C report valid with 9 registers, 4 operations and 1 port description |
| Generated accessors | I2C 9-register and UART 7-register drift checks; 12 generator tests |
| Firmware dependency graph | No `safe-mmio/custom-mmio` in the I2C firmware graph |
| ARM library builds | Both MCU families, with and without `defmt` |
| Linked example firmware | Four normal write/write-read bins, cleanup bin and two RXSTALL bins |
| Runner controls | 20 mutation-runner tests, 81 RXSTALL verdict controls and 19 startup controls |
| Formatting/patch hygiene | Rustfmt and `git diff --check` passed |

The wake policy inventories nine source files after ISR relocation and still
rejects PAC accesses, redirected sources and unreviewed helper/clock paths.
All 35 named mutation anchors were checked for unique applicability. The full
**committed-baseline mutation campaign was not rerun**: this work remains
uncommitted. Anchor checks and compiler port controls are not a substitute for
executing that campaign. Kani/cargo-mutants were not rerun; protocol code did
not change. The complete repository CI/board-farm matrix was not run locally.

Firmware used these flags (with `DEFMT_LOG=info`):

```text
-Dwarnings -C linker=flip-link -C link-arg=-Tlink.x -C link-arg=-Tdefmt.x -C link-arg=--nmagic
```

## Connected boards

Two FRDM-MCXA577 boards: controller `1fc9:0143:ZS1PGS20MCJBC`, target
`1fc9:0143:0YHQFRBFS4FIG`; SDA P3_20, SCL P3_21, common ground and the existing
pull-ups. Probe-rs 0.31.0, SWD 4000 kHz. Both roles exited successfully for each
two-board test. These are MCU target-driver observations, **not a new Saleae
waveform capture**.

| Test | Required and observed outcome | Evidence |
| --- | --- | --- |
| Write | Exact 16-byte `Hello from DMA!\n` payload and STOP | [Result](../target/i2c-one-shot-hil/6d0cdb4dab4340209cdffef9fbbb8381/result.json) |
| Combined write/read | Register 0x10, repeated START **without intermediate STOP**, exact four received bytes, RX quiescent and final STOP | [Result](../target/i2c-one-shot-hil/b29c388786ea475e8781b31436dd15c4/result.json) |
| Short RX-service stall | At least 5 ms withheld; exact write and STOP resume successfully; controller returned in 6909 us | [Verdict](../target/i2c-one-shot-rxstall/80c0dc2020374a15b6c64c540d83caf7/verdict.json) |
| Long RX-service stall | At least 150 ms withheld; exactly `Timeout`, return in 100112 us, passive local cleanup assertions pass | Same RXSTALL verdict |
| Restored normal pair | Exact 16 bytes and STOP after diagnostic tests | [Result](../target/i2c-one-shot-hil/c118c860f2f641b6998e3078aba3aebe/result.json) |

The combined target reported `EarlyStop(5)`: its count describes queued target
bytes, not five received bytes. The controller independently received exactly
four. Before reply, the target required `Restarted(1)` on its two-byte scratch;
`Stopped(1)` or `BufferFull` would fail even with the correct register value.

Two separate cleanup firmware invocations also exited zero, with their output
recorded in the task's terminal tool results (not a separate raw-log directory):

- `I2C_ONE_SHOT_CHECK=nack`: unserved address 0x2b returned exactly `Error::Nack`.
- `I2C_ONE_SHOT_CHECK=cancel`: poll once, require Pending and TDDE armed, then
  drop that polled future. This was not cancellation of an unpolled future.

Both retained the original post-Drop assertions: eDMA ERQ off and ACTIVE clear,
TDDE off, MEN off, LPI2C IRQ disabled. Neither reset/re-enabled the controller
before auditing. The peer was reset between cases, and the RXSTALL runner later
restored and reran the normal pair. These are targeted TX cancellation/NACK
checks, not a new RX cancellation race sweep or peer/bus-recovery proof.

The normal and combined runner now records selected source hashes (including
the shared bodies/peer decoder), dirty source status and ELF hashes. This is
not a complete transitive build attestation. RXSTALL preserves its build,
status, timing, endpoint and restoration records and now includes the shared
example paths in its declared source scope. Logs remain local ignored evidence.
The restored ELF pair still matches its recorded hashes. One selected source
hash changed afterward: rustfmt expanded the expected array in the host-only
`release_sequence.rs` test. No production expression changed in that formatting
edit; the dated hardware snapshot is retained rather than rewritten.

## Interpreter check

A separate Miri run passed **44/44 selected `i2c::one_shot` host tests and
exited zero**, including the unchanged real 65,536-poll reset-failure bound.
The other 17 HAL tests were filtered out; this is not a full-HAL or MCXA2xx
interpreter qualification. Exit-time leak checks remained enabled. The native
and board results above are separate evidence, not substitutes for this run.

```powershell
$env:RUSTFLAGS = '-Dwarnings'
$env:MIRIFLAGS = '-Zmiri-strict-provenance'
cargo +nightly miri test --locked --manifest-path embassy-mcxa/Cargo.toml --lib `
  --no-default-features --features mcxa5xx,one-shot-host-tests `
  --target x86_64-unknown-linux-gnu `
  --target-dir examples/mcxa5xx/target/miri-b984630ed/build `
  -- i2c::one_shot --test-threads=1
```

The Windows-hosted interpreter uses rustc
`1.95.0-nightly (f889772d6 2026-02-05)`, default Stacked Borrows plus strict
provenance, with no reduced poll bound or disabled alias/leak checks. This tests
executed host-memory paths, not physical DMA transactions or electrical timing.
