# Share the lifecycle, keep the transfer explicit

This records the first simplification stage. The later
[shared-control step](SHARED-CONTROL.md) has separate size and validation accounting;
the numbers below are not the aggregate after that subsequent change.

This is a small simplification of snapshot `84b2e3f98`, not a new framework or
a claim that a complete safe async DMA implementation is now a few dozen lines.
The production controller/target/eDMA drivers and the public one-shot API are
unchanged. No new crate or register abstraction is introduced.

## What is shared

- One private `DmaLease<E, B>` owns the register/channel pair and actual storage.
  TX specializes it with `&'static [u8]`, RX with `&'static mut [u8]`. These are
  distinct types, not a direction flag that can lose the RX ownership rule.
- Observation, stale-wake registration, DMA error/DONE polling and Drop now
  have one implementation. Pending and error leave cleanup armed. Success
  restores CPU authority and storage only after requests-off, quiescence of the
  same channel, and the acquire fence. Cancellation does not produce success.
- A single generic deadline selector wraps two explicit transaction recipes in
  [session.rs](../../../embassy-mcxa/src/i2c/one_shot/session.rs). It retains the
  same one 100 ms deadline and transfer-first polling when both futures are ready.

TX/RX hardware arming deliberately remains different. RX still checks live FIFO
space, arms RDDE before START-read/RECEIVE, and holds exclusive storage through
cleanup. DMA DONE still does not establish STOP or wire completion. CPU-state
shutdown retains its separate typed path. The four layers and L3/L4 unsafe
prohibitions remain intact.

## Size, counted honestly

These are nonblank source lines not beginning with `//`; braces, attributes and
signatures count. Test-only modules/hooks are excluded. This is not a statement
count, binary-size result, or a count of the entire dependency graph.

| Production source | Before | After |
| --- | ---: | ---: |
| Portable protocol, vocabulary and interface | 452 | 452 |
| Handwritten MCXA adapter/binding | 893 | 847 |
| Generated register access | 187 | 187 |
| Shared confirmed reset | 173 | 173 |
| **Total** | **1,705** | **1,659** |

The reduction is **46 code lines (2.7%)**, or 55 physical lines when comments and
blanks are included: 2,442 to 2,387. It removes duplication without moving it to
another library. It does not yet meet the larger ambition of a few hundred lines
per new driver. The board entry points and shared example bodies are unchanged.

The delta is reproducible by comparing the nonblank/non-`//` counts of
`session.rs` (83 to 69) and `session/vocabulary/registers.rs` (392 to 360) against
`84b2e3f98`. The latter includes unchanged inline tests in both measurements;
they cancel in the delta. The new RX deadline tie-break/returned-reference test
adds 30 net physical lines to the separate host test file, not firmware.

## Validation and its limits

Host execution passes 45 selected I2C tests. An additional deadline test
checks that a completed combined operation wins the existing ready-deadline tie
and returns the same RX storage. Existing effect tests exercise the shared DMA
implementation for both directions, including fault/cancellation cleanup.

The previous snapshot's mutation run was 34/35: `bytes[0] = 0` was correctly
rejected after forgetting the receive owner, but emitted E0503 and E0506 where
the strict fixture expected only E0506. The replacement mutation, `bytes.fill(0)`,
still attempts a forbidden safe write and produces one precisely matched E0499.
The runner's diagnostic, path, span and baseline requirements are unchanged.
An isolated fixture and the runner's 20 self-tests pass; a full committed-baseline
campaign is a separate result, not implied by that probe.

| Completed check | Result |
| --- | --- |
| Host HAL with custom MMIO | 71/71 per MCU family, MCXA2xx and MCXA5xx |
| Host HAL without custom MMIO | 27/27 per family |
| I2C protocol/vocabulary and boundary/compiler controls | 30 core tests and 11 vocabulary tests |
| Shared I2C/UART example bodies | 22/22 |
| Generated maps / generator tests | 17/9/7 registers match; 12/12 tests |
| Firmware feature exclusion | I2C and UART pass; 12/12 checker tests |
| ARM library matrix | I2C/UART, both families, defmt off/on: eight checks pass |
| Doctests | Five I2C and five DMA snippets per family pass |
| Full named mutation campaign | 35/35: 27 exact compiler rejections, eight effect-order rejections |
| Miri, selected MCXA5xx I2C host paths | 45/45, exit zero |
| Board write | Exact 16 bytes, observed STOP, both roles exit zero |
| Board combined write/read | Repeated START, no intermediate STOP, exact four bytes, both roles exit zero |
| Board NACK and polled-future cancellation | Expected outcomes, DMA inactive, requests/MEN/IRQ off |
| Board short service stall | Exact write/STOP resumes; controller 6,912 us |
| Board long service stall | Exactly Timeout at 100,117 us; local cleanup passes |
| Normal pair restored | Exact write/STOP passes after diagnostics |

Miri uses the existing nightly 1.95 interpreter with strict provenance and
default Stacked Borrows. The full 65,536-poll reset-failure bound was retained;
the selected run took 653.42 seconds. This checks executed host-memory paths,
not physical DMA or electrical timing. Native builds use Rust 1.92 and
warnings-denied flags where applicable.

Local board evidence (ignored build output, not part of the source snapshot):
[write](../target/i2c-one-shot-hil/23d96a9360944cab868350e4c8bbd67f/result.json),
[write/read](../target/i2c-one-shot-hil/2b319aa2e0584caf96be2041d7a77701/result.json),
[short/long stalls](../target/i2c-one-shot-rxstall/9a3b9bf900754f2aa5afcf6b9f23a3bc/verdict.json),
[restored pair](../target/i2c-one-shot-hil/6512092259fe4fb0a5922d87f9bc1298/result.json).
Build/native/Miri/cleanup logs are retained under
`C:/temp/i2c-simplification-84b2e3f98-001` on the validation machine.

The full mutation campaign passed against a clean **test-only copy**, commit
`8c07a944b9d7a9c92f8936d8d2bbc868874cb41c`, not an amended user snapshot. Its
source patch SHA-256 is
`338a6f66564f94b1c5884298c1d7d1a0958e4cd99717d950bce4d19ab9594e62`.
All 2,814 copied source files were verified with only Git newline conversion
permitted, and all 12 lockfiles were byte-identical. This note's subsequent
result updates are documentation-only. At validation, the original branch
remained at `84b2e3f98` with the refactor uncommitted; nothing was pushed.

Every positive baseline succeeded and every mutant failed for the exact named
reason, with no timeouts. A final audit verified all 420 saved lockfile copies
against the committed fixture after newline normalization, and confirmed the
changed Rust/test/mutation inputs still match the working source. Reports,
mutation diffs, command logs and lockfiles are retained at
`C:/temp/i2c-simplification-mutations-001/results`; only its 70 disposable build
caches were removed. The native workspace build caches were left in place.

This validates the named checks, not every possible implementation mistake.
No Kani rerun, new waveform capture, second-platform qualification, or
peer/bus-recovery proof is claimed.
