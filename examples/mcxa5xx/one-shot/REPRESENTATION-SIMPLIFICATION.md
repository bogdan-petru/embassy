# Fewer representations, the same ownership boundary

This follows [shared control](SHARED-CONTROL.md). It simplifies the short
async/DMA I2C example, not the full controller/target driver or its public API.
The C SDK example is unrelated and unchanged.

## Measured scope

Production source is counted as nonblank lines not beginning with `//`, including
braces, attributes and signatures. Test-only files, modules, declarations and
fixture hooks are excluded. Reused Embassy/PAC/eDMA libraries, generator source,
tests, docs and board entry points are outside this driver-source total.
The shared helper includes both supported timing policies, not just one compiled
feature combination. These are source counts, not binary-size measurements.

| Part | Before this step | After | Change |
| --- | ---: | ---: | ---: |
| Portable interface/vocabulary/protocol | 452 | 387 | -65 |
| Handwritten MCXA adapter/binding | 818 | 798 | -20 |
| Generated register access | 237 | 189 | -48 |
| Shared I2C reset recipe | 164 | 164 | 0 |
| Shared bounded confirmation | 46 | 46 | 0 |
| **Total** | **1,717** | **1,584** | **-133 (7.7%)** |

The earlier snapshot was 1,705 lines; this result is 121 below that baseline.
Do not add individual proposal estimates: removing a wrapper can also remove
the enum conversion counted by another proposal. The actual combined diff is
the measurement. This does not establish that the implementation is safer merely
because it is smaller.

## What changed

- **One status representation.** L2 decodes one MSR snapshot into `Status`; L3
  forwards it without rebuilding `Observation`/`FaultSet`. Fault priority still
  belongs to the pure L4 classifier. `bus_idle` affects initial START admission,
  not STOP completion or fault priority. FIFO readiness stays a fresh check at
  the command's point of use, never a retained permission from a status sample.
- **One ownership-carrying result shape.** `Step<PendingOwner, CompletedOwner>`
  keeps the exact pending owner, transforms a completed owner, or carries a
  failure. RX completion carries the controller and exclusive buffer together.
  Existing domain aliases remain readable without duplicating enum definitions.
- **No extra L3 DMA wrappers.** L3 reuses L2's opaque `InFlight` and `Receiving`.
  The L2 wrappers and their private device/lease fields remain. L3's `Hardware`
  remains narrow; it is not an alias exposing `Cpu::command` to its callers.
- **One HAL owner.** `I2c` owns `Idle<Hardware>` directly. `session.rs` contains
  the explicit write/write-read recipes and shared deadline selector, not a
  second `Session` owner forwarding into the same state. Both operations retain
  one 100-ms deadline, transfer-first polling, and preflight before START.
- **Completion includes release.** `Stopping::complete()` retains ownership
  throughout its await, requires fault-free STOP plus controller-idle, drops
  the backend, and only then returns success or exclusive RX storage. The
  intermediate `WireComplete` and separate `finish()` are gone. This intentionally
  changes the lower API's destruction point from a later caller action to the
  successful completion poll; the lifetime test now checks that exact promise.
- **Less generated ceremony.** Private PAC-derived offsets replace the dummy
  `repr(C)` layout and duplicate offset assertions. The same generator validates
  alignment/nonoverlap and emits the release-build PAC comparison and recorder
  access/extent metadata. `ControlIo` matches its explicit operation whitelist
  before one field-sized access per direction. Read-only temporaries cannot
  write; write-only temporaries cannot read. UART STAT remains non-writable.

## What deliberately did not change

All four responsibilities remain. L3 and L4 still forbid unsafe; L2 still owns
the trusted MMIO, clock/pin/interrupt and DMA boundary. Retained shared MSR/MFSR
handles, exclusive FIFO pops, concrete TX/RX endpoint types, DMA arm ordering,
request-off/quiescence/fence restoration, and forgotten static RX storage retain
their contracts. No borrowing poll API replaces the ownership-returning result.

The release-sequence effect test and WaitCell observation/injection hooks remain.
No reset acknowledgment, interrupt mask, DMA mask, timing policy, error variant,
or clock configuration was removed. Neither the START/STOP wire recipe nor
long-transfer support changed. UART shares the mechanical generator cleanup;
its protocol implementation is unchanged.

## Validation record

Logs and per-file counts are in `C:/temp/i2c-representations-20260915` on the
validation machine. Hardware was initially unavailable; the connected-probe
follow-up below now records fresh board results for the same source.

| Check | Result |
| --- | --- |
| Native HAL recorder, MCXA5xx and MCXA2xx | 77/77 each, warnings denied |
| Native plain-MMIO HAL, both families | 42/42 each, warnings denied |
| Public I2C doctests, both families | One positive and four negative examples pass |
| Portable interface / vocabulary / core | 3 / 11 / 31 pass, warnings denied |
| Portable core defmt tests compile | Pass, warnings denied |
| Generator tests / both PAC drift checks | 17/17 and both checks pass |
| ARM library matrix | Eight I2C/UART, family and defmt combinations pass |
| Mutation runner self-tests | 20/20 pass |
| Linked I2C/UART examples | Four I2C and two UART binaries build successfully |
| Firmware recorder-feature exclusion | I2C/UART pass; 12 checker tests pass |
| Strict named mutation campaign | 35/35: 27 compiler rejections and eight behavior checks; all positive baselines pass |
| Miri, I2C recorder and shared helpers | 68/68, no filters/ignored tests, 799.35 seconds |
| Miri, UART generated control adapter | Four selected tests pass, 0.72 seconds |
| Kani | All three harnesses verified, zero failures |
| Board tests | Follow-up passes: write, repeated-START write/read, NACK, cancellation, short/long RX stall, UART both directions, normal-pair restoration |

The status property test enumerates all seven status bits and `want_stop`:
256 combinations, with an independent fault-priority oracle. A separate test
keeps STOP completion independent of `bus_idle`. Neither test proves electrical
timing or physical DMA; Miri and bounded proofs also do not establish those.

Miri used the installed Rust 1.95.0 nightly with strict provenance, default
Stacked Borrows, leak checking, and the full frozen-clock reset bounds. Kani
0.67.0/CBMC 6.8.0 ran the three updated synchronous harnesses under WSL; its
successful log is `portable-kani-retry.log`. The earlier `portable-kani.log`
records a shell-launch quoting failure, not an executed proof or code failure.

The mutation campaign used a private test-only source fixture at
`9362c956b2b7b1979903431f0760fcce486f4fb2`, not a new commit on the user's
branch. Its 2,822-file inventory captures the tested source before the final
documentation cleanup. Subsequent edits are this results record and
platform-neutral wording in docs, a Rust doc comment and contract-report
metadata; transfer logic and generated access remain unchanged. The strict
matcher was rerun against every saved log;
expected diagnostic/path/span or test/assertion matches all pass. Reports,
mutation diffs and all twelve lockfiles per case remain under `mutations/`;
baseline and mutant locks match byte-for-byte. The 420 saved locks also match
the fixture after only CRLF-to-LF normalization, with no other content changes.
Disposable compilation caches and runner worktrees were removed, reclaiming
16.54 GB of rebuildable artifacts. See
`mutations/final-summary.json` and `mutations/final-ledger.md` for provenance and
per-case results. The worker handoff's deliberate existing-output refusal is
recorded separately, not counted as a mutation failure or a test execution.

The initial six linked firmware binaries are under
`C:/temp/i2c-representations-20260915/build/thumbv8m.main-none-eabihf/release`.
The connected-probe follow-up rebuilt into the runners' actual output directories
with `--locked` and warnings denied, rather than flashing older default-path
binaries. No old board result is used to qualify this refactor.

The validation runs used local source edits on the snapshot branch. A follow-up
commit records the tested changes and documentation cleanup at the user's
request; the original snapshot is not rewritten, and nothing is pushed.

## Connected-probe follow-up

On 2026-09-15, both FRDM-MCXA577 probes were reconnected. Controller/sender A
was `ZS1PGS20MCJBC`; target/receiver B was `0YHQFRBFS4FIG`. The second UART run
reversed those roles. All board processes exited zero; no failing case was
retried or accepted under a weaker verdict. At run time, the fresh source
inventory matched the mutation fixture except this results document.

| Hardware check | Observed result |
| --- | --- |
| UART A to B and B to A | Exact 16 bytes; no extras over 20 ms; DMA quiescent and transmission complete |
| I2C write | Exact 16 bytes and observed STOP on the peer |
| Combined write/read | Register 0x10, repeated START with no intermediate STOP, exact four received bytes, RX quiescent, final STOP |
| NACK | Exact `Error::Nack`; DMA inactive, requests off, MEN off, IRQ disabled |
| Cancellation | Future polled Pending with TDDE armed before dropping; same local shutdown assertions pass |
| Short RX stall | RX service withheld 5.129 ms; controller success in 6.888 ms, exact 16 bytes and STOP |
| Long RX stall | RX service withheld 150.274 ms; exact `Timeout` in 100.096 ms with local cleanup verified |
| Final restoration | Normal I2C pair freshly rebuilt, flashed, and passed again |

Short/long target readiness was 5,607/5,301 ms, below the unchanged 9,000-ms
budget. Both modes observed SCL through discrete GPIO samples; this is not a
continuous logic-analyzer capture. The long case explicitly aborts the target
after inspecting local controller cleanup: it does not prove automatic peer or
bus recovery. Cancellation coverage is the targeted TX case, not an exhaustive
race campaign or new RX-cancellation hardware qualification.

The orchestration, source/ELF hashes, build logs and result envelope are in
`C:/temp/i2c-representations-hil-20260915`. Detailed runner evidence under
`examples/mcxa5xx/target/`:

- UART: `uart-one-shot-hil/60059bb2c0d744409abf30219b29ef8a` and
  `uart-one-shot-hil/0c0bbcb38f614c25a2bd00dc62b271db`.
- I2C write: `i2c-one-shot-hil/4cf67a2ec71246859d3b7c2cab989d08`.
- Combined write/read: `i2c-one-shot-hil/a1aec8117ab14458bd1ab40861e86dee`.
- Short/long and restoration: `i2c-one-shot-rxstall/c024bb3c8016451d9e64004247f120b0`.
- Restored normal pair: `i2c-one-shot-hil/dc0260990f5c4867b0717f2444462b93`.
