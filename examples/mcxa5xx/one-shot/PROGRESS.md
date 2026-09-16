# Ownership and effect-testing checkpoints

Types constrain permitted operations. Effect tests check their sequencing.
Hardware tests challenge the assumptions underneath both. None replaces the
others.

Branch: `experiment/i2c-pac-safe-wrapper`. No push in this implementation run.
The [plan](IMPLEMENTATION-PLAN.md) remains the acceptance contract, not a claim
that every planned check has passed.

## Checkpoint 1 — committed as 77f190623

Retained pure-read shared handles, real-adapter MMIO/DMA/NVIC recording,
shared WaitCell/ISR bodies, firmware feature exclusion, and an isolated
compiler/behavior mutation runner. Production controller/target/eDMA and the
public sending application were not changed.

On this exact commit:

- All **9/9 mutations passed**, with successful positive controls, exact
  diagnostics or named assertions, and unchanged per-case lockfiles.
  Local evidence: `target/one-shot-mutations-77f190623/{summary.json,ledger.md}`.
- Both connected FRDM-MCXA577 boards passed the exact 16-byte/STOP transfer.
- NACK and cancellation checks passed: DMA inactive, requests off, MEN off,
  interrupt disabled. A final normal pair passed after those abort checks.
- Normal-pair logs:
  `examples/mcxa5xx/target/i2c-one-shot-hil/b05bb346c70243f5864b28c510fc4677`
  and `.../e58feeaf19584cd1ae004aa7a8526464`.
  Cleanup logs: `.../checkpoint-77f190623-51dfbf91c4c34200b5548bba30d9772d`.
- Host baseline: 26 HAL tests and 3 public doctests per chip; 51 DSL tests;
  fork 7 executable tests, 17 default/14 legacy doctests.
- ARM checks and linked normal/cleanup firmware passed with warnings denied.

The owner measured 24 bytes on this 64-bit host, versus 8 before retention:
three pointer words. Twelve ARM bytes is structural inference, not a measured
runtime allocation. No zero-overhead claim is made.

## Checkpoint 2 — committed as 5151ae792

Implemented after those baseline gates:

- A [protocol path crate](../../../embassy-mcxa/experimental/i2c-one-shot-core/README.md)
  without PAC, MMIO, HAL or timer dependencies. Six explicit moved owners
  replace the local transition macro. START queued, DMA quiescent and
  wire-complete are distinct promises.
- A declaration-only `unsafe trait Backend`, with root deny, forbidden logic
  modules and source/dependency policy tests. This is an open audited contract,
  not sealing. The policy rejects unreviewed source routes and lint lowering;
  actual compiler mutations complement it under recorded flags.
- A private controller/channel/static-source lease. Generated Streaming allows
  observation and wake rearming, but no MTDR, arbitrary MDER, or MCR reset.
  The concrete paired exit restores CPU authority only after requests-off,
  quiescence and Acquire. Pending retains the same lease.
- PAC decoding retains every co-latched fault from one MSR snapshot. The core
  keeps fault priority. STOP checks FIFO emptiness at the point of use.
- The one 100 ms deadline stays in the HAL around the complete sequence.
  Controlled-time tests exercise the actual wrapper, including all pending
  phases, preparation time, and the existing transfer-first deadline tie.
- The real hardware effect tests now exercise the extracted core and paired
  adapter. Separate mock tests exercise decisions; they do not simulate DMA
  internals or substitute for the effect recorder.
- CI gains the protocol tests/policy, runner controls and complete committed
  mutation manifest. The repository-wide CI entry point is not claimed run.

### Gate ledger

| Claim / check | Observed result |
| --- | --- |
| Protocol decisions (mock backend) | 5 tests pass, including 192 fault/STOP/idle combinations |
| Source/dependency policy | 9 tests pass; native/ARM, features off/all production graphs checked |
| Real HAL effects + deadline wrapper | 29 tests pass per MCXA2xx / MCXA5xx; warnings denied |
| Public API doctests, no custom-mmio | 3 pass per family |
| Optional defmt | Host tests type-check; full host execution has no embedded logger and fails to link; firmware link is a separate gate |
| ARM checks | Both chip families, defmt off/on, plus production-only MCXA5xx: 5 pass |
| Fork regression suites | 51 DSL; 7 executable + 17 default / 14 legacy doctests pass |
| Both PAC generators | Match: 9 one-shot / 17 production registers |
| Firmware exclusion | 12 checker controls pass; actual ARM graph has no custom-mmio |
| Mutation runner controls | 20 pass, including cap-lints survival detection |
| New driver mutation manifest | 16/16 pass on exact 5151ae792; 12 compiler rejections + 4 named behavioral failures |
| Linked MCXA5xx firmware | Normal controller, DMA target and cleanup checker pass with warnings denied |
| New hardware rerun | Exact data/STOP, NACK, armed-DMA cancellation and final normal pair all pass |
| New RXSTALL write characterization | Planned, not measured |

### Saved evidence for 5151ae792

Mutation results: `target/one-shot-mutations-5151ae792/{summary.json,ledger.md}`.
All positive runs succeeded; each mutation failed for its required reason,
not an unrelated build/link failure. Lockfiles stayed unchanged within each
case. Manifest SHA-256:
`857bab86a9eaf24fbb9a512b6446a8b88b4b77098136d21bbadf20445909b742`.

Two-board normal logs, each with both process exits zero:

- `examples/mcxa5xx/target/i2c-one-shot-hil/c5ac28770f474d50b6208f286cee19c2`
- `examples/mcxa5xx/target/i2c-one-shot-hil/52f89a87c3164a7d9ea3ad760c9e727a`
- NACK/cancel logs, each controller exit zero:
  `examples/mcxa5xx/target/i2c-one-shot-hil/checkpoint-5151ae792-38edf9ed941949689ddab1445709ee37`

Observed exact verdicts:

```text
ONE-SHOT CONTROLLER PASS: 16 bytes, DMA finished, STOP observed
ONE-SHOT TARGET PASS: exact 16 bytes and STOP
ONE-SHOT TWO-BOARD PASS
ONE-SHOT NACK: exact error observed
ONE-SHOT CLEANUP PASS: DMA inactive, requests off, MEN off, IRQ disabled
```

Controller probe `1fc9:0143:ZS1PGS20MCJBC`; target probe
`1fc9:0143:0YHQFRBFS4FIG`. Flashed ELF SHA-256 values, built with
`DEFMT_LOG=info`, `-Dwarnings` and the example's explicit linker flags:

```text
controller 1C38A2CE17696DEE7C6AB905B3690B91894D51373634421C53B5586541559C41
target     AF5E96FC0312F30CDBBB0C56DB69E2420B5A9C60D2B29BBDBA0FE25A24F5899E
nack       2261890727D7582DF7EABB924A3C6281C1D90AAE823C9267CB06A906459D8B9C
cancel     921D1FA9798A8136E4CDEFE0F1624D64782D068DEDE544C232E862F736A5ED73
```

Current normal controller ELF sections: text 39,960 bytes, data 260, bss 1,864.
No controlled before/after size comparison was performed. Source/diff formatting,
local documentation links and CI Bash syntax pass; the full CI job was not run.

### What remains trusted or assumed

- `Backend`, the Abacus generator's policy, retained-pointer acquisition and
  HAL DMA internals remain reviewed trusted code. An unsafe implementation can
  lie; private state fields cannot prove its silicon claims.
- The syntax gate trusts approved built-in/defmt derives and its own parser.
  Source checking alone does not constrain compiler flags. The real lint
  mutation is needed to notice a disabled rejection.
- Recorder evidence covers enumerated register/DMA/NVIC effect boundaries,
  not every machine-instruction interleaving, physical clock/pin construction,
  bus timing or active DMA memory-transaction progress.
- Drop covers ordinary errors and cooperative cancellation, not forgotten
  futures. Static resources keep a forgotten DMA source valid, not live.
- The fixed generated arming write has no firmware callback or await; an
  intentionally panicking custom-MMIO callback at that handover is not covered
  by a full-unwind engine-cleanup claim.
- Local shutdown is not bus-clear and cannot promise peer recovery. The
  deadline cannot preempt an unbounded synchronous HAL ACTIVE drain.

The production I2C controller, target and eDMA implementation remain intact.
The new target-fault characterization should land separately from this
architectural checkpoint so new experimental evidence is not confused with
regression evidence.

RXSTALL next-step prerequisites were inspected, not exercised: withhold receive
service after the target consumes its address event and before RX DMA arming;
use a short resumable control and a long timeout case, then explicitly abort
the target response. Target RDF/RDDE/ERQ/ACTIVE observations can corroborate
that service was withheld. No analyzer waveform was acquired; Timeout alone
or sparse GPIO samples cannot establish continuously held SCL past 100 ms.
Do not call this characterization complete or claim that controller shutdown
releases the peer. The two boards are left with the normal example firmware.

## Checkpoint 3 — committed as ebf59cce2

The [new probe](RXSTALL.md) adds two diagnostic firmware binaries and a bounded
PowerShell runner. Production code and the normal application are unchanged.
The runner has 81 passing host controls for exact verdicts, telemetry and
sampled-versus-unavailable electrical evidence. Both mode binaries
link with warnings denied, formatting passes and custom-mmio remains absent
from the firmware graph. The committed-revision board acceptance is below.

Exploratory runs before this checkpoint was committed are retained, not erased:

- `target/i2c-one-shot-rxstall/f134700d2e0449f685b5f1316eefb4b9` under
  `examples/mcxa5xx`: probe FAIL before target READY, because the initial probe
  assumed GPIO3 clock was enabled. The target released via abort and the runner
  restored a passing normal pair. No I2C driver fix was needed.
- `target/i2c-one-shot-rxstall/728d889625c54f88926863687e845ada`: after making
  the optional electrical observer explicitly unavailable, short and long
  service checks PASS and normal restoration PASS. Short: 6,880 us controller,
  5,128 us target hold / 6 samples. Long: exact Timeout at 100,062 us,
  150,273 us target hold / 149 samples. Both SCL observations unavailable.

Mandatory RDF/address/fault/RXSTALL/DMA, payload/STOP, timeout and cleanup
assertions were not weakened to obtain those passes. Only the optional SCL
observation is classified separately, and the final verdict names service
withholding rather than complete electrical characterization. No GPIO clock,
pin ownership, mux or input configuration is modified by the observer.

### Committed-revision acceptance

Run `examples/mcxa5xx/target/i2c-one-shot-rxstall/2126ccd92b164e6d8765af4219b12678`
records exact source commit `ebf59cce22257108a80e715e5aa2d1ae70fd2a9b`, with
scoped source inputs clean before/after both builds. Its `verdict.json` preserves
full repository status separately, including unrelated pre-existing untracked
files. Warnings-denied locked builds, both endpoint exits zero, exact markers,
and strict parsed telemetry all passed. No failed case was retried by the runner.

| Case | Controller | Target hold | RX samples | Outcome |
| --- | --- | --- | --- | --- |
| Short | Ok, 6,881 us | 5,128 us | 6 | Exact 16 bytes + STOP; local cleanup |
| Long | Exactly Timeout, 100,062 us | 150,273 us | 149 | Local cleanup before separate target abort |

SCL is explicitly `sampled=false`, `evidence=unavailable`, and
`continuous_waveform_qualified=false` in both mode records. The underlying
observation is CC3.GPIO3 reading its documented disabled setting; this is not
a broader clock-routing or waveform diagnosis. All mandatory status/DMA checks
remain active when this optional electrical observer is unavailable.

Flashed SHA-256 identities:

```text
short controller 1F1176194B54C42E9677B26FF190DCB472E505D13AACCC04FFDA6DF020D936F0
short target     541427552D2277FD72D7E0AB9F680601311879E588D68792053F1AF89AADF627
long controller  E4A4FE5F02A39A8DC0266A8CD43B0D6E03C63EBEB2E42069D9A844C53CA39CA8
long target      35A429CF5EF6A8EEC5EF4E2EBF800E6A96CD2B4D8688B0F8F1E79ABD81B63C35
```

Normal pair restoration passed with both exits zero in
`examples/mcxa5xx/target/i2c-one-shot-hil/e1b680770b014fa1bbe9eb262b010105`.
The restored ELF hashes match checkpoint 2's normal controller/target exactly.
Both boards are left running the normal example. No driver source or normal
application changes were needed. The 81 runner controls, formatting and firmware
feature-exclusion check passed; full repository CI was not rerun for this
diagnostic-only addition. At that checkpoint electrical characterization was
unresolved. The next experiment uses hardware thresholds rather than treating
another software timeout as continuous-SCL evidence.

## Checkpoint 4 — SCL-only watchdog experiment, recovery failure preserved

Implemented in the worktree based on `98d0d07cf`; the following runs record
dirty scoped source state and exact ELF hashes, **not** a clean-commit hardware
acceptance claim. [WATCHDOG.md](WATCHDOG.md) describes the dedicated full-driver
probe. Production controller/target/eDMA, one-shot ownership code, and normal
application files remain unchanged.

The shared observer now accepts GPIO3's existing ACC3 gate rather than rejecting
the block on CC3 alone. Both boards report PORT3 CC=true/ACC=false, GPIO3
CC=false/ACC=true, resets released. Mux2/IBE/PID checks and real input readback
succeed with no observer clock, reset, mux, input-gate or output writes.

### Measured cases

All run paths below are relative to `examples/mcxa5xx/target`.

| Case | Controller result / elapsed | Target hold | Whole-case result |
| --- | --- | --- | --- |
| Watchdog 120 ms | Exactly `PinLowTimeout`, 120,290 us | 150,224 us / 149 low samples | FAIL: SCL still low after target abort |
| Watchdog 140 ms | Exactly `PinLowTimeout`, 140,258 us | 150,224 us / 149 low samples | FAIL: same post-release low condition |
| Watchdog 200 ms | Ok, 152,043 us | 150,224 us / 149 low samples | PASS: exact 16 bytes, STOP, controller idle and SCL high |
| Original one-shot short | Ok, 6,885 us | 5,129 us / 6 low samples | PASS: exact data/STOP, high after release |
| Original one-shot long | Exactly `Timeout`, 100,062 us | 150,274 us / 149 low samples | PASS: original cleanup assertions, high after separate target abort |

120 ms failure: `i2c-one-shot-watchdog/6ffef3427ee343c6b116aae63252e92d`.
The separate remaining-cases run
`i2c-one-shot-watchdog/bcfeeb888d39457589b0e7bdc7b56646` explicitly selected
140 and 200, records `full_sweep=false`, and remains **FAIL** despite the passing
200 ms control. No failed case was retried by either invocation. Both preserved
the first failure and restored a passing normal pair.

Readbacks in all watchdog cases: nominal input 3 MHz, peripheral clock mux0
and divider0, PRESCALE=0, TIMECFG=0, FILTSCL=0; PINLOW=1407/1641/2344 for
120/140/200. The 500 ms software progress budget and 750 ms outer deadline did
not substitute for PLTF. These are nominal hardware-qualified threshold
observations, not calibrated time, complete waveforms or one-event brackets.

### Failure evidence, not a cleanup PASS

In both failing cases, after DMA requests/channels were quiescent:

| Snapshot | MCR | MSR | MFSR | SCL sample |
| --- | --- | --- | --- | --- |
| Immediately after `PinLowTimeout` return | `0x00000001` | `0x03002001` | 0 | Low |
| 100 ms later, no intervening repair | `0x00000001` | `0x03002401` | 0 | Low |

MCR says MEN=1/DBGEN=0. Initial MSR has TDF, PLTF, MBF and BBF; the later
snapshot additionally has NDF. Neither has SDF. The new NACK means controller
transaction state changed after the API returned its error, not merely a
delayed readback of the busy bit. Target abort cleared its FIFO and RX DMA, but
the line did not rise in its post-abort check either. In the successful 200 ms
case both controller snapshots are MCR=1, MSR=1, MFSR=0, SCL high.

Likely investigation area: full-driver `terminal_shutdown()` immediately
re-enables MEN after FIFO-reset read-modify-writes. RM p1902 describes MEN-off
as graceful close; the observed register-update latency also raises the
possibility of a following RMW re-copying stale MEN=1. These mechanisms are
**hypotheses**, not proven diagnoses. Target `abort_response()` has a similar
SEN-off/reset/on sequence, so physical ownership of the held line must not be
asserted solely from these samples. Production terminal shutdown and possibly
target abort sequencing need a separately authorized fix/investigation.

### Preserved exploratory failures

- `i2c-one-shot-watchdog/e3562de1ee4e42d49566b8eb98b560f1`: target did not reach
  READY inside the original 9 s host budget including flashing. No transfer
  ran. Only startup changed to 20 s quiet / 19 s conservative READY budget;
  the hold and watchdog thresholds were not relaxed.
- `i2c-one-shot-watchdog/1af64a867f0544faa04c1b2c306d15a8`: an immediate MEN-off
  readback assertion failed before TIMECFG was changed. The diagnostic now
  waits at most 1 ms for actual readback. Later observed setup waits were
  4–5 us disabling and 1–2 us enabling, not assumed CPU-cycle delays.
- `i2c-one-shot-watchdog/b5b3330a7e624cf6ae1d86ecc9ca5d25`: exact PLTF, followed
  by an overstrong immediate MBF-idle assertion. The manual does not promise
  immediate graceful completion against held SCL. That assertion was replaced
  with explicit raw-state observations, while post-release SCL and successful
  control-idle requirements remain mandatory. The target's later high sample
  in this run is confounded by the controller panic/debug halt with DBGEN=0;
  it is **not** evidence of successful recovery with running firmware.

Normal restoration passed after every invocation, including all failures.
Original one-shot short/long revalidation is
`i2c-one-shot-rxstall/06462168f9eb480d874fbc421d989850`, with sampled electrical
corroboration now present. Last normal restore is
`i2c-one-shot-hil/2efe35678c964807b4f564e819696a99` (both endpoint exits zero).
Both boards are left with the normal pair, whose ELF hashes still match
checkpoint 2. Hardware was not left in the failing watchdog state.

Static validation: 449 watchdog verdict/selection/aggregation controls plus
81 existing RXSTALL controls pass; all selected firmware builds link with
`-Dwarnings`; rustfmt/diff checks and firmware custom-mmio exclusion pass.
Full repository CI and unrelated driver host suites were not rerun for this
diagnostic-only change. No full fault-recovery PASS is claimed or inferred
from those static checks or the successful normal/200 ms cases.

## Checkpoint 5 — matched post-return repair diagnostics

Checkpoint 4 was first committed unchanged as `77a53f66d`. Production driver,
target, DMA and one-shot code remain unchanged in this checkpoint. The opt-in
`wait` and `reset` repairs run only after the original failing PLTF recovery.
They share full-word disable, observed MEN readback, and a return+100 ms disabled
checkpoint. Reset alone adds RST and verified restoration of eight master
configuration words. Both then require idle/high, re-enable and a fresh exact
`Recovered DMA!!\n` transaction/STOP; the target receives no extra reset.

| Repair | Threshold | MEN-off acknowledgment | Result |
| --- | --- | ---: | --- |
| Wait only | 120 ms | 30,201 us | PASS, fresh exact data/STOP |
| Wait only | 140 ms | 10,219 us | PASS, fresh exact data/STOP |
| Wait then RST | 120 ms | 30,178 us | PASS, fresh exact data/STOP |
| Wait then RST | 140 ms | 10,206 us | PASS, fresh exact data/STOP |

The acknowledgment took until the target's designated 150 ms release, not the
4–5 us seen during idle setup. This is strong evidence against using a short
graceful-disable spin as a production precondition during peer stretching.
Both repairs suffice under matched dwell; these post-return changes do not
isolate the original stale-RMW/premature-re-enable mechanism.

Original return snapshots remain MEN=1/MSR=0x03002001/FIFO=0/SCL low. Wait-only
disabled snapshots have MEN=0/MSR=9985/FIFO=0/SCL high; reset snapshots have
MEN=0/MSR=1/FIFO=0/SCL high. After re-enable, all have MEN=1/MSR=1/FIFO=0/high.
Each target held for 150,224 us with 149 low samples, then used the unchanged
explicit abort path. No hidden retry or target reinitialization qualified the
fresh transaction.

Evidence, relative to `examples/mcxa5xx/target`:

- Wait: `i2c-one-shot-watchdog/a1db8edcc6194874bd0f55581af6211a`.
- Reset: `i2c-one-shot-watchdog/c7ede66cf0f7421a8e3d54d2073d916b`.
- Normal restores passed: `i2c-one-shot-hil/f1e6dbc3cfaf4698b2af7345b7015623`
  and `i2c-one-shot-hil/b2215fb11eaf4f4d853e4280e92e7409`.

Both invocations record subsets, not a new full threshold sweep. Complete logs,
ELF hashes and source state are preserved. Static: 789 watchdog host controls,
81 RXSTALL controls, warnings-denied firmware builds, rustfmt and diff checks
pass. No production recovery qualification is inferred.

Next production-shape question: RM p1904 permits immediate abrupt RST after
PLTF, without waiting for graceful MEN-off. Test reset-before-acknowledgment and
immediate re-enable while the peer still holds, with configuration preserved,
before adopting that bounded local recovery path. A 300 ms diagnostic yield is
not an acceptable blanket cancellation/critical-section delay.

### Abrupt local-reset experiment — fresh-transfer failure preserved

`i2c-one-shot-watchdog/c0e9884ddcd948459dc387fda020ebb4` tests immediate
RST before any graceful MEN acknowledgment, confirms reset-held/released, then
restores configuration and re-enables while the peer still holds SCL. Both
120/140 cases return the original exact PLTF (120,290/140,258 us). Reset takes
11/12 us; MEN=1, MBF idle, FIFO empty, SCL low at `reset_complete`.

At return+100 ms, SCL is high but MSR remains `0x02000001`: **bus** busy, not
**controller** busy. The fresh transfer then returns `Timeout` after its 500 ms
progress budget; the target never sees its new address. Both cases FAIL. This
proves the local reset handshake is short here, not that immediate re-enable
recovers a usable bus. SDA has not yet been independently sampled in this arm;
physical SDA versus retained bus-idle history remains to be distinguished.

The failing experiment is not retried or marked green. Its runner restores the
normal pair successfully (`i2c-one-shot-hil/cd64f429ca6c498e82981628bed8baef`).
985 watchdog controls and 81 RXSTALL controls pass; linked firmware is warnings
denied. Production remains unchanged pending a complete recovery shape.

### Explicit bus-idle detector arm

`i2c-one-shot-watchdog/432e2afa018c4ce7baa2f351e9549230` adds only BUSIDLE=60
to the preserved abrupt-reset configuration (CLKLO=14, SETHOLD=14, CLKHI=12,
PRESCALE=0). Both 120/140 cases PASS: exact original PLTF, local reset in 12 us,
early MEN=1/MBF idle/SCL low, then MEN=1/MSR=1/FIFO=0 after release. A single
passive PDIR snapshot confirms both SDA and SCL high; BBF is clear. Both fresh
16-byte writes arrive exactly with STOP, without a retry or extra target reset.
The ordinary pair is restored successfully at
`i2c-one-shot-hil/21f94920c9d54893a397a9aeefb1131f`.

The earlier `d20cf3ba23114e368cfb051d81749b40` invocation failed at build time
on an incorrect PAC enum name (`Bbf`, corrected to `MsrBbf`), before either
board was touched. It is not a hardware attempt or a passing result.

This establishes a useful single-controller diagnostic policy, not permission
to enable a short BUSIDLE globally. A multi-controller peer may have a longer
high period; automatic idle inference must be opt-in. Preserve BUSIDLE in the
production local-reset fix and retain explicit `clear_bus()` for authorized
bus recovery. Reset confirmation and physical/logical bus usability remain
different promises. Static: 1,283 watchdog controls, 81 RXSTALL controls,
warnings-denied firmware builds, formatting and diff checks pass.

## Checkpoint 6 — confirmed local reset, production integration and regression pins

The production change is isolated in `f09e490d9`, after the independently
committed baseline (`77a53f66d`) and diagnostic arms (`f63e95a36`, `fa0c066a5`,
`48f5400e4`). The final validation record is a following documentation commit;
no production source changed during that final validation.

### What the measurements justify

Confirmed graceful MEN-off took 10–30 ms, until the peer's release, whereas
confirmed abrupt reset took 11–12 us. Consequently the shared production
primitive does **not** wait for graceful disable before asserting RST. It
writes full-word reset-held/disabled, confirms assertion, releases reset while
disabled, confirms release, and only then restores static configuration and
permits enable. Each handshake has both a 1 ms time bound and a 65,536-poll
ceiling. A failed handshake attempts to leave reset held and fails before
recovery evidence is resolved; it is not disguised as a successful cleanup.

The facade's `expect` is a deliberate fail-stop policy, including when terminal
recovery is reached from `Session::drop`. The tested `thumbv8m.main-none-eabihf`
firmware uses `panic="abort"` by target default, with no override in the
MCXA5xx example's Cargo profile or `.cargo/config.toml`, and uses `panic-probe`.
These binaries do not unwind, so this is not a second panic during stack
unwinding. Failed confirmation is treated as a
hardware/clock/configuration-contract failure, not a recoverable I2C protocol
error; it does not diagnose dead silicon. The final reset-held write is only
**attempted**, not confirmed on that failure path: software withholds recovery
evidence and stops, but cannot promise that unresponsive hardware accepted it.

The private non-Copy `Disabled` owner borrows the exact register adapter through
mode configuration and enable. Controller setup, PLTF termination, fallback
reset and bus-clear handoff use the shared sequence; target setup/abort use it
with their own configuration map. Mode bits are explicitly confirmed rather
than restoring an old mode at enable. IRQ/DMA requests are masked, but owned
DMA leases must already be quiescent: reset does not replace that protocol.
Ordinary FIFO/halt paths and the one-shot Abacus teardown are unchanged.

BUSIDLE is preserved, not silently set to the diagnostic's short value. The
API promises local-engine reset, not a generated STOP, peer release, BBF clear,
or automatic replay. Callers with bus ownership can use the existing explicit
`clear_bus()`; the full rig below checks fresh traffic after that operation.
The original stale-RMW mechanism is not claimed uniquely proven, and the
~0.226 ms threshold excess is not a FRO-tolerance measurement: controller
watchdog and OSTIMER share a FRO-derived source. Common oscillator error
cancels in this comparison; measuring absolute FRO accuracy needs an independent
calibrated time reference. The excess can include timer phase, detection and
software latency; these measurements do not isolate ISR-to-return latency or
scheduling alone. The peer board's independent clock still affects the
hold-versus-threshold margin, not the controller's absolute frequency estimate.

### Effect tests and mutation

Nine new tests run the actual reset sequence against scripted individual
register effects: delayed/stuck assertion and release, configuration destroyed
by reset, nonzero configuration restoration, mode changes, enable timeout,
drop while disabled and a stopped clock. The small concrete register maps are
reviewed, not electrically simulated by these tests.

The committed `omit-reset-release-confirmation` mutation at `f09e490d9` passed
acceptance: the exact positive test exited 0; the mutant compiled and exited
101 on `RESET_CONFIRMATION_BEFORE_DEPENDENT_EFFECT` in the named test. Baseline
and mutant lockfile hashes match; flags were `RUSTFLAGS=-Dwarnings` and
`CARGO_BUILD_TARGET=x86_64-pc-windows-msvc`. The existing CI manifest runner
includes this case. All 20 runner selftests also passed.

Preserved infrastructure failures are **not** mutation passes: the first
attempt under `embassy-mcxa/target/reset-mutation-f09e490d9` failed to link the
positive baseline (Windows LNK1104 at a deep build path); the short-path retry
`%TEMP%/reset-f09-f5214ac1` passed the positive but ran out of disk during mutant
compilation (OS 112). After free space became available, the unchanged third
attempt `%TEMP%/reset-f09-28b81c4f` passed. No test/matching rule was weakened.
Its reports, manifest, diff and raw logs are also copied, without build output,
to the full-rig evidence directory below under `reset-mutation-pass/`.

### Hardware results

Paths below are relative to `examples/mcxa5xx/target`. No timing thresholds,
cancellation assertions, target hold durations or RXSTALL acceptance windows
were relaxed for the production change.

The default complete watchdog sweep
`i2c-one-shot-watchdog/253d6b8c01884367a9d39b42a488717d` passes with **no
diagnostic repair**. It records dirty production inputs based on `48f5400e4`,
subsequently committed in `f09e490d9`; it is not a clean-ref run. Only comments
and documentation changed between that firmware build and the commit.

| Threshold | Result / elapsed | Local state and peer-release observation |
| --- | --- | --- |
| 120 ms | Exactly PinLowTimeout, 120,307 us | MBF idle at return and after wait; SCL low during hold, high after release |
| 140 ms | Exactly PinLowTimeout, 140,275 us | Same strengthened recovery checks |
| 200 ms | Ok, 152,036 us | Exact 16 bytes and STOP, MSR=1, SCL high |

All target holds are 150,224 us / 149 low samples. Fault snapshots have MCR=1,
MSR=`0x02000001`, MFSR=0: BBF remains set, **MBF does not**. The verdict
truthfully sets local-engine qualification true and full terminal/bus recovery
false. Normal restore passes at
`i2c-one-shot-hil/1f961218b40047fe8b47ea4540d005ae`.

Full-rig evidence: `reset-recovery-hil/3c3b82ed9b9f4cc9bbd839c4f6889e25`.
The first builds explicitly selected all four defined phases; the firmware
correctly labels any explicit filter rather than claiming an unfiltered run.

| Target | Async | DMA | Blocking | Pin-low / explicit bus-clear |
| --- | --- | --- | --- | --- |
| Interrupt | ALL PASS | ALL PASS | ALL PASS | PASS; fresh traffic recovered |
| DMA | ALL PASS before probe loss | ALL PASS in tail session | ALL PASS in tail session | PASS in tail session; fresh traffic recovered |

`interrupt.log` contains all four completed phases and runner exit 0. Async
and DMA each cancel 120/120 races with zero faulted. Both queued-suffix
address-NACK/halt-recovery cases pass; all three settle audits report zero.

`dma.log` records completed async (111 cancelled, 9 completed, zero faulted),
then a **probe access-port FAULT** during the DMA soak: overall runner exit 1,
not a successful full session. That failure is preserved. The supported phase
split rebuilt only the controller for `dma,blocking,pin_low`; `dma-tail.log`
ends with all those phases passed and runner exit 0. DMA cancellation reports
111 cancelled, 9 completed, zero faulted; its queued-suffix address-NACK passes.
The split does not erase the first failed session or claim uninterrupted-run
qualification. No firmware panic/assertion was reported by the lost session.

Recorded full-rig ELF SHA-256 identities:

```text
all-four controller 14E368DE6F30237DDBDAC7D51FB1895C5E94B6DDC8160CF69DEB176D73287AB8
interrupt target   60276EB18A72CACD7E6D8928A2D83006956A8EC682118F54DC18343EF75195E2
DMA target         4317F10331A9CFF7B96DD43EA57E1FD7934E0E0A8324D4F6E453DF03A7E660FD
tail controller    E6E2B9938823492F247E6A8F5281552FEAB50259CD4E5B768226F4BE4D9081B3
```

The unchanged original one-shot RXSTALL probe passes at scoped-clean
`f09e490d9`: `i2c-one-shot-rxstall/b67a7e38926b4960bfb5bde8ab0f09ad`.
Short: Ok in 6,886 us, target hold 5,129 us / 6 low samples, exact 16 bytes and
STOP. Long: exactly Timeout in 100,062 us, target hold 150,274 us / 149 low
samples, original DMA/local-cleanup assertions and SCL high after separate
target abort. Both endpoint exits are zero in each case. The normal pair is
restored with exact data/STOP and both exits zero at
`i2c-one-shot-hil/b0ca57ef7e7141d7892676ea2bdeba31`; this is the final board state.

### Size at checkpoint 6 — 2026-09-13 (historical)

Measured at this checkpoint with one script: *total* counts every line;
*code* excludes blank lines and lines beginning with `//`. The three adapter
test files are gated `#[cfg(all(test, feature = "one-shot-host-tests"))]` and
never reach firmware. `mod.rs` and `registers.rs` still carry small inline
`#[cfg(test)]` modules inside the hand-written figures. The earlier sizes in
VALIDATION.md are historical measurements of earlier layouts.

| What | Total | Code |
| --- | ---: | ---: |
| Application (sending) | 33 | 27 |
| Test peer (receiving) | 37 | 32 |
| Protocol core crate `src/` (no PAC/HAL/timer dependency) | 234 | 160 |
| HAL adapter, hand-written (`mod`, `session`, `vocabulary`, `registers`, `effects`, `dma_boundary`) | 773 | 563 |
| Generated register map (`mmio.rs`) | 126 | 98 |
| **Driver compiled into firmware** | **1133** | **821** |
| Host tests — protocol core `tests/` | 1047 | 969 |
| Host tests — adapter (`effect_tests`, `deadline_tests`, `test_support`) | 1055 | 945 |
| Shared reset sequence (production, both roles; not the example) | 524 | 445 |
| Local Abacus fork including its tests | 3440 | 2762 |

Against the original 567-line, four-file driver (432 code), the hand-written
driver surface is now 1007 lines (723 code): the protocol crate extraction,
the paired lease and `Streaming` state, and the two effect-seam files that let
the real adapter run under the recorder. Host tests grew from none to
2102 lines; one of them is the test that catches the removed reset
confirmation. `reset_sequence.rs` is listed because the example's characterization
produced it, but it is production code shared by the controller and target.

### Static acceptance and remaining limits

- HAL host tests: 17/17 on each MCXA family (including the nine reset tests).
- One-shot host tests: 38/38 on each family.
- Warnings-denied ARM library checks: MCXA2xx/MCXA5xx, each with and without
  defmt; full-rig and diagnostic firmware builds linked warnings-denied.
- Generated maps match: 17 full-driver registers and 9 one-shot registers.
- Firmware custom-mmio exclusion passes for the example and both HAL graphs;
  its 12 negative/positive checker controls pass.
- Watchdog verdict controls 1,301/1,301; unchanged RXSTALL controls 81/81.
- Rustfmt/diff checks pass (stable rustfmt only warns about nightly options).

Full repository CI was not run. These observations do not qualify arbitrary
clock rates, stopped peripheral clocks, electrical waveforms, externally held
SDA, multi-controller bus-idle policy, or a peer that never releases SCL.
Existing unrelated untracked C examples and root build directories are left
untouched; no push was performed.

## Checkpoint 7 — narrower unsafe obligations and confirmed one-shot control

Implemented and hardware-validated as working changes on `9439aecd0`, following
the book comparison; the source/tests/CI changes are now committed separately
as `e2167e972`. This following documentation/manifest commit records the evidence
without upgrading it to a clean-ref hardware run. No push was performed. The
public API, 33-line sender, transaction sequence, thresholds and receiver are
unchanged.

### What is enforced, and where

- `Info` no longer manually promises `Sync`. Its fields must satisfy the
  compiler's structural auto-trait rule, pinned by a compile-time assertion.
- Full-driver session, bus-clear and reset sequencing now forbid explicit
  unsafe code. This is NOT a PAC-free guarantee: safe PAC methods remain
  reachable inside the HAL. Broader full-driver extraction remains future work.
- The wake-only ISR and its inventoried local call/import/module path have a
  fail-closed AST policy with source-mutation controls. Added safe PAC writes,
  FIFO pops, helper/clock calls and redirected sources are rejected. A separate
  compile-only fixture demonstrates that the same PAC operations compile under
  `forbid(unsafe_code)`. This is CI policy, not compiler sealing of all HAL code;
  trusted dependencies/build machinery remain outside that policy's scope.
- `TxEndpoint<T>` acquisition consumes the matching singleton and derives its
  mapping and TX request together. A byte-typed, non-Copy opaque endpoint moves
  with the controller/channel lease. The instance brand is explicitly erased
  for the non-generic driver; it is not claimed to persist afterward.
- The generated map has no MCR setter or write-only enable/reset transitions.
  `Unconfirmed` grants no configuration authority. A concrete safe-mmio adapter
  runs the shared production reset sequence: Off follows observed assertion and
  release; Enabled follows actual MEN readback. DMA quiescence precedes reset.
  This avoids a peer-dependent graceful-disable wait during cancellation.

Each handshake retains the 1 ms / 65,536-poll bounds. Failure attempts to hold
reset, then fails before returning authority or handing off pins; it does not
prove unresponsive hardware accepted that last write. Firmware's fail-stop
policy matches checkpoint 6. No bus-idle, peer-release, or automatic-replay
guarantee was added. The shared reset algorithm remains unsafe-free; only its
new concrete adapter performs MMIO.

### Validation

- Warnings-denied host tests: **45/45 on each MCXA family**. Seven additions
  cover endpoint construction/movement, delayed reset and enable acknowledgment,
  failed enable, and failed reset including a stopped host clock. Existing
  cancellation, wake-boundary, quiescence, deadline and fault tests still pass.
- Protocol crate: **9 policy + 5 protocol + 5 wake-policy tests** pass. The
  compile-only PAC fixture also passes on ARM and is now checked in `ci.sh`.
- Warnings-denied ARM library checks pass for MCXA2xx/MCXA5xx, defmt on/off,
  with one-shot enabled and default features disabled. Firmware builds retain
  the example's flip-link/link.x/defmt.x/--nmagic flags as well as `-Dwarnings`.
- Both generated maps match (9 one-shot / 17 full-driver registers); firmware
  custom-mmio exclusion and its 12 controls pass. Mutation-runner selftests
  pass 20/20. Formatting and diff checks pass (stable rustfmt warns about
  nightly-only formatting options).
- Six new manifest cases cover the three unsafe prohibitions, ISR PAC bypass,
  unobserved reset release through the one-shot adapter, and cross-instance
  endpoint typing. Existing Streaming controls target the new confirmed-control
  methods. All 23 anchors are unique. **These new isolated source-mutation runs
  have not yet executed**; their declarations are not recorded PASS. The user
  subsequently authorized the source and documentation commits needed for the
  baseline, but disk space remains insufficient for fresh isolated builds.
  The runner's dirty-input refusal was not bypassed. Current-source policy
  mutations and effect tests above did execute; these are separate evidence claims.

### Connected-board evidence

Controller ZS1PGS20MCJBC; target 0YHQFRBFS4FIG; existing wiring and 4 MHz SWD.
Paths below are relative to `examples/mcxa5xx/target`.

- Normal exact 16 bytes + STOP, both exits zero:
  `i2c-one-shot-hil/0e19b1bcbf9b45cfb7ce7f01fad81b15`.
- Exact NACK and a polled/armed cancellation both pass local cleanup:
  `i2c-one-shot-safety/3bd861c508a14a5ab23c56c0dc4248dd/{nack,cancel}.log`.
  ERQ/ACTIVE/TDDE/MEN are off and the IRQ is disabled; this does not release
  a peer stranded mid-transaction.
- Short/long RXSTALL run:
  `i2c-one-shot-rxstall/aaf77abe86dd44dd8ced22e45a3dde01`.
  Short succeeds in 6,892 us, exact bytes/STOP after 5,129 us service withholding.
  Long returns exactly Timeout in 100,060 us while the target withholds service
  for 150,274 us. Local shutdown passes; target release remains a separate abort.
- The runner restores and re-verifies the normal pair, both exits zero:
  `i2c-one-shot-hil/5aed8a9c7dec4e548b7f7c6d607414c1`.
  This is the final board state.

Final normal firmware SHA-256:

```text
controller 55A970F8DB84D9FD990C3143D733FB52ADBF0F1D071E71336344D533FCD736CD
target     6B7C180931D9DD8B5A20B737C123945D9DAED5C548F422A2587AD246BEA3A770
```

The initial normal invocation, `i2c-one-shot-hil/1984826a48a44feba460747548bad5cc`,
failed before startup with missing `.defmt`: an overriding RUSTFLAGS value had
omitted the example linker flags. That failed launch is preserved, not counted
as a firmware PASS. The corrected build restored the configured linker flags.
Disk space was low; attempted cleanup of this turn's temporary build directory
was blocked, and no directory was deleted. Unrelated user files remain intact.
Full repository CI, the full multi-phase production rig, and new isolated
committed-baseline mutations were not rerun in this checkpoint.

### Follow-up: committed mutation validation after disk cleanup

The eight pending/updated cases ran on committed baseline `991eb9a21`: the
three full-driver unsafe prohibitions, wake-only ISR PAC bypass, one-shot reset
release observation, cross-instance endpoint, and Streaming reset/enable.
All eight positive controls exited 0; all eight mutants failed for their
specified diagnostic and exact span or named test/assertion. Lockfile hashes
were unchanged and all runner-owned worktrees were removed. The recorded
environment contains no RUSTFLAGS override; this run is not labelled
warnings-denied. Reports, raw logs, diffs and lock snapshots are preserved at
`target/one-shot-mutations-991eb9a21/` relative to `examples/mcxa5xx`.
This closes the earlier pending eight cases on that revision, not on later
uncommitted source. Other mutation cases were not rerun in this follow-up.

## Checkpoint 8 — capsule-style prohibition across the protocol library

Initial working-change validation on `991eb9a21`, requested after reviewing the
book's Tock capsule/HIL distinction. At that point no commit, push or hardware
run had been performed for this split; the committed follow-up is below.
The production I2C driver, the one-shot HAL adapter, transaction
algorithm and sending/receiving applications have no source changes.

### Boundary and scope

- The complete `i2c-one-shot-core` library now uses
  `#![forbid(unsafe_code)]`, without a declaration exception. This is the
  book's capsule source rule, not a claim that the example is a Tock capsule
  or has Tock's kernel/process isolation.
- The new sibling `i2c-one-shot-interface` package owns the declaration-only
  `unsafe trait Backend` and shared value types. It is the trusted contract,
  not another capsule. Its root denies unsafe except for that reviewed trait;
  its types module forbids unsafe. It performs no MMIO and contains no
  transaction algorithm. Its only external normal dependency is optional
  defmt formatting.
- The core re-exports the previous public names. The shared type definitions
  match their former source exactly apart from file placement/line endings;
  the trait's operations and safety obligations are unchanged. Moving these
  shared types alongside the contract avoids a dependency cycle.
- The HAL still supplies the audited unsafe implementation; test mocks are
  separate integration-test crates or HAL test code, not an allowance in the
  protocol library. The interface is open, not sealed. Trusted dependencies
  may contain unsafe; a source lint does not establish hardware correctness.
- Four responsibilities remain. Separating the contract into another package
  strengthens their boundary without adding a fifth runtime layer.

### Validation of the working changes

Rust 1.92; all Rust build/test invocations below used `RUSTFLAGS=-Dwarnings`.
The linked example build additionally retained its flip-link, link.x, defmt.x
and --nmagic flags. Validation build output was kept outside the checkout
under `C:/temp/i2c-capsule-validation` and `C:/temp/i2c-capsule-firmware`.

- Core: 11 source/dependency policy tests, 5 protocol-decision tests and
  5 wake-path policy tests pass. The 11 policy tests also pass with all
  features; the core's defmt-enabled test targets type-check.
- The policy reads both source inventories and manifests, checks host and
  ARM normal/build dependency graphs, requires unconditional core-level
  forbid, and rejects root unsafe declarations/implementations/blocks and
  allow/expect/conditional suppressions. The interface's sole allowance
  cannot acquire default bodies or other unsafe syntax. These controls are
  AST-policy tests, not newly executed compiler mutations.
- Both MCXA families: 45/45 HAL host tests pass. Public one-shot doctests
  pass for each family (one positive compile example, two compile-fail cases).
- ARM library checks pass for MCXA2xx/MCXA5xx, defmt on/off, with one-shot
  enabled and defaults disabled. The MCXA5xx one-shot controller and target
  firmware binaries both link. They were not flashed or run in this checkpoint.
- The new interface package checks independently, default and defmt enabled.
  CI now includes those first-party checks as well as the core's checks.
- Generated maps still match the PAC (9 one-shot / 17 full-driver registers).
  Firmware custom-mmio exclusion passes for both HAL family graphs and the
  example graph; its 12 checker controls pass. Mutation-runner selftests pass
  20/20. Rustfmt/diff checks pass, with the existing stable-rustfmt warnings
  for nightly-only formatting settings.

The mutation inventory now includes the interface dependency and a new
`forbidden-core-root-unsafe` case, explicitly targeting the root rather than
the previously forbidden protocol module. All 24 source anchors match once.
The new case and the full manifest still need a fresh committed-baseline run;
the runner's dirty-input refusal is not bypassed. The previous eight PASS
records remain evidence only for `991eb9a21`. Full repository CI and new
connected-board tests were not run. No new silicon or timing claim follows
from this packaging change.

### Follow-up: local commits and post-split validation

The user authorized local commits, without pushing. Source commit `3007e9a79`
contains the boundary split; `bb024eaf7` records its initial validation. The
following runs use exact baseline
`bb024eaf7a65725ef28e20ea2d9b5a40b2f0c5a2`. An independent diff review confirms
that `protocol.rs`, the one-shot adapter, production I2C/DMA and the example
applications remain unchanged from `991eb9a21`. The shared types are identical
moves, and the backend contract has no new methods or safety obligations.

Supported host runs again pass with `RUSTFLAGS=-Dwarnings`: core 11 policy +
5 protocol + 5 wake-policy tests, and 45 HAL tests on each MCXA family.
The RXSTALL verdict checker also passes all 81 controls. An additional full
core `cargo test --all-features` attempt failed to link the host defmt provider
symbols. That unsupported runtime configuration is not recorded PASS; the
supported defmt checks remain the compile-only test-target check and the
policy-only runtime suite recorded above. No firmware dependency or provider
was added to make that exploratory command link.

The isolated compiler mutation `forbidden-core-root-unsafe` now passes on this
committed baseline: the unmodified library compiles, and the injected root
unsafe block fails with `unsafe_code` at the expected primary source span.
This is compiler evidence in addition to the AST source-policy controls.
The full manifest also passes **24/24** on this baseline with
`RUSTFLAGS=-Dwarnings` and `CARGO_BUILD_TARGET=x86_64-pc-windows-msvc`, with
no encoded-flags override. All 24 positive controls exit zero; all 24 mutants
are rejected for their specified diagnostic and primary span or named behavior
test/assertion. There are no timeouts or lockfile hash changes, and all
runner-owned worktrees are removed. Reports, raw logs, mutation diffs and
baseline lock snapshots are preserved at
`target/one-shot-mutations-bb024eaf7/` relative to `examples/mcxa5xx`.
This is the complete current manifest, not full repository CI or a proof that
the tests detect every possible regression.

#### Connected-board outcomes

Controller ZS1PGS20MCJBC; target 0YHQFRBFS4FIG; existing wiring, MCXA577,
4 MHz SWD. All release builds use `--locked`, `-Dwarnings` and the example's
flip-link/link.x/defmt.x/--nmagic flags. Paths below are relative to
`examples/mcxa5xx/target`.

- Normal exact 16 bytes + STOP, both endpoints exit zero:
  `i2c-one-shot-hil/8ad3c6de0b604902bb73095af4dceac6`.
- Exact NACK and cancellation of a polled, DMA-armed future both pass:
  `i2c-one-shot-safety/1e8ed8ac84f94af8bed4fc0b8854041a/verdict.json`.
  Both controller processes exit zero after asserting ERQ/ACTIVE/TDDE/MEN
  off and I2C IRQ disabled. The peer is reset between cases. These are local
  cleanup assertions, not a claim of releasing a stranded peer.
- RXSTALL attempt
  `i2c-one-shot-rxstall/02608257833a4c40aa07899699424252/verdict.json`
  **fails startup**, not the transfer assertions: no target READY marker by
  the conservative 9,000 ms startup limit (checked at 9,049 ms). Both children
  are terminated by the runner. Short/long transfer results are unavailable.
  Its unconditional normal restore passes, both exits zero:
  `i2c-one-shot-hil/474bfd9f80eb40838399b1ce28d88d1f`.
- One fresh RXSTALL attempt, with unchanged source, limits and assertions,
  is preserved separately at
  `i2c-one-shot-rxstall/1a2447bab0a54142826a8d2b71112cc8/verdict.json`.
  It also fails target startup readiness (9,056 ms). No RXSTALL PASS is
  claimed and no further retries were made. Its normal restore passes:
  `i2c-one-shot-hil/8c0ed79013cb4004a67b23f9b911e98d`.
  The ordinary controller/target example is the final flashed board state.

Final normal firmware SHA-256 (also in both RXSTALL restore verdicts):

```text
controller BC3CA206E2AAB9FE7B3B03B49D6F424018B31506C59E5BEBD659EB23F4CA8462
target     430575B5FD76E8777F2A067A142C611AC9A236D9623B8C3DC00F08FDF1CE08D6
```

The earlier normal launch
`i2c-one-shot-hil/33211860ec2f421a89eb435c7583bf89` failed before firmware
startup with a missing `.defmt` section. In this PowerShell environment,
setting `CARGO_ENCODED_RUSTFLAGS` to null through .NET left an empty variable
present: Cargo used that instead of `RUSTFLAGS`, and the build fingerprint
showed `rustflags: []`. Explicit `Remove-Item Env:CARGO_ENCODED_RUSTFLAGS`
removed the override before rebuilding with the required linker flags. The
failed launch remains preserved and is not a firmware verdict.

The recorded repository status includes the user's untracked C SDK example
and `one-shot/prune-build-output.ps1`. The latter also matches the runner's
broad relevant-path inventory, so `relevant_dirty` is honestly true; neither
untracked item is a Rust input to these builds. Tracked inputs were clean.
No board test result or isolated mutation is attributed to a different source
revision. No push, full repository CI, full production multi-phase rig, new
watchdog qualification, or post-split short/long RXSTALL success is claimed.

## Checkpoint 9 — separate programming from the RXSTALL startup window

The user requested that the unqualified short/long checks be made to pass.
This is a runner-only correction on `4e071ea7f`: no driver, firmware, transfer
assertion or timing acceptance window changed. The two previous startup
failures remain failures in their original records.

### Diagnosis and correction

The old startup clock charged both complete flash operations against the
conservative 9 s budget. A separate observation run with unchanged short
firmware saw controller quiet-window output at 4,264 ms and target READY at
8,391 ms from host launch. Both firmware processes then exited zero; this was
diagnostic evidence, not a replacement runner verdict. Logs and host event
times: `target/i2c-one-shot-rxstall-startup/88519aa39dee4b9480677f3a5d72776c`.
The earlier diagnostic helper invocation
`target/i2c-one-shot-rxstall-startup/456db9c833d148539e222d3118ba1a79`
stopped on an empty-log/null handling error before gathering startup evidence;
it restored the normal pair and is not counted as a test result.

`Prepare-Board` now downloads and verifies both selected ELFs before starting
the startup clock. Preparation has its own bound and recorded result/time.
After preparation, both boards are reset and the normal controller-first
launch sequence runs. `run --preverify --verify` still checks the same ELF and
starts each board freshly; preparation does not stand in for startup. Any
preparation failure aborts the case and still triggers normal-pair restoration.

The 9,000 ms readiness gate still starts **before controller launch**. The
controller's 10 s quiet window, target's bounded listen, short/long hold times,
exact-result/data/STOP assertions, timeout range and cleanup checks are
unchanged. There is no hidden retry or extra time granted to a transfer.

### Hardware qualification

Full corrected runner exits zero, both modes and restoration PASS:
`target/i2c-one-shot-rxstall/44d3040ea14447e280b1d91faf81d980/verdict.json`.
All paths in this checkpoint are relative to `examples/mcxa5xx`.

| Mode | Programming before startup clock | Target READY within 9,000 ms | Controller result/time | Measured RX service hold |
| --- | --- | --- | --- | --- |
| Short | 8,057 ms | 4,430 ms | `Ok(())`, 6,890 us; exact 16 bytes + STOP | 5,129 us, 6 samples |
| Long | 8,620 ms | 4,440 ms | Exactly `Err(Timeout)`, 100,059 us; local cleanup | 150,274 us, 149 samples |

Every controller/target process exits zero with its mode-specific verdict.
Both modes observe idle-high / low-hold / released-high SCL samples. The long
target releases itself through its separate explicit abort; controller bus-clear
or continuous electrical-waveform qualification is not inferred.

The runner rebuilds/restores the normal pair and verifies exact data + STOP:
`target/i2c-one-shot-hil/2bfec77cb5304b8da95e79e4448bf869` (both exits zero).
This is the final flashed board state. Its ELF hashes match checkpoint 8's
final normal pair. The short ELFs also match the earlier failed-startup run,
confirming that the hardware fix required no firmware change.

Builds remain locked and warnings-denied with the required linker flags.
The verdict preserves `4e071ea7f` plus the working runner modification and the
pre-existing untracked cleanup script; this is not attributed to a clean
committed runner. The original **81/81 verdict controls** pass unchanged.
The new **19/19 startup controls** exercise the actual extracted runner
functions with fake probes: exact download/run arguments, both preparations
before reset/start, failed preparation preventing all starts, and missing
READY rejected by the real marker/startup helpers. A separate AST ordering
check pins both preparation calls before creation of the startup stopwatch.
These are host orchestration checks, not simulated hardware qualification.
The previous host suites and 24 mutation results remain attached to their
recorded code baseline; they were not rerun for this orchestration-only edit.
No full-repository-CI or new watchdog qualification is claimed.

## Checkpoint 10 — unsafe-free vocabulary over owned physical capabilities

**Types constrain permitted operations. Effect tests check their sequencing.
Hardware tests challenge the assumptions underneath both. None replaces the others.**

The user requested that architecture-note layer 3, not only layer 4, forbid
unsafe, and authorized local commits without pushing. `b736836d0` commits the
passing runner-only checkpoint 9 fix. Source commit `4efa39060` implements this
new boundary for the experimental one-shot controller; the production I2C
driver is not migrated.

### What moved, and what did not

Use one numbering scheme from here: layer 1 is the PAC's chip definitions;
layer 2 is trusted MMIO/resource capabilities; layer 3 is named hardware
operations; layer 4 is the typed transaction and public service. Crates and
legacy file nesting are implementation packaging, not additional layers.

- Layer 3 now lives in `i2c-one-shot-vocabulary`, whose entire library has
  `#![forbid(unsafe_code)]`. It composes START, fresh FIFO-empty/STOP decisions,
  observations and polling using only opaque `Cpu` and `InFlight` capabilities.
- Layer 4's `i2c-one-shot-core` retains whole-library `forbid(unsafe_code)`.
  Its sequence, fault-priority decisions and public ownership states do not
  change. The single 100 ms deadline stays in the HAL wrapper.
- Layer 2's interface defines the declaration-only open `unsafe trait Device`
  and safe owned wrappers. The audited HAL `DeviceHardware` implements it.
  Singleton/pin/clock/interrupt acquisition, destination/request pairing, DMA
  preparation and arming, request-off/quiescence/fencing, reset and release
  remain on that trusted side. No raw device, channel or inner lease accessor
  exists on the capability handed to layer 3.
- Normal dependency direction is core -> vocabulary -> interface, with only
  optional error formatting outside these packages. Neither upper library
  has PAC, MMIO, HAL or timer access through its normal dependencies. The
  bounded policy inventories all three libraries, lint/expansion sources,
  contracts and host/ARM feature graphs; it is not a general security sandbox.

The safe `Backend` trait now means **protocol service**, not permission to
perform unsafe work. An arbitrary safe mock can report false protocol facts;
generic `TxDmaQuiescent`/`WireComplete` wording explicitly records that limit.
No unsafe release or raw CPU authority restoration relies on those generic
states. In the actual implementation, only the lower physical lease can
return the same controller/channel pair after quiescence. Wrong layer-3
sequencing can still misoperate the bus, but must not invalidate DMA memory.
The immutable static source remains valid even if the complete future is
forgotten; that does not promise cleanup or forward progress for a leak.

This is not merely relabeling individual unsafe calls as safe helpers. The
lower operation consumes the whole owner, performs setup/arming synchronously,
and returns one indivisible in-flight owner. Upper code cannot reorder its
internal DMA steps or issue a CPU command through that in-flight capability.
Register and HAL effect bodies retain their prior ordering. Physical truth
still depends on the audited unsafe Device implementation, DMA HAL and silicon.

### Pre-commit source/build validation

Warnings-denied, locked checks pass:

- Core: 21 tests (11 policy, 5 protocol, 5 wake-policy); all-features policy
  11/11; defmt-enabled test targets compile. Unsupported host-defmt runtime
  linking is not silently converted into a passing configuration.
- New vocabulary: 7 tests cover START readiness, fresh STOP FIFO checks,
  preservation of co-latched status, pending/completion ownership, lower-owner
  drop, DMA errors, and positive capability examples. Its fake Device is an
  explicit unsafe implementation in an integration test, outside the forbidden
  library. These tests do not establish actual DMA effect ordering.
- HAL: 48/48 tests on each family. Three new tests run the real adapter with
  recorded lower effects while bypassing the layer-4 sequence: arm before
  START then drop; injected preparation error before START; and complete then
  re-arm before STOP. They require no request enable on preparation failure,
  and quiescence before reset/release or a second preparation. The fixture
  does not simulate HAL-internal memory transactions or electrical behavior.
- Four ARM HAL combinations (MCXA2xx/MCXA5xx, defmt off/on), and 3 public
  one-shot doctests on each family. The normal controller/target release
  binaries link with the required flip-link/defmt flags and `-Dwarnings`.
- Firmware custom-mmio exclusion passes with its 12 checker controls.
  RXSTALL runner controls pass 19/19 startup and 81/81 verdict checks.
  Rustfmt and diff checks pass; stable rustfmt still reports the repository's
  existing nightly-only formatting option warnings.

The manifest now has 28 cases, adding unsafe-at-vocabulary-root rejection,
private raw-device and raw-lease extraction rejection, and no CPU command on
`InFlight`. Existing behavior mutations remain unchanged. Fresh committed-
baseline mutation and connected-board results are recorded below when run;
the previous 24 mutation passes and checkpoint 9 board results are historical,
not new evidence for this extraction. No push or full-repository-CI run.

### Connected-board results for the new boundary

All code inputs are committed at
`4efa390607fcfe49d6a2dc6f968b34d257c54139`. Documentation was being rewritten
during these runs, so the records correctly report a dirty checkout (including
a transient README replacement), plus the user's existing untracked SDK
example and cleanup script. None is a firmware source modification. These
results are not attributed to a clean later documentation commit.

Controller ZS1PGS20MCJBC and target 0YHQFRBFS4FIG use the existing MCXA577
wiring and 4 MHz SWD. Release builds use `--locked`, `-Dwarnings` and the
required flip-link/link.x/defmt.x/--nmagic flags. All paths below are relative
to `examples/mcxa5xx/target`.

- Normal transfer: exact 16 bytes and STOP, both endpoints exit zero;
  `i2c-one-shot-hil/919cb53ee277440ab4088a7cd3593b3a`.
- NACK and armed-future cancellation: both pass, each process exits zero;
  `i2c-one-shot-safety/3fb4bb0b76a04ad389194f1bd9df46bf/verdict.json`.
  NACK requires the exact error; cleanup asserts ERQ/ACTIVE/TDDE/MEN off and
  I2C IRQ disabled. The target is reset between cases.
- Short and long RXSTALL plus normal restoration all pass:
  `i2c-one-shot-rxstall/d0a52bc46b1443a38bbd48acb5609661/verdict.json`.

| Mode | Programming before startup | Target READY within 9 s | Controller result/time | Target RX service hold |
| --- | --- | --- | --- | --- |
| Short | 8,938 ms | 4,706 ms | `Ok(())`, 6,886 us; exact 16 bytes + STOP | 5,129 us / 6 samples |
| Long | 9,052 ms | 4,817 ms | Exactly `Err(Timeout)`, 100,064 us; local cleanup | 150,274 us / 149 samples |

All four mode-specific endpoint processes exit zero. Both SCL records observe
idle-high, low-hold and released-high input samples, not a continuous waveform.
The long target separately aborts its own held transfer; controller bus-clear
or automatic recovery of a stranded peer is not inferred.

The runner restores and reruns the normal pair, both endpoints exiting zero:
`i2c-one-shot-hil/b5370132897b4d368f67302f819c43a3`. This is the final flashed
board state. Its ELF SHA-256 values are:

```text
controller BC7EBE6D7BF8BE4AF1024E7BE0CB198AA03F748E6F3CC47CD4F62D05BC0C3639
target     12696D79B5E1B747A8E68802491DF01F57D832478892FBDDCE2A68DE69145774
```

Both generator checks also pass (9 one-shot / 17 full-driver registers), as
do all 20 mutation-runner selftests. These complement rather than replace the
fresh full-manifest mutation run still to be recorded. No full production
multi-phase rig, new watchdog qualification, or full repository CI was run.

### Final committed-baseline mutation validation

The full manifest passes **28/28** on exact baseline
`5a20e71126632e8017202557a407c83869b44e95`: **21 compiler** and **7 behavioral**
checks. This documentation baseline includes implementation `4efa39060` and
runner correction `b736836d0`. Later overview commit `f8969d033` changes only
Markdown; an explicit Git comparison confirms validation source inputs still
match the tested baseline.

The runner uses `RUSTFLAGS=-Dwarnings`,
`CARGO_BUILD_TARGET=x86_64-pc-windows-msvc`, Rust 1.92.0, and no encoded-flags
override. All 28 positive controls exit zero. All mutants exit 101 with the
specified diagnostic/message/exact primary source span or the sole named
behavior assertion. There are no retries, infrastructure failures, timeouts,
lockfile inventory/hash changes, or retained runner-owned worktrees. These
checks do not mistake unrelated compilation or link failures for evidence.

The four newly added boundary rejections are:

| Deliberate mistake | Required compiler rejection |
| --- | --- |
| Add an unsafe block to the vocabulary root | `unsafe_code` at that block |
| Extract `Cpu.device` | `E0616` at the private field |
| Extract `InFlight.lease` | `E0616` at the private field |
| Call `InFlight.command(Stop)` | `E0599` at the unavailable method |

The remaining cases retain the prior register-direction, IRQ-source-policy,
reset-confirmation, pairing, moved-owner, wake and cleanup regressions. In
particular, omitted quiescence, quiescence before TDDE-off, DMA DONE treated as
wire completion, and omitted IRQ rearming fail their real-adapter effect tests.
This is evidence for these enumerated mistakes, not exhaustive verification.

Reports, raw positive/mutant logs, mutation diffs, saved manifest and lockfiles
are preserved in `target/one-shot-mutations-5a20e7112/` relative to
`examples/mcxa5xx`: 171 files, 5,852,920 bytes, each SHA-256 checked against
the original run at `C:/temp/capability-mutations-5a20e7112`. The evidence
archive contains no build caches. Original outputs, including disposable build
caches, remain intact; free C: space after validation is approximately 5.97 GiB.
No prior evidence or user files were removed.

The CI shell syntax check also passes, and all 52 local links across the eight
updated documents resolve. Together with the host/ARM checks and connected-
board results above, this completes the scoped boundary validation. All
changes are committed locally without pushing. The user's pre-existing
untracked C SDK example and cleanup script remain untouched. Full production
multi-phase hardware testing, new watchdog qualification and full repository
CI are still outside this run; the production driver has not been migrated.

## Checkpoint 11 — direct safe-mmio replacement (in progress)

Stage 1 changes generation and its tests, not the transfer protocol. The PAC
still owns offsets/field meanings. A direct nine-register generator replaces
the local macro dependency while preserving retained shared status handles,
exclusive FIFO access, confirmed reset, the fixed TX arm transition and the
paired requests-off/quiesce/fence restoration. The full production driver is
unchanged. See [the implementation note](DIRECT-SAFE-MMIO.md).

On the uncommitted stage-1 checkout based on `cb636388c`:

- HAL host tests: 49/49 on each MCU family (48 retained + one actual-map
  pure/destructive-read test). `-Dwarnings`; fresh root recheck on MCXA5xx.
- Direct generator tests: 7/7; nine-register `--check` passes. Compiler fixture
  and manifest now use the actual generated map instead of the Abacus Demo.
  Full committed-baseline mutation results are **pending**, not inferred from
  generator output checks or the earlier 28/28 run.
- Normal boards: both endpoints exit zero, exact 16 bytes and STOP;
  `target/i2c-one-shot-hil/071b75c097b248138231029ae72caa2a`.
- NACK/cancellation: exact NACK and local ERQ/ACTIVE/TDDE/MEN/IRQ cleanup pass;
  `target/i2c-one-shot-safety/865d8480d6ee4517a0efa395fbea7cf3/verdict.json`.
- Short/long RXSTALL and normal restoration pass;
  `target/i2c-one-shot-rxstall/1fc0808d56844306bb40419887aa18bc/verdict.json`.
  Short returns exact bytes+STOP at 6,889 us; long returns exactly Timeout at
  100,064 us with cleanup. The target independently ends its withheld service;
  this does not establish controller bus-clear.
- Final normal restoration: both endpoints exit zero;
  `target/i2c-one-shot-hil/6c7c1ea8613c4dcab5ebd2fb1a95ef4f`.

These paths are relative to `examples/mcxa5xx`. Builds/tests occurred while
the generator test port was being edited; the records are not attributed to a
clean later commit. A temporary compiler-negative fixture edit was restored
before the fresh positive host recheck. Firmware excludes that test fixture.
All future mutation checks use isolated worktrees, not temporary main-tree edits.

The user authorized deletion of only the previous mutation run's disposable
compilation caches. All 56 exact `baseline-target`/`mutant-target` directories
under `C:/temp/capability-mutations-5a20e7112` were removed; reports, logs,
diffs and lockfiles remain, with archive/original summary hashes rechecked.
Approximately 16 GiB was recovered. These build caches can be regenerated;
no validation evidence was removed.

Separately, WSL 2.7.14 and VirtualMachinePlatform installation completed with
exit zero. Windows requires a restart; none was requested or performed by the
script. Ubuntu/Kani execution remains pending. Installation log:
`C:/temp/install-wsl-for-kani.log`. The receive extension is being developed in
an isolated temporary worktree, not mixed into these stage-1 firmware results.

Independent stage-1 review additionally found a host-fixture ownership gap:
the direct-map callback checked for an existing owner but did not retain its
guard through the callback. It now holds an Rc guard through normal return and
unwind; a passing regression rejects nested direct/hardware owners. This is a
test-fixture correction, not a firmware change. The stage-1 source patch was
saved before RX integration at
`target/direct-safe-mmio-stage1-cb636388c/implementation.patch`; it is a source
checkpoint, not a validated mutation baseline or a git ref. Later manifest
validation made the no-clone diagnostic occurrence explicit.

## Checkpoint 12 — short combined RX transfer and Kani preparation

Validation below ran on uncommitted source based on `cb636388c`. The user
subsequently authorized a local commit of this implementation and record;
that checkpoint does not turn pending mutation or Kani work into a PASS.
Stage 1 was preserved above, then the separately developed receive extension
was integrated into the working branch. The full production controller,
target, DMA HAL, reset sequence and bus-clear implementation are unchanged by
these two stages. See [WRITE-READ.md](WRITE-READ.md) for the ownership contract.

The new path writes one register address, repeated START, reads four bytes,
then STOP. L2 owns the exclusive static RX slice, paired RX endpoint and DMA
channel. It prepares/enables DMA before RDDE, START-read and RECEIVE, in that
order; Streaming has no general CPU FIFO/command access. Both lengths are
preflighted and one deadline covers the whole operation. Cancellation/error
quiesce and consume the buffer; forgetting the future leaks static storage,
not a reusable alias. Only successful quiescence followed by STOP returns it.
No long-transfer/chunking/retry framework was added.

Measured integration results with `-Dwarnings`:

- HAL host tests: **59/59 on both MCXA5xx and MCXA2xx**. The added real-adapter
  tests include arm-before-command order, no launch on RX setup failure,
  invalid/busy preflight, pending RX cancellation, DMA error before DONE, and
  quiescence before RX memory return. The merged direct-map ownership/read
  fixture tests are included, not dropped during extraction.
- Core host suites: **24/24** (11 source/dependency policy, 8 protocol, 5 IRQ
  policy). Vocabulary: **11/11**, including a deliberately forgotten mock RX
  lease that does not invoke Drop or expose its buffer.
- ARM HAL checks: both families, defmt on/off, all four combinations pass.
  The four original/combined example binaries build with `--locked --release`.
- Public one-shot doctests: **1 positive and 4 compile-fail** pass without
  `custom-mmio`. The two new negatives cover stack RX and access after forget.
  Rustdoc does not pin their diagnostic reason; do not equate them to the
  separate exact-diagnostic mutation runner.
- Generator: **7/7**, nine-register `--check` passes. Firmware custom-MMIO
  exclusion and **12/12** checker controls pass. Abacus is absent from the
  active manifest/CI and resolved firmware graph; a stale entry remains in the
  ignored HAL lockfile and is not evidence of a compiled dependency.
- Mutation-runner self-tests: **20/20**. All **35 declared** current mutation
  anchors resolve uniquely (including the RX additions). The actual 35-case
  committed-baseline run is **not yet run**, not PASS. Local commit approval
  has now been received, making that baseline available for the next run.
  The unused fork remains retained until its ported rejection tests run.
- Formatting and `git diff --check` pass; stable rustfmt emits only the
  repository's existing nightly-option warnings.

Connected boards, main integrated tree, SWD 4000 kHz, controller
`1fc9:0143:ZS1PGS20MCJBC`, target `1fc9:0143:0YHQFRBFS4FIG`:

- Combined: both exit zero;
  `target/i2c-one-shot-hil/59b7aab557ca469792aa026b4411dcd6`.
  Target required `Restarted(1)` (an intermediate STOP fails) and then observed
  termination as `EarlyStop(5)` with spare TX capacity. The target count means
  STDR-queued bytes, including one stranded byte. Controller independently
  received exactly `[0x31, 0x82, 0xc7, 0x59]` and observed final STOP/idle.
- Original write-only restoration: both exit zero, exact 16 bytes and STOP;
  `target/i2c-one-shot-hil/8270771b3d084c3b8baf3465590ecd3a`.

Paths above are relative to `examples/mcxa5xx`. The isolated development run
also passed at `C:/temp/i2c-write-read-development/examples/mcxa5xx/target/`
`i2c-one-shot-hil/ea612c2e491e4f45957bc936385a1edd`. The two preceding failed
peer-assertion runs remain in that same directory (`077566c067834cc0b66324065df9c987`
and `c8649c87b670499b9a2b92167080c970`). Their controller passed but their target
returned `NeedMore(4)` instead of the test's incorrect `Complete(4)` assumption;
they are **failed runs**, not counted as passes. The spare-capacity peer now
requires termination without changing the production target implementation.
Final formatting after integration validation changes no transfer logic.

Kani: a separate host-only
[proof package](../../../embassy-mcxa/experimental/i2c-one-shot-proofs/README.md)
contains three harnesses calling the real synchronous fault-selection and
preflight functions. Ordinary host `cargo check` passes with `cfg(kani)` OFF;
it neither compiles those harness bodies nor proves anything. WSL 2.7.14 and
the virtualization component are installed, but Windows explicitly requires a
restart. Ubuntu setup/Kani installation and execution remain pending; no
reboot was performed. Do not report these as Kani passes.

Limits: real RX cancellation/fault injection on the boards, arbitrary lengths,
electrical waveforms and exhaustive interrupt schedules are not qualified by
the four-byte HIL success. Host effect tests observe the actual adapter's HAL
calls, not physical DMA memory transactions. The earlier TX NACK/cancellation
and stall results remain scoped to checkpoint 11. This local commit records
the implementation and its measured limits. No push, reboot, fork-source
removal or unrelated C SDK/cleanup-script edit is included.

## Checkpoint 13 — Kani runs after WSL restart

The user rebooted Windows. Ubuntu 26.04.1 installed and starts under WSL2;
Kani runs as the dedicated unprivileged Linux user `kani`. System build
dependencies and Rust were installed inside that new distribution only. No
further Windows reboot, push, board flash, driver edit or harness edit was
needed. The measured code is the clean tracked source at
`b984630ed808bd6b1e7a69f43d9b988f080d5ace`.

**All three harnesses passed**, Kani 0.67.0 / CBMC 6.8.0 / CaDiCaL 2.0.0,
with the default verification checks enabled. The verifier uses
`nightly-2025-11-21` (Rust 1.93.0-nightly, commit `53732d5e0`) and
`x86_64-unknown-linux-gnu`, a 64-bit target. Installed Rust 1.98.1 stable is
only the cargo-plugin dispatcher. No RUSTFLAGS, encoded flags, assumptions,
stubs, safety-only settings or harness filters were supplied.

- `fault_priority_for_every_snapshot`: exact priority and unchanged successful
  observations for all 128 status/`want_stop` combinations.
- `completion_observation_never_swallows_a_fault`: fault acceptance for all 64
  snapshots with `want_stop=true`; this does not call the async completion wait.
- `all_input_sizes_are_rejected_or_fit_one_receive_command`: exact preflight
  acceptance over all u8 addresses/host-usize lengths and lossless arithmetic
  for the accepted count-minus-one calculation in the harness. This does not
  execute the physical RECEIVE command or assert the rejection error variant.

The complete package run exits zero and reports exactly three successful
harnesses, zero failures, three total. The separate negative engine control
asserts an unconstrained byte is always zero: it exits one and reports the
expected `KANI_ENGINE_NEGATIVE_CONTROL` solver assertion failure. A compile
or link failure is not counted as that control's success. Four package
lockfiles have identical hashes before and after verification.

The full record is in `target/kani-b984630ed/` relative to `examples/mcxa5xx`:
`proofs.log`, `negative-control.log`, versions and exit logs, before/after
lock hashes, `summary.json` with the source SHA-256 inventory, and the exact
run/control sources. The log records all three named results, not a filtered
or empty run. See [the proof note](../../../embassy-mcxa/experimental/i2c-one-shot-proofs/README.md)
for commands and detailed limits.

This is not an ARM binary proof or proof of the unsafe Device, async state
machine, register decoding, DMA, NVIC, reset timing or electrical bus. The
priority proof establishes agreement with the stated oracle, not independent
validation of that oracle against the reference manual. The 35-case isolated
mutation run and subsequent unused-fork retirement remain pending. This
follow-up changes validation documentation only; no additional commit/push
has been made.
