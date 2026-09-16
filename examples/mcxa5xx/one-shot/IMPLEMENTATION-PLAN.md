# One-shot ownership and effect-testing plan

Status — **historical pre-split checkpoint**: protocol/lease implementation and
service-withholding tests complete; confirmed full-driver local reset
implemented; SCL-only watchdog sweep passes.
First checkpoint: `77f190623` on `experiment/i2c-pac-safe-wrapper`.
Checkpoint 7 adds the book-inspired unsafe-boundary safeguards: structural Sync,
unsafe-free full-driver sequencing modules, wake-only source policy, one paired
DMA endpoint capability, and observed reset/enable in the one-shot. These are
targeted follow-ups, not a full-driver protocol extraction or Tock OS migration.
Retained handles and real-adapter effect tests passed all nine baseline
mutations plus normal/NACK/cancellation/final-normal connected-board checks.
Protocol extraction, paired lease, source/dependency policy, controlled-time
deadline tests and expanded mutations are now implemented. Their new committed
mutation run and hardware evidence are recorded separately in the checkpoint
ledger. A separate historical write-path RXSTALL probe passed the short-service
and long-timeout checks on both boards. SCL observation was unavailable at
checkpoint 3; the corrected read-only observer now passes high/low/high controls. A new
[watchdog probe](WATCHDOG.md) addresses the low-duration
witness with hardware thresholds and corrects the observer's CC-only gate
precondition. Its threshold evidence must remain separate from full terminal
recovery qualification and exact waveform claims.
The original 120/140 ms recovery failures were committed independently. Matched
repair diagnostics then distinguished peer-dependent graceful disable from
bounded abrupt reset and explicit bus-idle policy. The production fix now
requires confirmed reset before restoring authority; the new default sweep
requires MBF idle at return and SCL high after release. BUSIDLE is preserved,
not globally shortened, and local recovery is not an automatic bus-clear
promise. The failed baseline checks remain recorded, not retried into passes.
Those results are evidence for that historical layout. Checkpoints 8 and 9
preserve the former two-package core/interface boundary and the runner-only
RXSTALL correction. The completed current layout follows the architecture-note
names directly: L1 PAC; L2 trusted physical MMIO/resource capabilities plus the
unsafe `Device` contract; L3 unsafe-free named operations and safe `Backend`;
and L4 unsafe-free core/transaction. [Checkpoint 10](PROGRESS.md) is the
current evidence and limits ledger. The sections below retain historical design
and acceptance requirements without turning those records into aggregate claims
for the newer extraction.

Read with [the current implementation](ABACUS-SAFE-MMIO.md),
[the broader framework proposal](FRAMEWORK.md), and
[historical validation](VALIDATION.md).

## Thesis and scope

The framework generates who owns an operation now, what evidence permits
the next operation, and what cleanup remains owed. Register typing supports
that story; it is not the whole story.
Organize a driver around ownership and a promised service, not its register map.

Keep the small async/DMA controller, existing public application, static
resources/buffer contract, and exact two-board verdict. The current extraction
uses three small source packages for L2, L3, and L4, while the physical HAL
implements L2. They are not extra runtime layers or a generalized framework.
L2 owns atomic resource construction, DMA arm, and shutdown; L3 owns named
START/STOP composition and status decoding; L4 owns typed sequencing and fault
priority. The high-level sequence, the one HAL-owned deadline, and the
source-static requirement stay unchanged. This checkout has per-package
manifests rather than a root Cargo workspace manifest; the HAL uses the core
through an optional path dependency.
Do not migrate the full I2C driver, invent a universal peripheral framework,
add a safe-mmio fork, submit upstream PRs, or rebuild the manager note in this
experiment. A future small target or eDMA abstraction is separate work.

Implementation order:

1. Retained shared handles for never-CPU-written pure registers.
2. Effect-level recording tests and isolated automated mutations.
3. L2/L3/L4 capability extraction and one paired controller/DMA lease, in
   reviewable steps under those tests.
4. Connected-board revalidation and write-path fault characterization.

## 1. Retain shared handles, not per-read uniqueness

Extend the generated owner to retain a private shared handle for each
`ReadPure` field. For this map, that means MSR and MFSR.

- Acquire each register-specific unique pointer once during exclusive
  initialization, with competing IRQ/DMA access excluded; immediately consume
  it into a shared pointer using safe-mmio 0.3.1's existing API.
- Pure getters subsequently use those shared handles directly. They do not
  reconstruct a unique pointer, create an MMIO reference, or expose a raw
  pointer/shared observer to callers.
- Writes and destructive reads keep short-lived, field-specific exclusive
  access. Do not reacquire uniqueness over the entire block.
- Do not apply this to `ReadPureWrite`: an ISR reader could overlap its
  exclusive write window. A register declared ReadPure must have no competing
  CPU writer under the actual hardware contract, not just a convenient label.
- Preserve private ownership, non-Copy/non-Clone and non-Send/non-Sync
  restrictions, state-gated access, live mappings/clocks, and shutdown ordering.
  Measure owner size and firmware effects; do not promise zero overhead.

Acceptance: positive/access-rejection backend tests still pass; generated
pure getters contain no per-call unique acquisition; the pointer/lifetime
argument is reviewed separately from functional tests. The ISR warning may
then be narrowed, not deleted wholesale: writes, FIFO pops, conflicting
accessors, and clock/power changes remain outside its permitted role.

## 2. Three effect seams, one ordered trace

Run the real adapter sequencing, cleanup, poll logic, and wake-only ISR body
against these effects. Do not replace the operation being tested with an
already-correct `poll_dma()` or `DataQueued` result.

| Seam | Real implementation | Host observation |
| --- | --- | --- |
| LPI2C register operations | Generated accessors using safe-mmio | Address/register, width, read/write, value, ordered sequence |
| DMA HAL operations | Delegate to DmaChannel | Configuration/arming, completion/error observations, quiesce entry/return |
| NVIC operations | Enable, disable, unpend the bound vector | IRQ state, pending delivery, exact order relative to register/DMA effects |

Use safe-mmio 0.3.1's existing `custom-mmio`, `MmioOps`, and
`set_mmio_ops!` for the first seam. It binds exactly once per final binary;
a plain RAM fixture by itself cannot record access order.
[Upstream example](https://github.com/google/safe-mmio/blob/9bef6d8c90fd60fc619018e21b43e3c584074a5b/tests/custom_mmio.rs).

The DMA seam sits below the one-shot adapter and above the HAL. It proves
that the adapter invokes quiescence at the right point, not that the HAL's
internal ERQ/ACTIVE/INT sequence is correct. Keep that boundary explicit.
Keep these HAL effect seams private/sealed. The L2 cross-crate boundary in
section 4 is the explicitly unsafe physical `Device` contract, not a public
safe extension point. L3's `Backend` is intentionally safe and open, so a
result from it must never release raw DMA authority or manufacture physical
cleanup evidence.

WaitCell and waker registration remain real software, not mocked methods.
Host fixtures must also provide safe construction and clock/pin teardown
counterparts so running the real transfer body cannot reach physical MMIO.
Physical pin/clock behavior is outside this initial host model.

### Two test layers, different claims

After extraction, use a safe `Backend` mock for protocol decisions: enumerate
fault sets and STOP/idle combinations, check legal sequences, and reject success
without the required observations. The mock can lie about protocol facts, so it
is not physical evidence; it has no `Device`, `Cpu`, or `InFlight` capability
and cannot access or release the real DMA/MMIO resources.

Retain the real adapter plus register/DMA/NVIC recording for effect ordering,
actual observation decoding, wake schedules and cleanup mutations. A mock
returning successful DMA-quiescence evidence cannot detect omitted quiescence.
Conversely, protocol decision enumeration need not exercise every MMIO effect.
The single whole-transfer deadline stays in the HAL's Embassy wrapper. Test
that real wrapper with a pending mock backend and controlled host time; do not
call it a pure core test. Keep the two evidence claims separate in the ledger.

### Recorder isolation and fail-closed rules

- One process-wide MmioOps implementation dispatches to a thread-local context.
  A scoped guard installs the script/log and restores state during unwinding.
  Reject accidental nested installation.
- Keep the executor and injected IRQ/DMA events on that test thread. Any future
  cross-thread scheduler must explicitly carry recording identity; an event
  must not silently become an unrecorded pass-through.
- Register fixture regions with base, length, ownership, and register metadata.
  Derive names/offsets/access directions from the reviewed generated map;
  validate bounds and alignment before interpreting an address.
- While scripted, reject unexpected addresses, non-u32 LPI2C accesses,
  wrong-direction operations, wrong order/value, and unconsumed expectations.
  Unknown operations never fall through to volatile access.
- With no script, permit pass-through only inside an explicitly registered,
  live RAM fixture. This preserves the existing host tests without treating
  arbitrary raw addresses as safe host memory.
- Inject callbacks only after releasing mutable recorder borrows/locks.
  Re-entrant ISR logging must not deadlock or trip a RefCell borrow panic.
- A generated legality table checks consistency; it cannot establish that
  the register policy itself is correct. Keep independent expected behavior
  and historical regression cases.

### Required schedules and effect assertions

1. Co-latched NDF and DMA DONE return Nack, never success.
2. SDF during drain returns UnexpectedStop.
3. DMA DONE alone never completes the I2C operation; STOP plus idle is required.
4. Fault and cancellation paths disable requests before calling quiesce and
   wait for its return before resetting the engine or releasing ownership.
5. Observe the full cleanup trace: NVIC disable/unpend, MIER=0, MDER=0,
   DMA quiesce entry/return, then the MEN/reset chain.
6. Cancel at each reachable pending stage: DMA drain, FIFO/STOP enqueue wait,
   and STOP completion. Include relevant preparation/arming error exits.
7. Exercise stale WaitCell tokens and completion before registration, between
   registration and the readiness read, and after that read before Pending.

For the wake test, execute the real sequence: drain/register WaitCell, write
MIER, enable NVIC, read MSR. Enumerate injection boundaries before/after those
operations and before returning Pending. Model latched device events and NVIC
masking/pending behavior: do not invoke a supposedly deliverable interrupt
while masked. Execute the same wake-only ISR body when delivery is eligible.

The success property is **Ready immediately, or Pending with a live wake path
that the modeled event actually triggers**. Do not demand a wake when the task
already observed readiness. This covers the enumerated schedules, not every
machine instruction or every possible hardware interleaving.

## 3. Automated mutations and a generated ledger

One reviewed manifest describes each mutation: stable ID, source anchor,
replacement, guarantee, test kind, expected compiler diagnostic or behavioral
failure, and associated host/HIL evidence.

- Use an isolated temporary git worktree per mutation, based on an explicit
  commit. Local runs must not silently test HEAD while relevant edits are
  uncommitted. Never auto-commit user changes.
- Require a positive baseline with the same command/features first. Require
  the mutation anchor to match exactly once; no-op/ambiguous mutations fail.
- Compiler cases check the intended diagnostic and source location, not any
  failed build. Behavioral cases must fail the named invariant assertion,
  not fail compilation.
- Keep logs tied to baseline commit, case, command and result. Clean only the
  runner's validated worktree paths; a crash must leave the main tree untouched.
- Generate the misuse table and ledger links from manifest plus recorded
  outcomes. Never generate a passing verdict from a declared expectation.

Initial behavioral mutations: remove quiesce, move quiesce before TDDE-off,
accept DONE as wire completion, and break wake registration/rearming.
Initial compile cases retain existing direction, private-boundary, moved-owner
and static-buffer checks; add lease-specific cases when that phase lands.

## 4. L2/L3/L4 capabilities and paired controller/DMA ownership

Keep the physical boundary and the streaming lease together after effect tests
exist. The extraction changes which code can name which authority; it does not
replace real adapter tests with a high-level mock.

### Cross-crate contract and lint boundary

- **L1** remains the Cargo-pinned PAC: it owns offsets and field/command
  encodings, not transaction policy.
- **L2** consists of the physical HAL plus `i2c-one-shot-interface`. Its sole
  declaration-only unsafe item is the open `Device` trait. The HAL's unsafe
  implementation owns the clocked controller, matching request/channel, IRQ
  binding, and static source. It must make every safe call order memory-safe:
  atomic arm/request enablement, Pending lease retention, and request-off plus
  quiescence before CPU authority returns. No default body or public raw
  resource-release primitive belongs on that trait.
- `Cpu<Device>` and `InFlight<Device>` are L2 opaque capabilities. They expose
  no device/channel/address extraction or partial DMA-setup operation. `arm`
  consumes CPU authority; `poll` returns the same in-flight owner on Pending
  and CPU authority only after the physical binding's terminal cleanup.
- **L3** is `i2c-one-shot-vocabulary`, a `no_std`,
  `#![forbid(unsafe_code)]` crate. It composes named START/STOP operations,
  decodes full status into observations, and maps lower completion ownership
  into the safe, open `Backend` trait. A safe Backend mock may lie about
  protocol facts, but cannot cause DMA-memory unsafety because it has no L2
  resource capability to split or release.
- **L4** is `i2c-one-shot-core`, also `#![forbid(unsafe_code)]`, with no direct
  PAC/MMIO/HAL dependency. It owns the typed sequence and fault priority and
  re-exports the safe L3 vocabulary so application/HAL imports remain stable.
  The public HAL API preflights address and static source before START; L4 also
  checks input at its owning transitions.
- The one 100 ms deadline remains around the complete transfer in the existing
  HAL wrapper. It does not restart between phases, preempt synchronous work, or
  bound a broken hardware ACTIVE drain. The source-static contract is unchanged.

The source-policy gate must inspect all three package inventories. L3/L4 reject
unsafe syntax and lint weakening entirely. The L2 interface allows only the
identified declaration-only `Device` trait and rejects extra unsafe syntax,
impls, default bodies, suppression routes, and unknown generated sources. The
physical HAL is separately trusted and tested. These are source-policy controls,
not electrical isolation or runtime-correctness proofs.

### Observation and completion semantics

L2 snapshots one MSR read into flat `Status`; L3 decodes it into the
protocol-facing `Observation` (`faults: FaultSet`, `stop_seen`, and
`controller_idle`) without selecting an error. L4 retains the priority rule
PLTF > NDF > ALF > FEF and checks faults before success. Do not collapse
co-latched faults to one lower-layer-selected error.

Wake arming is an input to observation, not evidence of FIFO readiness.
Keep the TX-FIFO-empty check inside L3's `try_stop`, at its point of use on
MFSR. Empty is the existing condition, not merely available space. This
separate register read is not part of an atomic MSR/DMA/FIFO snapshot, and
no fleeting readiness observation may be retained across an await as authority.

Rename `Started` to `StartQueued`. Rename/document successful DMA evidence
as `TxDmaQuiescent`: all requested bytes have been delivered to the FIFO,
DONE was accepted without DMA error, requests are off and the channel is
quiescent. Cancellation may quiesce the channel but never mints that success.
Make `WireComplete` a consuming completion transition of the current Stopping
owner, requiring its STOP observation, controller idle and prior fault checks.
Keep observations as data, not detachable or reusable authority tokens.

The public success promise is: the bus transaction is closed; whether the
addressed device has finished acting on the bytes is the device driver's
concern. Keep error/abandonment and peer-recovery limits explicit. L3 decodes
and composes commands; L4 chooses priority. Updating the external manager note
remains separate from this experiment.

### The paired lease

Start with one concrete lease before expanding the DSL. The lease owns the
controller register state, matching channel, and current static source buffer.
Its consuming completion returns the resources directly; do not issue a
detached receipt that could be reused for an earlier transfer.

Conceptual lifecycle:

`CPU-owned -> prepared -> streaming -> disarming -> CPU-owned`

One authority-separation principle applies at two scopes: Abacus `Off` gates
configuration registers; transaction `Streaming` gates CPU versus DMA access.
Both scopes are now implemented; the paired restoration still relies on the
reviewed hardware contract and effect tests, not register typing alone.

- Preparation configures the correct endpoint/request/channel before enabling
  transfer. Preserve current arming order and guard every intermediate owner.
- Streaming retains status reads and controlled wake rearming: MSR/MFSR
  reads and the necessary MIER operations remain available.
- Streaming exposes no MTDR write, arbitrary MDER setter, or MCR
  disable/reset/re-enable shortcut.
- Exit first disables TDDE, then quiesces the owned channel. Only after
  quiescence may the consuming lease return CPU register authority.
- Error/cancellation uses the same retained cleanup responsibility. Keep
  preparation and shutdown synchronous initially.
- Keep the restoration transition private to the lease implementation.
  A broadly callable state conversion must not bypass quiescence.
- Ownership of the live pair supplies transfer freshness without a detached
  epoch token. Correct instance/channel pairing still needs enforcement at
  construction and through the retained ownership types.
- Preserve static buffers/resources: mem::forget can bypass Drop. A lease
  does not prove that cleanup inevitably runs or that DMA eventually progresses.

Acceptance includes compiler rejection of CPU MTDR access during Streaming,
arbitrary TDDE enable, reset/re-enable bypass, cross-instance/channel misuse,
and reuse of consumed ownership; host traces must catch omitted/reordered
quiescence. DMA quiescence must remain distinct from wire completion.

## 5. Firmware feature exclusion and CI

Keep custom-mmio enabled only in the dedicated host-test dependency path.
It is not inherently test-only; firmware could supply a custom implementation.
Explicitly verify that our firmware does not activate it.

### Planned activation and registration ownership

In `embassy-mcxa`, add a separate opt-in feature rather than extending the
existing `test` feature, which also serves unrelated driver host tests:

```toml
[features]
one-shot-host-tests = ["test", "i2c-one-shot", "safe-mmio/custom-mmio"]
```

The HAL recorder and its single `set_mmio_ops!` invocation belong in a module
gated by `#[cfg(all(test, feature = "one-shot-host-tests"))]`. Run this suite
with `--lib --no-default-features --features mcxa5xx,one-shot-host-tests`, and
repeat for mcxa2xx. Keep HAL public doctests on the existing `test,i2c-one-shot`
feature path, without the recorder. Ordinary `test` and firmware `i2c-one-shot`
builds must not activate the hook. This activation is present in the first slice.
[Cargo dependency-feature syntax](https://doc.rust-lang.org/cargo/reference/features.html#dependency-features).

In the Abacus fork, enable the hook on its existing development dependency:

```toml
[dev-dependencies]
safe-mmio = { version = "=0.3.1", features = ["custom-mmio"] }
```

Make `tests/fixtures/mmio.rs` the registration-owning entry fixture for the
fork's MMIO doctests, both positive and negative. At this baseline, all 11
snippets in `SAFE_MMIO_TESTS.md` include it: one positive and ten negatives.
The other four default-feature doctests check unavailable imports and do not
use this fixture or custom MMIO. The integration executable
`tests/safe_mmio.rs` also imports the fixture once, so it must not add another
registration. Recorder implementation helpers can remain registration-free.

The HAL library-test registration is not linked into those executables.
A library's `#[cfg(test)]` registration is not available when that library
is compiled as a dependency of another test executable. Keep an inventory
of the actual unit, integration, and doctest link units rather than assuming
exactly two binaries.
[Cargo test targets](https://doc.rust-lang.org/cargo/commands/cargo-test.html#target-selection).

The fork currently uses edition 2021; the HAL uses edition 2024 but its
public doctests stay off custom-mmio. The standalone rule is therefore
future-proofing here: where rustdoc merges compatible doctests, mark
independently registering snippets `standalone_crate` so merging cannot
combine their providers. In the inspected Rust 1.92.0 implementation,
`compile_fail` snippets are already excluded from merging; this is not a
claim that edition-2024 negatives would merge and collide. A duplicate
introduced within a negative's own executable can still mask a lost rejection.
[Rustdoc standalone tests](https://doc.rust-lang.org/rustdoc/write-documentation/documentation-tests.html#standalone_crate).
[Rustdoc negative-test merge exclusion](https://github.com/rust-lang/rust/blob/1.92.0/src/librustdoc/doctest/make.rs#L110-L113).

Shared recorder code is allowed. A helper crate does not inherently collide:
even one helper-provided registration can serve several separately linked
binaries. Duplicate providers within the *same* final binary are the problem.
The entry fixture above is a deliberate exception to registration-free helpers:
it supplies exactly one provider to each including executable, not a second
provider alongside one already defined at that executable's root.

Acceptance for either path requires positive tests that execute generated
MMIO over registered host RAM and therefore exercise linking and dispatch;
an unused function containing MMIO calls is not sufficient. Run the fork's
default and legacy-tock integration/doctest suites, plus both HAL host suites.

A positive canary in its own executable does not establish that a negative
has a provider. The user's link-only probe on the current toolchain showed
that `compile_fail` accepts an undefined-symbol linker failure, consistent
with [Rustdoc's failure handling](https://github.com/rust-lang/rust/blob/1.92.0/src/librustdoc/doctest.rs#L719-L739).
Registration must therefore accompany the negatives too; the shared entry fixture closes
that specific missing-provider path. It does not exclude unrelated compile
or link failures, or prove that the intended restriction caused the rejection.
Likewise, keeping HAL doctests off custom-mmio avoids introducing this provider
dependency; it does not make their failures diagnostic-specific.

Keep compiler-rejection checks tied to the intended diagnostic and source
location as in section 3. On the current stable toolchain, an `E...` code in a
rustdoc fence is not an enforced diagnostic assertion; the inspected
[error-code collection is nightly-gated](https://github.com/rust-lang/rust/blob/1.92.0/src/librustdoc/doctest.rs#L201-L203).
New invariant negatives belong in the manifest runner; a missing provider, another linker failure, or
the wrong compiler error must never count as the expected rejection.
The firmware gate below checks the resulting safe-mmio feature set, not the
spelling of whichever feature activated it.

### Firmware graph check

From the repository root, the diagnostic command is:

```powershell
cargo tree --manifest-path examples/mcxa5xx/Cargo.toml --features i2c-one-shot --target thumbv8m.main-none-eabihf --locked -e normal,build,features -i safe-mmio@0.3.1 --charset ascii --color never
```

Do not use `-e features` alone: it includes development dependencies.
[Cargo tree documentation](https://doc.rust-lang.org/cargo/commands/cargo-tree.html).
Match the manifest, target, default-feature behavior and feature flags to the
actual firmware build. Tree output is a build-graph check, not a replacement
for linking the firmware.

Equivalent PowerShell check; the implemented CI gate is
[the tested Python checker](../../../embassy-mcxa/tools/check_one_shot_firmware_features.py):

```powershell
$taskGraph = @(cargo tree --manifest-path examples/mcxa5xx/Cargo.toml --features i2c-one-shot --target thumbv8m.main-none-eabihf --locked -e normal,build --prefix none --format '{p}|{f}' --color never)
if ($LASTEXITCODE -ne 0) { throw 'cargo tree failed' }
$taskRows = @($taskGraph | Where-Object { $_ -match '^safe-mmio v' })
if ($taskRows.Count -eq 0) { throw 'safe-mmio missing from firmware graph' }
foreach ($taskRow in $taskRows) {
    if ($taskRow -notmatch '^safe-mmio v0\.3\.1\|') {
        throw "Unexpected safe-mmio package: $taskRow"
    }
    $taskFeatures = ($taskRow -split '\|', 2)[1] -replace ' \(\*\)$', ''
    if (($taskFeatures -split ',') -contains 'custom-mmio') {
        throw 'custom-mmio enabled in firmware graph'
    }
}
```

Require failures for a missing package, an unexpected/additional version,
and the forbidden feature. Unit-test the checker itself with those fixtures.
Repeat for every firmware configuration being validated; feature unification
must not leak host instrumentation into an ARM build.

Keep this beside both generator --check commands, backend default/legacy-tock
tests, protocol dependency/lint gates, both host-test layers, mutation checks,
and warning-denied firmware builds. Both small protocol packages are
first-party targets, so lint policy is checked rather than merely appearing as
a capped dependency.
The historical pre-split gates are recorded in `ci.sh` and `PROGRESS.md`.
Checkpoint 8 records the former two-package core/interface source/build checks;
its `bb024eaf7` mutation and limited board evidence remain attached to that
baseline. Checkpoint 9 preserves the runner-only RXSTALL correction. Neither
historical record establishes evidence for the current L2/L3/L4 extraction;
see [Checkpoint 10](PROGRESS.md) for that layout's current results and limits.

## 6. Ledger: four evidence labels, not one green badge

Keep mechanism and status separate. A planned test is not a passed test.
Host-tested includes the proposed host-side dependency-policy checker, not just
adapter tests. Runtime checks count only for exercised cases; remaining runtime
obligations must be listed separately, not called static proofs.

| Invariant | Evidence category | Historical record or current ledger (details in PROGRESS.md) |
| --- | --- | --- |
| No wrong-state CPU FIFO write | Compile-enforced | Generated Streaming restriction and exact compiler rejection mutations pass |
| Fault priority and legal sequence | Host-tested (protocol decisions) | Mock-backend enumeration passes; separate decoder test preserves all co-latched faults |
| Correct decoding and TDDE-off / quiesce / reset order | Host-tested (real adapter effects) | Real recorder/ordering tests and committed omit/reorder mutations pass |
| One HAL-owned deadline covers the whole transfer | Host-tested (wrapper integration) | Actual wrapper passes controlled-time checks at every pending phase and across phase boundaries |
| No lost wake at enumerated event boundaries | Host-tested | Real shared WaitCell/ISR schedule tests pass; not exhaustive hardware interleavings |
| Exact payload and STOP on the connected pair | Hardware-tested | Baseline architecture passes normal and final normal reruns on both boards |
| NACK and ordinary cancellation shut down local hardware | Hardware-tested | Exact NACK and armed-DMA cancellation pass; other cancellation stages have host coverage, not newly measured HIL |
| No custom-mmio feature in firmware | Host-tested (build-policy checker) | Automated checker and negative fixtures pass; CI gate installed |
| Historical PAC-free core with one narrow unsafe declaration exception | Historical build-policy evidence | Parsed inventory/dependency checks and compiler mutation pass; does not validate the newer split |
| Historical two-package core/interface source boundary | Historical source/build evidence (Checkpoint 8) | Core/interface policy/tests and the `bb024eaf7` mutation/board evidence remain tied to that former layout; [Checkpoint 9](PROGRESS.md) records its runner-only RXSTALL correction. |
| Current L2/L3/L4 capability boundary | Current evidence/limits | [Checkpoint 10](PROGRESS.md) is the authoritative ledger. It must distinguish source/build, mutation, and hardware evidence rather than inheriting historical aggregate counts. |
| Read purity, register map and raw acquisition contract match silicon | Assumed / reviewed hardware contract | Not established by generating matching tables |
| Timing margins and behavior across operating conditions | Assumed outside tested conditions | Selected board results are not exhaustive timing qualification |
| Peer cooperation or eventual release of held lines | Assumed / external condition | No one-shot bus-recovery promise |
| An active DMA memory transaction eventually completes | Assumed | Software deadline cannot bound a broken hardware ACTIVE drain |

“Compile-enforced” describes callers using the intended safe boundaries, not
arbitrary unsafe code or trusted-module edits that bypass the contract.
Do not assume Drop runs for leaked futures: mem::forget remains possible;
the static resource contract prevents a dangling buffer rather than forcing cleanup.

## 7. Hardware acceptance and characterization

After retained-handle and lease changes, rerun exact payload/STOP, unserved
address NACK, and cancellation cleanup, then a final normal pair run. Record
commit, ELF identities, both board exits and complete verdicts.

Characterize a controller-write stall separately: target RXSTALL plus withheld
receive servicing is now exercised by the dedicated probe. eDMA must also stop
servicing receive requests or it may prevent the FIFO from filling. Prove an
actual sustained SCL stretch past the 100 ms deadline; do not recycle the
12 ms read/TXDSTALL probe or label a guessed stall duration as measured.

The separate probe observes pending RX data with no service/DMA, resumes the
short case to exact data/STOP, and measures exact Timeout plus cleanup in the
long case. The corrected passive observer supplies sampled high/low/high
corroboration; it does not provide a continuous waveform. The separate SCL-only
watchdog experiment now supplies a hardware-qualified threshold witness and a
completing 200 ms control. Its 120/140 ms cases also qualify local idle and SCL
high after the peer's designated abort. It does not replace the original
one-shot deadline test or promise automatic fresh traffic: reset can leave BBF
set, and the full rig tests explicit bus-clear separately. See RXSTALL.md,
WATCHDOG.md and the checkpoint ledger for measured outcomes and limits.

Data-phase NACK via RXNACK remains uncharacterized. Stuck SDA needs a controlled
bench setup. Neither becomes an acceptance-test PASS merely because a transfer
failed. Exact fault class, bounded local cleanup, and any peer-recovery limits
must be recorded separately.

## Exit criteria

Keep each implementation phase reviewable. Require positive tests, the intended
mutation failures, feature exclusion, unchanged application contract, and the
appropriate board rerun before advancing. Update the ledger with actual evidence.
No claim of full-driver migration, universal IP support, zero overhead, or
complete hardware correctness follows from completing this bounded experiment.
