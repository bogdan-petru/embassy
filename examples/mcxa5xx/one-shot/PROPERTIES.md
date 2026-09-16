# Protocol sequence properties

The question is: **does the real protocol still follow its contract through
sequences we did not write by hand?** This is a host test, not a new driver or
another framework. No firmware behavior changes.

[`tests/properties.rs`](../../../embassy-mcxa/experimental/i2c-one-shot-core/tests/properties.rs)
uses `proptest` 1.11.0 to generate and shrink sequences. A small reference model
has these phases:

`open → TX DMA → enqueue STOP → await STOP + idle → finished`

At each manual poll the model predicts the result and permitted backend calls.
The test drives the **real public core states and futures**, then compares both
the result and the call log. Pending retains ownership; errors and cancellation
release the same owner exactly once. DMA completion cannot substitute for bus
completion. An observed fault must prevent the downstream DMA poll or command.
The expected fault priority comes from an independently written ranked set,
not a call to the implementation's classifier.

The stimulus includes repeated Pending, DMA completion/error, command
readiness/error, STOP without idle, idle without STOP, co-latched faults and
cancellation, including cancellation before the first poll. A poll can traverse
several immediately ready phases. End of a finite trace means cancellation,
not proof that a perpetually pending transfer will eventually finish.

There are three generated properties:

- Write sequences agree with the reference model (0–64 events).
- Invalid arguments and backend errors propagate without forbidden operations.
- Address/length preflight agrees with the public contract, including arbitrary
  `usize` values and extra weight on boundary values.

Two deterministic tests supplement sampling: every prefix of a successful
transfer, and **all 128 classifier inputs**. That count is six observation bits
(four faults, STOP, controller-idle), plus `want_stop`; the seventh hardware
status bit, bus-idle, is not an input to this core classifier. Successful
classification must preserve the complete observation, not only return `Ok`.

## Reproduce

From the repository root in PowerShell:

```powershell
$core = 'embassy-mcxa/experimental/i2c-one-shot-core/Cargo.toml'
# Standalone package locks are ignored in this repository. Create a lock if
# this is a fresh checkout, then keep it for replay (also saved by the mutant runner).
cargo +1.92 generate-lockfile --manifest-path $core
cargo +1.92 test --locked --manifest-path $core --target x86_64-pc-windows-msvc --test properties
```

The existing `ci.sh` core `cargo test` invocation automatically includes these
tests. It uses Proptest's default 256 cases **per generated property**, a fresh
random seed, and persisted failure-seed replay. For a larger reproducible run:

```powershell
$env:PROPTEST_CASES = '4096'
$env:PROPTEST_RNG_SEED = '20260914'
try {
    cargo +1.92 test --locked --manifest-path $core --target x86_64-pc-windows-msvc --test properties
    if ($LASTEXITCODE -ne 0) { throw 'Protocol properties failed' }
} finally {
    Remove-Item Env:PROPTEST_CASES, Env:PROPTEST_RNG_SEED -ErrorAction SilentlyContinue
}
```

Failures print a shrunk input and save a seed in
[`tests/properties.proptest-regressions`](../../../embassy-mcxa/experimental/i2c-one-shot-core/tests/properties.proptest-regressions).
Commit new seeds, and turn an actual defect into a literal deterministic
regression too: a changed strategy can change what an old seed generates.
Do not interpret a passing sampled run as an exhaustive sequence proof.
See Proptest's [persistence documentation](https://proptest-rs.github.io/proptest/proptest/failure-persistence.html).

## Scope and evidence

`proptest` is a **dev-dependency only**, with just its `std` feature. There is no
new normal/firmware dependency, no unsafe fixture, and no allocation leaked per
generated case. The smaller plain `proptest` vector/model is sufficient here;
`proptest-state-machine` would currently add machinery without another guarantee.

The generated sequences cover **writes**, not arbitrary combined-read
sequences. A cancelled nonempty `&'static mut` receive loan cannot in general be
recovered by a safe reusable fixture through this API. Random RX coverage is
therefore deferred rather than introducing a forged lifetime or an accumulating
leak. Existing combined-read deterministic tests and the L2 effect/hardware
checks remain separate evidence; this addition does not expand their claims.

Manual polling with a noop waker verifies the requested `WakeOn` class, **not**
waker registration, IRQ scheduling or lost wakes. Mock DMA completion is a
protocol fact, **not** evidence that the hardware channel quiesced. No test here
proves bus timing, physical repeated START, absence of STOP on the wire, deadline
behavior in the outer HAL wrapper, or memory safety after `mem::forget`.
The recorder, Miri, Kani and board tests still have their distinct jobs.

Initial validation on 2026-09-14, production source at `b984630ed`:

- 4,096 cases × 3 properties = **12,288 generated cases** passed with seed
  `20260914`, plus the two deterministic properties.
- Core suite: **29/29** passed (11 policy, 5 properties, 8 existing protocol,
  5 IRQ-policy), with `RUSTFLAGS=-Dwarnings` on Rust 1.92 / Windows x86-64.
- The captured host output is in
  [`target/core-test-quality/host-tests.log`](../target/core-test-quality/host-tests.log).
- Core `defmt` host test compilation and a normal ARM library build passed
  with warnings denied; formatting and diff checks passed (stable rustfmt
  reports the repository's existing nightly-only option warnings).
- `cargo tree --locked --manifest-path $core --target thumbv8m.main-none-eabihf
  -e normal,build` contains only core → vocabulary → interface. No Proptest.
- The actual MCXA5xx firmware graph (`examples/mcxa5xx/Cargo.toml`, feature
  `i2c-one-shot`, same ARM target and dependency kinds) also includes the core
  and excludes Proptest.

These counts describe this run, not a permanent coverage percentage. The
[scoped mutation campaign](CORE-MUTATIONS.md) asks the complementary question:
**which deliberately changed implementations can still pass these tests?**
The existing named/diagnostic mutation runner is retained; generated mutants
do not replace its specific safety and compiler-rejection guarantees.

## A demonstrated improvement, not just more green tests

In an isolated copy we replaced only `validate_transfer` with an unconditional
`Ok(())`, removing its address and size checks. The existing eight protocol
tests still passed. The new preflight property failed and shrank the input to
**address 0, length 0**: the mutant accepted an input the contract rejects.
This is a deliberately injected bug, **not** a defect in the production code.

Both test binaries were built together and then run against that same mutant.
Warnings were capped, as in the mutation campaign, so unused parameters could
not turn this into a compiler-rejection result. The property failed on the
actual result assertion; there was no linker error and no regression seed was
written. All production files remained unchanged.

[Control evidence](../target/core-test-quality/control-validate-transfer-20260914/result.json),
[exact mutation](../target/core-test-quality/control-validate-transfer-20260914/mutation.diff),
[old tests](../target/core-test-quality/control-validate-transfer-20260914/existing-protocol.log),
[new property](../target/core-test-quality/control-validate-transfer-20260914/new-properties.log).

For Jeff: **we can now demonstrate a bad change that the old suite accepted
and the new suite rejects, with a minimal reproducible counterexample.** That
is a concrete improvement in the regression guard, not a claim that testing
proves the entire driver safe.
