# Native protocol mutation checks

`cargo-mutants` asks a different question from the named safety-mutation runner:
**when a tool changes an expression or return value, do our behavior tests notice?**
It complements the generated-sequence tests in [PROPERTIES.md](PROPERTIES.md).
Neither tool proves the hardware backend is correct.

## Reproduce

From the repository root, in PowerShell 7:

```powershell
cargo +1.92 install --locked cargo-mutants --version 27.1.0
pwsh -NoProfile -File examples/mcxa5xx/one-shot/run-core-mutants.ps1 -ListOnly
pwsh -NoProfile -File examples/mcxa5xx/one-shot/run-core-mutants.ps1
```

Each invocation creates a new directory below `examples/mcxa5xx/target/` and
prints its location. It preserves the exact source snapshot, lockfile, SHA-256
inventory, tool/compiler versions, command arguments, candidate list, logs,
mutation diffs, and machine-readable outcomes. No cache or evidence is deleted.
The source snapshot includes current uncommitted test changes: its hashes, not
the Git HEAD alone, identify the tested input.

To investigate only a named family of candidates, use the regular expression
shown by `-ListOnly`, for example:

```powershell
pwsh -NoProfile -File examples/mcxa5xx/one-shot/run-core-mutants.ps1 -Match 'checked|validate_transfer'
```

An empty selection is rejected. A filtered campaign is not a full-file result.
The script requires the pinned tool version so an update cannot silently change
the candidate set. Updating that pin is a reviewed test-tool change.

## Scope and isolation

- The runner copies only the three small protocol crates' library sources and
  manifests, the core's lockfile, the two selected integration tests, and any
  persisted property-test regression inputs. It never copies the repository's
  large build or hardware-evidence directories.
- `--in-place` applies **only to that disposable source snapshot**, never to the
  checkout. One worker reuses one native build directory; interrupted runs can
  leave a mutant in the snapshot without damaging the working tree.
- Mutation is restricted to `i2c-one-shot-core/src/protocol.rs`. The baseline
  and every mutant run only `tests/protocol.rs` and `tests/properties.rs`, with
  the native Windows target explicitly selected. Source-policy tests are not
  behavioral evidence and are intentionally absent from this campaign.
- The unmutated baseline must pass. The defaults allow 180 seconds per build
  and 60 seconds per test invocation, with no incremental compilation. Tests
  use 256 generated cases and a recorded fixed seed (`20260914`). Mutants are
  run in source order; the RNG is pinned to ChaCha and other inherited
  `PROPTEST_*` overrides are cleared for the run. `-Cases` and `-Seed` allow an explicitly recorded second
  campaign; a new seed is additional sampling, not exhaustive exploration.
- Property failures discovered under a mutant are not written into the seed
  corpus or fed to subsequent mutants. Existing regression inputs are copied
  for provenance but **are not replayed**: the Proptest switch disables loading
  as well as saving the seed corpus. Literal deterministic regression tests
  still execute. Ordinary `cargo test` remains the place to persist newly
  discovered real-code failures. Mutant counterexamples stay in their logs.
- `--cap-lints=true` is deliberate: a new unused variable should not make a
  behavior mutation look compiler-rejected. This is **not** used by the normal
  warning-denied build or the named unsafe-boundary rejection checks.
- The additional `--error=crate::Error::Dma` asks the tool to try always-return-an-
  error replacements as well as its default candidates. This is a mutation
  input, not a production change or a new accepted error policy.

The script restores its temporary environment changes and refuses to reuse an
existing output directory. It checks that the snapshot's input files have been
restored at completion. Compilation caches may be pruned later under the
repository's normal explicit cleanup rules; keep the remaining evidence.

On this Windows host, immediately relinking a just-executed test binary produced
`LNK1104`; a cooldown did not reliably fix it. The snapshot-only Cargo target
runner instead executes a uniquely named **SHA-256-verified byte-identical copy**
beside the original executable, preserving local DLL lookup. It logs the copied
binary hash/path, keeps the copies, and returns the original test exit code.
There is no test retry, security-setting change, or delay-based acceptance rule.

## Reading the result

| Outcome | What it establishes |
| --- | --- |
| Caught | A selected behavior test failed under that mutation. Inspect the log to confirm the intended assertion, rather than a harness failure. |
| Missed | The tests passed. Review whether the mutation changes observable behavior before adding a test or declaring equivalence. |
| Unviable | The changed program did not build. This is not a behavior-test success. Opaque owners make several constructor substitutions naturally unviable. |
| Timeout | The run exceeded its bound. Diagnose it; do not quietly call it a caught behavioral regression. |

No blanket survivor exclusions are installed. In particular, an equivalent
mutant should be recorded with its reasoning, not used to inflate a score.
Tool exit zero is not enough: the wrapper requires a passing baseline and
exactly one result per selected candidate, rejects linker/ICE failures, requires
an actual failed test phase, libtest failure summaries, and a panic in a selected test source for a
`Caught` outcome, and admits only the reviewed `E0277`/`E0599` constructor/type
rejections as `Unviable`. Unknown outcomes are unqualified and return nonzero.
Only the baseline may report `Success`, and any missed mutant forces nonzero
independently of the tool's exit code. The diagnostic screen is not a
diagnostic-and-location compiler proof; the current 27 rejections were also
inspected individually, while the named runner remains the stronger home for
pinned compile-rejection contracts.
The tool's automatic transformations are a finite, version-specific set; they
do not enumerate every missing MMIO operation, unsafe lifetime violation, or
possible source edit. The existing [named runner](mutations/README.md) retains
its diagnostic/location checks and effect-order controls.

The evidence boundary remains: the protocol tests exercise the real async
protocol over an independent model and scripted backend; the effect tests check
real adapter ordering; Miri checks the exercised pointer operations; hardware
checks the physical assumptions. A mock reporting DMA completion does not prove
that real eDMA was quiesced.

## Tool references

The upstream [installation guide](https://mutants.rs/installation.html) describes
native installation and separate tool/build toolchains. The
[cargo-mutants repository](https://github.com/sourcefrog/cargo-mutants) describes
file scoping and why mutation outcomes complement code coverage. The script's
flags were checked against `cargo mutants --help` and the installed 27.1.0
source, including the distinction between `--cargo-arg` (build and test) and
`--cargo-test-arg` (test only).

## Recorded campaign

2026-09-14, Rust 1.92, cargo-mutants 27.1.0, the default 256 cases/seed above:

| Result | Count |
| --- | ---: |
| Candidates examined | 49 |
| Caught by an actual behavior-test failure | 22 |
| Did not type-check (`E0599`: 18; `E0277`: 9) | 27 |
| Missed / timed out / infrastructure failures | 0 / 0 / 0 |

The unmutated baseline passed all 13 selected tests (five property/contract tests
and eight existing protocol tests). The full final campaign took about 44
seconds. Its wrapper gate returned zero, and every captured input file,
including the regression sidecar, was unchanged afterwards. This means **22
viable mutations were caught**, not "49 bugs proved absent."

Final evidence:
[`core-mutants-20260914-184047-dbd460b5`](../target/core-mutants-20260914-184047-dbd460b5/)
(`run.json`, `input-files.json`, `result.json`, `candidates.json`,
`unqualified-outcomes.json`, and `mutants.out/`). The immediately preceding
byte-copy campaign `core-mutants-20260914-183950-14429a71` produced the same counts;
the final run additionally put copies beside the originals for DLL lookup and
tightened the panic-location check.

After that campaign, the outcome gate was tightened without changing the tests
or driver. Direct evaluation of the current gate accepted the archived final
report and rejected synthetic reports with a mutant mislabeled `Success`, a
missed mutant despite a zero tool exit, and a `Caught` result lacking a failed
test phase. These are parser controls, not additional driver mutation runs.

The 27 unviable substitutions call nonexistent `new`/`from_iter` constructors
or require unavailable `Default` implementations on opaque owners/generic
payloads. They are separately counted, not credited to the tests. Examples of
the viable mutations caught include replacing `STOP observed AND idle` with
`OR`, accepting every input, suppressing fault classification, and returning
`Dma` without executing each transaction stage. There were no survivors to
waive or production changes to make for this campaign.
The independent old-tests/new-tests control in [PROPERTIES.md](PROPERTIES.md)
also demonstrates an actual test improvement: always accepting preflight input
passes the old eight tests but fails the new property, shrinking to address zero
and length zero.

Earlier setup runs remain as diagnostic evidence, **not additional passing
campaigns**: `core-mutants-20260914-183440-7a00189c` suffered executable-replacement
link failures before the wrapper gate existed;
`core-mutants-20260914-183759-028a9d32` correctly returned nonzero for eight such
failures despite the underlying tool returning zero. The intermediate
`core-mutants-20260914-183650-15a62405` stopped at its baseline because an initial
Cargo runner argument quoting attempt was invalid. None is counted above.
