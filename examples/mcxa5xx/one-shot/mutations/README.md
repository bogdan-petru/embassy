# One-shot mutation checks

The runner tests an explicit committed revision in one temporary Git worktree
per case. It never commits, stashes, resets, or cleans the working checkout.
Relevant staged, unstaged, and nonignored untracked inputs cause refusal, including when
the selected revision is older than HEAD. The manifest is reviewed input and
may be supplied separately from that revision; its bytes and SHA-256 are saved
with each run. Commands in a manifest are trusted executable arguments, not
untrusted data.

From the Embassy repository root:

```powershell
python examples/mcxa5xx/one-shot/mutations/runner.py --baseline <committed-revision>
python examples/mcxa5xx/one-shot/mutations/runner.py --baseline <committed-revision> --case write-only-read
python -m unittest discover -s examples/mcxa5xx/one-shot/mutations/tests -v
```

Do not substitute a clean old revision for the current work merely to obtain a
passing result. Commit the intended source explicitly before the real mutation
run. This runner deliberately has no option to waive its dirty-input check.
The self-tests create and commit only newly generated temporary fixture repos;
they do not commit any Embassy changes.

`--output` names a new directory and refuses an existing one. By default logs
are retained under a new system temporary directory. `summary.json` records the
commit, manifest hash, command, expected diagnostic, exact changed span, exit
status, elapsed time, and any retained worktree. The log records `rustc -vV`
and `cargo --version` from each case's worktree. Only `RUSTFLAGS`,
`RUSTDOCFLAGS`, `CARGO_ENCODED_RUSTFLAGS`, and `CARGO_BUILD_TARGET` are copied
from the inherited environment into the report; there is no full environment
dump. `ledger.md` is generated from
those recorded results. Each case directory retains baseline/mutant stdout and
stderr and the actual mutation diff. Baseline and mutant each use a separate
Cargo output directory under that case's results. This avoids stale build
state and overwriting a test executable Windows may still hold after exit.
The command and features are identical; only the recorded output-directory
environment value differs. Inherited user target directories are not modified.

The main repository ignores Cargo.lock, so its local lockfile is neither a
cleanliness input nor copied into the worktree. Cargo resolves dependencies
fresh in each committed worktree, using any lockfiles that revision tracks.
After the positive command, all worktree Cargo.lock bytes are saved under the
case's `baseline-lockfiles/` directory with SHA-256 hashes in `summary.json`.
The mutant must leave the exact set of lockfiles and their bytes unchanged;
otherwise the case fails even if its diagnostic or assertion matches. These
saved files establish the dependencies of that run, not reproducibility from
the source revision alone. Baseline and mutant use the same command and the
same in-worktree lockfiles.

The runner requires a successful positive execution of the identical command
and features before applying the unique source anchor. Compiler cases require
exactly one error with the specified Rust error code, literal message fragment,
source file, and **exact primary byte span** in the replacement. A missing MMIO
provider, wrong span, unrelated compiler error, or arbitrary nonzero exit is a
failure of the mutation check. The initial backend cases execute the real
positive integration test, so their baseline includes linking and MMIO access.

Compiler codes may be an exact `E####` diagnostic or the explicitly reviewed
named lint `unsafe_code`. No other named lint or wildcard is accepted. The
unsafe-code case still requires its exact diagnostic message, source file,
and whole primary span; `unused_unsafe` or an unrelated error cannot stand in
for the forbidden-code rejection. A real Rust self-test also reruns the same
fixture with `--cap-lints=allow`: the mutant then compiles, and the runner must
record `FAIL` with `Mutation survived: command succeeded`. That negative control
tests the runner's sensitivity to the lint being disabled, not a claim that
source lint attributes override compiler flags.

Behavioral cases must run exactly one named test with `--exact --nocapture`.
That test must pass the positive baseline and fail the mutant while emitting
the named invariant assertion. Compilation errors, zero tests, an unrelated
panic, another failing test, and timeouts never count as expected rejection.
Use a specific stable panic marker in each host assertion, such as
`ONE_SHOT_TDDE_OFF_BEFORE_QUIESCE`, instead of matching generic `assertion failed`.

Every normal cleanup resolves the runner-created worktree path, checks its
parent and Git registration, then removes only that worktree. `--keep-worktrees`
retains them for review. A crash or uncertain command termination leaves the
worktree in place; the runner never recursively deletes the temporary parent.
There is no claim that a software timeout bounds a faulty hardware DMA drain.

## Adding a case

Append a case to `manifest.json`; behavior test authors can use this shape:

```json
{
  "id": "omit-dma-quiesce",
  "kind": "behavior",
  "source": "embassy-mcxa/src/i2c/one_shot/session/vocabulary/registers.rs",
  "anchor": "An exact, unique reviewed source fragment",
  "replacement": "The corresponding source with the fault introduced",
  "guarantee": "DMA quiescence returns before controller reset or release.",
  "cwd": "embassy-mcxa",
  "command": [
    "cargo", "test", "--lib", "--no-default-features",
    "--features", "mcxa5xx,one-shot-host-tests", "--message-format=json",
    "fully::qualified::test_name", "--", "--exact", "--nocapture"
  ],
  "expected": {
    "test": "fully::qualified::test_name",
    "assertion": "ONE_SHOT_QUIESCE_BEFORE_RESET"
  },
  "evidence": ["IMPLEMENTATION-PLAN.md section 2: real-adapter cleanup trace"]
}
```

For a compiler case use `kind: "compiler"` and `expected` containing `code`,
`message_contains`, and `span: {"text": "changed_identifier"}`. If that text
occurs repeatedly in the replacement, supply a one-based `span.occurrence`.
The reported primary span must match the selected bytes exactly; expansion
notes or a nearby line cannot substitute for the mutation's diagnostic.

Keep `relevant_paths` complete for local build inputs, including added path
dependencies and configuration. The initial list includes the HAL's direct
local dependencies and their in-repository path dependencies, including the
executor macros and timer queue. It is a reviewed inventory of tracked and
nonignored sources, not automatic dependency discovery or a universal security
gate. Git-ignored local source inputs must not be used as dependencies: Git
status cannot detect their edits. Cargo.lock is handled separately through
fresh resolution and the recorded snapshots described above. HAL behavior
tests must exercise the real adapter over the three effect seams; protocol
mocks do not establish quiescence order, wake correctness, or MMIO ownership.

Manifest declarations are planned checks until a committed-baseline run has
produced the expected result. Existing host/HIL references are displayed as
declared evidence, never converted into a new passing verdict by this runner.
