# From one transfer to repeatable peripheral drivers

Design proposal and source review — 2026-09-12, updated 2026-09-14. The broad
framework below remains a proposal. A narrow implementation now exists:
[one reviewed Rust contract format](../../../embassy-mcxa/tools/driver-contracts/README.md),
a shared PAC/layout/access emitter, and a [UART DMA TX pilot](UART-PILOT.md)
alongside I2C. This is not a generated async protocol engine: operation text
records reviewed obligations, while reset, DMA quiescence and transactions
remain handwritten. Checkpoints 8 and 9 in [PROGRESS.md](PROGRESS.md) are
historical records for the prior two-package core/interface layout and the
runner-only RXSTALL startup correction. The completed current extraction uses
the architecture-note names directly: L1 PAC; L2 trusted physical MMIO/resource
capabilities and the unsafe `Device` contract; L3 an unsafe-free named-operation
vocabulary with safe `Backend`; and L4 an unsafe-free typed protocol and
transaction. It keeps async transitions and recovery handwritten. [Checkpoint
10](PROGRESS.md) records that completed extraction; checkpoints 11/12 record
the direct-generator and short receive extension (with their remaining gates).
This proposal does
not repeat aggregate mutation, HIL, or CI outcomes.

The current [UART porting walkthrough](PORTING.md) makes the first same-IP/new-SoC
boundary concrete: an owned MCXA resource binding, unchanged PAC-free upper
libraries, and shared example logic. Its L2 adapter is still concretely bound
to MCXA; it is a reference for the next port, not cross-SoC qualification.

## Recommendation

Keep Embassy and the existing PAC. Keep this small driver as the reference
implementation. Build a **peripheral-contract framework**, rather than another
register generator or a universal I2C/SPI/UART driver.

The reusable unit should be an operation with a contract: who may request it,
what it does, what observation justifies its result, what wakes a pending call,
and what cleanup remains obligatory. Generate the repetitive API and checks;
keep chip-specific actions small and inspectable.

The relevant prior work was **Abacus**, a Rust framework wrapping Tock registers
with generated state-dependent access rules. That evaluation is now recorded in
[the historical experiment](ABACUS-SAFE-MMIO.md). The active examples directly
generate safe-mmio accessors and have no Abacus dependency. We retain the useful
state-dependent access idea, without claiming that it generates protocol truth.
[Abacus source](https://github.com/abacus-rs/abacus-registers).

## What Jeff's questions imply

Taking the earlier feedback about repeatable drivers for N IP blocks, machine-
readable inputs, encapsulation, and the location of `unsafe` as the requirements:

| Required outcome | What an instance of the model supplies |
| --- | --- |
| Repeatable across IP | A register source, a reviewed behavioral contract, a hardware adapter, and executable acceptance cases |
| Clear unsafe boundary | L2 `Device` owns physical access and memory-safety obligations; L3/L4 receive restricted safe capabilities/operations |
| Readable driver | A short transaction sequence; expanded generated code remains available for inspection |
| Machine-readable specification | Existing register descriptions plus a small behavior description referencing their symbols, not copying their bit positions |
| A real worked example | This one-shot I2C controller and two-board test, with its intentionally narrow scope stated |
| Evidence of improvement | Misuse rejected by the compiler, bad event orders rejected by host tests, and hardware results tied to the tested build |

These are four responsibilities, not four runtime engines. Use the
architecture-note numbering directly: L1 PAC; L2 trusted physical capability;
L3 named operations; L4 core/transaction. The interface, vocabulary, and core
packages express the L2/L3/L4 source boundaries, not three extra runtime
layers. The old example-local application/hardware-boundary/vocabulary
numbering is retired.

For a manager: we want the next engineer to fill in a small, reviewable contract
and implementation, not rediscover all the ordering rules scattered across the
last driver's error paths.

## What is already good, and what is still handwritten

The current [transaction](../../../embassy-mcxa/src/i2c/one_shot/session.rs)
still reads start, arm DMA, drain, stop, complete. The six explicit L4 owner
types in the [core](../../../embassy-mcxa/experimental/i2c-one-shot-core/src/protocol.rs)
restrict its legal sequence; the L3 vocabulary carries its opaque physical
capability rather than a raw hardware owner. The earlier local `Port<State>`
transition-macro description is historical and is not the current boundary.

These other rules are still handwritten decisions:

- The `WakeOn` selection, `want_stop` classification input, and interrupt enables.
- Which co-latched fault wins over readiness.
- The meaning of DMA completion versus completion on the wire.
- Disabling requests, waiting for DMA inactivity, then resetting the peripheral.
- The relationship between an evidence token and the hardware that produced it.

L2 keeps the physical controller/channel/source pair in one opaque capability,
so safe L3/L4 code cannot mix or release its raw pieces. The current call graph
still relies on the trusted physical `Device` implementation to report status
truthfully and to retain the pair until quiescence. These are improvement
opportunities, not newly demonstrated runtime bugs.

## The central idea: describe permission, effects, and outstanding obligations

An operation contract should answer six questions:

1. **Authority:** which owned state and resources are required?
2. **Effect:** which bounded hardware operation is requested?
3. **Observation:** what actually happened, on which owner and transfer?
4. **Outcomes:** pending, success, and each classified fault; what state survives?
5. **Wake plan:** which level/interrupt/DMA events can change the outcome?
6. **Obligations:** what must remain owned until completion or cancellation cleanup?

Example contract notation below is a proposal, not existing macro syntax:

```text
operation drain_tx
  requires:       Transfer<Streaming, owner, epoch>
  action:         backend.poll_tx
  pending:        retain Streaming and its cleanup owner
  success:        DataFed + DmaQuiescent -> Transfer<Queued, same owner, same epoch>
  faults:         classify I2C faults before accepting completion; classify DMA errors
  wake:           DMA completion/error, I2C faults, unexpected STOP, deadline
  cancellation:   mask sources -> disable DMA requests -> drain ACTIVE -> reset I2C
  does_not_prove:  bytes ACKed, STOP observed, bus released
```

The hardware-specific realization of those lines remains reviewed code. In
particular, status in I2C and status in eDMA are not one atomic hardware snapshot;
the sampling protocol and stability assumptions must be explicit.

The useful innovation is to generate **obligations as well as permissions**.
A pending transfer must retain its shutdown owner. Observing DMA DONE must not
produce the receipt needed to claim STOP. A final success path should require
the distinct completion receipt, rather than an unrelated boolean at a caller.
Ordinary error/cancellation drops follow the same resource shutdown contract.

This does not make Rust linear: values can be dropped or deliberately forgotten.
The framework must preserve the static-buffer leak-safety contract; it cannot
use the hope that Drop always runs to justify borrowed DMA memory.

## Hardware state is not the same as software progress

This is the most important lesson to reuse from Abacus. It distinguishes states
which hardware can leave autonomously from states that require software action,
and reconciles observations before allowing state-sensitive access. The research
also discusses limitations for timing-only rules and large state spaces.
[Abacus paper, sections 5–7](https://tylerpotyondy.com/assets/pdf/asplos26-potyondy.pdf).

Our `Open` means a START was queued; it does not mean an address was ACKed or
that the bus is still active. A future name such as `StartQueued` would make
that fact clearer. A type should represent durable authority or a precisely
scoped observation, not pretend that asynchronous hardware has stopped moving.

For our framework:

- Use types for ownership, legal operations, and resource lifetimes.
- Use freshly classified events for changing hardware facts.
- Mark whether evidence is durable until software invalidates it, or only usable
  within one checked operation. Do not store a fleeting readiness observation
  across an await and call it a proof of current readiness.
- Keep software cancellation separate from hardware faults. The production I2C
  driver's queued-command recovery taught us that those can require different
  actions even at the same software phase.
- Compose protocol progress with a DMA lease and a wake registration; do not
  enumerate every product of FIFO occupancy, DMA state, and interrupt state as
  a separate public type.

## Three concrete safety improvements

### 1. Replace boolean polling options with operation-specific event APIs

Generate `poll_tx_drain`, `poll_stop_room`, and `poll_stop_complete` contracts.
Each specifies the relevant status classes and wake sources together, then calls
a shared, reviewed register-waker/arm/recheck primitive. Callers cannot choose a
wake mask that omits an outcome they are waiting for.

Keep new fault variants exhaustive in generated matches. A catch-all success or
retry arm must not silently turn a newly added error into an accepted outcome.
Reject incomplete contracts at generation time instead of choosing defaults.

This directly targets the earlier missed-wake and raw-status/classifier drift.
It does not eliminate the need to validate interrupt masking and level/latch
semantics against the chip.

### 2. Bind receipts to the resource and transfer that produced them

For a reusable driver, carry an owner identity and fresh transfer epoch through
permits and receipts. A completion from I2C0 cannot finish I2C3; an old completion
cannot finish a later transaction on I2C3.

Prefer private transitions which return the next owned object directly, so
receipts need not circulate independently at all. Where separate receipts are
necessary, evaluate invariant lifetime brands. The `generativity` crate provides
unique invariant brands; ordinary lifetime annotations alone are not enough.
Its fit with async storage and Embassy task lifetimes must be tested, not assumed.
[Generativity documentation](https://docs.rs/generativity/latest/generativity/).

### 3. Model access side effects, not just read/write direction

A FIFO pop, a status observation, a write-one-to-clear acknowledgement, and a
command enqueue are different effects. Their APIs should differ:

- `snapshot_status`: observe without acknowledging.
- `pop_rx`: consumes exclusive FIFO access and returns a byte/result.
- `acknowledge_fault`: accepts the appropriate resolved-fault authority.
- `enqueue_stop`: consumes the permitted command transition.

Google's `safe-mmio` explicitly distinguishes side-effecting reads from pure
reads, requires exclusive access for the former, and offers pointer-based field
access plus a custom MMIO backend. It is a strong lower-layer reference or
alternative to evaluate, not something to stack independently over Tock on the
same addresses. It does not supply our I2C recovery policy or synchronize CPU
ownership with a DMA engine by itself.
[safe-mmio](https://github.com/google/safe-mmio).

## What to reuse, and what not to assume

| Project | Reuse or learn from it | Decision for this work |
| --- | --- | --- |
| [Abacus](https://github.com/abacus-rs/abacus-registers) | Generated state-dependent Tock access and hardware-state reconciliation | First candidate for a bounded compatibility experiment, not an immediate production dependency |
| [safe-mmio](https://github.com/google/safe-mmio) | Pure versus destructive reads; unique MMIO ownership; replaceable access backend | Study for the lower boundary; changing it and the state framework simultaneously would obscure the comparison |
| [device-driver](https://device-driver.com/book/) | Machine-readable registers, commands, buffers, and a transport-independent interface | Useful precedent, especially for external chips accessed over I2C/SPI; not a replacement for this MCU bus-controller state machine |
| [chiptool](https://github.com/embassy-rs/chiptool) | Register-description transformation and PAC generation | Keep the existing PAC pipeline; do not invent another source of bitfield values |
| [trybuild](https://github.com/dtolnay/trybuild) | Assert both accepted code and intended compiler diagnostics | Automate API misuse tests instead of relying on one-off deliberate edits |
| [Kani](https://model-checking.github.io/kani/usage.html) | Check assertions over nondeterministic inputs; replay counterexamples | Later, on an extracted host-compatible decision core and explicit hardware model, not directly on live MMIO |

Abacus is available as `abacus-registers` 0.1.0, not merely a presentation. But
source inspection gives reasons to do a fit experiment first: its manifest uses
Tock 0.9, while its DSL docs mention 0.10; its register model uses Tock bitfields,
where ours retains PAC field semantics. Its synchronous `AbacusCell::map` also
needs careful integration with owned async futures. The source contains an
explicit soundness-review TODO around that cell. None of this demonstrates an
exploitable bug, but it rules out treating adoption as a mechanical import.
[Manifest](https://raw.githubusercontent.com/abacus-rs/abacus-registers/main/Cargo.toml),
[DSL docs](https://docs.rs/abacus-registers-dsl/latest/abacus_registers_dsl/),
[cell implementation](https://raw.githubusercontent.com/abacus-rs/abacus-registers/main/src/lib.rs).

A second unusually relevant research direction is **Efeu**: it generates I2C
software/hardware implementations and verification models from a specification.
Its key relevance to us is modeling the peer and bus too, rather than assuming a
locally correct controller guarantees interoperability. Its C/HDL generation is
not a drop-in Embassy implementation. Borrow the model-and-conformance approach,
not its entire toolchain for this small driver.
[ETH project](https://enzian-test.systems.ethz.ch/projects/trustworthy-bmc/),
[paper](https://people.inf.ethz.ch/troscoe/pubs/schwyn_eurosys_2025.pdf).

## Machine-readable inputs without duplicated truth

Use two specifications with different responsibilities:

1. **Register source:** retain the Cargo-pinned PAC and existing generated map.
   Vendor CMSIS-SVD is XML and can describe access direction and read/write side
   effects; it is not merely offsets. Chiptool uses register IR/YAML transforms.
   Preserve available metadata and flag omissions rather than guessing semantics.
   [CMSIS-SVD format](https://arm-software.github.io/CMSIS_5/latest/SVD/html/index.html),
   [side-effect fields](https://arm-software.github.io/CMSIS_5/5.8.0/SVD/html/elem_registers.html).
2. **Behavior contract:** a small Rust-native declaration beside the backend,
   referencing PAC symbols and reviewed helper functions. It adds sequencing,
   arbitration/error policy, observation stability, DMA ownership, and recovery
   obligations that register metadata alone cannot supply.

Generate a readable expanded API, contract table, transition diagram, and test
inventory from that one behavior declaration. If another tool needs JSON, export
it; do not require engineers to maintain the same contract in Rust and YAML.
Freeze input revision and generator version. A check-only invocation must use
locked dependency resolution and must not update the lockfile.

The generator must not infer hardware behavior from names such as DONE or READY.
An AI may propose a contract with reference-manual citations, but an unsupported
assumption must stay an explicit review item, not become an invented safe API.

## Test the contract, and independently test its implementation

Use the same protocol implementation with a real backend and a deterministic
host backend. The backend seam must reach the effects whose ordering is tested;
a mock which simply returns a correct `DataQueued` cannot catch a real backend
that disables DMA in the wrong order.

Model relevant MMIO effects and let the test scheduler inject hardware progress
between accesses: co-latched faults and DONE, completion before waker registration,
ACTIVE persisting after request-disable, and cancellation at every pending point.
Then use the board rig to check that these abstractions match actual hardware.

Crucially, **do not generate the expected behavior solely from the same code
being checked**. Keep independently reviewed invariants and old regression cases.
Generated tests can check consistency while sharing the specification's mistake.

Maintain a small invariant ledger. Each entry links:

`hardware rule -> contract -> enforcing API -> misuse test -> fault trace -> HIL case`

For example, “DMA DONE is not wire completion” links to the distinct Queued and
Stopping states, the missing `stop` method on Streaming, a delayed-STOP host trace,
and the target's exact STOP-based receive verdict. Label each guarantee as
compile-enforced, runtime-checked, modeled, hardware-tested, or still assumed.

Add behavioral mutation checks: intentionally remove quiescence, accept DONE as
success, or omit a wake source. The appropriate test must fail. For private APIs,
use fixtures inside the generated module structure; do not make internals public
just so external trybuild tests can reach them. Keep a corresponding positive
case so an unrelated import or target-build failure cannot masquerade as safety.

## Where unsafe belongs

L2 is the trusted physical boundary. The HAL acquires clock/reset, pins, IRQ,
controller, and matched DMA resources, then implements the interface's only
declaration-only unsafe item: `Device`. Its soundness contract requires that
every safe call order remains memory-safe: static DMA memory stays valid;
arming and request enablement are indivisible; Pending retains the same live
lease; and error/Drop paths quiesce before authority returns. “Unsafe only at
the memory load/store” is too narrow: the trusted computing base includes
resource construction, DMA/IRQ ordering, and dependencies whose effects uphold
that contract.

L2 exposes opaque safe `Cpu` and `InFlight` capabilities. L3 cannot unwrap
them, recover a channel, issue an arbitrary MMIO operation, or independently
release resources. L3 and L4 therefore both use `#![forbid(unsafe_code)]` with
no exception. L3's `Backend` is deliberately safe and open: a mock can lie
about a protocol result, but it cannot turn that lie into DMA memory unsafety.
L4 keeps state transitions and fault priority. These rules are source-level
discipline, not a literal Tock capsule, electrical isolation, or a proof that
the HAL/runtime protocol is correct. PAC methods can still be safe to call in
trusted L2 code, so dependency, visibility, effect-order, and hardware evidence
remain separate concerns.

No framework here proves an electrically stuck bus will recover, all external
masters will cooperate, or hardware will eventually complete a DMA memory access.
State those assumptions separately from memory safety and protocol correctness.

## A small next step, not another large rewrite

Historical update after the first experiment: step 2 below was deliberately
narrowed to Off/Enabled/ResetHeld register access. The pre-split baseline adds
Streaming plus a reviewed confirmed-control boundary: acquisition is
Unconfirmed, Off requires observed reset release, and Enabled requires observed
MEN. Generated MCR write-only transitions were removed rather than presented as
hardware acknowledgment. That baseline's PAC-free core, paired lease, and
effect tests are documented in [the implemented scope](ABACUS-SAFE-MMIO.md).
Checkpoints 8 and 9 retain their historical two-package and runner evidence.
The completed current L2/L3/L4 extraction has its own [Checkpoint
10](PROGRESS.md) evidence and limits; do not infer its aggregate results from
the historical counts above.

1. Freeze this working one-shot driver and its two-board verdicts as a baseline.
2. Compare Abacus with the current small macro for just START, TX drain, and STOP.
   Require PAC field reuse, correct queued-versus-completed meaning, matching IRQ
   bindings, cancellation ownership, and useful compiler messages.
3. Extract one operation-specific poll contract and its deterministic host backend.
   Demonstrate that it catches the historical DONE/wake/cleanup mutations.
4. Automate compile-fail tests and the invariant ledger. Keep the application and
   transaction sequence unchanged and readable.
5. Apply the reusable pieces to the I2C target/DMA path next. Only claim cross-IP
   generality after a genuinely different peripheral validates the abstraction;
   that second peripheral is future work, not part of the current I2C scope.

Adoption gates: no duplicated PAC bitfields, no expanded public unsafe surface,
no loss of negative tests or hardware verdicts, and measured code-size/RAM/runtime
effects. Prefer an adaptation of existing work if it passes those gates. If it
does not, reuse its stable/transient model and build only the missing contract
and async-resource machinery. Do not promise zero overhead without measuring it.
