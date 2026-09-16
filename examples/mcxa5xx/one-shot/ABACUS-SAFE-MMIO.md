# Abacus + safe-mmio: historical experiment

**Stage-1 replacement:** the current one-shot generates safe-mmio directly and
no longer compiles the fork. [Direct generation](DIRECT-SAFE-MMIO.md) describes
the preserved contracts and the test port. The text below records the earlier
Abacus implementation and its evidence; references to "current" below belong
to that historical checkpoint, not the new generator. The unused fork is
retained until committed-baseline mutation validation completes.

The controller now uses **real Abacus-generated accessors with a local
safe-mmio backend**. The retained-handle/effect-testing checkpoint `77f190623`
and Checkpoints 8/9 in [current progress](PROGRESS.md) are historical evidence
for earlier source layouts and the runner-only RXSTALL correction. The current
layout completes the architecture-note L2/L3/L4 split: a trusted physical
`Device` capability below an unsafe-free named-operation vocabulary and an
unsafe-free protocol core. [Checkpoint 10](PROGRESS.md) records the evidence
and limits for this layout; this overview does not duplicate its aggregate
results. This is an experimental local adaptation, not an integration supported
by upstream Abacus as shipped.

Start with [the register declaration](../../../embassy-mcxa/src/i2c/one_shot/session/vocabulary/registers/mmio.rs)
and [the hardware adapter](../../../embassy-mcxa/src/i2c/one_shot/session/vocabulary/registers.rs).
The 33-line sending application and its public API are unchanged.

## Who does what?

| Architecture layer / component | Responsibility | Does not prove |
| --- | --- | --- |
| L1: pinned nxp-pac | Register offsets and bit/command encodings | Correct protocol sequencing |
| L2: Abacus + safe-mmio + physical HAL + interface | State-restricted register access; resource construction; atomic arm/shutdown; opaque `Cpu`/`InFlight`; sole unsafe `Device` contract | Truth of the physical hardware contract, electrical behavior, or peer recovery |
| L3: vocabulary crate | Safe named START/STOP operations, status decoding, and safe `Backend` | Truthfulness of a safe mock's protocol facts |
| L4: protocol core / transaction | START → DMA → STOP state sequence and fault priority | DMA quiescence or I2C bus completion beyond L2's audited promise |
| Embassy | Singleton/pin ownership, interrupt bindings, DMA HAL, executor and timer | Correct chip-specific implementation of our adapter |

These are responsibilities, not new runtime engines. The older example-local
numbering is retired: this document uses the architecture-note L1–L4 names.
Abacus/safe-mmio participate in trusted L2 access; the interface, vocabulary,
and core packages are the L2/L3/L4 source boundaries, not extra layers.

## The actual declaration is small

This comes from the new register map:

```rust
#[RegAttributes([Off], WriteOnly)]
mccr0: WriteOnly<u32>,

#[RegAttributes([Enabled], WriteOnly)]
mtdr: WriteOnly<u32>,

#[RegAttributes([Enabled], ReadOnly)]
mrdr: ReadOnly<u32>,
```

It generates timing writes only for `Lpi2cRegisters<Off>`, transmit writes only
for `Lpi2cRegisters<Enabled>`, and a destructive receive read requiring
`&mut self`. The receive accessor is included to demonstrate access semantics;
this driver still performs **writes only**.

MCR is deliberately **not** a generated write-only state transition:

```rust
#[RegAttributes([Unconfirmed, Off, Enabled], ReadOnly)]
mcr: ReadWrite<u32>,
```

The [control boundary](../../../embassy-mcxa/src/i2c/one_shot/session/vocabulary/registers/control_boundary.rs)
runs the production reset sequence over individual safe-mmio effects. It
returns `Off` only after abrupt RST assertion and release are observed, and
`Enabled` only after MEN readback succeeds. No arbitrary `write_mcr`, generated
enable/disable shortcut, or Streaming reset operation exists. Acquisition
starts as `Unconfirmed`; timing writes exist only after confirmed reset.

Cleanup quiesces DMA **before** entering this boundary. Each handshake has a
1 ms and 65,536-poll bound; failure attempts reset-held and fails before authority
or pin handoff. Firmware treats failed confirmation as a fail-stop contract
failure, not an ordinary NACK. It does not establish physical bus idle, generate
STOP, release a peer, or guarantee unresponsive hardware accepted the last write.

The [generator](../../../embassy-mcxa/tools/gen_one_shot_mmio.py) gets all nine
offsets from Cargo's locked direct PAC dependency. A reviewed manifest supplies
the state/access policy. Generated compile-time offsets and runtime PAC-address
checks guard layout drift. No second set of bitfields is introduced.

## The important ownership adjustment

The interrupt handler now only disables its NVIC vector and wakes the task.
It never creates another MMIO accessor. The task registers its waker, arms
MIER/NVIC, then checks level-latched status.

The generated owner stores a raw address but never a Rust reference or a
persistent unique pointer to the entire register block. Exclusive access covers
**one register for one operation**. This allows the CPU to inspect MSR while
eDMA writes MTDR without claiming exclusive ownership of both simultaneously.

Safe-mmio 0.3.1 requires raw acquisition through a UniqueMmioPointer. The
constructor now acquires each never-CPU-written ReadPure field once, consumes
uniqueness into a SharedMmioPointer, and retains that private handle across
state changes. MSR/MFSR getters no longer acquire uniqueness per call.
ReadPureWrite is deliberately excluded: its CPU write window still needs
exclusivity. Destructive reads and writes remain short-lived exclusive accesses.
The owner is neither Send nor Sync and exposes no observer handle or MMIO
reference. Its private erased lifetime is not proof of static storage; the
unsafe constructor still requires a live mapping and clocks for all owner uses.

This is still an audited obligation: the public PAC remains reachable inside
the HAL crate. A new PAC write or FIFO pop in the ISR could bypass the owner
without a Rust error. Retention removes the pure-read acquisition conflict,
not those remaining obligations. A bounded AST policy now pins the wake-only
ISR and its inventoried local callees/imports/source routing. Its negative
controls reject PAC writes/pops and helper/clock injection. A compile-only
fixture confirms that these same PAC operations compile under `forbid(unsafe_code)`:
the protection is a source-policy gate, **not** Rust visibility or full-HAL
compiler sealing. Changes to trusted dependencies and build/compiler machinery
remain outside this gate's claim.

The current extraction makes both L3 and L4 follow the
`#![forbid(unsafe_code)]` source rule. The interface owns the declaration-only,
open `unsafe trait Device`; the trusted physical HAL implements it and turns it
into opaque `Cpu`/`InFlight` capabilities. L3 owns the safe open `Backend` and
re-exports its vocabulary through the core, so HAL transfer imports stay
unchanged. A safe Backend mock can lie about protocol facts, but cannot extract
the physical controller/channel/source pair or return DMA authority. Thus an L3
ordering error is not a DMA-memory-safety escape. Source prohibition does not
make L2, the HAL, or trusted dependencies unsafe-free, isolate hardware, or
prove runtime behavior.

The DMA endpoint is now another small capability: acquisition consumes one
`Peri<T>` and derives the mapping and request from that same `T`. Address, byte
width and request cannot be supplied independently. The non-generic driver then
explicitly erases the instance brand and moves the opaque, non-Copy pair with
its controller/channel lease. There is no claim of type branding after erasure.

The generated `Streaming` state now removes CPU writes to **MTDR itself**,
arbitrary MDER writes, and MCR disable/reset. It retains pure status reads and
MIER wake rearming. A private lease owns the controller, channel and static
source together. The only CPU restoration is a reviewed concrete helper that
consumes the pair, turns requests off, quiesces the channel, and returns that
same pair. A detached receipt cannot be reused with another channel.
Register policy supports this boundary; it does not prove that the helper's
hardware ordering or the shared DMA HAL is correct.

## Historical baseline: what was verified

The [validation record](VALIDATION.md) contains historical hardware results and
limits. Counts below describe the pre-split baseline. Checkpoints 8 and 9
preserve the earlier two-package source boundary and runner correction,
including their original evidence and limitations. See [Checkpoint
10](PROGRESS.md) for the current L2/L3/L4 extraction's evidence and limits;
this document deliberately does not merge those checkpoint records or imply a
full repository CI run.

- Exact 16-byte reception and STOP, both boards exiting successfully.
- Unserved address produces exactly Nack; local DMA/I2C shutdown is checked.
- Cancellation after DMA arming leaves ERQ/ACTIVE/TDDE/MEN off and IRQ disabled.
- Warning-denied ARM checks for MCXA2xx/MCXA5xx, defmt on/off.
- Existing host tests: 11 per chip family; public API doctests: 3.
- Backend: 48 macro tests, 3 executable generated-MMIO tests, 11 doctests
  (one legal program plus ten compiler rejection cases).

The review cleanup adds four default-build compiler checks proving the legacy
exports are absent: 15 doctests in total. An explicit legacy-tock build runs
the original eleven plus one positive compatibility check (12 in total).

The negative compiler cases include wrong-state access, moved-owner reuse,
wrong register direction, a destructive read through a shared reference,
raw-base access, unsafe acquisition without unsafe, and cross-thread sharing.
They use the same generated fixture as positive tests. They do not merely
assert that token strings look right.

To run the additional checks from the repository root:

```powershell
cargo test --manifest-path embassy-mcxa/experimental/abacus-registers/abacus-registers-dsl/Cargo.toml
cargo test --manifest-path embassy-mcxa/experimental/abacus-registers/Cargo.toml
python embassy-mcxa/tools/gen_one_shot_mmio.py --check
```

## Does the combination make sense?

**Yes as a foundation; not yet as a production migration.** Abacus handles
state-dependent API generation, while safe-mmio handles the memory-access
discipline. Neither needs to replace the PAC's bitfield knowledge. The
interesting gain is generating a restricted interface from a small reviewed
contract instead of hand-maintaining those restrictions everywhere.

The cost is real. At the first hardware validation, the four driver files
totaled **580 lines**, versus 567 before this experiment, plus a **96-line
generated map**. The framework tree contained **2,976 Rust lines including
tests**, of which 388 were the new backend and its tests. These are baseline
measurements, not counters kept constant as comments and tests are added.
Counts include comments/blanks and exclude shared HALs, the source generator
and documentation. The framework source is outside the driver, but somebody
must maintain it.

The unused upstream `AbacusCell`/`AbacusStaticRef` runtime is now isolated behind
the non-default `legacy-tock` feature. The HAL explicitly disables Abacus default
features. The original Tock backend export is likewise opt-in; its generator
source still compiles in the shared macro crate. Gating unused runtime does not
resolve its upstream soundness TODO or remove the fork's maintenance cost.

This backend supports flat stable states and u32 fields only. It deliberately
rejects unsupported transient/composite state declarations. Async pending,
fault classification, cancellation and paired DMA ownership remain handwritten.
The earlier local transition macro has been replaced by six explicit owner
types in the L4 core; L3 supplies the safe named operation vocabulary and L2
retains the physical lease. This is not a generated async protocol engine.

This narrows step 2 of [the framework proposal](FRAMEWORK.md): that proposed
START/drain/STOP, whereas this first experiment implements register-access
states. It is not a completed async protocol-generator migration.

The production I2C controller, target and eDMA implementations are unchanged.
The receiving test peer still uses the existing production DMA target.
This does not establish bus-clear, timing margins, every cancellation race,
or safety against mistakes inside the trusted contract/adapter.

The **CPU-versus-DMA FIFO lease** remains the bounded experiment. Its L2
capability hides the controller/channel/source pair from L3/L4, but the
physical ordering is still an audited implementation obligation. A broader
framework remains deferred until a second small peripheral demonstrates what
actually generalizes.

See [fork provenance and constraints](../../../embassy-mcxa/experimental/abacus-registers/EXPERIMENT.md).
Upstream declares MIT in its metadata but the pinned source lacks a standalone
license file; resolve that packaging detail before independently distributing
the fork.
