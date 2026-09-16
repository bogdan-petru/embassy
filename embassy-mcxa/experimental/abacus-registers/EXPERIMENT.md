# Abacus + safe-mmio experiment

This directory is a local experimental fork, not an upstream release or an
upstream-supported integration. The I2C and UART `one_shot` controllers opt into it; the
production I2C driver does not depend on it when that feature is disabled.

## Provenance

Source: [abacus-rs/abacus-registers](https://github.com/abacus-rs/abacus-registers),
pinned to [`a18dc3cca72fc98f990b15cc864002b9bb58966b`](https://github.com/abacus-rs/abacus-registers/tree/a18dc3cca72fc98f990b15cc864002b9bb58966b),
package version 0.1.0. Its package metadata declares the MIT license. No separate
LICENSE file was present in that source tree; the existing attribution and
source comments are retained. Resolve the missing license text with upstream
before distributing this fork as an independently packaged library.

The upstream files copied here were the root Cargo.toml, README.md and
src/lib.rs, plus the DSL crate's Cargo.toml and five source files (lib.rs,
register.rs, generating.rs, macro_input.rs, parsing.rs). The local Cargo path
ensures we use that pinned source, not whichever published version is latest.

Current integration additionally supports `mmio_initial_state(State)` for a
closed initial acquisition and `mmio_private` for fields restricted to reviewed
companions. This mode emits no state-erasing enum or generic constructor.
It shares parsing and access generation across I2C and UART, without DMA-specific
assumptions in the macro. See the one-shot ABACUS-INTEGRATION.md record.

Local changes:

- Normalized imported trailing whitespace in Cargo metadata, README and parser.
- Added `process_mmio_block`, a separate backend using the existing Abacus
  declaration parser, state types, state enum and consuming conversions.
- Added `safe_mmio_backend.rs`; the original Tock backend's runtime exports
  are available only with the explicit, non-default `legacy-tock` feature.
- Exported the new macro and linked the two local crates by path.
- Moved the upstream `AbacusCell`/`AbacusStaticRef` implementation intact
  into `src/legacy_tock.rs`, retaining its soundness TODO and attribution.
  Fixed its stale `StaticRef` intra-doc link/name to `AbacusStaticRef`.
  The Tock dependency is optional and enabled only by `legacy-tock`.
- Added a `safe-mmio = "=0.3.1"` dev dependency, real generated host fixtures,
  positive and compile-fail documentation checks, and this explanation.
- Retained private shared handles for never-CPU-written `ReadPure` fields;
  kept `ReadPureWrite` shared acquisition transient because it can be written.
- Enabled `custom-mmio` only on that development dependency. Its shared entry
  fixture supplies exactly one provider to each integration/doctest executable,
  validates live registered host RAM and access directions, and logs accesses.
- Repaired one stale upstream unit test that still named the removed
  `StateChangeRW` enum variant; it now checks that spelling is rejected.

## What the new backend actually enforces

An annotated register block generates a non-Copy, non-Clone, non-Send, non-Sync
owner parameterized by a hardware state. A register operation exists only in
its declared states and cannot exceed its declared access mode. A state
transition consumes the old owner, writes its declared word, and returns the
new owner. It cannot return the new state without doing that write.

For this experiment, declarations support flat **stable** states and `u32`
fields only. Transient/composite states, unannotated registers, non-byte
padding, conflicting operations, and unsupported layout attributes are
rejected. This is deliberate limited scope, not full compatibility with the
upstream backend. `register_base_addr = 0` is parser-compatible syntax; the
actual base is supplied to the unsafe constructor from the PAC.

`StateChange` uses one zero-argument function returning a complete `u32` write
word, rather than the original backend's Tock field expressions. The I2C
consumer supplies PAC-based builders, preserving the PAC as the source of
bitfield meanings. No register read/modify/write is implicitly generated.

`safe-mmio` performs the volatile operation and expresses pure versus
side-effecting reads: pure getters take `&self`; destructive reads and writes
take `&mut self`. The generator creates no Rust reference to device memory,
does not implement `Deref`, and never exposes a field pointer to callers.

## Ownership and the unsafe boundary

There is no persistent `UniqueMmioPointer` spanning the entire peripheral.
Each exclusive access covers **one register**, not the whole block. This
matters because an eDMA transfer can own MTDR while the CPU observes a
different status register.

During exclusive initialization, each `ReadPure` field gets exactly one
`UniqueMmioPointer::new` acquisition, immediately consumed into a retained
`SharedMmioPointer`. Pure getters read that private handle directly without
reacquiring uniqueness. State changes and enum conversions preserve the
handles. No CPU writer, destructive reader, or competing exclusive accessor
may cover those registers while a retained handle can be used. Competing
IRQ/DMA access must be excluded during the initial unique acquisitions.

The internal handles carry an erased `'static` marker because the existing
owner/state enum has no lifetime parameter. They are raw-pointer wrappers,
not references, and never escape: this is **not** a compiler proof of static
storage. The unsafe constructor's live-mapping obligation applies throughout
all owner uses, including every consuming state transition. The test backing
allocation can be reclaimed only after relinquishing the owner.

`ReadPureWrite` retains no observer, even if the declaration currently exposes
only reads. Its getter still creates and consumes a temporary unique pointer;
a concurrent ISR reader could overlap a future exclusive write. Destructive
reads and writes also keep field-specific temporary exclusive access. Those
paths still need no competing CPU/ISR/DMA access during the operation. The
owner remains non-Send/non-Sync through its private `NonNull` base; retained
handles do not authorize interrupts to write, pop a FIFO, claim uniqueness,
or change clocks/power. Functional tests cannot prove these pointer contracts.

Storage is no longer one pointer word: it is the raw base plus one pointer
word per `ReadPure` field. The host fixture measures two words (16 bytes on
the 64-bit host), versus its previous one word. The one-shot map has two such
fields, so its register owner has three words; firmware effects still require
the separate integration build/measurement. There is no zero-overhead claim.

Acquiring the owner is explicitly unsafe. The caller must establish the
correct live/aligned layout, hardware state, access modes/read purity,
exclusive CPU ownership, and absence of competing access to each register.
Annotated transition words must truly establish their claimed states.
Alignment or the actual silicon state is not inferred by the macro.

These obligations do **not** disappear because the methods generated above
them are safe. The trusted boundary module and its declarations are part of
the implementation to audit; the compiler protects callers outside that
boundary. In particular, it does not validate an incorrectly annotated
write-one-to-clear register or incorrectly encoded command.

## What remains handwritten

Abacus is used here for the small Off/Enabled/ResetHeld **register-access**
state machine. ResetHeld exposes only reset release. The existing Session
and transaction vocabulary still enforce
START → DMA → STOP, completion evidence, cancellation cleanup and fault
handling. This experiment does not generate those asynchronous transitions.
The default runtime does not compile or export `AbacusCell` or `AbacusStaticRef`.
The default crate also does not re-export `process_register_block` or Tock.
Explicit `legacy-tock` opt-in preserves those upstream APIs for compatibility,
but does not claim to solve their upstream soundness TODO. The original
generator source still compiles in the shared DSL crate, with its tests preserved;
gating the runtime is not a claim that the whole vendored source is small or
that the legacy backend has been audited.

The generated register owner has no hardware Drop behavior. Its lifetime is
not itself a DMA lease, and it cannot prevent an incorrectly written trusted
adapter from accessing MTDR during DMA. The enclosing I2C owner/transaction
states and Drop cleanup must provide those guarantees. Host checks do not
prove bus timing, pin behavior, peer recovery, or actual DMA completion.

## Repeatable backend checks

From the Embassy repository root:

```powershell
cargo test --manifest-path embassy-mcxa/experimental/abacus-registers/abacus-registers-dsl/Cargo.toml
cargo test --manifest-path embassy-mcxa/experimental/abacus-registers/Cargo.toml
cargo test --manifest-path embassy-mcxa/experimental/abacus-registers/Cargo.toml --features legacy-tock
```

The first command runs the upstream parser/generator tests plus backend
rejection tests (51 total). The second executes seven generated-access/fixture
host tests and thirteen documentation checks: one legal program and twelve programs
the compiler must reject. The same real macro fixture is shared by the
positive and negative checks. The positive program executes real MMIO through
the provider; each negative includes that same single provider so a missing
or duplicate registration is not an alternative reason to pass. Rustdoc still
accepts unrelated compile failures; the planned diagnostic-matching mutation
runner must pin the intended reason separately. The host backing store and
ordered trace verify actual offsets and transition writes, not just generated
token strings. The I2C integration
still needs its separate ARM builds, host tests and connected-board checks.

The default feature matrix adds four independent compile-fail checks for
the absent legacy exports (17 documentation checks total). The opt-in matrix
instead adds one positive compatibility check (14 documentation checks total).
Both preserve the seven safe-mmio host tests and all thirteen backend
documentation checks. Run both matrices with `RUSTFLAGS=-Dwarnings` and
`RUSTDOCFLAGS=-Dwarnings` when validating a change to this boundary.

The custom provider is a single linker registration with thread-local fixture
identity. Unknown/unregistered addresses, padding, incorrect widths or
directions fail before dereference. Registration is removed before freeing
the allocation, including during unwinding; nested fixtures are rejected.
This small fork fixture has no IRQ/DMA simulator or fault script: it tests
generated accesses and provider dispatch, not the real HAL's effect ordering.
