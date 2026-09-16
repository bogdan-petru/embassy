# One-shot protocol core — Layer 4

The service is deliberately small: one async DMA write, then an observed STOP
and idle controller. Bus completion does not mean the addressed device has
finished processing the bytes.

Read [protocol.rs](src/protocol.rs) first, then the safe named-operation
[`i2c-one-shot-vocabulary`](../i2c-one-shot-vocabulary/README.md), and finally
the trusted lower [`i2c-one-shot-interface`](../i2c-one-shot-interface/README.md).
The protocol carries the owned sequence:

`Idle -> StartQueued -> Streaming -> TxDmaQuiescent -> Stopping -> complete()`

- START queued is not address acknowledged.
- `Streaming` retains the backend's live DMA lease. A Pending poll returns that
  same lease, not a detached completion receipt.
- `TxDmaQuiescent` means the backend reported successful DMA completion. For
  the audited Device-backed backend, that includes requests off and quiescence;
  a generic safe `Backend` can only report a protocol fact. Cancellation cannot
  produce it.
- `Stopping::complete()` owns cleanup throughout the wait. After fault-free
  STOP plus controller-idle it drops the backend before returning the payload.
  Cancellation/error still drops the retained owner. There is no detached
  completion token or separate `finish()` step.

This is **L4** in the architecture note. Its root is
`#![forbid(unsafe_code)]` with no local exception, unsafe declaration, block,
or implementation. It has no PAC, MMIO, HAL, allocator, or timer dependency.
It re-exports the safe `Backend`, common `Status`, `DmaPoll`, `ReceivePoll`, and
shared error/wake types from Layer 3 so the HAL transfer flow keeps its public
imports.

Layer 4 owns protocol policy. It validates the address before START, repeats
the 1..=32 static-source constraint at the transmit transition, preserves all
co-latched status facts, and chooses fault priority before accepting readiness.
The public HAL API preflights both address and source before it starts a
transaction. Layer 3 composes START/STOP and forwards the one decoded status
snapshot; Layer 2 atomically
arms and shuts down physical DMA; the existing HAL wrapper retains the one
100 ms deadline around the complete sequence. None of those source-boundary
changes alter the high-level sequence, deadline placement, or static-source
requirement.

`Status` preserves all seven decoded facts across the lower layers without
copying them into a second representation. Bus-idle admits the initial START;
it neither selects fault priority nor substitutes for controller-idle at STOP.
The exhaustive classifier test covers all 256 snapshot/phase combinations;
the actual async completion test varies bus-idle independently. Shared result
shapes still carry the live owner on Pending and return RX storage only with
its completed owner. Physical quiescence remains the lower binding's duty.

## A safe protocol boundary, not a hardware proof

`Backend` is deliberately safe and open in Layer 3. That makes inexpensive
mocks useful for checking state transitions and fault priority, but a safe mock
can report false protocol facts. Such a mock cannot release real DMA memory or
recover CPU authority: those operations are unavailable above Layer 2's opaque
`Cpu`/`InFlight` capabilities. The physical HAL is the sole audited
implementation of the unsafe `Device` contract below this core.

`forbid(unsafe_code)` is a source prohibition, not electrical isolation or a
proof of the protocol. Trusted dependencies and the physical HAL can contain
unsafe code. Real-adapter effect tests still need to establish request-off,
quiescence, reset, and wake ordering; connected-board tests still need to
establish the bounded hardware behavior they exercise.

The interface, vocabulary, and core packages are L2, L3, and L4 source
boundaries. They are not additional runtime engines or a migration of the
production I2C driver.

## Evidence status

[Checkpoint 8](../../../examples/mcxa5xx/one-shot/PROGRESS.md) and
[Checkpoint 9](../../../examples/mcxa5xx/one-shot/PROGRESS.md) are historical
records for the earlier two-package core/interface layout and the runner-only
RXSTALL startup correction. Their mutation and board results remain attached to
their recorded commits. The completed L2/L3/L4 code extraction is a newer
layout; [Checkpoint 10](../../../examples/mcxa5xx/one-shot/PROGRESS.md) is the
current source of its results and limits. This README deliberately leaves the
detailed mutation, hardware, and CI evidence to that ledger.

## Two different test claims

A safe `Backend` mock tests **protocol decisions**: fault priority, pending
ownership, completion conditions, and cleanup transitions. It cannot establish
that physical DMA stopped. The HAL's custom-MMIO recorder instead runs the
real Layer-2 binding and records register, DMA HAL, and NVIC effects in one
trace. Removing quiescence or moving it before request shutdown should fail
those effect tests. Controlled-time tests must exercise the actual HAL deadline
wrapper, not a second copy of the sequence.

From the repository root (resolve ignored lockfiles once without `--locked` if
needed):

```text
cargo test --locked --manifest-path embassy-mcxa/experimental/i2c-one-shot-interface/Cargo.toml
cargo test --locked --manifest-path embassy-mcxa/experimental/i2c-one-shot-vocabulary/Cargo.toml
cargo test --locked --manifest-path embassy-mcxa/experimental/i2c-one-shot-core/Cargo.toml
cargo check --locked --manifest-path embassy-mcxa/experimental/i2c-one-shot-core/Cargo.toml --features defmt --tests
```

These are path packages because this checkout uses per-package manifests, not a
root Cargo workspace manifest. They are not a generic peripheral framework.
See the experiment's [plan](../../../examples/mcxa5xx/one-shot/IMPLEMENTATION-PLAN.md)
and [validation ledger](../../../examples/mcxa5xx/one-shot/PROGRESS.md).
