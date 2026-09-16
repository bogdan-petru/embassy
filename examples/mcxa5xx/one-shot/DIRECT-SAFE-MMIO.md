# Direct safe-mmio generation

The one-shot controller now generates its small register owner directly,
without compiling the local Abacus fork. This is a maintenance simplification,
not evidence that fewer source lines are intrinsically safer. The full
production I2C driver is unchanged.

The [generator](../../../embassy-mcxa/tools/gen_one_shot_mmio.py) combines
Cargo's pinned PAC offsets/access directions with a reviewed nine-register
policy. The PAC still supplies every field/command encoding. The policy, not
the PAC, specifies pure versus destructive reads and which state permits an
operation. The generated output is [mmio.rs](../../../embassy-mcxa/src/i2c/one_shot/session/vocabulary/registers/mmio.rs).

## What must survive the replacement

- One non-Copy, non-Clone, non-Send, non-Sync register owner.
- Retained shared MSR/MFSR handles, acquired once and moved across transitions.
- Field-sized exclusive writes and destructive MRDR reads; no reference over
  the whole register block, no exposed base/field pointer.
- `Unconfirmed` grants no operations. Only the reviewed reset/enable sequence,
  after actual readback, returns `Off`/`Enabled`.
- Fixed TX request-arm writes consume `Enabled` and return `Streaming`.
  Streaming permits observation and interrupt rearming, not CPU FIFO/reset
  access. Only requests-off followed by channel quiescence and fencing restores
  CPU authority.
- The same paired controller, endpoint, channel and static source survive
  pending polls and cancellation. No transfer sequence was changed in stage 1.

The handwritten reset and quiescence companions remain audited code. The
generator is not a general-purpose protocol compiler. Unused Abacus state-enum
conversions and generic DSL machinery are deliberately not reproduced.

## Regression evidence

The four former Abacus-Demo compiler checks now target the **actual generated
LPI2C map** through a compile-only HAL fixture. Additional cases cover timing
writes on Enabled, FIFO pops while Streaming, and copying the owner. The
manifest still requires the exact diagnostic and source span; arbitrary
compiler/link failures cannot count as a rejection.

The [generator tests](../../../embassy-mcxa/tools/gen_one_shot_mmio_test.py)
check an independent offset/direction/size oracle, the exact allowed method
surface, malformed policies, and CRLF behavior. The real HAL recorder additionally
checks retained pure reads and destructive receive reads. Existing reset,
DMA, cancellation and wake-order tests remain in place.

See [checkpoint 11](PROGRESS.md) for measured results and what remains pending.
The unused fork is retained on disk until the committed-baseline mutation port
is validated; it is no longer a firmware dependency. Its original experiment
and attribution remain in [the historical record](ABACUS-SAFE-MMIO.md).

## The subsequent receive extension

A short DMA [`write_read`](WRITE-READ.md) was developed separately and then
integrated: write one register address, repeated START, receive four bytes,
STOP. Its new obligation is
exclusive receive-memory ownership, including cancellation and forgotten
futures. Both endpoints pass, including the peer's rejection of an intermediate
STOP. Its measurements are recorded in checkpoint 12, not attributed to the
earlier behavior-preserving stage-1 runs.

Kani is a separate evidence source for bounded synchronous decisions. It will
not establish real DMA quiescence, interrupt interleavings, or electrical bus
behavior. After the user rebooted, WSL/Ubuntu/Kani setup completed and all
three synchronous-decision harnesses passed at `b984630ed`. See checkpoint 13
and [the exact scope](../../../embassy-mcxa/experimental/i2c-one-shot-proofs/README.md).
The separate 35-case compiler/effect mutation run remains pending.
