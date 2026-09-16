# One-shot named operations — Layer 3

This `no_std` crate is the safe named-hardware-operation vocabulary for the
one-shot controller. Its crate root is `#![forbid(unsafe_code)]`: it cannot
acquire MMIO, construct a physical device binding, or implement the Layer-2
unsafe `Device` contract.

`Hardware<D>` owns an opaque `Cpu<D>` capability from Layer 2 and keeps that
field private: its public API does not expose the lower command methods.
Its streaming and receiving associated types are Layer 2's opaque `InFlight<D>`
and `Receiving<D>` directly, without another forwarding wrapper.
The crate composes the narrow commands that matter to this controller:

- check bus/controller idle and queue `START`;
- arm the indivisible DMA operation;
- forward one decoded `Status` without discarding any co-latched facts;
- retain the live DMA capability on a pending poll; and
- check the FIFO at the point of use before queueing `STOP`.

It exposes the deliberately **safe, open** `Backend` trait used by the typed
protocol. A safe test mock may implement `Backend` and lie about protocol
facts. That can make a protocol test wrong; it cannot cause DMA memory
unsafety, because a `Backend` implementation has no way to extract the
physical `Device`, channel, raw MMIO, or `InFlight` lease from the production
capabilities. Physical authority remains below it in Layer 2.

Layer 3 does not decide which co-latched fault wins. It carries the full
`Status`; the Layer-4 core applies the I2C fault-priority policy. Likewise
Layer 3 does not own the whole-transfer deadline or change the static-source
contract. Those remain in the existing transaction/HAL flow.

All poll results use the lower `Step` shape: Pending owns the live lease,
Complete owns the restored controller (and the RX buffer when applicable),
and Failed carries the error. Mapping these values cannot extract private
raw-device/lease fields. The `Cpu`/`InFlight`/`Receiving` boundary is retained;
this is not a public alias from `Hardware` to the broader `Cpu` API.

The three small packages map directly to the architecture-note responsibilities:
L2 is the trusted `Device` and its opaque capabilities, L3 is this named
operation vocabulary, and L4 is the unsafe-free protocol core. They are source
boundaries, not three new runtime layers or a generic peripheral framework.

## Verification

The separate [Kani proof package](../i2c-one-shot-proofs/README.md) exercises
this real vocabulary over a modeled Layer-2 `Device`, rather than replacing
Layer 3 with a mock `Backend`. Nine bounded proofs check admission, fresh FIFO
gating, status/wake forwarding, and TX/RX ownership and result preservation.
The same property helpers also run as native tests in `ci.sh`. Physical DMA
quiescence and interrupt delivery remain assumptions of that model, covered by
separate lower-layer and hardware evidence.
