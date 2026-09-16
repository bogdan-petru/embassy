# One-shot trusted device interface — Layer 2

This small `no_std` package is the lower capability boundary for the one-shot
experiment. It owns no register mapping and performs no MMIO itself. The
physical HAL constructs the actual binding from the clocked controller, its
matched DMA request/channel, IRQ binding, and static source contract.

`Device` is the **only declaration-only unsafe item** in the three small
one-shot packages. It is an open `unsafe trait` with no default bodies; the
audited physical HAL supplies its implementation. The declaration is a promise
from that binding, not compiler sealing or a proof that arbitrary hardware is
correct.

The safe `Cpu<D>`, `InFlight<D>`, and `Receiving<D>` wrappers are deliberately opaque. Upper
layers cannot recover a raw `Device`, a DMA channel, an address, or a resource
release operation from any capability. `arm` consumes CPU authority and
performs the paired setup as one lower-layer action. `poll_dma` retains the
same lease on Pending and restores CPU authority only after request shutdown
and DMA quiescence; its error and Drop paths retain the same cleanup duty.
This is the useful memory-safety boundary: an error in a safe upper-layer
protocol sequence cannot split the pair, enable a request independently, or
return a caller buffer while DMA may still access it.

`Status`, `Command`, `WakeOn`, `Error`, and `Step` are the small value
vocabulary needed by that boundary. A snapshot reports facts without choosing
I2C fault priority; it does not manufacture a durable FIFO-readiness permit.
The physical binding owns atomic DMA arming, request ordering, interrupt wake
registration, quiescence, and ordinary shutdown. Memory safety must hold even
if START/STOP are requested in a semantically wrong order. A forgotten future
retains its static source and resources, but need not run cleanup or progress.

`DmaResult`/`ReceiveResult` and the opaque capability completion aliases share
`Step<L, C>`: Pending carries the retained lease, Complete carries the restored
owner, and Failed carries the error. RX completion's owner is a tuple containing
the controller and exclusive static buffer. `Step::map` transforms only the
active variant and does not manufacture completion evidence; the audited
physical binding still establishes successful quiescence before constructing it.

The architecture note numbers this boundary as **L2**:

| Layer | Owner |
| --- | --- |
| L1 | PAC register definitions and bit/command encodings |
| L2 | This `Device` contract, opaque capabilities, and the trusted physical HAL |
| L3 | Safe named operations in [`i2c-one-shot-vocabulary`](../i2c-one-shot-vocabulary/README.md) |
| L4 | Unsafe-free typed protocol in [`i2c-one-shot-core`](../i2c-one-shot-core/README.md) and its transaction wrapper |

These are responsibilities, not extra runtime engines. The three small
packages express the L2/L3/L4 source boundaries; the physical HAL remains the
implementation of L2.

The package uses `#![deny(unsafe_code)]` except for the identified `Device`
declaration. That source rule does not make the physical HAL or trusted
dependencies unsafe-free, isolate an electrically live peripheral, prove Drop
runs, or establish runtime protocol correctness. Tests of real adapter effect
ordering and hardware behavior remain necessary.
