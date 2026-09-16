# The same porting recipe, with I2C's own obligations

The [UART reference](PORTING.md) separates a promised service from its owned
SoC binding. This I2C example applies that recipe without copying UART's wire
completion condition or expanding the existing transfer behavior.

See [I2C-PORTING-VALIDATION.md](I2C-PORTING-VALIDATION.md) for the dated build,
host/compiler and connected-board results and their explicit limits.

The service remains one consuming async DMA **write**, or one **write/repeated
START/read/STOP**. It accepts non-reserved 7-bit addresses, 1 through 32 bytes
per phase, and only a **12 MHz functional clock for 100 kHz I2C**. There are no
chunks, retries, bus-clear operation, standalone read, or production-driver
changes. The whole transaction retains its original 100 ms cooperative deadline;
synchronous cleanup is not preempted by that timer.

## Start with the small application

After the board initializes clocks and binds both LPI2C3 and DMA0_CH0 handlers:

```rust,ignore
let port = McxaI2cPort::acquire(
    p.LPI2C3, p.P3_21, p.P3_20, p.DMA0_CH0, Irqs, ClockConfig::default(),
)?;
let bus = I2c::new(port)?;
nxp_shared_examples::i2c::write_read(RX.init([0; 4]), |address, register, rx| {
    bus.write_read(address, register, rx)
}).await?;
```

`RX` is a board-owned `StaticCell<[u8; 4]>`. See the complete
[controller binary](../src/bin/i2c-one-shot-write-read-controller.rs) for imports,
clock initialization and bindings. The default port configuration uses FroLfDiv
without a peripheral divider. Other selections are accepted only when the HAL's
derived functional clock is still exactly 12 MHz; this is **not** UART's
variable-frequency baud solver. The application never supplies a frequency
integer, loose FIFO address or DMA request number.

| Read next | Responsibility |
| --- | --- |
| [Shared I2C example bodies](../../nxp-shared/src/i2c.rs) | Common payload, combined operation and independent peer expectations |
| [MCXA target-peer adapter](../src/bin/i2c_one_shot/peer.rs) | Translate the existing target HAL's actual outcomes, without inventing termination |
| [L4 protocol](../../../embassy-mcxa/experimental/i2c-one-shot-core/src/protocol.rs) | Typed transaction sequence and fault priority |
| [L3 vocabulary](../../../embassy-mcxa/experimental/i2c-one-shot-vocabulary/src/lib.rs) | Safe named operations over opaque L2 capabilities |
| [L2 register adapter](../../../embassy-mcxa/src/i2c/one_shot/session/vocabulary/registers.rs) | Confirmed reset/enable, command ordering, DMA leases and cleanup |
| [MCXA binding](../../../embassy-mcxa/src/i2c/one_shot/session/vocabulary/registers/binding.rs) | Owned clocks/power, SCL/SDA pins, paired endpoints/requests, DMA and IRQ integration |

The four layers remain: L1 PAC, L2 trusted physical capabilities, L3 vocabulary,
L4 protocol. The L3/L4 libraries remain unchanged and SoC-free, with unsafe
forbidden. The HAL's session wrapper supplies the deadline. L2 still implements
the audited unsafe `Device` contract; moving acquisition does not make it
unsafe-free or prove its hardware assumptions.

## Test the bus event, not just the bytes

The common `TargetPeer` interface is for the **existing independent test peer**,
not a replacement target driver. Its board adapter preserves `Stopped`,
`Restarted` and `BufferFull` as different outcomes; unknown outcomes and driver
errors fail the check.

For the combined transfer, `serve_write_read` requires **`Restarted(1)`** after
receiving register address `0x10`. `Stopped(1)` fails even if every byte matches.
The two-byte receive scratch deliberately prevents a one-byte DMA-full result
from masquerading as a repeated START. Only then may the peer accept the read
request and reply. Its spare reply suffix requires the previously characterized
`EarlyStop(4 | 5)` outcome, which counts queued target bytes, not exact wire
progress. The controller independently checks exactly four received bytes and
STOP/idle completion. See [WRITE-READ.md](WRITE-READ.md) for that distinction.

The shared tests deliberately inject a matching-data `Stopped(1)` case and
reject it. A new board adapter must not translate STOP or DMA fullness into
`Restarted` just to satisfy those tests. Reuse the expectations, not a fabricated
event label; hardware qualification still depends on the target HAL's report.

## Receive memory and shutdown

TX uses immutable static storage. RX transfers the actual **exclusive static
mutable reference** into the private DMA lease. CPU access cannot resume until
requests are off, the channel is quiescent and the acquire fence has run; L4
retains the buffer until final STOP/idle too. Cancellation or error consumes
that ownership without a reclaim API. Forgetting a future may leave DMA active
and retains the buffer indefinitely; it does not grant safe CPU access to it.
Static lifetime is not a proof of DMA reachability, and a fence is not cache
maintenance.

One intentional difference from UART: **the I2C peripheral clock gate remains
enabled after Drop, as it did before this refactor**. The final
[release helper](../../../embassy-mcxa/src/i2c/one_shot/session/vocabulary/registers/release_sequence.rs)
orders IRQ-off, DMA quiescence, unpend, pins-off, then **WakeGuard release**.
This permits passive MCR/MDER cleanup audits without re-enabling/resetting the
peripheral and hiding a defect. UART's binding gates its peripheral off; I2C
does not adopt that policy merely for structural symmetry. Neither binding
exposes clock-reconfiguration authority. A wake guard inhibits deep sleep but
cannot stop foreign PAC writes or another core changing root clocks.

## What the next SoC port must answer

The existing [I2C Rust contract](../../../embassy-mcxa/tools/driver-contracts/src/contracts/i2c.rs)
now carries `mcxa-i2c-short-transfer`, not another independently maintained
manifest. It requires all nine porting answers:

| Obligation | I2C-specific answer to preserve or requalify |
| --- | --- |
| Clock/power | Valid 12 MHz profile; retain guard through cleanup/forgetting; explicit post-Drop gate policy |
| Pins/interrupts | Matching SCL/SDA and both IRQ bindings; wake-only ISR plus register/arm/recheck |
| DMA pairing | MTDR/TX and MRDR/RX endpoint/request pairs from the same instance; switch direction while quiescent |
| DMA completion | Faults precede success; requests-off/quiesce before CPU or RX authority returns |
| Wire completion | Actual repeated START without intermediate STOP; final fault-free STOP-seen plus controller-idle |
| Reset/shutdown | Confirmed authority transitions; local reset is not recovery of a peer-held bus |
| Buffer lifetime | Static TX, exclusive static RX and no memory return from forgotten/cancelled work |
| DMA reachability | Qualify every source/destination memory region for the selected DMA master |
| Cache coherency | Provide correct ownership/maintenance for a cached port or reject that memory configuration |

The report checks completeness and references, not truth or test results. Real
effect tests observe the adapter's MMIO and HAL-call ordering; they do not prove
physical eDMA progress. Port compiler controls challenge cloning, double use
and extraction; they do not establish electrical correctness.

`DeviceHardware::new` still consumes **concrete `McxaI2cPort`** and the selected
MCXA PAC layout. This is not a cross-family port or a fully generic L2 backend. For the
next SoC, first match the IP revision/register semantics, then extract the
appropriate owned-resource interface and implement that binding. Keep the
shared protocol, bodies and independent expectations where the service still
applies; run the same obligations on the new hardware. Two MCXA boards alone
cannot establish cross-family portability.

From the repository root, useful focused checks are:

```text
cargo run --locked --manifest-path embassy-mcxa/tools/driver-contracts/Cargo.toml -- report i2c
cargo run --locked --manifest-path embassy-mcxa/tools/driver-contracts/Cargo.toml -- check i2c .
cargo test --locked --manifest-path embassy-mcxa/experimental/i2c-one-shot-core/Cargo.toml
cargo test --manifest-path examples/nxp-shared/Cargo.toml --test i2c
cargo test --locked --manifest-path embassy-mcxa/tools/driver-contracts/Cargo.toml --test abacus_surface
```

Build the four existing I2C example bins with `i2c-one-shot`. Use the existing
[two-board runner](run-two-board.ps1), with `-WriteRead` for the combined pair.
Keep actual run results, source/build identity and qualification limits in the
validation record; this guide intentionally declares no new passed test count.
