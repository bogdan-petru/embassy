# One combined write/read

The second example writes register address `0x10` to device `0x2a`, sends a
repeated START, receives `[0x31, 0x82, 0xc7, 0x59]`, then closes with STOP.
It uses the same one-shot controller and the same DMA channel in both
directions. The existing full controller/target/DMA implementation is unchanged.

Start with the [controller application](../src/bin/i2c-one-shot-write-read-controller.rs)
and [peer assertions](../src/bin/i2c-one-shot-write-read-target.rs).

```rust
let received = i2c.write_read(0x2a, &[0x10], RX.init([0; 4])).await?;
assert_eq!(received, &[0x31, 0x82, 0xc7, 0x59]);
```

## What owns the receive memory?

The API accepts `&'static mut [u8]`, not a borrowed stack buffer. The consuming
L2 operation holds that actual exclusive reference together with the matching
MRDR/RX-request endpoint and DMA channel. No buffer or raw lease accessor
exists while it is active. Ordinary completion requires requests-off, channel
quiescence and an acquire fence before CPU authority and storage can return.
L4 retains the storage until it additionally observes STOP and controller idle.

Cancellation and errors quiesce and reset locally, but consume the one-shot
buffer: there is no recovery/retry handle in this small API. Forgetting a
future bypasses Drop; it may leave hardware running and leaks its owned static
storage indefinitely. It does **not** allow safe Rust to reuse that storage.
Static lifetime prevents dangling memory; exclusive ownership prevents CPU
aliases. Neither promises cleanup of a deliberately leaked future.

Both lengths are preflighted before START. The scope is 1..=32 bytes per phase,
non-reserved 7-bit addresses, 100 kHz, one 100 ms deadline, and no chunks,
retries, bus-clear or reusable embedded-hal transaction API. The board example
qualifies the one-byte/four-byte case, not every permitted length.

## The important ordering

After TX DMA becomes quiescent, the vocabulary waits for FIFO space without
issuing read commands. L2 rechecks FIFO space and consumes the buffer, then:

1. Prepares the exact RX endpoint/channel and enables DMA IRQ/request.
2. Writes RDDE, then queues START-read and RECEIVE (with no intermediate STOP).
3. Returns the opaque receiving owner, which can only observe, poll or drop.

This is synchronous and ordered, **not interrupt-atomic**. An interrupt may
delay it, but read traffic cannot begin before its DMA destination is armed.
The generated compound transition exposes no general MTDR/MRDR access on
Streaming. A failed RX setup queues neither read command.

## What the hardware test actually checks

The peer requires `WriteStatus::Restarted(1)`. `Stopped(1)` fails even if all
bytes match, so this tests the repeated START rather than assuming it from
matching data. The controller independently checks four received bytes and
STOP/idle completion.

The peer offers eight bytes, including a four-byte spare suffix. This keeps
its TX DMA below major-loop completion so it must observe bus termination.
It requires `EarlyStop(4 | 5)`: the existing target API counts bytes queued
into STDR and can include one stranded byte. This is not a claim that five
bytes reached the controller, nor does `EarlyStop` distinguish STOP from
another START. The controller supplies the final STOP assertion separately.

The first peer version incorrectly required `Complete(4)` from an exactly
four-byte buffer; the actual target returned `NeedMore(4)` while the controller
passed. Those failed runs are preserved, not counted as successful validation.
Using spare capacity makes the intended termination check explicit without
changing the production target driver.

Build both `i2c-one-shot-write-read-controller` and
`i2c-one-shot-write-read-target` with feature `i2c-one-shot`, then use the
existing runner with `-WriteRead` and the two explicit probe IDs. Build/run
the original pair without that switch to restore the write-only example.

## Evidence and limits

[Checkpoint 12](PROGRESS.md) records actual host/board results. The recorder
runs the real adapter and checks RX arm order, request pairing, no launch on
setup failure, pending cancellation, error-before-DONE and quiescence before
memory return. Its DMA seam observes HAL calls, not physical memory writes.
Protocol mocks independently check priority, sequence and the single deadline.

The public API has compile-fail RX stack/forgotten-buffer examples. More exact
diagnostic/location mutations are declared but need a new committed baseline;
the previous mutation run is not evidence for this extension. The Abacus fork
is therefore unused but retained until its ported rejections are validated.

[Kani harnesses](../../../embassy-mcxa/experimental/i2c-one-shot-proofs/README.md)
passed for the real synchronous fault/preflight decisions at `b984630ed`, not
for the async DMA implementation. Checkpoint 13 records the three named
results and the Linux/64-bit verification scope. Connected-board
RX cancellation/fault injection and exhaustive electrical/interrupt schedules
remain unqualified; successful transfer and host cleanup tests do not imply them.
