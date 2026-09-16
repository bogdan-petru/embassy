# Shared example bodies

This `no_std`, dependency-free, unsafe-forbidden crate is application/test
logic, not another HAL. The UART sender requests one static 16-byte write. Its
callback is the already-constructed driver's consuming async operation, so the
driver retains its deadline, DMA ownership and cleanup responsibilities.

The receiver body requires exact bytes within 20 seconds, then checks for extra
bytes and peripheral errors for 20 milliseconds. A board supplies a nonblocking
read callback and a monotonic microsecond clock. Empty FIFO is `Ok(None)`;
hardware errors are errors, never evidence to retry the transfer into a pass.

Board-specific binaries own clock initialization, pins, driver construction,
interrupt bindings, startup coordination, logs and the final process verdict.
The MCXA receiver remains an independent peer using its existing UART driver;
this crate does not prescribe how another SoC implements those operations.

Run the independent host acceptance fixtures from the repository root:

```text
cargo test --manifest-path examples/nxp-shared/Cargo.toml --target x86_64-pc-windows-msvc
```

These fixtures check the example's payload, error classification and deadlines.
They do not establish DMA quiescence, clock correctness or UART wire timing.
