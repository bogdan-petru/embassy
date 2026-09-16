# Compile-only PAC bypass control

Do not execute this crate or include it in a firmware application. It deliberately
accesses an unowned peripheral. `cargo check` emits metadata, not running code.

From the Embassy repository root:

```powershell
cargo check --manifest-path embassy-mcxa/experimental/i2c-one-shot-core/tests/fixtures/pac-bypass-control/Cargo.toml --target thumbv8m.main-none-eabihf
cargo test --manifest-path embassy-mcxa/experimental/i2c-one-shot-core/Cargo.toml --locked --test wake_only_irq_policy
```

The first command must succeed even though the fixture forbids unsafe code: the
current PAC's register write and FIFO pop are safe Rust methods. The second
checks the actual IRQ path and rejects these same operations when injected into
its parsed source. This distinguishes **source-policy rejection** from Rust
compiler rejection. It does not claim PAC access is inherently unsound or that
this policy controls arbitrary HAL code, dependencies, compiler flags, or build
scripts. The real IRQ body and wake ordering also remain covered by effect tests.

The existing isolated mutation runner can additionally mutate the actual handler
at a clean committed baseline and require `ONE_SHOT_WAKE_ONLY_IRQ_POLICY`. A dirty
checkout must not bypass that runner's preflight guard. The in-memory mutation
controls run on the current source without altering it.
