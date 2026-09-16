# One contract, two small peripheral instances

This host-only tool contains the **reviewed policy inputs** for I2C and UART.
The Rust files in `src/contracts/` are the only maintained contract manifests.
JSON and Markdown are exported views, not independently edited sources.

The first reusable unit is an operation contract, not a register map:

- **Authority:** what mapping, channel, pins and memory it owns.
- **Effect:** the hardware action it requests.
- **Pending:** what remains owned, including any intermediate register state.
- **Wake:** events and recheck obligations needed to make progress.
- **Completion:** the observed condition that permits the next owner.
- **Error, cancellation and forgetting:** three different cleanup/lifetime cases.

The contracts also contain two **port descriptions**: `mcxa-uart-short-tx` and
`mcxa-i2c-short-transfer`. They are part of the same Rust inputs, not second
manifests. Each records its exact service, locked PAC/layout selection and MCXA
binding, plus supported and deliberately unsupported capabilities. Neither
calls the shared MCXA PAC selector a silicon revision or claims cross-family
compatibility. I2C retains its distinct exclusive-RX and repeated-START/STOP
requirements; it does not copy UART's transmit-complete condition.

`Operation::from/to` name register-authority states before the operation and
after its successful completion; for DMA, the intermediate `Streaming` lease
is described in `pending`. Direct register transitions separately describe the
instantaneous `Enabled -> Streaming` arm. Neither declaration executes the
protocol or manufactures its success evidence.

## What is reused now

Both one-shot drivers use the same Abacus safe-MMIO backend. Their Rust
register declarations specify allowed state/access pairs. This host tool checks
those declarations against the independent register/operation contract; it does
not generate driver source.

The Abacus backend rejects invalid field modes, duplicate state operations and
undeclared states. It emits private offsets, recorder metadata, owned handles
and state-specific accessors. PAC layout and direction checks execute/compile
with the HAL. Read purity remains a reviewed assertion, not an inference from
PAC read/write bits.

Reset confirmation, the private ControlIo adapters, DMA arm/quiescence and
restoration are reviewed companion code. Their sequences have not changed.
See [the integration](../../../examples/mcxa5xx/one-shot/ABACUS-INTEGRATION.md)
for the precise scope and the local adaptation's upstream provenance.

## Commands

From the repository root (the normal pinned Rust toolchain applies):

```text
cargo test --locked --manifest-path embassy-mcxa/tools/driver-contracts/Cargo.toml
cargo run --locked --manifest-path embassy-mcxa/tools/driver-contracts/Cargo.toml -- export i2c
cargo run --locked --manifest-path embassy-mcxa/tools/driver-contracts/Cargo.toml -- report uart
cargo run --locked --manifest-path embassy-mcxa/tools/driver-contracts/Cargo.toml -- check i2c .
cargo run --locked --manifest-path embassy-mcxa/tools/driver-contracts/Cargo.toml -- check uart .
cargo test --locked --manifest-path embassy-mcxa/tools/driver-contracts/Cargo.toml --test abacus_surface
cargo test --locked --manifest-path embassy-mcxa/experimental/abacus-registers/Cargo.toml
```

The optional repository path checks that evidence files and hook anchors exist.
It does **not** establish that the referenced hook meets the stated obligation,
that a referenced test passed, or that the hardware behaves like its model.
The exporter returns JSON to stdout; redirect it to an artifact if needed.
No JSON file needs to be checked in or maintained alongside the Rust input.

`report uart` and `report i2c` include each port's capability table and obligation
ledger; `check uart .` and `check i2c .` also check provenance paths/anchors.
Schema version 1 includes `ports` metadata and independent `registers`/`states`
expectations for the Abacus declaration checker. Port descriptions do not
generate a new expected-results file or alter transfer behavior.

The tool's lockfile is tracked independently of the HAL lockfile. Neither the
contract tool nor its `serde` dependencies belongs in a firmware dependency graph.

The host package also applies a bounded AST/dependency policy to the three UART
upper libraries. Its explicit source inventories reject hidden modules, macros,
unknown attributes, conditional lint weakening and build/source-generation
paths. Only the interface's body-free `unsafe Device` declaration receives an
allowance; core/vocabulary require root `forbid(unsafe_code)`. Inline negative
controls challenge the policy itself. This is not a general Rust security
checker or a substitute for compiler rejection under the actual CI flags.

## Recipe for a third IP

1. Define the externally promised service and the boundaries of the first example.
2. Describe each operation's ownership and eight obligations in a Rust contract.
3. Select the minimal PAC fields and states needed to perform those operations.
4. Implement and audit the hardware-specific hooks; explicitly map eligible
   control fields, then declare the Abacus register layout and permitted accesses.
   Reuse bounded confirmation without assuming the IP's reset recipe is identical.
   Add the new IP to the exporter deliberately.
5. Write **independent** expectations: compiler rejection controls, a protocol
   reference model and an effect seam beneath the actual adapter.
6. Qualify hardware assumptions on a real target. Keep assumptions, compiler
   checks, host results and hardware results as distinct evidence categories.

## Recipe for another SoC with the same IP

Keep the shared protocol/vocabulary and example body unchanged if their promised
service is still correct. Write a new owned SoC binding and a new `Port` value in
the existing IP contract. Do not substitute a chip name in the old description
and assume register compatibility. Each port must answer all nine questions:

| Concern | What the binding must establish |
| --- | --- |
| Clock/power | How actual frequency is obtained and resources remain live/stable through cleanup or a forgotten future |
| Pins/interrupts | Instance-correct pins and both interrupt bindings; arming and recheck rules |
| DMA pairing | Matching endpoint/request/channel without separately supplied arbitrary addresses |
| DMA completion | What establishes FIFO-fed success, request shutdown and quiescence |
| Wire completion | The separate observation proving the last bit has left the transmitter |
| Reset/shutdown | Confirmed authority transitions and fail-closed behavior, including unbounded assumptions |
| Buffer lifetime | Storage remains valid even when an in-flight future is forgotten |
| DMA reachability | Which source regions the bus master can actually read |
| Cache coherency | Why CPU/DMA views agree, or which ownership/maintenance steps are required |

The schema rejects missing answers, missing evidence references and missing
provenance. Unsupported capabilities require explicit explanations. This is a
**review completeness gate**, not an executable hardware compatibility decision:
the actual binding API must reject unsupported operations/configurations too.
Natural-language answers cannot prove that a clock stays enabled or a memory
region is DMA-accessible. In particular, a static Rust slice establishes
lifetime, not reachability or coherency. Current unqualified memory/cache
questions are retained as `assumed`, not promoted to host-tested guarantees.

Use the same independently maintained compiler, protocol, effect and hardware
acceptance expectations for the new binding. A valid report inventories evidence
sources; it does not say those tests passed for that port. Record actual runs,
board identities and timing qualifications separately. A receiver matching all
bytes does not establish that software returned only after the final wire bit;
that timing claim needs its own observation. Reusing code across two MCXA boards
does not establish cross-family portability.

The tests' expected I2C/UART method inventories are intentionally not generated
from the contract under test. A contract edit must not silently edit its own
oracle. Structural validation catches missing obligations and broken references;
it cannot prove natural-language obligations, clock timing or DMA memory safety.

Types constrain permitted operations. Effect tests check their sequencing.
Hardware tests challenge the assumptions underneath both. None replaces the others.
