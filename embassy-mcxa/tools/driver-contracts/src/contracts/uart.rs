use crate::ports::{Capability, Concern, Obligation, Port};
use crate::{Contract, ControlIo, ControlRegister, Direct as D, Evidence, Operation, Reference, Register};

const CONTROL: &str = "embassy-mcxa/src/lpuart/one_shot/hardware/control_boundary.rs";
const BINDING: &str = "embassy-mcxa/src/lpuart/one_shot/hardware/binding.rs";

pub fn contract() -> Contract {
    let register = |name, mode, readable, writable, direct| Register {
        name,
        mode,
        readable,
        writable,
        control_io: false,
        direct,
    };
    let control_register = |name, mode, readable, writable, direct| {
        let mut value = register(name, mode, readable, writable, direct);
        value.control_io = true;
        value
    };
    Contract {
        schema_version: 1,
        id: "uart",
        peripheral: "LPUART",
        owner: "UartRegisters",
        ports: vec![mcxa_port()],
        states: vec!["Unconfirmed", "Off", "Enabled", "Streaming"],
        registers: vec![
            control_register("global", "ReadWrite", true, true, vec![]),
            control_register("baud", "ReadWrite", true, true, vec![]),
            control_register(
                "stat",
                "ReadOnly",
                true,
                false,
                vec![
                    D::access("Enabled", "read_exclusive", "read_stat"),
                    D::access("Streaming", "read_exclusive", "read_stat"),
                ],
            ),
            control_register("ctrl", "ReadWrite", true, true, vec![]),
            register("data", "WriteOnly", false, true, vec![]),
            register(
                "fifo",
                "WriteOnly",
                false,
                true,
                vec![D::access("Off", "write", "write_fifo")],
            ),
            register(
                "water",
                "WriteOnly",
                false,
                true,
                vec![D::access("Off", "write", "write_water")],
            ),
        ],
        control_io: ControlIo {
            module: "super::reset_sequence",
            registers: vec![
                ControlRegister {
                    variant: "Global",
                    register: "global",
                    read: Some("ReadWrite"),
                    write: Some("ReadWrite"),
                },
                ControlRegister {
                    variant: "Baud",
                    register: "baud",
                    read: Some("ReadWrite"),
                    write: Some("ReadWrite"),
                },
                ControlRegister {
                    variant: "Ctrl",
                    register: "ctrl",
                    read: Some("ReadWrite"),
                    write: Some("ReadWrite"),
                },
                ControlRegister {
                    variant: "Stat",
                    register: "stat",
                    read: Some("ReadOnly"),
                    write: None,
                },
            ],
        },
        operations: vec![
            Operation {
                id: "confirmed_setup",
                from: "Unconfirmed",
                to: "Enabled",
                owns: "exclusive UART mapping, clock and TX pin",
                effect: "assert/release GLOBAL reset, configure fixed short-TX settings, enable transmitter",
                pending: "bounded synchronous readback; enabled owner unavailable until confirmation",
                wake: "none: bounded synchronous setup",
                completion: "reset and transmitter-enable observations confirm the configured state",
                error: "return failure without granting enabled authority",
                cancel: "no await inside setup",
                forget: "leaks resources rather than returning unconfirmed authority",
                hooks: vec!["reset_confirmed", "configure_confirmed"],
                evidence: vec!["register_surface", "reset_effects", "port_timing_effects"],
            },
            Operation {
                id: "tx_dma",
                from: "Enabled",
                to: "Enabled",
                owns: "UART, matching TX DMA channel/request and static immutable source",
                effect: "program channel, then enable BAUD.TDMAE",
                pending: "Streaming lease owns all resources; no CPU DATA writer or arbitrary reset API",
                wake: "DMA completion interrupt and state recheck after waker registration; error-only DMA may await the HAL deadline wake",
                completion: "DMA DONE without error, TDMAE off/readback then channel quiescent; bytes may still be on wire",
                error: "request-off then quiesce before cleanup; no successful DMA evidence",
                cancel: "request-off then quiesce before reset and resource release",
                forget: "static source remains valid and owner/channel remain leaked instead of returning to caller",
                hooks: vec!["into_arm_dma", "disable_dma_and_restore"],
                evidence: vec![
                    "register_surface",
                    "completion_contract",
                    "reset_effects",
                    "port_timing_effects",
                ],
            },
            Operation {
                id: "wire_complete",
                from: "Enabled",
                to: "Enabled",
                owns: "quiescent UART owner until the final stop bit leaves the transmitter",
                effect: "arm transmit-complete interrupt and observe STAT.TC",
                pending: "retain owner; do not equate DMA DONE or DATA space with wire completion",
                wake: "UART transmit-complete interrupt, followed by status recheck",
                completion: "STAT.TC is observed after successful DMA quiescence",
                error: "no DMA-success shortcut; overall deadline belongs to the HAL wrapper",
                cancel: "drop still resets owned UART; do not return a success result",
                forget: "no completion is returned and peripheral remains owned by the forgotten future",
                hooks: vec!["wire_complete"],
                evidence: vec!["completion_contract", "reset_effects", "wire_assumption"],
            },
        ],
        hooks: vec![
            Reference {
                id: "control_io_definition",
                source: "embassy-mcxa/src/lpuart/one_shot/hardware/reset_sequence.rs",
                anchor: "pub(super) trait RegisterIo {",
            },
            Reference {
                id: "reset_confirmed",
                source: CONTROL,
                anchor: "fn reset_confirmed(",
            },
            Reference {
                id: "configure_confirmed",
                source: CONTROL,
                anchor: "fn configure_confirmed(",
            },
            Reference {
                id: "into_arm_dma",
                source: CONTROL,
                anchor: "fn into_arm_dma(",
            },
            Reference {
                id: "disable_dma_and_restore",
                source: CONTROL,
                anchor: "fn disable_dma_and_restore(",
            },
            Reference {
                id: "wire_complete",
                source: CONTROL,
                anchor: "fn wire_complete(",
            },
        ],
        evidence: vec![
            Evidence {
                id: "register_surface",
                kind: "host",
                source: "embassy-mcxa/tools/driver-contracts/tests/abacus_surface.rs",
                claim: "independent UART register/state permissions and PAC layout checks",
            },
            Evidence {
                id: "completion_contract",
                kind: "host",
                source: "embassy-mcxa/experimental/uart-one-shot-core/tests/acceptance.rs",
                claim: "DMA completion cannot substitute for transmit-complete observation",
            },
            Evidence {
                id: "reset_effects",
                kind: "host",
                source: "embassy-mcxa/src/lpuart/one_shot/hardware/reset_sequence.rs",
                claim: "real reset/request-off/TC arming sequence over scripted register effects",
            },
            Evidence {
                id: "port_release_effects",
                kind: "host",
                source: "embassy-mcxa/src/lpuart/one_shot/hardware/release_sequence.rs",
                claim: "real resource-release helper orders IRQ shutdown, DMA quiesce, unpend, pin-off and clock release; not a proof of the HAL quiesce implementation",
            },
            Evidence {
                id: "port_timing_effects",
                kind: "host",
                source: "embassy-mcxa/src/lpuart/one_shot/hardware.rs",
                claim: "real generated MMIO arm/restore over host memory preserves selected port timing and disables requests before the quiesce callback; no physical DMA or reset-side-effect model",
            },
            Evidence {
                id: "wire_assumption",
                kind: "assumed",
                source: CONTROL,
                claim: "STAT.TC meaning, clock correctness and external peer behavior require hardware validation",
            },
            Evidence {
                id: "resource_binding",
                kind: "assumed",
                source: BINDING,
                claim: "reviewed owned-resource binding; singleton, clock stability and SoC DMA routing assumptions are not proved by this inventory",
            },
            Evidence {
                id: "dma_memory_assumption",
                kind: "assumed",
                source: "embassy-mcxa/src/dma.rs",
                claim: "static lifetime does not establish DMA reachability or coherency; the port must qualify its actual memory/cache configuration",
            },
            Evidence {
                id: "board_acceptance",
                kind: "hardware",
                source: "examples/mcxa5xx/one-shot/UART-PILOT.md",
                claim: "located two-board qualification procedure and historical records; not proof of a new run, all instances, or TC-to-pin timing",
            },
        ],
    }
}

fn mcxa_port() -> Port {
    use Concern::*;
    let obligation = |concern, requirement, evidence| Obligation {
        concern,
        requirement,
        evidence,
    };
    Port {
        id: "mcxa-uart-short-tx",
        soc_scope: "MCXA2xx/MCXA5xx build bindings; hardware qualification is separately scoped to FRDM-MCXA577 LPUART5",
        service: "one consuming async DMA TX of 1..=32 bytes, 115200 baud, 8N1; return only after wire completion",
        ip_revision: "nxp-pac meta_peripherals/mcxa/LPUART.rs selected by the HAL's direct locked dependency; not a silicon revision assertion",
        compatibility_limit: "seven mapped u32 fields and reviewed MCXA hooks only; no cross-family or Flexcomm compatibility or FIFO depth is inferred",
        provenance: vec![
            Reference {
                id: "pac_selection",
                source: "embassy-mcxa/tools/gen_lpi2c_regs.py",
                anchor: "def find_pac(",
            },
            Reference {
                id: "uart_layout_generation",
                source: "embassy-mcxa/src/lpuart/one_shot/hardware/mmio.rs",
                anchor: "process_mmio_block(",
            },
            Reference {
                id: "soc_binding",
                source: BINDING,
                anchor: "pub struct McxaUartPort",
            },
        ],
        hooks: vec![
            "reset_confirmed",
            "configure_confirmed",
            "into_arm_dma",
            "disable_dma_and_restore",
            "wire_complete",
        ],
        capabilities: vec![
            Capability {
                id: "short_tx_dma",
                supported: true,
                constraint: "1..=32 immutable static bytes; fixed 115200 8N1; reject unsupported length before arm",
            },
            Capability {
                id: "rx_dma",
                supported: false,
                constraint: "no receive operation or writable application buffer in this port",
            },
            Capability {
                id: "long_transfer",
                supported: false,
                constraint: "no chunking or retries; the shared core rejects lengths above 32",
            },
            Capability {
                id: "runtime_reclock",
                supported: false,
                constraint: "clock configuration must remain stable while any CPU or streaming owner is live",
            },
        ],
        obligations: vec![
            obligation(
                ClockPower,
                "SoC binding owns the peripheral clock/power lifetime and supplies its actual frequency; retain its guard through pending, cleanup and forgetting; no gate or reclock behind the owner",
                vec!["resource_binding", "port_release_effects"],
            ),
            obligation(
                PinsInterrupts,
                "consume the matching TX pin and UART singleton; require both UART and DMA interrupt bindings; arm/recheck around the real wait cells; keep UART ISR wake-only",
                vec!["resource_binding", "reset_effects"],
            ),
            obligation(
                DmaPairing,
                "derive DATA destination and TX request from the same sealed instance, retain the uniquely owned channel in the lease, and enable requests only after TCD setup",
                vec!["resource_binding"],
            ),
            obligation(
                DmaCompletion,
                "DONE without error is only FIFO-fed evidence; TDMAE off/readback, channel quiescence and final error recheck precede CPU authority restoration",
                vec!["completion_contract", "reset_effects", "port_timing_effects"],
            ),
            obligation(
                WireCompletion,
                "after successful DMA quiescence, register waiter, arm TCIE/NVIC and observe fresh STAT.TC; exact receiver bytes alone do not prove return followed the final transmitted bit",
                vec![
                    "completion_contract",
                    "reset_effects",
                    "wire_assumption",
                    "board_acceptance",
                ],
            ),
            obligation(
                ResetShutdown,
                "confirmed reset and enable gate usable authority; cancellation disables peripheral requests then quiesces before reset/release; quiescence assumes hardware progress and is not independently time-bounded",
                vec![
                    "reset_effects",
                    "resource_binding",
                    "port_release_effects",
                    "port_timing_effects",
                ],
            ),
            obligation(
                BufferLifetime,
                "immutable static source remains valid even if the future is forgotten; resources remain retained; cancellation is not successful completion",
                vec!["completion_contract", "resource_binding"],
            ),
            obligation(
                DmaReachability,
                "qualify every source memory region used by the board image for eDMA reads; a static Rust slice proves lifetime, not bus-master accessibility; arbitrary mappings remain unqualified",
                vec!["dma_memory_assumption", "board_acceptance"],
            ),
            obligation(
                CacheCoherency,
                "no cache-maintenance operation is provided by this pilot; qualify the actual memory/cache configuration separately; a cached or incoherent new port needs explicit ownership/maintenance before DMA arm",
                vec!["dma_memory_assumption"],
            ),
        ],
    }
}
