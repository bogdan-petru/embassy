use crate::ports::{Capability, Concern, Obligation, Port};
use crate::{Contract, ControlIo, ControlRegister, Direct as D, Evidence, Operation, Reference, Register};

const MMIO: &str = "embassy-mcxa/src/i2c/one_shot/session/vocabulary/registers/mmio.rs";
const CONTROL: &str = "embassy-mcxa/src/i2c/one_shot/session/vocabulary/registers/control_boundary.rs";
const DMA: &str = "embassy-mcxa/src/i2c/one_shot/session/vocabulary/registers/dma_boundary.rs";
const ADAPTER: &str = "embassy-mcxa/src/i2c/one_shot/session/vocabulary/registers.rs";
const BINDING: &str = "embassy-mcxa/src/i2c/one_shot/session/vocabulary/registers/binding.rs";

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
    let read = |name| {
        vec![
            D::access("Enabled", "read_shared", name),
            D::access("Streaming", "read_shared", name),
        ]
    };
    Contract {
        schema_version: 1,
        id: "i2c",
        peripheral: "LPI2C",
        owner: "Lpi2cRegisters",
        ports: vec![mcxa_port()],
        states: vec!["Unconfirmed", "Off", "Enabled", "Streaming"],
        registers: vec![
            control_register("mcr", "ReadWrite", true, true, vec![]),
            register("msr", "ReadPure", true, false, read("read_msr")),
            control_register(
                "mier",
                "WriteOnly",
                false,
                true,
                vec![
                    D::access("Off", "write", "write_mier"),
                    D::access("Enabled", "write", "write_mier"),
                    D::access("Streaming", "write", "write_mier"),
                ],
            ),
            control_register(
                "mder",
                "WriteOnly",
                false,
                true,
                vec![
                    D::transition("Enabled", "into_arm_dma", "Streaming", "arm_dma_word"),
                    D::transition("Enabled", "into_arm_rx_dma", "Streaming", "arm_receive_dma_word"),
                ],
            ),
            register(
                "mcfgr1",
                "WriteOnly",
                false,
                true,
                vec![D::access("Off", "write", "write_mcfgr1")],
            ),
            register(
                "mccr0",
                "WriteOnly",
                false,
                true,
                vec![D::access("Off", "write", "write_mccr0")],
            ),
            register("mfsr", "ReadPure", true, false, read("read_mfsr")),
            register(
                "mtdr",
                "WriteOnly",
                false,
                true,
                vec![D::access("Enabled", "write", "write_mtdr")],
            ),
            register(
                "mrdr",
                "ReadOnly",
                true,
                false,
                vec![D::access("Enabled", "read_exclusive", "read_mrdr")],
            ),
        ],
        control_io: ControlIo {
            module: "crate::i2c::reset_sequence",
            registers: vec![
                ControlRegister {
                    variant: "Control",
                    register: "mcr",
                    read: Some("ReadWrite"),
                    write: Some("WriteOnly"),
                },
                ControlRegister {
                    variant: "InterruptEnable",
                    register: "mier",
                    read: None,
                    write: Some("WriteOnly"),
                },
                ControlRegister {
                    variant: "DmaEnable",
                    register: "mder",
                    read: None,
                    write: Some("WriteOnly"),
                },
            ],
        },
        operations: vec![
            Operation {
                id: "confirmed_setup",
                from: "Unconfirmed",
                to: "Enabled",
                owns: "exclusive controller mapping and its clock/pin ownership",
                effect: "full-word reset assert/release, then configuration and enable",
                pending: "synchronous bounded readback; no enabled owner before confirmation",
                wake: "none: bounded polling, no asynchronous wait",
                completion: "reset release and enable observed in MCR",
                error: "failed handshake attempts reset-held fallback and returns an error",
                cancel: "no suspension point; caller cannot cancel halfway through the call",
                forget: "leaks ownership; does not mint an enabled owner",
                hooks: vec!["reset_confirmed", "enable_confirmed"],
                evidence: vec!["reset_effects", "direct_surface"],
            },
            Operation {
                id: "tx_dma",
                from: "Enabled",
                to: "Enabled",
                owns: "controller, its mapped TX request/channel, and a static immutable source",
                effect: "program DMA before enabling the fixed TDDE request",
                pending: "Streaming lease retains controller/channel/source and denies CPU FIFO writes",
                wake: "DMA completion/error plus armed I2C fault/completion interrupts",
                completion: "all source bytes delivered to FIFO, no error, requests off, channel quiescent; not wire completion",
                error: "classify faults before completion; request-off then quiesce before recovery",
                cancel: "same request-off/quiesce order, then abrupt confirmed reset; no success evidence",
                forget: "static source remains live and inaccessible owned capabilities are leaked, not returned",
                hooks: vec!["arm_dma_word", "tx_lease", "quiesce_with"],
                evidence: vec!["effect_order", "protocol_traces", "static_buffers"],
            },
            Operation {
                id: "rx_dma_repeated_start",
                from: "Enabled",
                to: "Enabled",
                owns: "controller, RX channel and exclusive static mutable receive buffer",
                effect: "RDDE on before the fixed repeated START-read/RECEIVE pair; no intermediate STOP",
                pending: "Streaming lease alone owns the writable buffer; no CPU access",
                wake: "RX DMA completion/error and armed I2C fault/completion interrupts",
                completion: "successful DMA, requests off and channel quiescent before returning the buffer",
                error: "fault precedence before DMA completion; quiesce before releasing any memory authority",
                cancel: "quiesce and reset; do not manufacture a completed receive",
                forget: "owned static mutable storage remains unavailable to the application while DMA may still write",
                hooks: vec!["arm_receive_dma_word", "rx_lease", "quiesce_with"],
                evidence: vec!["effect_order", "protocol_traces", "static_buffers"],
            },
            Operation {
                id: "close_transaction",
                from: "Enabled",
                to: "Enabled",
                owns: "quiescent controller and any received payload until wire completion",
                effect: "enqueue STOP only when FIFO-empty is freshly observed",
                pending: "retain the owner and payload; DMA completion alone does not release either",
                wake: "TX-space while enqueuing, then STOP/fault interrupt; recheck after arming",
                completion: "one fault-free status snapshot has STOP seen and controller idle",
                error: "faults precede completion; separate application bus clear may still be required",
                cancel: "drop retains cleanup authority through STOP wait and resets the local engine",
                forget: "no completion value or buffer is returned; controller resources remain leaked",
                hooks: vec!["observe", "tx_lease"],
                evidence: vec!["protocol_traces", "effect_order"],
            },
        ],
        hooks: vec![
            Reference {
                id: "control_io_definition",
                source: "embassy-mcxa/src/i2c/reset_sequence.rs",
                anchor: "pub(super) trait RegisterIo {",
            },
            Reference {
                id: "arm_dma_word",
                source: MMIO,
                anchor: "fn arm_dma_word()",
            },
            Reference {
                id: "arm_receive_dma_word",
                source: MMIO,
                anchor: "fn into_arm_rx_dma(",
            },
            Reference {
                id: "reset_confirmed",
                source: CONTROL,
                anchor: "fn reset_confirmed(",
            },
            Reference {
                id: "enable_confirmed",
                source: CONTROL,
                anchor: "fn enable_confirmed(",
            },
            Reference {
                id: "quiesce_with",
                source: DMA,
                anchor: "fn quiesce_with<",
            },
            Reference {
                id: "tx_lease",
                source: ADAPTER,
                anchor: "fn poll_dma(",
            },
            Reference {
                id: "rx_lease",
                source: ADAPTER,
                anchor: "fn poll_receive(",
            },
            Reference {
                id: "observe",
                source: ADAPTER,
                anchor: "fn observe<",
            },
        ],
        evidence: vec![
            Evidence {
                id: "direct_surface",
                kind: "host",
                source: "embassy-mcxa/tools/driver-contracts/tests/abacus_surface.rs",
                claim: "independent expected register/state/method inventory",
            },
            Evidence {
                id: "reset_effects",
                kind: "host",
                source: "embassy-mcxa/src/i2c/reset_sequence.rs",
                claim: "actual reset sequence over scripted register effects",
            },
            Evidence {
                id: "effect_order",
                kind: "host",
                source: "embassy-mcxa/src/i2c/one_shot/session/vocabulary/effect_tests.rs",
                claim: "real adapter request-off/quiesce and command effects",
            },
            Evidence {
                id: "protocol_traces",
                kind: "host",
                source: "embassy-mcxa/experimental/i2c-one-shot-core/tests/protocol.rs",
                claim: "protocol ordering, fault precedence and retained owners; not hardware proof",
            },
            Evidence {
                id: "static_buffers",
                kind: "compiler",
                source: "embassy-mcxa/src/i2c/one_shot/mod.rs",
                claim: "public API requires static DMA storage, including forgotten futures",
            },
            Evidence {
                id: "owned_port_compiler",
                kind: "compiler",
                source: "embassy-mcxa/experimental/i2c-one-shot-core/tests/port_compiler.rs",
                claim: "positive public constructor and exact-diagnostic negative controls for clone, reuse and resource extraction; no fixture runs on hardware",
            },
            Evidence {
                id: "wake_only_policy",
                kind: "host",
                source: "embassy-mcxa/experimental/i2c-one-shot-core/tests/wake_only_irq_policy.rs",
                claim: "source AST policy pins the actual relocated ISR and local wake callees; this is not compiler sealing or NVIC hardware proof",
            },
            Evidence {
                id: "resource_binding",
                kind: "assumed",
                source: BINDING,
                claim: "reviewed MCXA resource binding; root clock stability and DMA routing remain SoC obligations, not consequences of a port report",
            },
            Evidence {
                id: "port_release_effects",
                kind: "host",
                source: "embassy-mcxa/src/i2c/one_shot/session/vocabulary/registers/release_sequence.rs",
                claim: "real final resource-release helper orders IRQ shutdown, DMA quiesce, unpend, pins-off and clock-power guard release; the peripheral gate remains enabled as before; this does not prove HAL quiescence internals",
            },
            Evidence {
                id: "dma_memory_assumption",
                kind: "assumed",
                source: "embassy-mcxa/src/dma.rs",
                claim: "TX/RX lifetime and exclusive ownership do not prove bus-master reachability or cache coherency for arbitrary memory regions",
            },
            Evidence {
                id: "board_acceptance",
                kind: "hardware",
                source: "examples/mcxa5xx/one-shot/WRITE-READ.md",
                claim: "located repeated-START/STOP and exact-data qualification procedure/records; not a new-run pass or automatic qualification of another SoC",
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
        id: "mcxa-i2c-short-transfer",
        soc_scope: "MCXA2xx/MCXA5xx build bindings; physical qualification is separately scoped to the FRDM-MCXA577 board pair",
        service: "one consuming 100 kHz controller write or write/repeated-START/read/STOP; 1..=32 bytes per phase, 7-bit address, 12 MHz functional clock",
        ip_revision: "nxp-pac meta_peripherals/mcxa/LPI2C.rs selected by the HAL's direct locked dependency; not a silicon revision assertion",
        compatibility_limit: "nine mapped u32 fields and reviewed MCXA reset/DMA hooks; no cross-family, target-mode or arbitrary clock compatibility is inferred",
        provenance: vec![
            Reference {
                id: "pac_selection",
                source: "embassy-mcxa/tools/gen_lpi2c_regs.py",
                anchor: "def find_pac(",
            },
            Reference {
                id: "i2c_layout_generation",
                source: "embassy-mcxa/src/i2c/one_shot/session/vocabulary/registers/mmio.rs",
                anchor: "process_mmio_block(",
            },
            Reference {
                id: "soc_binding",
                source: BINDING,
                anchor: "pub struct McxaI2cPort",
            },
        ],
        hooks: vec![
            "reset_confirmed",
            "enable_confirmed",
            "arm_dma_word",
            "arm_receive_dma_word",
            "quiesce_with",
            "observe",
        ],
        capabilities: vec![
            Capability {
                id: "short_tx_dma",
                supported: true,
                constraint: "1..=32 immutable static bytes; validate address and length before START",
            },
            Capability {
                id: "short_write_read_dma",
                supported: true,
                constraint: "static TX and exclusively owned static mutable RX; both lengths validated before START; no intermediate STOP",
            },
            Capability {
                id: "standalone_rx",
                supported: false,
                constraint: "no standalone read operation in this service",
            },
            Capability {
                id: "long_transfer",
                supported: false,
                constraint: "no chunking or retry framework; lengths above 32 rejected",
            },
            Capability {
                id: "runtime_reclock",
                supported: false,
                constraint: "only a 12 MHz functional clock is accepted; no clock changes while an owner/lease is live",
            },
            Capability {
                id: "automatic_bus_clear",
                supported: false,
                constraint: "local reset does not free a peer-held bus; reset peer or use a separate qualified recovery procedure",
            },
        ],
        obligations: vec![
            obligation(
                ClockPower,
                "derive and validate 12 MHz from the initialized HAL clock tree; retain the clock-power guard through pending and cleanup, releasing WakeGuard only after DMA quiescence while leaving the gate enabled for passive audit; forgotten owners retain the guard; root-clock/PAC changes remain outside the supported model",
                vec!["owned_port_compiler", "resource_binding", "port_release_effects"],
            ),
            obligation(
                PinsInterrupts,
                "consume matching SCL/SDA pins, instance and DMA channel; require both interrupt bindings; preserve wake-only ISR and arm/register/recheck sequencing",
                vec!["wake_only_policy", "effect_order", "resource_binding"],
            ),
            obligation(
                DmaPairing,
                "derive fixed MTDR-TX and MRDR-RX endpoints/requests from the same consumed instance; retain the one channel with each lease and select the direction only while quiescent",
                vec!["effect_order", "resource_binding"],
            ),
            obligation(
                DmaCompletion,
                "faults precede success; requests off then channel quiescence before restoring CPU authority; RX acquire fence precedes return of exclusive receive storage; DMA success alone is not bus completion",
                vec!["effect_order", "protocol_traces"],
            ),
            obligation(
                WireCompletion,
                "combined transfer must contain repeated START and no intermediate STOP; final success requires one fault-free STOP-seen/controller-idle snapshot after DMA quiescence",
                vec!["protocol_traces", "effect_order", "board_acceptance"],
            ),
            obligation(
                ResetShutdown,
                "confirmed full-word reset/enable grant register authority; cancel/error disable requests and quiesce before abrupt local reset; quiescence assumes DMA progress and local reset cannot guarantee bus release",
                vec![
                    "reset_effects",
                    "effect_order",
                    "resource_binding",
                    "port_release_effects",
                ],
            ),
            obligation(
                BufferLifetime,
                "static immutable TX and owned static mutable RX remain live across forgetting; no RX accessor escapes before quiescence; error/cancel consumes RX ownership without fabricating success",
                vec!["static_buffers", "protocol_traces", "effect_order"],
            ),
            obligation(
                DmaReachability,
                "qualify DMA reads for every TX source region and writes for every RX destination region in the board image; static lifetime alone does not establish reachability",
                vec!["dma_memory_assumption", "board_acceptance"],
            ),
            obligation(
                CacheCoherency,
                "no cache-maintenance strategy is supplied by this pilot; a cached/noncoherent port needs explicit TX clean/RX ownership and invalidate rules or must reject that memory profile; a fence alone is not cache maintenance",
                vec!["dma_memory_assumption"],
            ),
        ],
    }
}
