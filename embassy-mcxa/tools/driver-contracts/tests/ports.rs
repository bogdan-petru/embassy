use driver_contracts::ports::Concern;
use driver_contracts::{contracts, report, validate, validate_sources};

// Deliberately independent of Concern::ALL and the manifest. Removing an
// obligation from both the schema and its instance must not edit this oracle.
const REQUIRED: [Concern; 9] = [
    Concern::ClockPower,
    Concern::PinsInterrupts,
    Concern::DmaPairing,
    Concern::DmaCompletion,
    Concern::WireCompletion,
    Concern::ResetShutdown,
    Concern::BufferLifetime,
    Concern::DmaReachability,
    Concern::CacheCoherency,
];

#[test]
fn uart_reference_port_has_every_required_answer_and_explicit_limits() {
    let contract = contracts::get("uart").unwrap();
    validate(&contract).unwrap();
    assert_eq!(contract.ports.len(), 1);
    let port = &contract.ports[0];
    assert_eq!(port.id, "mcxa-uart-short-tx");
    assert_eq!(port.obligations.len(), REQUIRED.len());
    for concern in REQUIRED {
        assert_eq!(port.obligations.iter().filter(|o| o.concern == concern).count(), 1);
    }
    // These promises intentionally do not follow from the generated surface.
    let actual: Vec<_> = port.capabilities.iter().map(|c| (c.id, c.supported)).collect();
    assert_eq!(
        actual,
        [
            ("short_tx_dma", true),
            ("rx_dma", false),
            ("long_transfer", false),
            ("runtime_reclock", false),
        ]
    );
    let export = serde_json::to_value(&contract).unwrap();
    assert_eq!(export["ports"][0]["id"], "mcxa-uart-short-tx");
    let markdown = report(&contract);
    assert!(markdown.contains("Evidence inventory (not run results)"));
    assert!(markdown.contains("dma_memory_assumption (assumed)"));
    assert!(markdown.contains("not a qualified port"));
}

#[test]
fn each_missing_port_obligation_is_rejected() {
    for id in ["uart", "i2c"] {
        for concern in REQUIRED {
            let mut contract = contracts::get(id).unwrap();
            contract.ports[0].obligations.retain(|o| o.concern != concern);
            let error = validate(&contract).unwrap_err();
            assert!(error.contains(&format!("omits {concern:?}")), "{error}");
        }
    }
}

#[test]
fn duplicate_or_unexplained_port_obligations_are_rejected() {
    let mut contract = contracts::get("uart").unwrap();
    let duplicate = contract.ports[0].obligations[0].clone();
    contract.ports[0].obligations.push(duplicate);
    assert!(validate(&contract).unwrap_err().contains("duplicate"));
    let mut contract = contracts::get("uart").unwrap();
    contract.ports[0].obligations[0].requirement = " ";
    assert!(validate(&contract).unwrap_err().contains("ClockPower"));
}

#[test]
fn port_evidence_cannot_be_missing_or_invented() {
    for evidence in [vec![], vec!["not_a_test"]] {
        let mut contract = contracts::get("uart").unwrap();
        contract.ports[0].obligations[0].evidence = evidence;
        assert!(validate(&contract).unwrap_err().contains("ClockPower"));
    }
    let mut contract = contracts::get("uart").unwrap();
    contract.ports[0].hooks.push("imaginary_dma_stop");
    assert!(validate(&contract).unwrap_err().contains("unresolved"));
}

#[test]
fn missing_capability_constraints_and_revision_selection_are_rejected() {
    let mut contract = contracts::get("uart").unwrap();
    contract.ports[0].capabilities[1].constraint = "";
    assert!(validate(&contract).unwrap_err().contains("capabilities"));
    let mut contract = contracts::get("uart").unwrap();
    contract.ports[0].ip_revision = "";
    assert!(validate(&contract).unwrap_err().contains("incomplete port"));
    let mut contract = contracts::get("uart").unwrap();
    let duplicate = contract.ports[0].clone();
    contract.ports.push(duplicate);
    assert!(validate(&contract).unwrap_err().contains("duplicate port"));
}

#[test]
fn port_provenance_is_checked_and_cannot_escape_the_repository() {
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .ancestors()
        .nth(3)
        .unwrap();
    let contract = contracts::get("uart").unwrap();
    validate_sources(&contract, root).unwrap();
    for source in ["../outside.rs", "/outside.rs", "missing-pac-selection.rs"] {
        let mut changed = contract.clone();
        changed.ports[0].provenance[0].source = source;
        assert!(validate_sources(&changed, root).unwrap_err().contains("source"));
    }
    let mut changed = contract;
    changed.ports[0].provenance[0].anchor = "fn no_such_pac_selector(";
    assert!(validate_sources(&changed, root).unwrap_err().contains("anchor"));
}

#[test]
fn i2c_reference_port_keeps_its_distinct_completion_and_buffer_contract() {
    let contract = contracts::get("i2c").unwrap();
    validate(&contract).unwrap();
    assert_eq!(contract.ports.len(), 1);
    let port = &contract.ports[0];
    assert_eq!(port.id, "mcxa-i2c-short-transfer");
    assert_eq!(port.obligations.len(), REQUIRED.len());
    for concern in REQUIRED {
        assert_eq!(port.obligations.iter().filter(|o| o.concern == concern).count(), 1);
    }
    let actual: Vec<_> = port.capabilities.iter().map(|c| (c.id, c.supported)).collect();
    assert_eq!(
        actual,
        [
            ("short_tx_dma", true),
            ("short_write_read_dma", true),
            ("standalone_rx", false),
            ("long_transfer", false),
            ("runtime_reclock", false),
            ("automatic_bus_clear", false),
        ]
    );
    let markdown = report(&contract);
    assert!(markdown.contains("exclusively owned static mutable RX"));
    assert!(markdown.contains("no intermediate STOP"));
    assert!(markdown.contains("dma_memory_assumption (assumed)"));
}
