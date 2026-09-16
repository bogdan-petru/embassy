use driver_contracts::{contracts, report, validate, validate_sources};

#[test]
fn both_instances_validate_and_export_from_rust() {
    for id in ["i2c", "uart"] {
        let contract = contracts::get(id).unwrap();
        validate(&contract).unwrap();
        let json = serde_json::to_value(&contract).unwrap();
        assert_eq!(json["id"], id);
        assert!(!report(&contract).contains("Evidence entries passed"));
    }
}

#[test]
fn missing_lifecycle_obligations_fail_closed() {
    for field in [
        "owns",
        "effect",
        "pending",
        "wake",
        "completion",
        "error",
        "cancel",
        "forget",
    ] {
        let mut contract = contracts::get("i2c").unwrap();
        let op = &mut contract.operations[1];
        match field {
            "owns" => op.owns = "",
            "effect" => op.effect = "",
            "pending" => op.pending = "",
            "wake" => op.wake = "",
            "completion" => op.completion = "",
            "error" => op.error = "",
            "cancel" => op.cancel = "",
            "forget" => op.forget = "",
            _ => unreachable!(),
        }
        assert!(validate(&contract).unwrap_err().contains(field));
    }
}

#[test]
fn unknown_states_hooks_and_evidence_are_rejected() {
    let mut contract = contracts::get("i2c").unwrap();
    contract.operations[0].to = "Imaginary";
    assert!(validate(&contract).is_err());
    let mut contract = contracts::get("i2c").unwrap();
    contract.operations[0].hooks.push("unreviewed_reset");
    assert!(validate(&contract).is_err());
    let mut contract = contracts::get("uart").unwrap();
    contract.operations[0].evidence.clear();
    assert!(validate(&contract).is_err());
}

#[test]
fn modes_and_transition_targets_cannot_be_broadened() {
    let mut contract = contracts::get("i2c").unwrap();
    contract.registers[1].direct[0].kind = "write";
    assert!(validate(&contract).unwrap_err().contains("exceeds"));
    let mut contract = contracts::get("i2c").unwrap();
    contract.registers[3].direct[0].target = Some("Unknown");
    assert!(validate(&contract).unwrap_err().contains("unresolved transition"));
    let mut contract = contracts::get("i2c").unwrap();
    contract.registers[1].writable = true;
    assert!(validate(&contract).unwrap_err().contains("recorder permissions"));
}

#[test]
fn retained_shared_reads_are_not_permitted_for_cpu_writable_fields() {
    let mut contract = contracts::get("uart").unwrap();
    contract.registers[2].mode = "ReadPureWrite";
    contract.registers[2].writable = true;
    contract.registers[2].direct[0].kind = "read_shared";
    assert!(validate(&contract).unwrap_err().contains("exceeds"));
}

#[test]
fn duplicate_registers_states_and_methods_fail() {
    let mut contract = contracts::get("i2c").unwrap();
    contract.registers.push(contract.registers[0].clone());
    assert!(validate(&contract).unwrap_err().contains("duplicate register"));
    let mut contract = contracts::get("i2c").unwrap();
    contract.states.push("Enabled");
    assert!(validate(&contract).unwrap_err().contains("duplicate state"));
    let mut contract = contracts::get("uart").unwrap();
    let duplicate = contract.registers[2].direct[0].clone();
    contract.registers[2].direct.push(duplicate);
    assert!(validate(&contract).unwrap_err().contains("duplicate direct operation"));
}

#[test]
fn source_inventory_rejects_escapes_missing_files_and_stale_anchors() {
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .ancestors()
        .nth(3)
        .unwrap();
    let contract = contracts::get("i2c").unwrap();
    validate_sources(&contract, root).unwrap();
    for source in ["../outside.rs", "/outside.rs", "missing-contract-hook.rs"] {
        let mut changed = contract.clone();
        changed.hooks[0].source = source;
        assert!(validate_sources(&changed, root).unwrap_err().contains("source"));
    }
    let mut changed = contract;
    changed.hooks[0].anchor = "fn this_hook_does_not_exist(";
    assert!(validate_sources(&changed, root).unwrap_err().contains("anchor"));
}
