use driver_contracts::{ControlRegister, contracts, validate};

type Mapping = (&'static str, &'static str, Option<&'static str>, Option<&'static str>);

// Independent expected adapter surface. Do not generate this oracle from the
// contract or emitter: it must reject an accidental expansion of either.
const I2C: &[Mapping] = &[
    ("Control", "mcr", Some("ReadWrite"), Some("WriteOnly")),
    ("InterruptEnable", "mier", None, Some("WriteOnly")),
    ("DmaEnable", "mder", None, Some("WriteOnly")),
];
const UART: &[Mapping] = &[
    ("Global", "global", Some("ReadWrite"), Some("ReadWrite")),
    ("Baud", "baud", Some("ReadWrite"), Some("ReadWrite")),
    ("Ctrl", "ctrl", Some("ReadWrite"), Some("ReadWrite")),
    ("Stat", "stat", Some("ReadOnly"), None),
];

#[test]
fn exact_control_surfaces_and_seven_eligible_fields_are_reviewed_independently() {
    for (id, module, mappings, eligible, definition) in [
        (
            "i2c",
            "crate::i2c::reset_sequence",
            I2C,
            &["mcr", "mier", "mder"][..],
            "embassy-mcxa/src/i2c/reset_sequence.rs",
        ),
        (
            "uart",
            "super::reset_sequence",
            UART,
            &["global", "baud", "stat", "ctrl"][..],
            "embassy-mcxa/src/lpuart/one_shot/hardware/reset_sequence.rs",
        ),
    ] {
        let contract = contracts::get(id).unwrap();
        validate(&contract).unwrap();
        assert_eq!(contract.schema_version, 1, "additive metadata, not a second manifest");
        assert_eq!(contract.control_io.module, module);
        let actual: Vec<_> = contract
            .control_io
            .registers
            .iter()
            .map(|mapping| (mapping.variant, mapping.register, mapping.read, mapping.write))
            .collect();
        assert_eq!(actual, mappings);
        assert_eq!(
            contract
                .registers
                .iter()
                .filter(|register| register.control_io)
                .map(|register| register.name)
                .collect::<Vec<_>>(),
            eligible
        );
        let hook = contract
            .hooks
            .iter()
            .find(|hook| hook.id == "control_io_definition")
            .unwrap();
        assert_eq!(hook.source, definition);
        assert_eq!(hook.anchor, "pub(super) trait RegisterIo {");
        let exported = serde_json::to_value(&contract).unwrap();
        assert_eq!(exported["control_io"]["module"], module);
        assert_eq!(
            exported["control_io"]["registers"].as_array().unwrap().len(),
            mappings.len()
        );
    }
}

#[test]
fn arbitrary_paths_expressions_and_enum_patterns_are_not_emitter_input() {
    for module in [
        "",
        "reset_sequence",
        "::crate::i2c::reset_sequence",
        "crate::i2c::reset_sequence::<u32>",
        "crate::i2c::{reset_sequence}",
        "crate::super::reset_sequence",
        "crate :: i2c :: reset_sequence",
        "super::../reset_sequence",
        "super::",
        "super::super",
        "self::reset_sequence",
        "crate::reset_sequence; include!(\"other.rs\")",
    ] {
        let mut contract = contracts::get("i2c").unwrap();
        contract.control_io.module = module;
        assert!(validate(&contract).is_err(), "accepted module: {module}");
    }
    for variant in [
        "",
        "Config(_) ",
        "Control::<u32>",
        "Control, DmaEnable",
        "r#Control",
        "Self",
    ] {
        let mut contract = contracts::get("i2c").unwrap();
        contract.control_io.registers[0].variant = variant;
        assert!(validate(&contract).is_err(), "accepted variant: {variant}");
    }
}

#[test]
fn duplicate_unknown_empty_and_unlocated_control_mappings_fail_closed() {
    let mut contract = contracts::get("i2c").unwrap();
    contract.control_io.registers[1].variant = "Control";
    assert!(validate(&contract).unwrap_err().contains("duplicate"));
    let mut contract = contracts::get("i2c").unwrap();
    contract.control_io.registers[1].register = "mcr";
    assert!(validate(&contract).unwrap_err().contains("duplicate"));
    let mut contract = contracts::get("i2c").unwrap();
    contract.control_io.registers[0].register = "missing_register";
    assert!(validate(&contract).unwrap_err().contains("unknown"));
    let mut contract = contracts::get("i2c").unwrap();
    contract.control_io.registers.clear();
    assert!(validate(&contract).is_err());
    let mut contract = contracts::get("i2c").unwrap();
    contract.control_io.registers[0].read = None;
    contract.control_io.registers[0].write = None;
    assert!(validate(&contract).unwrap_err().contains("no operation"));
    let mut contract = contracts::get("i2c").unwrap();
    contract.hooks.retain(|hook| hook.id != "control_io_definition");
    assert!(validate(&contract).unwrap_err().contains("source-inventory"));
}

#[test]
fn fifo_data_and_retained_status_handles_do_not_enter_control_io() {
    for (id, field, read, write) in [
        ("i2c", "mtdr", None, Some("WriteOnly")),
        ("i2c", "mrdr", Some("ReadOnly"), None),
        ("uart", "data", None, Some("WriteOnly")),
    ] {
        let mut contract = contracts::get(id).unwrap();
        contract.control_io.registers.push(ControlRegister {
            variant: "Payload",
            register: field,
            read,
            write,
        });
        assert!(validate(&contract).unwrap_err().contains("not eligible"));
    }
    for field in ["msr", "mfsr"] {
        let mut contract = contracts::get("i2c").unwrap();
        // Even a mistaken eligibility edit cannot overlap retained shared
        // handles. Other side-effect semantics remain reviewed input.
        contract
            .registers
            .iter_mut()
            .find(|register| register.name == field)
            .unwrap()
            .control_io = true;
        contract.control_io.registers.push(ControlRegister {
            variant: "Status",
            register: field,
            read: Some("ReadOnly"),
            write: None,
        });
        assert!(validate(&contract).unwrap_err().contains("retained shared"));
    }
}

#[test]
fn eligibility_and_mapping_inventory_must_remain_identical() {
    let mut contract = contracts::get("i2c").unwrap();
    contract.registers[0].control_io = false;
    assert!(validate(&contract).unwrap_err().contains("not eligible"));
    let mut contract = contracts::get("i2c").unwrap();
    contract
        .registers
        .iter_mut()
        .find(|register| register.name == "mccr0")
        .unwrap()
        .control_io = true;
    assert!(validate(&contract).unwrap_err().contains("inventory"));
    let mut contract = contracts::get("uart").unwrap();
    contract.control_io.registers.pop();
    assert!(validate(&contract).unwrap_err().contains("inventory"));
}

#[test]
fn operation_kinds_and_field_modes_cannot_be_broadened() {
    for mode in ["WriteOnly", "ReadPure", "ReadPureWrite", "Unknown"] {
        let mut contract = contracts::get("i2c").unwrap();
        contract.control_io.registers[0].read = Some(mode);
        assert!(validate(&contract).is_err(), "accepted read mode: {mode}");
    }
    for mode in ["ReadOnly", "ReadPure", "ReadPureWrite", "Unknown"] {
        let mut contract = contracts::get("i2c").unwrap();
        contract.control_io.registers[0].write = Some(mode);
        assert!(validate(&contract).is_err(), "accepted write mode: {mode}");
    }
    for (read, write) in [
        (Some("ReadWrite"), None),
        (Some("ReadOnly"), Some("WriteOnly")),
        (Some("ReadOnly"), Some("ReadWrite")),
    ] {
        let mut contract = contracts::get("uart").unwrap();
        let stat = contract
            .control_io
            .registers
            .iter_mut()
            .find(|mapping| mapping.variant == "Stat")
            .unwrap();
        stat.read = read;
        stat.write = write;
        assert!(validate(&contract).unwrap_err().contains("exceeds"));
    }
    let mut contract = contracts::get("i2c").unwrap();
    contract.control_io.registers[1].read = Some("ReadOnly");
    assert!(validate(&contract).unwrap_err().contains("exceeds"));
    let mut contract = contracts::get("i2c").unwrap();
    contract.control_io.registers[1].write = Some("ReadWrite");
    assert!(validate(&contract).unwrap_err().contains("exceeds"));
}
