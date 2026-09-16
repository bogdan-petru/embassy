//! Reviewed mapping for a generated, private register-effect adapter only.
//! Reset timing, confirmation, failure handling and state reconstruction stay
//! in handwritten code. Eligibility is semantic input, not inferred safety.

use std::collections::BTreeSet;

use serde::Serialize;

use crate::Contract;

#[derive(Clone, Debug, Serialize)]
pub struct ControlIo {
    /// Fixed Rust path to the reviewed Register/RegisterIo definitions.
    pub module: &'static str,
    pub registers: Vec<ControlRegister>,
}

#[derive(Clone, Debug, Serialize)]
pub struct ControlRegister {
    pub variant: &'static str,
    pub register: &'static str,
    /// Field-sized, exclusive safe-mmio mode used by this operation.
    pub read: Option<&'static str>,
    pub write: Option<&'static str>,
}

fn identifier(value: &str) -> bool {
    syn::parse_str::<syn::Ident>(value).is_ok()
        && value
            .bytes()
            .enumerate()
            .all(|(index, byte)| byte.is_ascii_alphabetic() || byte == b'_' || index > 0 && byte.is_ascii_digit())
}

fn module_path(value: &str) -> bool {
    let segments: Vec<_> = value.split("::").collect();
    if segments.len() < 2 || !matches!(segments[0], "crate" | "super") {
        return false;
    }
    let mut names_started = false;
    for segment in &segments[1..] {
        if *segment == "super" && segments[0] == "super" && !names_started {
            continue;
        }
        if !identifier(segment) {
            return false;
        }
        names_started = true;
    }
    names_started
}

pub(crate) fn validate(contract: &Contract) -> Result<(), String> {
    let control = &contract.control_io;
    if !module_path(control.module) || control.registers.is_empty() {
        return Err("invalid or empty control-I/O module/mapping".into());
    }
    if !contract.hooks.iter().any(|hook| hook.id == "control_io_definition") {
        return Err("control-I/O definitions need a reviewed source-inventory hook".into());
    }
    let mut variants = BTreeSet::new();
    let mut fields = BTreeSet::new();
    for mapping in &control.registers {
        if !identifier(mapping.variant) || !variants.insert(mapping.variant) {
            return Err("invalid or duplicate control-I/O variant".into());
        }
        if !fields.insert(mapping.register) {
            return Err("duplicate control-I/O register".into());
        }
        let register = contract
            .registers
            .iter()
            .find(|register| register.name == mapping.register)
            .ok_or("unknown control-I/O register")?;
        if !register.control_io {
            return Err(format!("{} is not eligible for control I/O", mapping.register));
        }
        if register.mode == "ReadPure" || register.direct.iter().any(|direct| direct.kind == "read_shared") {
            return Err("retained shared register cannot enter exclusive control I/O".into());
        }
        if mapping.read.is_none() && mapping.write.is_none() {
            return Err("control-I/O entry grants no operation".into());
        }
        for (mode, reading) in [(mapping.read, true), (mapping.write, false)] {
            let Some(mode) = mode else { continue };
            let legal_kind = if reading {
                matches!(mode, "ReadOnly" | "ReadWrite")
            } else {
                matches!(mode, "WriteOnly" | "ReadWrite")
            };
            let (needs_read, needs_write) = match mode {
                "ReadOnly" => (true, false),
                "WriteOnly" => (false, true),
                "ReadWrite" => (true, true),
                _ => return Err("unreviewed control-I/O mode".into()),
            };
            if !legal_kind || needs_read && !register.readable || needs_write && !register.writable {
                return Err(format!(
                    "{} control-I/O mode exceeds register permissions",
                    mapping.register
                ));
            }
        }
    }
    // A newly eligible field must have a reviewed mapping, not merely sit in
    // an unused whitelist waiting for a later expansion of the adapter.
    let eligible: BTreeSet<_> = contract
        .registers
        .iter()
        .filter(|register| register.control_io)
        .map(|register| register.name)
        .collect();
    if eligible != fields {
        return Err("control-I/O eligibility and mapping inventory disagree".into());
    }
    Ok(())
}
