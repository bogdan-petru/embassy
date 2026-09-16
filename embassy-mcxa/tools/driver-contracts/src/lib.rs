//! Reviewed policy, not a protocol prover. Rust is the only maintained input;
//! the JSON and Markdown views are exports of these same values.
#![forbid(unsafe_code)]

use std::collections::BTreeSet;

use serde::Serialize;

pub mod boundary;
pub mod contracts;
pub mod control_io;
pub mod ports;

pub use control_io::{ControlIo, ControlRegister};

#[derive(Clone, Debug, Serialize)]
pub struct Contract {
    pub schema_version: u32,
    pub id: &'static str,
    pub peripheral: &'static str,
    pub owner: &'static str,
    pub states: Vec<&'static str>,
    pub registers: Vec<Register>,
    /// Explicit private reset-I/O mapping, not an inferred reset recipe.
    pub control_io: ControlIo,
    pub operations: Vec<Operation>,
    pub hooks: Vec<Reference>,
    pub evidence: Vec<Evidence>,
    /// Additive inventory: existing IP contracts need not claim a qualified port.
    pub ports: Vec<ports::Port>,
}

#[derive(Clone, Debug, Serialize)]
pub struct Register {
    pub name: &'static str,
    pub mode: &'static str,
    pub readable: bool,
    pub writable: bool,
    /// Reviewed eligibility for the private control adapter. This is not
    /// inferred from PAC access direction or proof of non-destructive reads.
    pub control_io: bool,
    pub direct: Vec<Direct>,
}

#[derive(Clone, Debug, Serialize)]
pub struct Direct {
    pub state: &'static str,
    pub kind: &'static str,
    pub method: &'static str,
    pub target: Option<&'static str>,
    pub hook: Option<&'static str>,
}

impl Direct {
    pub fn access(state: &'static str, kind: &'static str, method: &'static str) -> Self {
        Self {
            state,
            kind,
            method,
            target: None,
            hook: None,
        }
    }

    pub fn transition(state: &'static str, method: &'static str, target: &'static str, hook: &'static str) -> Self {
        Self {
            state,
            kind: "transition",
            method,
            target: Some(target),
            hook: Some(hook),
        }
    }
}

/// All eight obligations are mandatory, including explicit "none" explanations.
/// Text is reviewed intent; validation cannot establish that hardware obeys it.
#[derive(Clone, Debug, Serialize)]
pub struct Operation {
    pub id: &'static str,
    pub from: &'static str,
    pub to: &'static str,
    pub owns: &'static str,
    pub effect: &'static str,
    pub pending: &'static str,
    pub wake: &'static str,
    pub completion: &'static str,
    pub error: &'static str,
    pub cancel: &'static str,
    pub forget: &'static str,
    pub hooks: Vec<&'static str>,
    pub evidence: Vec<&'static str>,
}

#[derive(Clone, Debug, Serialize)]
pub struct Reference {
    pub id: &'static str,
    /// Repository-relative file. An anchor names the reviewed code, not a proof.
    pub source: &'static str,
    pub anchor: &'static str,
}

#[derive(Clone, Debug, Serialize)]
pub struct Evidence {
    pub id: &'static str,
    pub kind: &'static str,
    pub source: &'static str,
    pub claim: &'static str,
}

fn unique<'a>(values: impl Iterator<Item = &'a str>, category: &str) -> Result<BTreeSet<&'a str>, String> {
    let mut result = BTreeSet::new();
    for value in values {
        if value.trim().is_empty() || !result.insert(value) {
            return Err(format!("empty or duplicate {category}: {value:?}"));
        }
    }
    Ok(result)
}

pub fn validate(contract: &Contract) -> Result<(), String> {
    if contract.schema_version != 1
        || contract.id.is_empty()
        || contract.peripheral.is_empty()
        || contract.owner.is_empty()
    {
        return Err("unsupported or incomplete contract header".into());
    }
    let states = unique(contract.states.iter().copied(), "state")?;
    if states.is_empty() || contract.registers.is_empty() || contract.operations.is_empty() {
        return Err("states, registers and operations are required".into());
    }
    unique(contract.registers.iter().map(|r| r.name), "register")?;
    unique(contract.operations.iter().map(|o| o.id), "operation")?;
    let hooks = unique(contract.hooks.iter().map(|h| h.id), "hook")?;
    let evidence = unique(contract.evidence.iter().map(|e| e.id), "evidence")?;
    for hook in &contract.hooks {
        if hook.source.is_empty() || hook.anchor.is_empty() {
            return Err(format!("unlocated hook {}", hook.id));
        }
    }
    for item in &contract.evidence {
        if !["compiler", "host", "hardware", "assumed"].contains(&item.kind)
            || item.source.is_empty()
            || item.claim.is_empty()
        {
            return Err(format!("incomplete evidence {}", item.id));
        }
    }
    let mut direct_names = BTreeSet::new();
    for register in &contract.registers {
        let (readable, writable) = match register.mode {
            "ReadPure" | "ReadOnly" => (true, false),
            "WriteOnly" => (false, true),
            "ReadWrite" | "ReadPureWrite" => (true, true),
            _ => return Err(format!("unknown mode {}", register.mode)),
        };
        if (register.readable, register.writable) != (readable, writable) {
            return Err(format!("recorder permissions disagree with mode for {}", register.name));
        }
        for direct in &register.direct {
            if !states.contains(direct.state)
                || direct.method.is_empty()
                || !direct_names.insert((direct.state, direct.method))
            {
                return Err(format!("unknown state or duplicate direct operation {}", direct.method));
            }
            let legal = match direct.kind {
                "read_shared" => register.mode == "ReadPure",
                "read_exclusive" => ["ReadOnly", "ReadWrite"].contains(&register.mode),
                "write" | "transition" => writable,
                _ => false,
            };
            if !legal {
                return Err(format!("{}: {} exceeds {}", register.name, direct.kind, register.mode));
            }
            if direct.kind == "transition" {
                if !direct.target.is_some_and(|s| states.contains(s)) || !direct.hook.is_some_and(|h| hooks.contains(h))
                {
                    return Err(format!("unresolved transition {}", direct.method));
                }
            } else if direct.target.is_some() || direct.hook.is_some() {
                return Err(format!("non-transition {} carries a transition", direct.method));
            }
        }
    }
    control_io::validate(contract)?;
    for operation in &contract.operations {
        if !states.contains(operation.from) || !states.contains(operation.to) {
            return Err(format!("unknown state for {}", operation.id));
        }
        for (label, value) in [
            ("owns", operation.owns),
            ("effect", operation.effect),
            ("pending", operation.pending),
            ("wake", operation.wake),
            ("completion", operation.completion),
            ("error", operation.error),
            ("cancel", operation.cancel),
            ("forget", operation.forget),
        ] {
            if value.trim().is_empty() {
                return Err(format!("{} omits {label}", operation.id));
            }
        }
        if operation.hooks.is_empty()
            || operation.evidence.is_empty()
            || operation.hooks.iter().any(|h| !hooks.contains(h))
            || operation.evidence.iter().any(|e| !evidence.contains(e))
        {
            return Err(format!("unresolved hooks/evidence for {}", operation.id));
        }
    }
    ports::validate(&contract.ports, &hooks, &evidence)?;
    Ok(())
}

/// Inventory validation only: existence does not verify a hook's implementation.
pub fn validate_sources(contract: &Contract, repo: &std::path::Path) -> Result<(), String> {
    for source in contract
        .hooks
        .iter()
        .map(|h| h.source)
        .chain(contract.evidence.iter().map(|e| e.source))
        .chain(
            contract
                .ports
                .iter()
                .flat_map(|p| p.provenance.iter().map(|r| r.source)),
        )
    {
        let path = std::path::Path::new(source);
        if path.is_absolute()
            || path.components().any(|c| !matches!(c, std::path::Component::Normal(_)))
            || !repo.join(path).is_file()
        {
            return Err(format!("unresolved or out-of-tree source {source}"));
        }
    }
    for hook in contract
        .hooks
        .iter()
        .chain(contract.ports.iter().flat_map(|p| &p.provenance))
    {
        let source = std::fs::read_to_string(repo.join(hook.source)).map_err(|e| e.to_string())?;
        if !source.contains(hook.anchor) {
            return Err(format!("missing hook anchor {}", hook.id));
        }
    }
    Ok(())
}

pub fn report(contract: &Contract) -> String {
    let mut text = format!(
        "# {} operation contract\n\nReviewed policy; not generated proof. JSON is an export, not another input.\n\n| Operation | Ownership/state | Requested effect | Completion evidence |\n| --- | --- | --- | --- |\n",
        contract.id
    );
    for op in &contract.operations {
        text.push_str(&format!(
            "| {} | {} -> {}; {} | {} | {} |\n",
            op.id, op.from, op.to, op.owns, op.effect, op.completion
        ));
    }
    for op in &contract.operations {
        text.push_str(&format!("\n## {} obligations\n\n- Pending: {}\n- Wake: {}\n- Error: {}\n- Cancellation: {}\n- Forgetting: {}\n- Reviewed hooks: {}\n- Evidence inventory: {}\n", op.id, op.pending, op.wake, op.error, op.cancel, op.forget, op.hooks.join(", "), op.evidence.join(", ")));
    }
    text.push_str("\nEvidence entries identify checks/assumptions; this report does not assert they passed.\n");
    text.push_str(&ports::report(contract));
    text
}
