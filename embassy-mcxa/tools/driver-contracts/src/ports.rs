//! Port descriptions extend the operation contract, not its generated test oracle.
//! These are reviewed answers to a fixed set of porting questions. Validation
//! checks completeness/references; it cannot establish silicon compatibility.

use std::collections::BTreeSet;

use serde::Serialize;

use crate::{Contract, Reference, unique};

#[derive(Clone, Debug, Serialize)]
pub struct Port {
    pub id: &'static str,
    pub soc_scope: &'static str,
    pub service: &'static str,
    /// Name a reviewed version/selector, or explicitly state that revision is
    /// unqualified. A shared peripheral name is not a compatibility claim.
    pub ip_revision: &'static str,
    pub compatibility_limit: &'static str,
    /// Existing code anchors for PAC selection, generation and SoC binding.
    pub provenance: Vec<Reference>,
    pub hooks: Vec<&'static str>,
    pub capabilities: Vec<Capability>,
    pub obligations: Vec<Obligation>,
}

#[derive(Clone, Debug, Serialize)]
pub struct Capability {
    pub id: &'static str,
    pub supported: bool,
    /// An unsupported entry explains the rejection/omission; never a default.
    pub constraint: &'static str,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Concern {
    ClockPower,
    PinsInterrupts,
    DmaPairing,
    DmaCompletion,
    WireCompletion,
    ResetShutdown,
    BufferLifetime,
    DmaReachability,
    CacheCoherency,
}

impl Concern {
    pub const ALL: [Self; 9] = [
        Self::ClockPower,
        Self::PinsInterrupts,
        Self::DmaPairing,
        Self::DmaCompletion,
        Self::WireCompletion,
        Self::ResetShutdown,
        Self::BufferLifetime,
        Self::DmaReachability,
        Self::CacheCoherency,
    ];
}

#[derive(Clone, Debug, Serialize)]
pub struct Obligation {
    pub concern: Concern,
    pub requirement: &'static str,
    /// References the contract's evidence inventory, including explicit assumed
    /// entries. An identified test is not a claim that it passed on this port.
    pub evidence: Vec<&'static str>,
}

pub(crate) fn validate(ports: &[Port], hooks: &BTreeSet<&str>, evidence: &BTreeSet<&str>) -> Result<(), String> {
    unique(ports.iter().map(|p| p.id), "port")?;
    for port in ports {
        if [port.soc_scope, port.service, port.ip_revision, port.compatibility_limit]
            .iter()
            .any(|s| s.trim().is_empty())
            || port.provenance.is_empty()
            || port.hooks.is_empty()
            || port.capabilities.is_empty()
        {
            return Err(format!("incomplete port {}", port.id));
        }
        unique(port.provenance.iter().map(|p| p.id), "port provenance")?;
        unique(port.capabilities.iter().map(|c| c.id), "port capability")?;
        if port
            .provenance
            .iter()
            .any(|p| p.source.is_empty() || p.anchor.is_empty())
            || port.hooks.iter().any(|h| !hooks.contains(h))
            || port.capabilities.iter().any(|c| c.constraint.trim().is_empty())
        {
            return Err(format!("unresolved provenance/hooks/capabilities for port {}", port.id));
        }
        let mut concerns = BTreeSet::new();
        for item in &port.obligations {
            if !concerns.insert(item.concern)
                || item.requirement.trim().is_empty()
                || item.evidence.is_empty()
                || item.evidence.iter().any(|e| !evidence.contains(e))
            {
                return Err(format!(
                    "incomplete or duplicate {:?} obligation for port {}",
                    item.concern, port.id
                ));
            }
        }
        for concern in Concern::ALL {
            if !concerns.contains(&concern) {
                return Err(format!("port {} omits {concern:?}", port.id));
            }
        }
    }
    Ok(())
}

pub(crate) fn report(contract: &Contract) -> String {
    let mut text = String::new();
    for port in &contract.ports {
        text.push_str(&format!(
            "\n## Port: {}\n\n- SoC scope: {}\n- Service: {}\n- IP selection: {}\n- Compatibility limit: {}\n\n",
            port.id, port.soc_scope, port.service, port.ip_revision, port.compatibility_limit,
        ));
        for source in &port.provenance {
            text.push_str(&format!("- {}: `{}` (`{}`)\n", source.id, source.source, source.anchor));
        }
        text.push_str("\n| Capability | Supported | Constraint |\n| --- | --- | --- |\n");
        for capability in &port.capabilities {
            text.push_str(&format!(
                "| {} | {} | {} |\n",
                capability.id, capability.supported, capability.constraint
            ));
        }
        text.push_str(
            "\n| Port obligation | Required behavior | Evidence inventory (not run results) |\n| --- | --- | --- |\n",
        );
        for item in &port.obligations {
            let evidence: Vec<_> = item
                .evidence
                .iter()
                .map(|id| {
                    let kind = contract
                        .evidence
                        .iter()
                        .find(|e| e.id == *id)
                        .map_or("unresolved", |e| e.kind);
                    format!("{id} ({kind})")
                })
                .collect();
            text.push_str(&format!(
                "| {:?} | {} | {} |\n",
                item.concern,
                item.requirement,
                evidence.join(", ")
            ));
        }
        text.push_str("\nA valid description is not a qualified port. Test outcomes and hardware limits belong in the run record; assumed rows remain assumptions.\n");
    }
    text
}
