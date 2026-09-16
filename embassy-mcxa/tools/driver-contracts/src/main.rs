use driver_contracts::{contracts, report, validate, validate_sources};

fn run() -> Result<(), String> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    let [action, ip, rest @ ..] = args.as_slice() else {
        return Err("usage: driver-contracts export|report|check i2c|uart [repository-root]".into());
    };
    if rest.len() > 1 {
        return Err("expected at most one repository-root argument".into());
    }
    let contract = contracts::get(ip)?;
    validate(&contract)?;
    if let Some(root) = rest.first() {
        validate_sources(&contract, std::path::Path::new(root))?;
    }
    match action.as_str() {
        "export" => println!(
            "{}",
            serde_json::to_string_pretty(&contract).map_err(|e| e.to_string())?
        ),
        "report" => print!("{}", report(&contract)),
        "check" => println!(
            "{} contract valid ({} registers, {} operations, {} port descriptions; not qualification results)",
            contract.id,
            contract.registers.len(),
            contract.operations.len(),
            contract.ports.len()
        ),
        _ => return Err("expected export, report or check".into()),
    }
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("{error}");
        std::process::exit(1);
    }
}
