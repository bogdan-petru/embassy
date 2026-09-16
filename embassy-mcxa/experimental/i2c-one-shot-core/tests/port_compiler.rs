//! Compile-only controls for the public, owned MCXA port bundle. No fixture is
//! linked or executed; checking the HAL's host graph does not operate hardware.
#![cfg(not(miri))]
#![forbid(unsafe_code)]

use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::{SystemTime, UNIX_EPOCH};

use serde_json::Value;

const HEADER: &str = "#![forbid(unsafe_code)]\n#![allow(dead_code, unused_imports)]\nuse embassy_mcxa::i2c::one_shot::{Error, McxaI2cPort, I2c};\n";
const MARKER: &str = "/*PRIMARY*/";

struct Case {
    name: &'static str,
    body: &'static str,
    error: Option<(&'static str, &'static str)>,
}

const CASES: &[Case] = &[
    Case {
        name: "positive-owned-port",
        body: "fn construct(port: McxaI2cPort) -> Result<I2c, Error> {\n    let _clock_hz = port.clock_hz();\n    I2c::new(port)\n}\n",
        error: None,
    },
    Case {
        name: "port-clone",
        body: "fn wrong(port: McxaI2cPort) {\n    let _ = port./*PRIMARY*/clone();\n}\n",
        error: Some(("E0599", "clone")),
    },
    Case {
        name: "port-consumed-twice",
        body: "fn wrong(port: McxaI2cPort) {\n    let _ = I2c::new(port);\n    let _ = I2c::new(/*PRIMARY*/port);\n}\n",
        error: Some(("E0382", "port")),
    },
    Case {
        name: "port-effects-private",
        body: "fn wrong(port: McxaI2cPort) {\n    let _ = port./*PRIMARY*/effects;\n}\n",
        error: Some(("E0616", "effects")),
    },
];

#[test]
fn owned_soc_port_compiles_and_rejects_exact_ownership_bypasses() {
    let core = Path::new(env!("CARGO_MANIFEST_DIR"));
    let hal = core.ancestors().nth(2).expect("HAL directory");
    let repo = hal.parent().expect("repository directory");
    let target = repo.join("examples/mcxa5xx/target/i2c-port-pilot/port-compiler-build");
    let run = std::env::temp_dir().join(format!(
        "i2c-port-compiler-{}-{}",
        std::process::id(),
        SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_nanos()
    ));
    fs::create_dir(&run).unwrap();
    println!("Preserved port fixtures and diagnostics: {}", run.display());
    fs::write(
        run.join("flags.log"),
        format!(
            "RUSTFLAGS={:?}\nCARGO_ENCODED_RUSTFLAGS={:?}\n",
            std::env::var_os("RUSTFLAGS"),
            std::env::var_os("CARGO_ENCODED_RUSTFLAGS")
        ),
    )
    .unwrap();

    let cargo = Path::new(env!("CARGO"));
    let rustc = std::env::var_os("RUSTC")
        .map(PathBuf::from)
        .unwrap_or_else(|| cargo.with_file_name(format!("rustc{}", std::env::consts::EXE_SUFFIX)));
    let version = Command::new(&rustc)
        .arg("-vV")
        .output()
        .expect("read test toolchain version");
    assert!(version.status.success());
    fs::write(run.join("rustc-version.log"), &version.stdout).unwrap();
    let version = String::from_utf8(version.stdout).unwrap();
    let host = version
        .lines()
        .find_map(|line| line.strip_prefix("host: "))
        .expect("host triple");

    for case in CASES {
        let probe = run.join(case.name);
        fs::create_dir_all(probe.join("src")).unwrap();
        let source = format!("{HEADER}{}", case.body.replace(MARKER, ""));
        fs::write(probe.join("src/lib.rs"), &source).unwrap();
        fs::write(
            probe.join("Cargo.toml"),
            format!(
                "[package]\nname = \"i2c-port-probe-{}\"\nversion = \"0.0.0\"\nedition = \"2024\"\n\n[workspace]\n\n[dependencies]\nembassy-mcxa = {{ path = {}, default-features = false, features = [\"mcxa5xx\", \"i2c-one-shot\", \"test\"] }}\n",
                case.name,
                serde_json::to_string(hal.to_str().unwrap()).unwrap()
            ),
        )
        .unwrap();
        // This is cargo CHECK, never a build/run. Test feature enables the
        // HAL's host compilation path without a firmware MMIO/linker provider.
        // Preserve actual CI flags rather than hiding lint/feature failures.
        let output = Command::new(cargo)
            .current_dir(&probe)
            .env("RUSTC", &rustc)
            .args([
                "check",
                "--offline",
                "--message-format=json",
                "--target",
                host,
                "--target-dir",
            ])
            .arg(&target)
            .output()
            .expect("check port fixture without linking or running it");
        fs::write(probe.join("diagnostics.jsonl"), &output.stdout).unwrap();
        fs::write(probe.join("stderr.log"), &output.stderr).unwrap();
        fs::write(probe.join("exit.log"), format!("{}\n", output.status)).unwrap();
        let diagnostics: Vec<Value> = String::from_utf8(output.stdout)
            .unwrap()
            .lines()
            .map(|line| serde_json::from_str::<Value>(line).expect("Cargo JSON diagnostic"))
            .filter(|message| message["reason"] == "compiler-message" && message["message"]["level"] == "error")
            .map(|message| message["message"].clone())
            .collect();
        if let Some((code, primary)) = case.error {
            assert!(
                !output.status.success(),
                "{} unexpectedly compiled; {}",
                case.name,
                probe.display()
            );
            assert_eq!(
                diagnostics.len(),
                1,
                "{}: unrelated diagnostics: {diagnostics:?}; {}",
                case.name,
                probe.display()
            );
            let diagnostic = &diagnostics[0];
            assert_eq!(diagnostic["code"]["code"], code, "{}: {diagnostic}", case.name);
            let start = HEADER.len() + case.body.find(MARKER).unwrap();
            assert_eq!(&source[start..start + primary.len()], primary);
            assert!(
                diagnostic["spans"].as_array().unwrap().iter().any(|span| {
                    span["is_primary"] == true
                        && span["file_name"]
                            .as_str()
                            .is_some_and(|name| name.replace('\\', "/") == "src/lib.rs")
                        && span["byte_start"] == start
                        && span["byte_end"] == start + primary.len()
                }),
                "{}: wrong primary source span: {diagnostic}; {}",
                case.name,
                probe.display()
            );
        } else {
            assert!(
                output.status.success(),
                "positive port fixture failed: {diagnostics:?}; {}",
                probe.display()
            );
            assert!(diagnostics.is_empty());
        }
    }
}
