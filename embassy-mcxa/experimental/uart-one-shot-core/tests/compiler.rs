//! Compile the public ownership API in isolated packages. These checks spawn
//! Cargo and therefore run in the native suite, independently of Miri.
#![cfg(not(miri))]
#![forbid(unsafe_code)]

use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::{SystemTime, UNIX_EPOCH};

use serde_json::Value;

const HEADER: &str = "#![forbid(unsafe_code)]\n#![allow(dead_code, unused_imports)]\nuse uart_one_shot_core::{Backend, Error, Idle, Streaming, DmaQuiescent, WireComplete};\n";
const MARKER: &str = "/*PRIMARY*/";

struct Case {
    name: &'static str,
    body: &'static str,
    error: Option<(&'static str, &'static str)>,
}

const CASES: &[Case] = &[
    Case {
        name: "positive-sequence",
        body: "static BYTES: [u8; 1] = [7];\nasync fn sequence<B: Backend>(backend: B) -> Result<(), Error> {\n    let streaming: Streaming<B> = Idle::new(backend).transmit(&BYTES)?;\n    let drained: DmaQuiescent<B> = streaming.drain().await?;\n    let complete: WireComplete<B> = drained.complete().await?;\n    complete.finish();\n    Ok(())\n}\n",
        error: None,
    },
    Case {
        name: "streaming-complete",
        body: "fn wrong<B: Backend>(streaming: Streaming<B>) {\n    let _ = streaming./*PRIMARY*/complete();\n}\n",
        error: Some(("E0599", "complete")),
    },
    Case {
        name: "streaming-clone",
        body: "fn wrong<B: Backend>(streaming: Streaming<B>) {\n    let _ = streaming./*PRIMARY*/clone();\n}\n",
        error: Some(("E0599", "clone")),
    },
    Case {
        name: "finish-twice",
        body: "fn wrong<B: Backend>(complete: WireComplete<B>) {\n    complete.finish();\n    /*PRIMARY*/complete.finish();\n}\n",
        error: Some(("E0382", "complete")),
    },
    Case {
        name: "stack-source",
        body: "fn wrong<B: Backend>(backend: B) {\n    let bytes = [7_u8; 1];\n    let _ = Idle::new(backend).transmit(/*PRIMARY*/&bytes);\n}\n",
        error: Some(("E0597", "&bytes")),
    },
    Case {
        name: "forbid-unsafe-control",
        body: "fn wrong(pointer: *const u8) -> u8 {\n    /*PRIMARY*/unsafe { *pointer }\n}\n",
        error: Some(("unsafe_code", "unsafe { *pointer }")),
    },
];

#[test]
fn public_ownership_contract_compiles_and_rejects_exact_misuses() {
    let core = Path::new(env!("CARGO_MANIFEST_DIR"));
    let target = core
        .ancestors()
        .nth(3)
        .unwrap()
        .join("examples/mcxa5xx/target/uart-contract-pilot/compiler-build");
    let run = std::env::temp_dir().join(format!(
        "uart-contract-compiler-{}-{}",
        std::process::id(),
        SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_nanos()
    ));
    fs::create_dir(&run).unwrap();
    println!("Preserved compiler fixtures and diagnostics: {}", run.display());
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
        .expect("read the test toolchain's rustc version");
    assert!(version.status.success());
    fs::write(run.join("rustc-version.log"), &version.stdout).unwrap();
    let version = String::from_utf8(version.stdout).unwrap();
    let host = version
        .lines()
        .find_map(|line| line.strip_prefix("host: "))
        .expect("rustc host triple");

    for case in CASES {
        let probe = run.join(case.name);
        fs::create_dir_all(probe.join("src")).unwrap();
        let source = format!("{HEADER}{}", case.body.replace(MARKER, ""));
        fs::write(probe.join("src/lib.rs"), &source).unwrap();
        fs::write(
            probe.join("Cargo.toml"),
            format!(
                "[package]\nname = \"uart-probe-{}\"\nversion = \"0.0.0\"\nedition = \"2024\"\n\n[workspace]\n\n[dependencies]\nuart-one-shot-core = {{ path = {} }}\n",
                case.name,
                serde_json::to_string(core.to_str().unwrap()).unwrap()
            ),
        ).unwrap();
        // Preserve CI flags, including -Dwarnings. The unsafe-code case must
        // fail even if an inherited cap-lints option would suppress the lint.
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
            .expect("run Cargo check without executing fixture code");
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
                "positive sequence failed: {diagnostics:?}; {}",
                probe.display()
            );
            assert!(diagnostics.is_empty());
        }
    }
}
