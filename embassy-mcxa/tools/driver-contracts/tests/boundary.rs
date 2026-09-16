use std::collections::BTreeMap;
use std::path::Path;
use std::process::Command;

use driver_contracts::boundary::{UART_LIBRARIES, check_graph, check_package, check_sources, read_sources};
use serde_json::Value;

fn root() -> &'static Path {
    Path::new(env!("CARGO_MANIFEST_DIR")).ancestors().nth(3).unwrap()
}
fn metadata(directory: &Path) -> Value {
    let output = Command::new("cargo")
        .args([
            "metadata",
            "--locked",
            "--no-deps",
            "--format-version",
            "1",
            "--manifest-path",
        ])
        .arg(directory.join("Cargo.toml"))
        .output()
        .unwrap();
    assert!(output.status.success(), "{}", String::from_utf8_lossy(&output.stderr));
    let value: Value = serde_json::from_slice(&output.stdout).unwrap();
    value["packages"]
        .as_array()
        .unwrap()
        .iter()
        .find(|p| {
            p["manifest_path"].as_str().unwrap().replace('\\', "/")
                == directory.join("Cargo.toml").to_string_lossy().replace('\\', "/")
        })
        .unwrap()
        .clone()
}

#[test]
fn uart_upper_libraries_preserve_the_bounded_trust_boundary() {
    for library in UART_LIBRARIES {
        let directory = root().join("embassy-mcxa/experimental").join(library.name);
        check_sources(library, &read_sources(&directory).unwrap()).unwrap();
        check_package(library, &directory, &metadata(&directory)).unwrap();
    }
}

#[test]
fn lint_weakening_expansion_and_hidden_unsafe_fail_the_policy() {
    let library = &UART_LIBRARIES[0];
    let original = read_sources(&root().join("embassy-mcxa/experimental").join(library.name)).unwrap();
    for addition in [
        "#[allow(unsafe_code)] fn hidden() {}",
        "#[expect(unsafe_code)] fn hidden() {}",
        "#[cfg_attr(feature=\"defmt\", allow(unsafe_code))] fn hidden() {}",
        "include!(\"hidden.rs\");",
        "fn hidden() { unsafe {} }",
        "mod hidden { fn f() {} }",
        "#[path=\"other.rs\"] mod hidden;",
        "macro_rules! hidden { () => { unsafe {} }; }",
    ] {
        let mut changed = original.clone();
        changed.get_mut("src/lib.rs").unwrap().push_str(addition);
        assert!(check_sources(library, &changed).is_err(), "accepted {addition}");
    }
    let mut changed = original;
    changed.insert("src/forgotten.rs".into(), "fn hidden() {}".into());
    assert!(check_sources(library, &changed).is_err());
}

#[test]
fn trait_allowance_cannot_cover_a_default_body_or_second_unsafe_item() {
    let library = &UART_LIBRARIES[2];
    let original = read_sources(&root().join("embassy-mcxa/experimental").join(library.name)).unwrap();
    let mutate = |needle: &str, replacement: &str| -> BTreeMap<String, String> {
        let mut changed = original.clone();
        let source = changed.get_mut("src/lib.rs").unwrap();
        assert_eq!(source.matches(needle).count(), 1);
        *source = source.replace(needle, replacement);
        changed
    };
    assert!(
        check_sources(
            library,
            &mutate("type InFlight;", "type InFlight; fn bad() { unsafe {} }")
        )
        .is_err()
    );
    assert!(
        check_sources(
            library,
            &mutate(
                "pub unsafe trait Device",
                "#[expect(unsafe_code)] pub unsafe trait Device"
            )
        )
        .is_err()
    );
    assert!(
        check_sources(
            library,
            &mutate(
                "mod capabilities;",
                "mod capabilities; #[allow(unsafe_code)] unsafe trait Other {} "
            )
        )
        .is_err()
    );
}

#[test]
fn metadata_controls_reject_dependency_and_source_generation_drift() {
    let library = &UART_LIBRARIES[0];
    let directory = root().join("embassy-mcxa/experimental").join(library.name);
    let original = metadata(&directory);
    for key in ["name", "kind", "target"] {
        let mut changed = original.clone();
        changed["dependencies"][0][key] = Value::String(
            match key {
                "name" => "nxp-pac",
                "kind" => "build",
                _ => "cfg(target_arch = \"arm\")",
            }
            .into(),
        );
        assert!(check_package(library, &directory, &changed).is_err());
    }
    let mut changed = original;
    changed["targets"][0]["kind"] = serde_json::json!(["custom-build"]);
    assert!(check_package(library, &directory, &changed).is_err());
}

fn cargo_output(directory: &Path, arguments: &[&str]) -> String {
    let output = Command::new("cargo")
        .args(arguments)
        .current_dir(directory)
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "cargo {arguments:?}: {}",
        String::from_utf8_lossy(&output.stderr)
    );
    String::from_utf8(output.stdout).unwrap()
}

#[test]
fn resolved_normal_build_closures_exclude_hardware_tools_and_unreviewed_formatter_sources() {
    let rustc = Command::new("rustc").arg("-vV").output().unwrap();
    assert!(rustc.status.success());
    let version = String::from_utf8(rustc.stdout).unwrap();
    let host = version.lines().find_map(|line| line.strip_prefix("host: ")).unwrap();
    for target in [host, "thumbv8m.main-none-eabihf"] {
        for formatting in [false, true] {
            for library in UART_LIBRARIES {
                let directory = root().join("embassy-mcxa/experimental").join(library.name);
                let selection = if formatting {
                    "--all-features"
                } else {
                    "--no-default-features"
                };
                let graph = cargo_output(
                    &directory,
                    &[
                        "tree",
                        "--locked",
                        "--target",
                        target,
                        selection,
                        "-e",
                        "normal,build",
                        "--prefix",
                        "depth",
                        "--format",
                        "{p}",
                        "--charset",
                        "ascii",
                        "--color",
                        "never",
                    ],
                );
                let metadata: Value = serde_json::from_str(&cargo_output(
                    &directory,
                    &[
                        "metadata",
                        "--locked",
                        "--format-version",
                        "1",
                        "--filter-platform",
                        target,
                        selection,
                    ],
                ))
                .unwrap();
                assert_eq!(
                    check_graph(library, &directory, &graph, &metadata, formatting),
                    Ok(()),
                    "{} {target} {selection}:\n{graph}",
                    library.name
                );
            }
        }
    }
}

#[test]
fn resolved_graph_negative_controls_reject_transitive_leaks_patches_and_wrong_edges() {
    let library = &UART_LIBRARIES[0];
    let directory = root().join("embassy-mcxa/experimental").join(library.name);
    let graph = "0uart-one-shot-core v0.1.0\n1uart-one-shot-vocabulary v0.1.0\n2uart-one-shot-interface v0.1.0\n3defmt v1.0.0\n4syn v2.0.0\n";
    let mut packages: Vec<Value> = UART_LIBRARIES.iter().map(|library| serde_json::json!({"name": library.name, "version": "0.1.0", "source": null, "manifest_path": directory.parent().unwrap().join(library.name).join("Cargo.toml")})).collect();
    packages.extend([serde_json::json!({"name": "defmt", "version": "1.0.0", "source": "registry+https://github.com/rust-lang/crates.io-index"}), serde_json::json!({"name": "syn", "version": "2.0.0", "source": "registry+https://github.com/rust-lang/crates.io-index"})]);
    let metadata = serde_json::json!({"packages": packages});
    check_graph(library, &directory, graph, &metadata, true).unwrap();
    for name in [
        "nxp-pac",
        "embassy-mcxa",
        "embassy-time",
        "proptest",
        "driver-contracts",
    ] {
        assert!(
            check_graph(
                library,
                &directory,
                &format!("{graph}5{name} v1.0.0\n"),
                &metadata,
                true
            )
            .is_err()
        );
    }
    assert!(check_graph(library, &directory, &graph.replace("4syn", "1syn"), &metadata, true).is_err());
    assert!(check_graph(library, &directory, graph, &metadata, false).is_err());
    for source in [Value::Null, Value::String("git+https://example.com/defmt".into())] {
        let mut patched = metadata.clone();
        patched["packages"][3]["source"] = source;
        assert!(
            check_graph(library, &directory, graph, &patched, true)
                .unwrap_err()
                .contains("source")
        );
    }
}
