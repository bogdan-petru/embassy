//! Bounded macro-free library policy, not a general Rust security checker.
//! Explicit inventories and fail-closed attributes keep expansion paths visible.
//! Compiler rejection under recorded CI flags remains a separate control.

use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};

use serde_json::Value;
use syn::punctuated::Punctuated;
use syn::visit::{self, Visit};
use syn::{Attribute, Expr, Item, ItemTrait, Lit, Meta, Token, TraitItem};

pub struct Library {
    pub name: &'static str,
    pub files: &'static [&'static str],
    pub dependency: Option<&'static str>,
    pub unsafe_trait: Option<&'static str>,
}

pub const UART_LIBRARIES: &[Library] = &[
    Library {
        name: "uart-one-shot-core",
        files: &["src/lib.rs"],
        dependency: Some("uart-one-shot-vocabulary"),
        unsafe_trait: None,
    },
    Library {
        name: "uart-one-shot-vocabulary",
        files: &["src/lib.rs"],
        dependency: Some("uart-one-shot-interface"),
        unsafe_trait: None,
    },
    Library {
        name: "uart-one-shot-interface",
        files: &["src/lib.rs", "src/capabilities.rs"],
        dependency: None,
        unsafe_trait: Some("Device"),
    },
];

fn lint(attr: &Attribute, level: &str) -> bool {
    attr.path().is_ident(level)
        && attr
            .parse_args_with(Punctuated::<syn::Path, Token![,]>::parse_terminated)
            .is_ok_and(|p| p.len() == 1 && p[0].is_ident("unsafe_code"))
}

fn literal(expr: &Expr, expected: Option<&str>) -> bool {
    matches!(expr, Expr::Lit(value) if matches!(&value.lit, Lit::Str(value) if expected.is_none_or(|s| value.value() == s)))
}

fn defmt_derive(attr: &Attribute) -> bool {
    let Ok(args) = attr.parse_args_with(Punctuated::<Meta, Token![,]>::parse_terminated) else {
        return false;
    };
    if args.len() != 2 {
        return false;
    }
    let (Meta::NameValue(feature), Meta::List(derive)) = (&args[0], &args[1]) else {
        return false;
    };
    feature.path.is_ident("feature")
        && literal(&feature.value, Some("defmt"))
        && derive.path.is_ident("derive")
        && derive
            .parse_args_with(Punctuated::<syn::Path, Token![,]>::parse_terminated)
            .is_ok_and(|paths| {
                paths.len() == 1
                    && paths[0].leading_colon.is_none()
                    && paths[0].segments.len() == 2
                    && paths[0].segments[0].ident == "defmt"
                    && paths[0].segments[1].ident == "Format"
                    && paths[0]
                        .segments
                        .iter()
                        .all(|s| matches!(s.arguments, syn::PathArguments::None))
            })
}

struct Checker<'a> {
    trait_item: Option<&'a ItemTrait>,
    allowed: Vec<&'a Attribute>,
    errors: Vec<String>,
}

impl<'ast> Visit<'ast> for Checker<'ast> {
    fn visit_attribute(&mut self, attr: &'ast Attribute) {
        let accepted = self.allowed.iter().any(|a| std::ptr::eq(*a, attr))
            || (attr.path().is_ident("doc") && matches!(&attr.meta, Meta::NameValue(v) if literal(&v.value, None)))
            || (attr.path().is_ident("must_use") && matches!(&attr.meta, Meta::Path(_)));
        if !accepted {
            self.errors
                .push("unreviewed attribute (including conditional lint/expansion)".into());
        }
        // Do not recursively accept nested cfg_attr; only the exact root Error
        // defmt derive is admitted by pointer identity below.
    }
    fn visit_macro(&mut self, _node: &'ast syn::Macro) {
        self.errors.push("unreviewed macro/expansion".into());
    }
    fn visit_expr_unsafe(&mut self, node: &'ast syn::ExprUnsafe) {
        self.errors.push("unsafe block".into());
        visit::visit_expr_unsafe(self, node);
    }
    fn visit_signature(&mut self, node: &'ast syn::Signature) {
        if node.unsafety.is_some() || node.abi.is_some() {
            self.errors.push("unsafe or foreign function".into());
        }
        visit::visit_signature(self, node);
    }
    fn visit_item_impl(&mut self, node: &'ast syn::ItemImpl) {
        if node.unsafety.is_some() {
            self.errors.push("unsafe impl".into());
        }
        visit::visit_item_impl(self, node);
    }
    fn visit_item_trait(&mut self, node: &'ast ItemTrait) {
        if node.unsafety.is_some() && !self.trait_item.is_some_and(|t| std::ptr::eq(t, node)) {
            self.errors.push("unreviewed unsafe trait".into());
        }
        visit::visit_item_trait(self, node);
    }
    fn visit_item_foreign_mod(&mut self, _node: &'ast syn::ItemForeignMod) {
        self.errors.push("foreign module".into());
    }
    fn visit_item_mod(&mut self, node: &'ast syn::ItemMod) {
        // Only root, inventoried, out-of-line module declarations are accepted
        // explicitly by check_sources. Nested/inline modules fail here.
        self.errors.push(format!("unreviewed module {}", node.ident));
    }
}

pub fn check_sources(library: &Library, sources: &BTreeMap<String, String>) -> Result<(), String> {
    let expected: BTreeSet<_> = library.files.iter().map(|s| s.to_string()).collect();
    if sources.keys().cloned().collect::<BTreeSet<_>>() != expected {
        return Err("source inventory drift".into());
    }
    let mut found_trait = 0;
    let mut declarations = BTreeSet::new();
    for (file, source) in sources {
        let tree = syn::parse_file(source).map_err(|e| e.to_string())?;
        let root = file == "src/lib.rs";
        let mut checker = Checker {
            trait_item: None,
            allowed: vec![],
            errors: vec![],
        };
        if root {
            let level = if library.unsafe_trait.is_some() {
                "deny"
            } else {
                "forbid"
            };
            let lints: Vec<_> = tree.attrs.iter().filter(|a| lint(a, level)).collect();
            let no_std: Vec<_> = tree
                .attrs
                .iter()
                .filter(|a| a.path().is_ident("no_std") && matches!(a.meta, Meta::Path(_)))
                .collect();
            if lints.len() != 1 || no_std.len() != 1 {
                return Err("missing exact root no_std/unsafe lint".into());
            }
            checker.allowed.extend(lints);
            checker.allowed.extend(no_std);
        }
        for item in &tree.items {
            if let Item::Trait(t) = item {
                if root && library.unsafe_trait.is_some_and(|name| t.ident == name) {
                    found_trait += 1;
                    if t.unsafety.is_none()
                        || !matches!(t.vis, syn::Visibility::Public(_))
                        || t.items.iter().any(|item| {
                            !matches!(item, TraitItem::Type(i) if i.default.is_none())
                                && !matches!(item, TraitItem::Fn(i) if i.default.is_none())
                        })
                    {
                        return Err("Device contract must be public, unsafe and declaration-only".into());
                    }
                    let allowance: Vec<_> = t.attrs.iter().filter(|a| lint(a, "allow")).collect();
                    if allowance.len() != 1 {
                        return Err("Device needs exactly one unsafe declaration allowance".into());
                    }
                    checker.allowed.extend(allowance);
                    checker.trait_item = Some(t);
                }
            }
            if let Item::Enum(item) = item {
                for attr in &item.attrs {
                    let builtins = attr.path().is_ident("derive")
                        && attr
                            .parse_args_with(Punctuated::<syn::Path, Token![,]>::parse_terminated)
                            .is_ok_and(|ps| {
                                !ps.is_empty()
                                    && ps.iter().all(|p| {
                                        ["Debug", "Clone", "Copy", "PartialEq", "Eq"]
                                            .iter()
                                            .any(|n| p.is_ident(n))
                                    })
                            });
                    if builtins
                        || (root && library.unsafe_trait.is_some() && item.ident == "Error" && defmt_derive(attr))
                    {
                        checker.allowed.push(attr);
                    }
                }
            }
            if let Item::Mod(module) = item {
                let path = format!("src/{}.rs", module.ident);
                if !root
                    || module.content.is_some()
                    || !expected.contains(&path)
                    || !declarations.insert(path)
                    || module.attrs.len() != 1
                    || !lint(&module.attrs[0], "forbid")
                {
                    return Err("unreviewed module or missing module forbid".into());
                }
            }
        }
        for attr in &tree.attrs {
            checker.visit_attribute(attr);
        }
        for item in &tree.items {
            if !matches!(item, Item::Mod(_)) {
                checker.visit_item(item);
            }
        }
        if !checker.errors.is_empty() {
            return Err(format!("{file}: {}", checker.errors.join(", ")));
        }
    }
    let children: BTreeSet<_> = expected.into_iter().filter(|p| p != "src/lib.rs").collect();
    if declarations != children || found_trait != usize::from(library.unsafe_trait.is_some()) {
        return Err("module or Device declaration inventory drift".into());
    }
    Ok(())
}

pub fn read_sources(directory: &Path) -> Result<BTreeMap<String, String>, String> {
    fn walk(root: &Path, at: &Path, result: &mut BTreeMap<String, String>) -> Result<(), String> {
        for item in std::fs::read_dir(at).map_err(|e| e.to_string())? {
            let item = item.map_err(|e| e.to_string())?;
            let kind = item.file_type().map_err(|e| e.to_string())?;
            if kind.is_symlink() {
                return Err("source symlink is outside bounded inventory".into());
            }
            if kind.is_dir() {
                walk(root, &item.path(), result)?;
            } else if item.path().extension().is_some_and(|e| e == "rs") {
                result.insert(
                    item.path()
                        .strip_prefix(root)
                        .unwrap()
                        .to_string_lossy()
                        .replace('\\', "/"),
                    std::fs::read_to_string(item.path()).map_err(|e| e.to_string())?,
                );
            }
        }
        Ok(())
    }
    let mut result = BTreeMap::new();
    walk(directory, &directory.join("src"), &mut result)?;
    Ok(result)
}

pub fn check_package(library: &Library, directory: &Path, package: &Value) -> Result<(), String> {
    if package["name"] != library.name || package["edition"] != "2024" || directory.join("build.rs").exists() {
        return Err("package/build-script policy drift".into());
    }
    let targets = package["targets"].as_array().ok_or("missing targets")?;
    let expected_lib = directory.join("src/lib.rs").canonicalize().map_err(|e| e.to_string())?;
    let mut libraries = 0;
    for target in targets {
        let kinds = target["kind"].as_array().ok_or("missing target kind")?;
        if kinds.iter().any(|k| k == "custom-build" || k == "proc-macro") {
            return Err("unreviewed source-generation target".into());
        }
        if kinds.iter().any(|k| k == "lib") {
            libraries += 1;
            if PathBuf::from(target["src_path"].as_str().ok_or("missing lib path")?)
                .canonicalize()
                .map_err(|e| e.to_string())?
                != expected_lib
            {
                return Err("library source override".into());
            }
        }
    }
    if libraries != 1 {
        return Err("expected one library target".into());
    }
    let dependencies = package["dependencies"].as_array().ok_or("missing dependencies")?;
    let mut normal = 0;
    for dependency in dependencies {
        if dependency["kind"] == "dev" {
            continue;
        }
        if !dependency["kind"].is_null() || !dependency["target"].is_null() || !dependency["rename"].is_null() {
            return Err("build, conditional or renamed dependency".into());
        }
        if dependency["features"] != serde_json::json!([]) {
            return Err("unreviewed dependency feature activation".into());
        }
        normal += 1;
        if let Some(expected) = library.dependency {
            let wanted = directory
                .parent()
                .ok_or("missing package parent")?
                .join(expected)
                .canonicalize()
                .map_err(|e| e.to_string())?;
            let actual = PathBuf::from(dependency["path"].as_str().ok_or("nonlocal dependency")?)
                .canonicalize()
                .map_err(|e| e.to_string())?;
            if dependency["name"] != expected
                || actual != wanted
                || dependency["uses_default_features"] != false
                || dependency["optional"] != false
            {
                return Err("normal dependency boundary drift".into());
            }
        } else if dependency["name"] != "defmt"
            || dependency["optional"] != true
            || !dependency["path"].is_null()
            || dependency["source"] != "registry+https://github.com/rust-lang/crates.io-index"
        {
            return Err("interface may depend only on optional defmt".into());
        }
    }
    if normal != 1 {
        return Err("normal dependency count drift".into());
    }
    let expected_features = if let Some(dependency) = library.dependency {
        serde_json::json!({"default": [], "defmt": [format!("{dependency}/defmt")]})
    } else {
        serde_json::json!({"default": [], "defmt": ["dep:defmt"]})
    };
    if package["features"] != expected_features {
        return Err("feature graph drift".into());
    }
    Ok(())
}

/// Validate the actually resolved normal/build graph, not merely declarations.
/// The optional formatter has a finite reviewed expansion closure. Metadata
/// supplies source identity so an identically named local/git patch is rejected.
pub fn check_graph(
    library: &Library,
    directory: &Path,
    graph: &str,
    metadata: &Value,
    formatting: bool,
) -> Result<(), String> {
    const FORMATTER: &[&str] = &[
        "defmt",
        "bitflags",
        "defmt-macros",
        "defmt-parser",
        "proc-macro2",
        "quote",
        "syn",
        "thiserror",
        "thiserror-impl",
        "unicode-ident",
    ];
    let packages = metadata["packages"].as_array().ok_or("missing resolved packages")?;
    let mut seen = BTreeSet::new();
    let mut parents: Vec<&str> = Vec::new();
    let mut root_seen = false;
    for line in graph.lines() {
        let end = line
            .find(|ch: char| !ch.is_ascii_digit())
            .ok_or("malformed graph depth")?;
        let depth: usize = line[..end].parse().map_err(|_| "malformed graph depth")?;
        let mut words = line[end..].split_whitespace();
        let name = words.next().ok_or("missing graph name")?;
        let version = words
            .next()
            .and_then(|v| v.strip_prefix('v'))
            .ok_or("missing graph version")?;
        let layer = UART_LIBRARIES.iter().find(|l| l.name == name);
        if layer.is_none() && (!formatting || !FORMATTER.contains(&name)) {
            return Err(format!("unreviewed resolved package {name}"));
        }
        let matching: Vec<_> = packages
            .iter()
            .filter(|p| p["name"] == name && p["version"] == version)
            .collect();
        if matching.len() != 1 {
            return Err(format!("ambiguous resolved package {name} {version}"));
        }
        let package = matching[0];
        if let Some(layer) = layer {
            let expected = directory
                .parent()
                .ok_or("no package parent")?
                .join(layer.name)
                .join("Cargo.toml")
                .canonicalize()
                .map_err(|e| e.to_string())?;
            let actual = PathBuf::from(package["manifest_path"].as_str().ok_or("missing resolved manifest")?)
                .canonicalize()
                .map_err(|e| e.to_string())?;
            if !package["source"].is_null() || actual != expected {
                return Err(format!("resolved layer source drift {name}"));
            }
        } else if package["source"] != "registry+https://github.com/rust-lang/crates.io-index" {
            return Err(format!("unreviewed resolved source {name}"));
        }
        if depth == 0 {
            if root_seen || name != library.name {
                return Err("wrong/duplicate graph root".into());
            }
            root_seen = true;
        } else {
            if depth > parents.len() {
                return Err("graph skips a parent".into());
            }
            parents.truncate(depth);
            let accepted = match parents
                .last()
                .and_then(|p| UART_LIBRARIES.iter().find(|l| l.name == *p))
            {
                Some(parent) => match parent.dependency {
                    Some(next) => name == next,
                    None => formatting && name == "defmt",
                },
                None => formatting && layer.is_none() && name != "defmt",
            };
            if !accepted {
                return Err(format!("unreviewed resolved edge to {name}"));
            }
        }
        parents.push(name);
        seen.insert(name);
    }
    let mut next = Some(library);
    while let Some(layer) = next {
        if !seen.contains(layer.name) {
            return Err("incomplete resolved layer chain".into());
        }
        next = layer
            .dependency
            .and_then(|name| UART_LIBRARIES.iter().find(|l| l.name == name));
    }
    if !root_seen || formatting && !seen.contains("defmt") {
        return Err("incomplete resolved graph".into());
    }
    Ok(())
}
