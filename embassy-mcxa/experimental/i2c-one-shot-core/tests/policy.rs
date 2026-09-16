//! Bounded source-policy gate for the protocol core, vocabulary and interface,
//! not a general Rust security checker. Core and vocabulary admit no unsafe
//! exception; only the interface may declare the body-free unsafe Device trait.
//! No expansion is silently skipped: these libraries intentionally use no macros
//! except reviewed built-in derives and the interface's defmt derive on Error.
//! Compiler rejection under actual CI flags is a separate mutation-runner control.

use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::path::Path;
use std::process::Command;

use serde_json::{Value, json};
use syn::punctuated::Punctuated;
use syn::visit::{self, Visit};
use syn::{Attribute, Expr, Item, ItemMod, ItemTrait, Lit, Meta, Token, TraitItem};

#[derive(Clone, Copy, PartialEq, Eq)]
enum Library {
    Core,
    Vocabulary,
    Interface,
}

const LIBRARIES: [Library; 3] = [Library::Core, Library::Vocabulary, Library::Interface];

impl Library {
    fn name(self) -> &'static str {
        match self {
            Self::Core => "i2c-one-shot-core",
            Self::Vocabulary => "i2c-one-shot-vocabulary",
            Self::Interface => "i2c-one-shot-interface",
        }
    }

    fn modules(self) -> &'static [&'static str] {
        match self {
            Self::Core => &["protocol"],
            Self::Vocabulary => &["vocabulary", "types"],
            Self::Interface => &["types", "capabilities"],
        }
    }

    fn inventory(self) -> &'static [&'static str] {
        match self {
            Self::Core => &["src/lib.rs", "src/protocol.rs"],
            Self::Vocabulary => &["src/lib.rs", "src/vocabulary.rs", "src/types.rs"],
            Self::Interface => &["src/lib.rs", "src/types.rs", "src/capabilities.rs"],
        }
    }

    fn contract(self) -> Option<&'static str> {
        match self {
            Self::Core => None,
            Self::Vocabulary => Some("Backend"),
            Self::Interface => Some("Device"),
        }
    }

    fn dependency(self) -> Option<Library> {
        match self {
            Self::Core => Some(Self::Vocabulary),
            Self::Vocabulary => Some(Self::Interface),
            Self::Interface => None,
        }
    }
}

type Sources = BTreeMap<String, String>;

fn lint(attr: &Attribute, level: &str) -> bool {
    attr.path().is_ident(level)
        && attr
            .parse_args_with(Punctuated::<syn::Path, Token![,]>::parse_terminated)
            .is_ok_and(|names| names.len() == 1 && names[0].is_ident("unsafe_code"))
}

fn literal_string(expr: &Expr, expected: Option<&str>) -> bool {
    matches!(expr, Expr::Lit(value) if matches!(&value.lit, Lit::Str(value)
        if expected.is_none_or(|expected| value.value() == expected)))
}

fn defmt_derive(attr: &Attribute) -> bool {
    let Ok(args) = attr.parse_args_with(Punctuated::<Meta, Token![,]>::parse_terminated) else {
        return false;
    };
    if args.len() != 2 {
        return false;
    }
    let Meta::NameValue(feature) = &args[0] else {
        return false;
    };
    let Meta::List(derive) = &args[1] else {
        return false;
    };
    if !feature.path.is_ident("feature")
        || !literal_string(&feature.value, Some("defmt"))
        || !derive.path.is_ident("derive")
    {
        return false;
    }
    derive
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
                    .all(|part| matches!(part.arguments, syn::PathArguments::None))
        })
}

#[derive(Default)]
struct Allowed<'a> {
    backend: Option<&'a ItemTrait>,
    exception: Option<&'a Attribute>,
    deny: Option<&'a Attribute>,
    modules: Vec<&'a ItemMod>,
    forbids: Vec<&'a Attribute>,
    type_attributes: Vec<&'a Attribute>,
    error_attributes: Vec<&'a Attribute>,
}

fn same<T>(node: &T, expected: Option<&T>) -> bool {
    expected.is_some_and(|expected| std::ptr::eq(node, expected))
}

fn listed<T>(node: &T, expected: &[&T]) -> bool {
    expected.iter().any(|expected| std::ptr::eq(node, *expected))
}

struct Checker<'a, 'ast> {
    library: Library,
    file: &'a str,
    allowed: &'a Allowed<'ast>,
    errors: &'a mut Vec<String>,
}

impl Checker<'_, '_> {
    fn reject(&mut self, reason: &str) {
        self.errors.push(format!("{}: {reason}", self.file));
    }
}

impl<'ast> Visit<'ast> for Checker<'_, 'ast> {
    fn visit_attribute(&mut self, attr: &'ast Attribute) {
        let path = attr.path();
        let accepted = if path.is_ident("allow") {
            same(attr, self.allowed.exception) && lint(attr, "allow")
        } else if path.is_ident("deny") {
            same(attr, self.allowed.deny) && lint(attr, "deny")
        } else if path.is_ident("forbid") {
            listed(attr, &self.allowed.forbids) && lint(attr, "forbid")
        } else if path.is_ident("no_std") {
            self.file == "src/lib.rs"
                && matches!(attr.style, syn::AttrStyle::Inner(_))
                && matches!(attr.meta, Meta::Path(_))
        } else if path.is_ident("doc") {
            matches!(&attr.meta, Meta::NameValue(value) if literal_string(&value.value, None))
        } else if path.is_ident("must_use") {
            matches!(&attr.meta, Meta::Path(_))
                || matches!(&attr.meta, Meta::NameValue(value) if literal_string(&value.value, None))
        } else if path.is_ident("derive") {
            listed(attr, &self.allowed.type_attributes)
                && attr
                    .parse_args_with(Punctuated::<syn::Path, Token![,]>::parse_terminated)
                    .is_ok_and(|paths| {
                        !paths.is_empty()
                            && paths.iter().all(|path| {
                                ["Debug", "Clone", "Copy", "PartialEq", "Eq", "Default"]
                                    .iter()
                                    .any(|name| path.is_ident(name))
                            })
                    })
        } else if path.is_ident("cfg_attr") {
            listed(attr, &self.allowed.error_attributes) && defmt_derive(attr)
        } else {
            false
        };
        if !accepted {
            self.reject("unreviewed attribute or lint weakening");
        }
        // Every accepted token-bearing attribute is parsed above. In particular,
        // doc/include expressions, conditional suppressions and unknown proc-
        // macros never reach an unexamined token stream.
    }

    fn visit_item_mod(&mut self, item: &'ast ItemMod) {
        if !listed(item, &self.allowed.modules) {
            self.reject("unreviewed module/source path");
        }
        visit::visit_item_mod(self, item);
    }

    fn visit_item_trait(&mut self, item: &'ast ItemTrait) {
        if !same(item, self.allowed.backend) {
            self.reject("only this library's reviewed contract trait is permitted");
        }
        if item.unsafety.is_some() != (self.library == Library::Interface)
            || !matches!(item.vis, syn::Visibility::Public(_))
            || item.auto_token.is_some()
            || item.restriction.is_some()
        {
            self.reject("contract must be public, with unsafe confined to interface Device");
        }
        for param in &item.generics.params {
            if matches!(param, syn::GenericParam::Type(value) if value.default.is_some())
                || matches!(param, syn::GenericParam::Const(value) if value.default.is_some())
            {
                self.reject("contract generic defaults are not declaration-only");
            }
        }
        for member in &item.items {
            let declaration_only = match member {
                TraitItem::Fn(value) => {
                    value.default.is_none()
                        && value.semi_token.is_some()
                        && value.sig.unsafety.is_none()
                        && value.sig.abi.is_none()
                }
                TraitItem::Const(value) => value.default.is_none(),
                TraitItem::Type(value) => value.default.is_none(),
                _ => false,
            };
            if !declaration_only {
                self.reject("contract members must be safe declarations without defaults");
            }
        }
        visit::visit_item_trait(self, item);
    }

    fn visit_item_impl(&mut self, item: &'ast syn::ItemImpl) {
        if item.unsafety.is_some() {
            self.reject("unsafe impl is outside the sole declaration exception");
        }
        visit::visit_item_impl(self, item);
    }

    fn visit_item_use(&mut self, item: &'ast syn::ItemUse) {
        fn reviewed_root(tree: &syn::UseTree, library: Library) -> bool {
            let name = match tree {
                syn::UseTree::Path(path) => &path.ident,
                syn::UseTree::Name(name) => &name.ident,
                syn::UseTree::Rename(rename) => &rename.ident,
                syn::UseTree::Group(group) => {
                    return group.items.iter().all(|tree| reviewed_root(tree, library));
                }
                syn::UseTree::Glob(_) => return false,
            };
            ["core", "crate", "self", "super"].iter().any(|root| name == root)
                || library.modules().iter().any(|module| name == module)
                || library
                    .dependency()
                    .is_some_and(|dependency| name.to_string() == dependency.name().replace('-', "_"))
        }
        // Do not let `use defmt::Format as Clone` disguise a proc-macro as an
        // approved built-in derive. Each layer may import only its immediate
        // reviewed dependency; defmt derives stay explicitly qualified.
        if !reviewed_root(&item.tree, self.library) {
            self.reject("unreviewed import or macro alias");
        }
        visit::visit_item_use(self, item);
    }

    fn visit_signature(&mut self, signature: &'ast syn::Signature) {
        if signature.unsafety.is_some() {
            self.reject("unsafe fn is outside the sole declaration exception");
        }
        visit::visit_signature(self, signature);
    }

    fn visit_expr(&mut self, expr: &'ast Expr) {
        if matches!(expr, Expr::Unsafe(_) | Expr::Verbatim(_)) {
            self.reject("unsafe or unparsed expression");
        }
        visit::visit_expr(self, expr);
    }

    fn visit_item(&mut self, item: &'ast Item) {
        if matches!(item, Item::Verbatim(_) | Item::ForeignMod(_) | Item::ExternCrate(_)) {
            self.reject("foreign or unparsed source is not permitted");
        }
        visit::visit_item(self, item);
    }

    fn visit_type(&mut self, ty: &'ast syn::Type) {
        if matches!(ty, syn::Type::Verbatim(_)) {
            self.reject("unparsed type is not permitted");
        }
        visit::visit_type(self, ty);
    }

    fn visit_impl_item(&mut self, item: &'ast syn::ImplItem) {
        if matches!(item, syn::ImplItem::Verbatim(_)) {
            self.reject("unparsed implementation member is not permitted");
        }
        visit::visit_impl_item(self, item);
    }

    fn visit_pat(&mut self, pattern: &'ast syn::Pat) {
        if matches!(pattern, syn::Pat::Verbatim(_)) {
            self.reject("unparsed pattern is not permitted");
        }
        visit::visit_pat(self, pattern);
    }

    fn visit_type_param_bound(&mut self, bound: &'ast syn::TypeParamBound) {
        if matches!(bound, syn::TypeParamBound::Verbatim(_)) {
            self.reject("unparsed type bound is not permitted");
        }
        visit::visit_type_param_bound(self, bound);
    }

    fn visit_lit(&mut self, literal: &'ast Lit) {
        if matches!(literal, Lit::Verbatim(_)) {
            self.reject("unparsed literal is not permitted");
        }
        visit::visit_lit(self, literal);
    }

    fn visit_macro(&mut self, _: &'ast syn::Macro) {
        self.reject("macro expansion/include source requires explicit policy review");
    }
}

fn check(library: Library, sources: &Sources) -> Result<(), Vec<String>> {
    let expected: BTreeSet<_> = library.inventory().iter().map(|path| path.to_string()).collect();
    if sources.keys().cloned().collect::<BTreeSet<_>>() != expected {
        return Err(vec!["library source inventory changed".into()]);
    }
    let parsed: BTreeMap<_, _> = sources
        .iter()
        .map(|(path, text)| {
            syn::parse_file(text)
                .map(|file| (path.as_str(), file))
                .map_err(|error| vec![format!("{path}: {error}")])
        })
        .collect::<Result<_, _>>()?;
    let lib = &parsed["src/lib.rs"];
    let mut errors = Vec::new();
    let mut allowed = Allowed::default();
    match library {
        Library::Core | Library::Vocabulary => {
            let forbids: Vec<_> = lib.attrs.iter().filter(|attr| lint(attr, "forbid")).collect();
            if forbids.len() == 1 {
                allowed.forbids.push(forbids[0]);
            } else {
                errors.push("safe library root must forbid unsafe_code exactly once, with no exception".into());
            }
        }
        Library::Interface => {
            let denies: Vec<_> = lib.attrs.iter().filter(|attr| lint(attr, "deny")).collect();
            if denies.len() == 1 {
                allowed.deny = Some(denies[0]);
            } else {
                errors.push("interface root must deny unsafe_code exactly once".into());
            }
        }
    }
    if lib.attrs.iter().filter(|attr| attr.path().is_ident("no_std")).count() != 1 {
        errors.push("root must declare no_std exactly once".into());
    }
    for item in &lib.items {
        match item {
            Item::Use(_) => {}
            Item::Trait(backend)
                if library.contract().is_some_and(|name| backend.ident == name) && allowed.backend.is_none() =>
            {
                allowed.backend = Some(backend);
                if library == Library::Interface {
                    let exceptions: Vec<_> = backend.attrs.iter().filter(|attr| lint(attr, "allow")).collect();
                    if exceptions.len() == 1 {
                        allowed.exception = Some(exceptions[0]);
                    } else {
                        errors.push("Device must own exactly one unsafe_code allowance".into());
                    }
                }
            }
            Item::Mod(module)
                if library.modules().iter().any(|name| module.ident == name) && module.content.is_none() =>
            {
                allowed.modules.push(module);
                let forbids: Vec<_> = module.attrs.iter().filter(|attr| lint(attr, "forbid")).collect();
                if forbids.len() == 1 {
                    allowed.forbids.push(forbids[0]);
                } else if library == Library::Interface || !forbids.is_empty() {
                    errors.push(format!("{} must forbid unsafe_code exactly once", module.ident));
                }
            }
            _ => errors.push("unreviewed root item; logic belongs in forbidden modules".into()),
        }
    }
    if library.contract().is_some() && allowed.backend.is_none() {
        errors.push("missing reviewed contract trait".into());
    }
    for name in library.modules() {
        if allowed.modules.iter().filter(|module| module.ident == name).count() != 1 {
            errors.push(format!("root must declare exactly one external {name} module"));
        }
    }
    if library != Library::Core {
        for item in parsed.values().flat_map(|file| &file.items) {
            let (name, attrs) = match item {
                Item::Enum(item) => (&item.ident, &item.attrs),
                Item::Struct(item) => (&item.ident, &item.attrs),
                _ => continue,
            };
            allowed.type_attributes.extend(attrs);
            if library == Library::Interface && name == "Error" {
                allowed.error_attributes.extend(attrs);
            }
        }
    }
    for (path, file) in &parsed {
        Checker {
            library,
            file: path,
            allowed: &allowed,
            errors: &mut errors,
        }
        .visit_file(file);
    }
    if errors.is_empty() { Ok(()) } else { Err(errors) }
}

fn read_sources(root: &Path) -> Result<Sources, String> {
    fn collect(root: &Path, directory: &Path, sources: &mut Sources) -> Result<(), String> {
        for entry in fs::read_dir(directory).map_err(|error| error.to_string())? {
            let entry = entry.map_err(|error| error.to_string())?;
            let kind = entry.file_type().map_err(|error| error.to_string())?;
            if kind.is_symlink() {
                return Err("source inventory contains a symlink".into());
            }
            let path = entry.path();
            if kind.is_dir() {
                collect(root, &path, sources)?;
            } else if path.extension().is_some_and(|extension| extension == "rs") {
                let name = path
                    .strip_prefix(root)
                    .map_err(|error| error.to_string())?
                    .to_string_lossy()
                    .replace('\\', "/");
                sources.insert(name, fs::read_to_string(path).map_err(|error| error.to_string())?);
            }
        }
        Ok(())
    }
    let mut sources = Sources::new();
    collect(root, &root.join("src"), &mut sources)?;
    Ok(sources)
}

fn package_policy(library: Library, package: &Value, root: &Path) -> Result<(), String> {
    let require = |condition, message: &str| if condition { Ok(()) } else { Err(message.to_string()) };
    require(package["name"] == library.name(), "wrong reviewed package")?;
    let features = match library {
        Library::Core => json!({"default": [], "defmt": ["i2c-one-shot-vocabulary/defmt"]}),
        Library::Vocabulary => json!({"default": [], "defmt": ["i2c-one-shot-interface/defmt"]}),
        Library::Interface => json!({"default": [], "defmt": ["dep:defmt"]}),
    };
    require(package["features"] == features, "unreviewed feature definition")?;
    let targets = package["targets"].as_array().ok_or("missing Cargo targets")?;
    let mut libraries = 0;
    for target in targets {
        let source = Path::new(target["src_path"].as_str().ok_or("target source is missing")?);
        if target["kind"] == json!(["lib"]) {
            libraries += 1;
            require(
                source == root.join("src/lib.rs") && target["edition"] == "2024",
                "unreviewed library source/edition",
            )?;
        } else {
            // Mocks and the checker belong to separate integration link units.
            // No build script (even a renamed one), binary or example is hidden
            // outside the library inventory and then trusted by default.
            require(
                target["kind"] == json!(["test"]) && source.starts_with(root.join("tests")),
                "unreviewed build/target source",
            )?;
        }
    }
    require(libraries == 1, "expected exactly one library target")?;
    let dependencies = package["dependencies"]
        .as_array()
        .ok_or("missing dependency declarations")?;
    let mut normal = 0;
    for dependency in dependencies {
        if dependency["kind"] == "dev" {
            continue;
        }
        normal += 1;
        require(
            dependency["kind"].is_null()
                && dependency["rename"].is_null()
                && dependency["target"].is_null()
                && dependency["features"] == json!([]),
            "production dependencies must be unconditional, unrenamed, and not force features",
        )?;
        match library.dependency() {
            Some(expected) => require(
                dependency["name"] == expected.name()
                    && dependency["optional"] == false
                    && dependency["source"].is_null()
                    && dependency["path"].as_str().is_some_and(|path| {
                        root.parent()
                            .is_some_and(|parent| Path::new(path) == parent.join(expected.name()))
                    }),
                "only the exact next-layer sibling is a reviewed dependency",
            )?,
            None => require(
                dependency["name"] == "defmt"
                    && dependency["optional"] == true
                    && dependency["path"].is_null()
                    && dependency["source"] == "registry+https://github.com/rust-lang/crates.io-index",
                "only optional registry defmt is a reviewed interface dependency",
            )?,
        }
    }
    require(normal == 1, "expected exactly one reviewed production dependency")
}

fn graph_policy(library: Library, graph: &str, formatting: bool) -> Result<(), String> {
    // The current reviewed defmt expansion closure, not an open-ended exception
    // for anything that happens to become one of its transitive dependencies.
    let approved = [
        "i2c-one-shot-core",
        "i2c-one-shot-vocabulary",
        "i2c-one-shot-interface",
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
    let mut seen = BTreeSet::new();
    let mut root = false;
    let mut parents = Vec::new();
    for line in graph.lines() {
        let depth_end = line
            .find(|ch: char| !ch.is_ascii_digit())
            .ok_or("malformed graph depth")?;
        let depth: usize = line[..depth_end].parse().map_err(|_| "malformed graph depth")?;
        let mut fields = line[depth_end..].split_whitespace();
        let name = fields.next().ok_or("missing graph package")?;
        let version = fields.next().ok_or("missing graph version")?;
        if !version.starts_with('v') || !approved.contains(&name) {
            return Err(format!("unreviewed resolved package: {line}"));
        }
        if depth == 0 {
            if root || name != library.name() {
                return Err("wrong graph root".into());
            }
            root = true;
        } else {
            if depth > parents.len() {
                return Err("dependency graph skips a parent".into());
            }
            parents.truncate(depth);
            let accepted = match parents.last().copied() {
                Some("i2c-one-shot-core") => name == "i2c-one-shot-vocabulary",
                Some("i2c-one-shot-vocabulary") => name == "i2c-one-shot-interface",
                Some("i2c-one-shot-interface") => formatting && name == "defmt",
                Some(_) => formatting && !LIBRARIES.iter().any(|library| library.name() == name) && name != "defmt",
                None => false,
            };
            if !accepted {
                return Err(format!("unreviewed dependency edge: {line}"));
            }
        }
        parents.push(name);
        seen.insert(name);
    }
    let mut required = Some(library);
    while let Some(library) = required {
        if !seen.contains(library.name()) {
            return Err("incomplete reviewed layer chain".into());
        }
        required = library.dependency();
    }
    if !root || formatting && !seen.contains("defmt") {
        return Err("incomplete protocol dependency graph".into());
    }
    Ok(())
}

fn cargo_output(root: &Path, args: &[&str]) -> String {
    let output = Command::new(env!("CARGO"))
        .args(args)
        .current_dir(root)
        .output()
        .expect("execute Cargo policy query");
    assert!(
        output.status.success(),
        "Cargo policy query {args:?}: {}",
        String::from_utf8_lossy(&output.stderr)
    );
    String::from_utf8(output.stdout).expect("Cargo UTF-8 policy output")
}

#[test]
fn cargo_declares_only_reviewed_sources_and_resolves_no_hardware_or_timer_dependencies() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let metadata: Value =
        serde_json::from_str(&cargo_output(root, &["metadata", "--format-version", "1", "--locked"])).unwrap();
    let packages = metadata["packages"].as_array().expect("Cargo packages");
    // Resolve all three manifests through the core's lockfile. Vocabulary and
    // interface remain dependencies, not hidden workspace/code-generation paths.
    let mut core_id = None;
    for library in LIBRARIES {
        let selected: Vec<_> = packages
            .iter()
            .filter(|package| package["name"] == library.name())
            .collect();
        assert_eq!(
            selected.len(),
            1,
            "CORE_DEPENDENCY_POLICY: missing/ambiguous reviewed package"
        );
        let package = selected[0];
        let package_root = root.parent().unwrap().join(library.name());
        assert_eq!(
            package_policy(library, package, &package_root),
            Ok(()),
            "CORE_DEPENDENCY_POLICY: {}",
            library.name()
        );
        if library == Library::Core {
            core_id = Some(package["id"].clone());
        }
    }
    assert_eq!(
        metadata["workspace_members"],
        json!([core_id.unwrap()]),
        "CORE_DEPENDENCY_POLICY: workspace shape changed"
    );
    let rustc = Command::new("rustc")
        .args(["-vV"])
        .current_dir(root)
        .output()
        .expect("record host target");
    assert!(rustc.status.success());
    let version = String::from_utf8(rustc.stdout).unwrap();
    let host = version
        .lines()
        .find_map(|line| line.strip_prefix("host: "))
        .expect("rustc host target");
    for target in [host, "thumbv8m.main-none-eabihf"] {
        for (selection, formatting) in [("--no-default-features", false), ("--all-features", true)] {
            for library in LIBRARIES {
                let args = [
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
                ];
                // Cargo cannot select features for a non-workspace dependency
                // via -p. Query each package as its own root, using the ignored
                // lockfiles explicitly primed by CI before this policy runs.
                let package_root = root.parent().unwrap().join(library.name());
                let graph = cargo_output(&package_root, &args);
                println!(
                    "Reviewed {} dependency graph {target} {selection}:\n{graph}",
                    library.name()
                );
                assert_eq!(
                    graph_policy(library, &graph, formatting),
                    Ok(()),
                    "CORE_DEPENDENCY_POLICY"
                );
            }
        }
    }
}

fn graph_fixture(library: Library, formatting: bool) -> String {
    let mut graph = String::new();
    let mut next = Some(library);
    let mut depth = 0;
    while let Some(library) = next {
        graph.push_str(&format!("{depth}{} v0.1.0 (/fixture)\n", library.name()));
        next = library.dependency();
        depth += 1;
    }
    if formatting {
        graph.push_str(&format!("{depth}defmt v1.1.1\n{}syn v2.0.119\n", depth + 1));
    }
    graph
}

#[test]
fn dependency_controls_reject_missing_hardware_and_unreviewed_transitive_packages() {
    for library in LIBRARIES {
        let plain = graph_fixture(library, false);
        let formatting = graph_fixture(library, true);
        assert_eq!(graph_policy(library, &plain, false), Ok(()));
        assert_eq!(graph_policy(library, &formatting, true), Ok(()));
        assert!(graph_policy(library, &plain, true).is_err());
        assert!(graph_policy(library, &formatting, false).is_err());
        for graph in [
            String::new(),
            "0defmt v1.1.1\n".into(),
            format!("{plain}1safe-mmio v0.3.1\n"),
            format!("{plain}1nxp-pac v0.1.0\n"),
            format!("{formatting}1embassy-time v0.5.1\n"),
            format!("{formatting}1syn v2.0.119\n"),
            formatting.replace("syn v2.0.119", "unknown v1.0.0"),
            formatting.replace("\n1", "\n9"),
        ] {
            assert!(
                graph_policy(library, &graph, true).is_err(),
                "unexpected graph acceptance: {graph}"
            );
        }
    }
    for graph in [
        "0i2c-one-shot-core v0.1.0\n",
        "0i2c-one-shot-core v0.1.0\n1i2c-one-shot-interface v0.1.0\n",
        "0i2c-one-shot-core v0.1.0\n1defmt v1.1.1\n",
    ] {
        assert!(
            graph_policy(Library::Core, graph, false).is_err(),
            "layer was bypassed: {graph}"
        );
    }
}

fn package_fixture(library: Library, root: &Path) -> Value {
    let (features, dependency) = match library.dependency() {
        Some(expected) => (
            json!({"default": [], "defmt": [format!("{}/defmt", expected.name())]}),
            json!({"name": expected.name(), "kind": null, "optional": false,
                "rename": null, "target": null, "source": null, "features": [],
                "path": root.parent().unwrap().join(expected.name())}),
        ),
        None => (
            json!({"default": [], "defmt": ["dep:defmt"]}),
            json!({"name": "defmt", "kind": null, "optional": true, "rename": null,
                "target": null, "source": "registry+https://github.com/rust-lang/crates.io-index",
                "path": null, "features": []}),
        ),
    };
    json!({"name": library.name(), "features": features,
        "targets": [{"kind": ["lib"], "src_path": root.join("src/lib.rs"), "edition": "2024"}],
        "dependencies": [dependency]})
}

#[test]
fn declared_dependency_and_build_script_controls_fail_closed() {
    for library in LIBRARIES {
        let root = Path::new("/fixture").join(library.name());
        let package = package_fixture(library, &root);
        assert_eq!(package_policy(library, &package, &root), Ok(()));
        let mut altered = package.clone();
        altered["targets"]
            .as_array_mut()
            .unwrap()
            .push(json!({"kind": ["custom-build"], "src_path": "/fixture/renamed-codegen.rs"}));
        assert!(package_policy(library, &altered, &root).is_err());
        altered = package.clone();
        altered["targets"][0]["src_path"] = json!("/fixture/out/generated.rs");
        assert!(package_policy(library, &altered, &root).is_err());
        altered = package.clone();
        altered["features"]["default"] = json!(["defmt"]);
        assert!(package_policy(library, &altered, &root).is_err());
        altered = package.clone();
        altered["features"]["defmt"] = json!(["unreviewed/defmt"]);
        assert!(package_policy(library, &altered, &root).is_err());
        for (field, value) in [
            ("name", json!("nxp-pac")),
            ("kind", json!("build")),
            ("rename", json!("pac")),
            ("target", json!("cfg(target_arch = \"arm\")")),
            ("optional", json!(library != Library::Interface)),
            ("features", json!(["defmt"])),
            ("path", json!("/unreviewed/replacement")),
            ("source", json!("git+https://example.invalid/replacement")),
        ] {
            altered = package.clone();
            altered["dependencies"][0][field] = value;
            assert!(
                package_policy(library, &altered, &root).is_err(),
                "unexpected {} dependency acceptance: {field}",
                library.name()
            );
        }
        altered = package.clone();
        altered["dependencies"] = json!([]);
        assert!(package_policy(library, &altered, &root).is_err());
    }
}

#[test]
fn actual_library_obeys_reviewed_source_policy() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    for library in LIBRARIES {
        let directory = root.parent().unwrap().join(library.name());
        let sources = read_sources(&directory).expect("read source inventory");
        assert_eq!(
            check(library, &sources),
            Ok(()),
            "CORE_SOURCE_POLICY: {}",
            library.name()
        );
    }
}

fn fixture(library: Library) -> Sources {
    if library == Library::Core {
        return Sources::from([
            (
                "src/lib.rs".into(),
                r#"#![no_std]
#![forbid(unsafe_code)]
#[forbid(unsafe_code)] mod protocol;
pub use i2c_one_shot_vocabulary::{Backend, Error};
"#
                .into(),
            ),
            ("src/protocol.rs".into(), "pub fn decision() -> bool { true }".into()),
        ]);
    }
    if library == Library::Vocabulary {
        return Sources::from([
            (
                "src/lib.rs".into(),
                r#"#![no_std]
#![forbid(unsafe_code)]
#[forbid(unsafe_code)] mod vocabulary;
#[forbid(unsafe_code)] mod types;
pub use types::Error;
pub use vocabulary::Hardware;
pub trait Backend: Sized { type Streaming; fn start(&mut self); }
"#
                .into(),
            ),
            (
                "src/vocabulary.rs".into(),
                "pub struct Hardware; pub fn decision() -> bool { true }".into(),
            ),
            ("src/types.rs".into(), "pub use i2c_one_shot_interface::Error;".into()),
        ]);
    }
    Sources::from([
        (
            "src/lib.rs".into(),
            r#"#![no_std]
#![deny(unsafe_code)]
#[allow(unsafe_code)]
pub unsafe trait Device: Sized { type Streaming; fn start(&mut self); }
#[forbid(unsafe_code)] mod types;
#[forbid(unsafe_code)] mod capabilities;
pub use types::Error;
"#
            .into(),
        ),
        (
            "src/types.rs".into(),
            r#"#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error { Timeout }
"#
            .into(),
        ),
        ("src/capabilities.rs".into(), "pub struct Cpu;".into()),
    ])
}

fn rejected(library: Library, path: &str, from: &str, to: &str) {
    let mut sources = fixture(library);
    let source = sources.get_mut(path).unwrap();
    assert_eq!(source.matches(from).count(), 1, "unambiguous fixture mutation");
    *source = source.replacen(from, to, 1);
    assert!(
        check(library, &sources).is_err(),
        "{} policy unexpectedly accepted {to}",
        library.name()
    );
}

#[test]
fn reviewed_shape_is_accepted() {
    for library in LIBRARIES {
        assert_eq!(check(library, &fixture(library)), Ok(()));
    }
    for library in [Library::Core, Library::Vocabulary] {
        let mut sources = fixture(library);
        let lib = sources.get_mut("src/lib.rs").unwrap();
        *lib = lib.replace("#[forbid(unsafe_code)] mod", "mod");
        assert_eq!(
            check(library, &sources),
            Ok(()),
            "root forbid already covers every module"
        );
    }
}

#[test]
fn alternate_suppressions_and_expansion_attributes_are_rejected() {
    for attr in [
        "#[expect(unsafe_code)]",
        "#[warn(unsafe_code)]",
        "#[allow(warnings)]",
        "#[cfg_attr(all(), allow(unsafe_code))]",
        "#[cfg_attr(all(), expect(unsafe_code))]",
        "#[cfg_attr(all(), cfg_attr(all(), expect(unsafe_code)))]",
        "#[unknown::generate]",
    ] {
        for library in LIBRARIES {
            for path in &library.inventory()[1..] {
                let mut sources = fixture(library);
                sources
                    .get_mut(*path)
                    .unwrap()
                    .push_str(&format!("{attr}\npub fn hidden() {{}}"));
                assert!(
                    check(library, &sources).is_err(),
                    "{} {path} accepted {attr}",
                    library.name()
                );
            }
        }
    }
    rejected(
        Library::Interface,
        "src/types.rs",
        "derive(defmt::Format)",
        "expect(unsafe_code)",
    );
    rejected(
        Library::Interface,
        "src/types.rs",
        "#[derive(Debug",
        "use defmt::Format as Clone;\n#[derive(Debug",
    );
}

#[test]
fn moved_duplicated_or_removed_policy_attributes_are_rejected() {
    rejected(Library::Interface, "src/lib.rs", "#[allow(unsafe_code)]", "");
    rejected(
        Library::Interface,
        "src/lib.rs",
        "#[allow(unsafe_code)]",
        "#[allow(unsafe_code)] #[allow(unsafe_code)]",
    );
    rejected(
        Library::Interface,
        "src/lib.rs",
        "#[allow(unsafe_code)]\npub unsafe trait",
        "#[allow(unsafe_code)] use core::marker::PhantomData;\npub unsafe trait",
    );
    rejected(
        Library::Interface,
        "src/lib.rs",
        "#[forbid(unsafe_code)] mod types;",
        "mod types;",
    );
    rejected(
        Library::Interface,
        "src/lib.rs",
        "#![deny(unsafe_code)]",
        "#![allow(unsafe_code)]",
    );
    rejected(
        Library::Interface,
        "src/lib.rs",
        "#[forbid(unsafe_code)] mod capabilities;",
        "mod capabilities;",
    );
    for library in [Library::Core, Library::Vocabulary] {
        rejected(library, "src/lib.rs", "#![forbid(unsafe_code)]", "");
        rejected(
            library,
            "src/lib.rs",
            "#![forbid(unsafe_code)]",
            "#![forbid(unsafe_code)] #![forbid(unsafe_code)]",
        );
        for level in ["deny", "allow", "warn", "expect"] {
            rejected(
                library,
                "src/lib.rs",
                "#![forbid(unsafe_code)]",
                &format!("#![{level}(unsafe_code)]"),
            );
        }
    }
}

#[test]
fn safe_library_roots_admit_no_unsafe_declaration_implementation_or_exception() {
    // These are AST-policy controls, not compiler-diagnostic controls. They also
    // reject declarations at the root, which the old module-only forbid could
    // not cover. The real mutation runner checks compiler reasons separately.
    for addition in [
        "pub unsafe trait Hidden {}",
        "#[allow(unsafe_code)] pub unsafe trait Hidden {}",
        "#[expect(unsafe_code)] pub unsafe trait Hidden {}",
        "unsafe impl Hidden for () {}",
        "#[allow(unsafe_code)] unsafe impl Hidden for () {}",
        "#[expect(unsafe_code)] unsafe impl Hidden for () {}",
        "pub unsafe fn hidden() {}",
        "pub fn hidden() { unsafe {} }",
        "const HIDDEN: () = unsafe {};",
        "#[allow(unsafe_code)] use core::marker::PhantomData;",
        "#[expect(unsafe_code)] use core::marker::PhantomData;",
        "#[cfg_attr(all(), allow(unsafe_code))] use core::marker::PhantomData;",
        "#[cfg_attr(all(), expect(unsafe_code))] use core::marker::PhantomData;",
    ] {
        for library in [Library::Core, Library::Vocabulary] {
            let mut sources = fixture(library);
            sources.get_mut("src/lib.rs").unwrap().push_str(addition);
            assert!(
                check(library, &sources).is_err(),
                "{} root accepted {addition}",
                library.name()
            );
        }
    }
}

#[test]
fn unsafe_expressions_and_implementations_are_rejected_in_all_logic_modules() {
    for library in LIBRARIES {
        for addition in [
            "unsafe impl Hidden for () {}",
            "pub unsafe fn hidden() {}",
            "pub fn hidden() { unsafe {} }",
            "#[allow(unsafe_code)] pub fn hidden() {}",
            "pub unsafe trait Hidden {}",
        ] {
            for path in &library.inventory()[1..] {
                let mut sources = fixture(library);
                sources.get_mut(*path).unwrap().push_str(addition);
                assert!(
                    check(library, &sources).is_err(),
                    "{} {path} accepted {addition}",
                    library.name()
                );
            }
        }
    }
}

#[test]
fn reviewed_traits_cannot_acquire_implementation_bodies_or_defaults() {
    for member in [
        "fn start(&mut self) {}",
        "unsafe fn start(&mut self);",
        "fn start(&mut self) { unsafe {} }",
        "const VALUE: u8 = 1;",
        "type Hidden = u8;",
        "generate!();",
        "fn start(&mut self, _: [u8; { unsafe { 1 } }]);",
    ] {
        for library in [Library::Interface, Library::Vocabulary] {
            rejected(library, "src/lib.rs", "fn start(&mut self);", member);
        }
    }
    rejected(
        Library::Interface,
        "src/lib.rs",
        "trait Device:",
        "trait Device<T = u8>:",
    );
    rejected(
        Library::Vocabulary,
        "src/lib.rs",
        "pub trait Backend:",
        "pub unsafe trait Backend:",
    );
    rejected(
        Library::Vocabulary,
        "src/lib.rs",
        "pub trait Backend:",
        "#[allow(unsafe_code)] pub trait Backend:",
    );
    rejected(
        Library::Interface,
        "src/lib.rs",
        "pub unsafe trait Device:",
        "pub trait Device:",
    );
}

#[test]
fn unknown_files_modules_and_includes_fail_closed() {
    for library in LIBRARIES {
        for extra in ["src/hidden.rs", "src/bin/check-policy.rs"] {
            let mut sources = fixture(library);
            sources.insert(extra.into(), "pub fn hidden() {}".into());
            assert!(check(library, &sources).is_err());
        }
        for module in library.modules() {
            let declaration = format!("mod {module};");
            rejected(
                library,
                "src/lib.rs",
                &declaration,
                &format!("#[path = \"elsewhere.rs\"] {declaration}"),
            );
        }
        for source in [
            "include!(\"generated.rs\");",
            "mod hidden;",
            "mod hidden {}",
            "macro_rules! generate { () => {} }",
            "#[doc = include_str!(\"doc.md\")] pub fn f() {}",
        ] {
            for path in &library.inventory()[1..] {
                let mut sources = fixture(library);
                sources.get_mut(*path).unwrap().push_str(source);
                assert!(
                    check(library, &sources).is_err(),
                    "{} {path} accepted {source}",
                    library.name()
                );
            }
        }
    }
}
