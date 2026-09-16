//! Fail-closed policy for the inventoried one-shot wake path, not all HAL code.
//!
//! A safe PAC write or FIFO pop needs no `unsafe` block. Rust's unsafe-code lint
//! therefore cannot protect this ISR's no-MMIO obligation. Pin its actual syntax,
//! its local callees, relevant imports, and source routing instead. Harmless
//! formatting/comments may change; a changed operation requires policy review.
//! This is a source-policy result, not compiler sealing, NVIC correctness, or
//! protection against arbitrary dependency/build-script/compiler modifications.

use std::collections::BTreeMap;
use std::fs;
use std::path::Path;

use syn::{Attribute, ImplItem, Item, UseTree};

const REGISTERS: &str = "src/i2c/one_shot/session/vocabulary/registers.rs";
const BINDING: &str = "src/i2c/one_shot/session/vocabulary/registers/binding.rs";
const EFFECTS: &str = "src/i2c/one_shot/session/vocabulary/registers/effects.rs";
const INFO: &str = "src/i2c/instance_info.rs";
const INVENTORY: [&str; 9] = [
    "src/lib.rs",
    "src/i2c/mod.rs",
    "src/i2c/one_shot/mod.rs",
    "src/i2c/one_shot/session.rs",
    "src/i2c/one_shot/session/vocabulary.rs",
    REGISTERS,
    BINDING,
    EFFECTS,
    INFO,
];
const WAKE_CALL: &str = "effects::wake_interrupt(T::Interrupt::disable, T::info().wait_cell());";
const PAC_WRITE: &str = "crate::pac::LPI2C0.mtdr().write(|word| word.set_data(0));";
const PAC_POP: &str = "let _discarded = crate::pac::LPI2C0.mrdr().read();";
type Sources = BTreeMap<String, String>;

fn without_docs(attrs: &mut Vec<Attribute>) {
    attrs.retain(|attr| !attr.path().is_ident("doc"));
}

fn normalized(mut item: Item) -> Item {
    match &mut item {
        Item::Fn(item) => without_docs(&mut item.attrs),
        Item::Struct(item) => without_docs(&mut item.attrs),
        Item::Use(item) => without_docs(&mut item.attrs),
        Item::Mod(item) => without_docs(&mut item.attrs),
        Item::Macro(item) => without_docs(&mut item.attrs),
        Item::Impl(item) => {
            without_docs(&mut item.attrs);
            for member in &mut item.items {
                if let ImplItem::Fn(member) = member {
                    without_docs(&mut member.attrs);
                }
            }
        }
        _ => {}
    }
    item
}

fn same_item(actual: &Item, expected: &str) -> bool {
    normalized(actual.clone()) == normalized(syn::parse_str(expected).expect("reviewed policy syntax"))
}

fn named_type(ty: &syn::Type, name: &str) -> bool {
    matches!(ty, syn::Type::Path(path) if path.qself.is_none()
        && path.path.segments.last().is_some_and(|part| part.ident == name))
}

fn bindings(tree: &UseTree, names: &mut Vec<String>) -> Result<(), String> {
    match tree {
        UseTree::Path(path) => bindings(&path.tree, names)?,
        UseTree::Name(name) => names.push(name.ident.to_string()),
        UseTree::Rename(rename) => names.push(rename.rename.to_string()),
        UseTree::Group(group) => {
            for tree in &group.items {
                bindings(tree, names)?;
            }
        }
        UseTree::Glob(_) => return Err("unreviewed wildcard import in wake-path module".into()),
    }
    Ok(())
}

fn require_item(file: &syn::File, expected: &str, select: impl Fn(&Item) -> bool) -> Result<(), String> {
    let selected: Vec<_> = file.items.iter().filter(|item| select(item)).collect();
    if selected.len() != 1 || !same_item(selected[0], expected) {
        return Err(format!("missing, duplicated, or changed wake-path item: {expected}"));
    }
    Ok(())
}

fn require_import(file: &syn::File, name: &str, expected: &str) -> Result<(), String> {
    let mut selected = Vec::new();
    for item in &file.items {
        if let Item::Use(import) = item {
            let mut names = Vec::new();
            bindings(&import.tree, &mut names)?;
            if names.iter().any(|bound| bound == name) {
                selected.push(item);
            }
        }
    }
    if selected.len() != 1 || !same_item(selected[0], expected) {
        return Err(format!("unreviewed wake-path binding: {name}"));
    }
    Ok(())
}

fn check(sources: &Sources) -> Result<(), String> {
    if sources.len() != INVENTORY.len() || INVENTORY.iter().any(|path| !sources.contains_key(*path)) {
        return Err("wake-path source inventory changed".into());
    }
    let parsed: BTreeMap<_, _> = sources
        .iter()
        .map(|(path, source)| syn::parse_file(source).map(|file| (path.as_str(), file)))
        .collect::<Result<_, _>>()
        .map_err(|error| format!("unparsed wake-path source: {error}"))?;

    for (path, name, expected) in [
        ("src/lib.rs", "i2c", "#[cfg(mcxa_lpi2c)] pub mod i2c;"),
        (
            "src/i2c/mod.rs",
            "one_shot",
            "#[cfg(feature = \"i2c-one-shot\")] pub mod one_shot;",
        ),
        ("src/i2c/mod.rs", "instance_info", "mod instance_info;"),
        ("src/i2c/one_shot/mod.rs", "session", "mod session;"),
        ("src/i2c/one_shot/session.rs", "vocabulary", "mod vocabulary;"),
        ("src/i2c/one_shot/session/vocabulary.rs", "registers", "mod registers;"),
        (REGISTERS, "effects", "mod effects;"),
        (REGISTERS, "binding", "mod binding;"),
    ] {
        require_item(
            &parsed[path],
            expected,
            |item| matches!(item, Item::Mod(item) if item.ident == name),
        )?;
    }
    for (path, expected) in [
        ("src/i2c/one_shot/mod.rs", "pub use session::InterruptHandler;"),
        ("src/i2c/one_shot/session.rs", "pub use vocabulary::InterruptHandler;"),
        (
            "src/i2c/one_shot/session/vocabulary.rs",
            "pub use registers::InterruptHandler;",
        ),
        (REGISTERS, "pub use binding::InterruptHandler;"),
    ] {
        require_import(&parsed[path], "InterruptHandler", expected)?;
    }
    require_import(&parsed["src/i2c/mod.rs"], "Info", "pub(crate) use instance_info::Info;")?;

    let registers = &parsed[REGISTERS];
    let binding = &parsed[BINDING];
    require_import(binding, "PhantomData", "use core::marker::PhantomData;")?;
    require_import(binding, "Instance", "use crate::i2c::{Info, Instance, SclPin, SdaPin};")?;
    require_import(binding, "effects", "use super::effects;")?;
    for imported in ["Binding", "Handler", "Interrupt"] {
        require_import(
            binding,
            imported,
            "use crate::interrupt::typelevel::{Binding, Handler, Interrupt};",
        )?;
    }
    require_item(
        binding,
        "pub struct InterruptHandler<T: Instance>(PhantomData<T>);",
        |item| matches!(item, Item::Struct(item) if item.ident == "InterruptHandler"),
    )?;
    require_item(
        binding,
        "impl<T: Instance> Handler<T::Interrupt> for InterruptHandler<T> {
            unsafe fn on_interrupt() {
                effects::wake_interrupt(T::Interrupt::disable, T::info().wait_cell());
            }
        }",
        |item| matches!(item, Item::Impl(item) if named_type(&item.self_ty, "InterruptHandler")),
    )?;

    let effects = &parsed[EFFECTS];
    require_import(effects, "WaitCell", "use maitake_sync::WaitCell;")?;
    require_item(
        effects,
        "pub(super) fn wake_interrupt(disable: impl FnOnce(), wait: &WaitCell) {
            disable();
            wait.wake();
        }",
        |item| matches!(item, Item::Fn(item) if item.sig.ident == "wake_interrupt"),
    )?;

    let info = &parsed[INFO];
    require_import(info, "WaitCell", "use maitake_sync::WaitCell;")?;
    let mut accessors = Vec::new();
    for item in &info.items {
        if let Item::Impl(item) = item {
            if named_type(&item.self_ty, "Info") {
                for member in &item.items {
                    if let ImplItem::Fn(member) = member {
                        if member.sig.ident == "wait_cell" {
                            if item.attrs.iter().any(|attr| !attr.path().is_ident("doc"))
                                || item.trait_.is_some()
                                || item.unsafety.is_some()
                                || !item.generics.params.is_empty()
                                || item.generics.where_clause.is_some()
                            {
                                return Err("unreviewed wrapper around Info::wait_cell".into());
                            }
                            let mut accessor = member.clone();
                            without_docs(&mut accessor.attrs);
                            accessors.push(accessor);
                        }
                    }
                }
            }
        }
    }
    let expected: syn::ImplItemFn =
        syn::parse_str("#[inline(always)] pub(in crate::i2c) fn wait_cell(&self) -> &WaitCell { &self.wait_cell }")
            .unwrap();
    if accessors != [expected] {
        return Err("Info::wait_cell is no longer a plain shared software accessor".into());
    }

    // T::info() must still return its static software Info. Checking only the
    // handler would miss a PAC operation inserted into this generated callee.
    require_item(
        &parsed["src/i2c/mod.rs"],
        "#[macro_export]
        macro_rules! impl_lpi2c_instance {
            ($n:literal) => {
                paste::paste! {
                    impl crate::i2c::SealedInstance for crate::peripherals::[<LPI2C $n>] {
                        fn info() -> &'static crate::i2c::Info {
                            static INFO: crate::i2c::Info = crate::i2c::Info::new(crate::pac::[<LPI2C $n>]);
                            &INFO
                        }
                        const CLOCK_INSTANCE: crate::clocks::periph_helpers::Lpi2cInstance
                            = crate::clocks::periph_helpers::Lpi2cInstance::[<Lpi2c $n>];
                        const PERF_INT_INCR: fn() = crate::perf_counters::[<incr_interrupt_i2c $n>];
                        const PERF_INT_WAKE_INCR: fn() = crate::perf_counters::[<incr_interrupt_i2c $n _wake>];
                        const TX_DMA_REQUEST: DmaRequest = DmaRequest::[<Lpi2C $n Tx>];
                        const RX_DMA_REQUEST: DmaRequest = DmaRequest::[<Lpi2C $n Rx>];
                    }
                    impl crate::i2c::Instance for crate::peripherals::[<LPI2C $n>] {
                        type Interrupt = crate::interrupt::typelevel::[<LPI2C $n>];
                    }
                }
            };
        }",
        |item| matches!(item, Item::Macro(item) if item.ident.as_ref().is_some_and(|name| name == "impl_lpi2c_instance")),
    )?;

    // These modules route or contain the wake bodies. Their own attributes, module
    // routing and expansions cannot silently replace or wrap the pinned items.
    // The existing register-access macro is deliberately not an open expansion
    // exception: both its body and its only two invocations are inventoried.
    for (path, file) in [(REGISTERS, registers), (BINDING, binding), (EFFECTS, effects)] {
        if file.attrs.iter().any(|attr| !attr.path().is_ident("doc")) {
            return Err(format!("unreviewed module attribute in {path}"));
        }
        let macros: Vec<_> = file
            .items
            .iter()
            .filter(|item| matches!(item, Item::Macro(_)))
            .collect();
        let expected = if path == REGISTERS {
            vec![
                "macro_rules! observable_state {
                    ($state:ty) => {
                        impl ObservableRegisters for mmio::Lpi2cRegisters<$state> {
                            fn status(&self) -> u32 { self.read_msr() }
                            fn interrupt_mask(&mut self, word: u32) { self.write_mier(word); }
                        }
                    };
                }",
                "observable_state!(mmio::Enabled);",
                "observable_state!(mmio::Streaming);",
            ]
        } else {
            vec![]
        };
        if macros.len() != expected.len()
            || macros
                .iter()
                .zip(expected)
                .any(|(item, expected)| !same_item(item, expected))
        {
            return Err(format!("unreviewed macro/include expansion in {path}"));
        }
    }
    Ok(())
}

fn actual_sources() -> Sources {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    INVENTORY
        .iter()
        .map(|relative| {
            let mut path = root.clone();
            for component in relative.split('/') {
                path.push(component);
                assert!(
                    !fs::symlink_metadata(&path)
                        .expect("inventoried path")
                        .file_type()
                        .is_symlink(),
                    "wake-path source must not be redirected by a symlink"
                );
            }
            (
                relative.to_string(),
                fs::read_to_string(path)
                    .expect("inventoried wake-path source")
                    .replace("\r\n", "\n"),
            )
        })
        .collect()
}

fn changed(sources: &Sources, path: &str, from: &str, to: &str) -> Sources {
    let mut result = sources.clone();
    let source = result.get_mut(path).unwrap();
    assert_eq!(source.matches(from).count(), 1, "unique mutation anchor: {from}");
    *source = source.replacen(from, to, 1);
    result
}

#[test]
fn one_shot_irq_executes_only_the_reviewed_wake_path() {
    assert_eq!(check(&actual_sources()), Ok(()), "ONE_SHOT_WAKE_ONLY_IRQ_POLICY");
}

#[test]
fn safe_pac_accesses_and_clock_or_helper_indirection_are_rejected() {
    let sources = actual_sources();
    assert_eq!(check(&sources), Ok(()));
    for operation in [
        PAC_WRITE,
        PAC_POP,
        "crate::clocks::with_clocks(|_| ());",
        "hidden_helper();",
        "unsafe { hidden_helper(); }",
        "include!(\"unreviewed.rs\");",
    ] {
        assert!(
            check(&changed(
                &sources,
                BINDING,
                WAKE_CALL,
                &format!("{operation}\n{WAKE_CALL}")
            ))
            .is_err(),
            "unreviewed IRQ operation accepted: {operation}"
        );
    }
    for operation in [
        "hidden_helper();",
        "crate::pac::LPI2C0.mtdr().write(|word| word.set_data(0));",
    ] {
        assert!(
            check(&changed(
                &sources,
                EFFECTS,
                "    disable();",
                &format!("    {operation}\n    disable();")
            ))
            .is_err()
        );
    }
    assert!(
        check(&changed(
            &sources,
            INFO,
            "        &self.wait_cell",
            "        hidden_helper();\n        &self.wait_cell"
        ))
        .is_err()
    );
}

#[test]
fn redirected_sources_bindings_attributes_and_expansions_are_rejected() {
    let sources = actual_sources();
    for (path, from, to) in [
        (REGISTERS, "mod effects;", "#[path = \"other.rs\"] mod effects;"),
        (REGISTERS, "mod effects;", "mod effects { include!(\"other.rs\"); }"),
        (REGISTERS, "mod effects;", "mod effects; include!(\"other.rs\");"),
        (REGISTERS, "mod binding;", "#[path = \"other.rs\"] mod binding;"),
        (REGISTERS, "mod binding;", "mod binding { include!(\"other.rs\"); }"),
        (
            BINDING,
            "unsafe fn on_interrupt()",
            "#[cfg_attr(all(), unknown)] unsafe fn on_interrupt()",
        ),
        (
            BINDING,
            "use core::marker::PhantomData;",
            "use crate::hidden::PhantomData;",
        ),
        (
            BINDING,
            "use crate::i2c::{Info, Instance, SclPin, SdaPin};",
            "use crate::hidden::{Info, Instance, SclPin, SdaPin};",
        ),
        (BINDING, "use super::effects;", "use crate::hidden::effects;"),
        (
            BINDING,
            "use super::effects;",
            "use super::effects; include!(\"other.rs\");",
        ),
        (EFFECTS, "use maitake_sync::WaitCell;", "use crate::hidden::WaitCell;"),
        (
            EFFECTS,
            "use maitake_sync::WaitCell;",
            "use maitake_sync::WaitCell; use hidden::*;",
        ),
        (INFO, "impl Info {", "#[unreviewed::rewrite] impl Info {"),
        (
            EFFECTS,
            "    disable();\n    wait.wake();",
            "    wait.wake();\n    disable();",
        ),
        (
            "src/i2c/one_shot/mod.rs",
            "mod session;",
            "#[path = \"hidden.rs\"] mod session;",
        ),
        (
            "src/i2c/mod.rs",
            "                    &INFO",
            "                    hidden_helper();\n                    &INFO",
        ),
    ] {
        assert!(
            check(&changed(&sources, path, from, to)).is_err(),
            "unreviewed source accepted: {to}"
        );
    }
    let mut unreviewed = sources;
    unreviewed.insert("src/i2c/one_shot/hidden.rs".into(), "".into());
    assert!(check(&unreviewed).is_err());
}

#[test]
fn ordinary_comments_and_formatting_do_not_change_policy() {
    let sources = actual_sources();
    let changed = changed(
        &sources,
        BINDING,
        WAKE_CALL,
        "// Reviewed explanation may improve without weakening the gate.\n effects :: wake_interrupt ( T::Interrupt::disable, T::info().wait_cell() );",
    );
    assert_eq!(check(&changed), Ok(()));
}

#[test]
fn compile_control_uses_the_same_pac_operations_under_forbid_unsafe_code() {
    // This checks fixture identity, not a compiler result. The documented
    // separate ARM cargo-check command is the real compiler control.
    let source = include_str!("fixtures/pac-bypass-control/src/lib.rs").replace("embassy_mcxa::", "crate::");
    let parsed = syn::parse_file(&source).unwrap();
    let expected = syn::parse_file("#![no_std]\n#![forbid(unsafe_code)]").unwrap();
    let mut attrs = parsed.attrs.clone();
    without_docs(&mut attrs);
    assert_eq!(attrs, expected.attrs);
    require_item(
        &parsed,
        &format!("pub fn safe_pac_access_compiles() {{ {PAC_WRITE} {PAC_POP} }}"),
        |item| matches!(item, Item::Fn(item) if item.sig.ident == "safe_pac_access_compiles"),
    )
    .unwrap();
}
