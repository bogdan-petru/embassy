//! Independent source contract check; does not generate any driver code.
//! Actual expansion, ownership rejection and effect ordering are separate tests.
use std::collections::BTreeSet;
use syn::{Token, parse::Parse, parse::ParseStream, punctuated::Punctuated};

struct Access {
    states: Vec<syn::Ident>,
    mode: syn::Ident,
}
impl Parse for Access {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        let states;
        syn::bracketed!(states in input);
        let states = Punctuated::<syn::Ident, Token![,]>::parse_terminated(&states)?
            .into_iter()
            .collect();
        input.parse::<Token![,]>()?;
        let mode = input.parse()?;
        if !input.is_empty() {
            return Err(input.error("unexpected access declaration"));
        }
        Ok(Self { states, mode })
    }
}

fn check(ip: &str, relative: &str, expected_offsets: &[(&str, usize)]) {
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let source = std::fs::read_to_string(root.join(relative)).unwrap();
    let file = syn::parse_file(&source).unwrap();
    let layout = file
        .items
        .iter()
        .find_map(|item| match item {
            syn::Item::Struct(item) if item.ident == "Layout" => Some(item),
            _ => None,
        })
        .unwrap();
    assert_eq!(
        layout
            .attrs
            .iter()
            .filter(|a| a.path().is_ident("mmio_initial_state"))
            .count(),
        1
    );
    assert_eq!(
        layout
            .attrs
            .iter()
            .find(|a| a.path().is_ident("mmio_initial_state"))
            .unwrap()
            .parse_args::<syn::Ident>()
            .unwrap(),
        "Unconfirmed"
    );
    let contract = driver_contracts::contracts::get(ip).unwrap();
    let mut actual = BTreeSet::new();
    let mut offsets = Vec::new();
    let mut position = 0;
    for field in &layout.fields {
        let name = field.ident.as_ref().unwrap().to_string();
        if name.starts_with("_reserved") {
            let syn::Type::Array(array) = &field.ty else {
                panic!("padding array required")
            };
            let syn::Expr::Lit(length) = &array.len else {
                panic!("literal padding required")
            };
            let syn::Lit::Int(length) = &length.lit else {
                panic!("integer padding required")
            };
            position += length.base10_parse::<usize>().unwrap();
            continue;
        }
        assert_eq!(position % 4, 0);
        offsets.push((name.clone(), position));
        position += 4;
        let declared = contract.registers.iter().find(|r| r.name == name).unwrap();
        let syn::Type::Path(ty) = &field.ty else {
            panic!("field type required")
        };
        assert_eq!(ty.path.segments.last().unwrap().ident, declared.mode);
        let private = field.attrs.iter().any(|a| a.path().is_ident("mmio_private"));
        let mut count = 0;
        for attribute in field.attrs.iter().filter(|a| a.path().is_ident("RegAttributes")) {
            count += 1;
            let access: Access = attribute.parse_args().unwrap();
            for state in access.states {
                assert_ne!(state, "Unconfirmed", "unconfirmed acquisition grants no operation");
                let (kind, method) = match access.mode.to_string().as_str() {
                    "ReadOnly" if declared.mode == "ReadPure" => ("read_shared", format!("read_{name}")),
                    "ReadOnly" => ("read_exclusive", format!("read_{name}")),
                    "WriteOnly" => ("write", format!("write_{name}")),
                    _ => panic!("unexpected/broadened accessor"),
                };
                assert!(actual.insert((state.to_string(), kind.to_owned(), method)));
            }
        }
        assert_eq!(private, count == 0, "private fields cannot also expose accessors");
    }
    let expected: BTreeSet<_> = contract
        .registers
        .iter()
        .flat_map(|r| &r.direct)
        .filter(|d| d.kind != "transition")
        .map(|d| (d.state.to_owned(), d.kind.to_owned(), d.method.to_owned()))
        .collect();
    assert_eq!(
        actual, expected,
        "Abacus surface differs from independently reviewed contract"
    );
    assert_eq!(
        offsets,
        expected_offsets
            .iter()
            .map(|(n, o)| (n.to_string(), *o))
            .collect::<Vec<_>>()
    );
}

#[test]
fn i2c_abacus_surface_matches_reviewed_policy() {
    check(
        "i2c",
        "src/i2c/one_shot/session/vocabulary/registers/mmio.rs",
        &[
            ("mcr", 0x10),
            ("msr", 0x14),
            ("mier", 0x18),
            ("mder", 0x1c),
            ("mcfgr1", 0x24),
            ("mccr0", 0x48),
            ("mfsr", 0x5c),
            ("mtdr", 0x60),
            ("mrdr", 0x70),
        ],
    );
}
#[test]
fn uart_abacus_surface_matches_reviewed_policy() {
    check(
        "uart",
        "src/lpuart/one_shot/hardware/mmio.rs",
        &[
            ("global", 8),
            ("baud", 0x10),
            ("stat", 0x14),
            ("ctrl", 0x18),
            ("data", 0x1c),
            ("fifo", 0x28),
            ("water", 0x2c),
        ],
    );
}
