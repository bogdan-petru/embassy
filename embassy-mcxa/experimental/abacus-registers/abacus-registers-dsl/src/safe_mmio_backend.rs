//! Local experimental backend. Reuses Abacus's parser and state/enum generator;
//! changes only register storage/access emission. No Tock cells or MMIO references.
use std::collections::HashSet;

use proc_macro2::TokenStream;
use quote::{format_ident, quote};
use syn::{Fields, GenericArgument, ItemStruct, PathArguments, Type};

use crate::macro_input::MacroInput;
use crate::parsing::{RegisterAttributes, RegisterType, State};

pub fn expand(attr: TokenStream, item: TokenStream) -> syn::Result<TokenStream> {
    let input: MacroInput = syn::parse2(attr)?;
    let mut block: ItemStruct = syn::parse2(item)?;
    if !block.generics.params.is_empty() || block.generics.where_clause.is_some() {
        return Err(syn::Error::new_spanned(
            &block.generics,
            "MMIO block must not be generic",
        ));
    }
    let mut initial_state: Option<syn::Ident> = None;
    for attribute in &block.attrs {
        if attribute.path().is_ident("mmio_initial_state") {
            if initial_state.replace(attribute.parse_args()?).is_some() {
                return Err(syn::Error::new_spanned(attribute, "duplicate initial state"));
            }
            continue;
        }
        if attribute.path().is_ident("doc") {
            continue;
        }
        // Do not silently erase cfg/packed/align attributes: they can change
        // which bytes a generated operation addresses.
        if attribute.path().is_ident("repr") && attribute.parse_args::<syn::Ident>().is_ok_and(|ident| ident == "C") {
            continue;
        }
        return Err(syn::Error::new_spanned(
            attribute,
            "only doc and repr(C) block attributes are supported",
        ));
    }
    let mut states = HashSet::new();
    for definition in &input.state_definitions {
        if definition.transient || !definition.state.substates.is_empty() {
            return Err(syn::Error::new_spanned(
                &definition.state.state_name,
                "experimental safe-mmio backend supports flat stable states only",
            ));
        }
        if !states.insert(definition.state.state_name.to_string()) {
            return Err(syn::Error::new_spanned(&definition.state.state_name, "duplicate state"));
        }
    }
    if states.is_empty() {
        return Err(syn::Error::new_spanned(&block.ident, "at least one state is required"));
    }
    if initial_state
        .as_ref()
        .is_some_and(|state| !states.contains(&state.to_string()))
    {
        return Err(syn::Error::new_spanned(&initial_state, "undeclared initial state"));
    }
    let check_state = |state: &State| -> syn::Result<()> {
        if !state.substates.is_empty() || !states.contains(&state.state_name.to_string()) {
            return Err(syn::Error::new_spanned(
                &state.state_name,
                "undeclared or unsupported state",
            ));
        }
        Ok(())
    };
    // The generated block is a layout description, never instantiated at MMIO.
    let block_name = block.ident.clone();
    block.vis = syn::Visibility::Inherited;
    block.attrs = vec![syn::parse_quote!(#[repr(C)])];
    let Fields::Named(fields) = &mut block.fields else {
        return Err(syn::Error::new_spanned(&block, "named fields required"));
    };
    let register = format_ident!("{}Registers", input.peripheral_name);
    let state_enum = format_ident!("{}StateEnum", input.peripheral_name);
    let mut methods = TokenStream::new();
    let mut offsets = TokenStream::new();
    let mut access_rows = Vec::new();
    let mut emitted = HashSet::new();
    let mut shared_fields = Vec::new();
    let mut shared_types = Vec::new();
    let mut shared_offsets = Vec::new();
    let mut needs_transient_shared = false;
    // Collect handles before emitting any consuming transition, including when
    // its source register occurs before the pure fields in the declaration.
    // The validation loop below still checks every collected field's contract.
    for field in &fields.named {
        let name = field.ident.as_ref().unwrap();
        if matches!(&field.ty, Type::Path(path) if path.path.segments.last().is_some_and(|segment| segment.ident == "ReadPure"))
        {
            shared_fields.push(format_ident!("__read_pure_{}", name));
            shared_types.push(quote!(safe_mmio::fields::ReadPure<u32>));
            shared_offsets.push(quote!(core::mem::offset_of!(#block_name, #name)));
        }
    }
    for field in &mut fields.named {
        let name = field.ident.clone().unwrap();
        field.vis = syn::Visibility::Inherited;
        let annotations: Vec<RegisterAttributes> = field
            .attrs
            .iter()
            .filter(|a| a.path().is_ident("RegAttributes"))
            .map(|a| a.parse_args())
            .collect::<syn::Result<_>>()?;
        let private = field.attrs.iter().any(|a| a.path().is_ident("mmio_private"));
        if field.attrs.iter().filter(|a| a.path().is_ident("mmio_private")).count() > 1
            || field
                .attrs
                .iter()
                .any(|a| a.path().is_ident("mmio_private") && !matches!(a.meta, syn::Meta::Path(_)))
        {
            return Err(syn::Error::new_spanned(
                &field,
                "expected one bare mmio_private attribute",
            ));
        }
        if field.attrs.iter().any(|a| {
            !a.path().is_ident("RegAttributes") && !a.path().is_ident("doc") && !a.path().is_ident("mmio_private")
        }) {
            return Err(syn::Error::new_spanned(field, "unsupported field attribute"));
        }
        field.attrs.retain(|a| a.path().is_ident("doc"));
        if name.to_string().starts_with("_reserved") {
            if !annotations.is_empty() || private {
                return Err(syn::Error::new_spanned(name, "reserved padding cannot have operations"));
            }
            if !matches!(&field.ty, Type::Array(array) if matches!(&*array.elem, Type::Path(path) if path.path.is_ident("u8")))
            {
                return Err(syn::Error::new_spanned(
                    &field.ty,
                    "reserved padding must be a u8 array",
                ));
            }
            continue;
        }
        if private && !annotations.is_empty() {
            return Err(syn::Error::new_spanned(&name, "private field cannot expose operations"));
        }
        if annotations.is_empty() && !private {
            return Err(syn::Error::new_spanned(
                name,
                "register requires an explicit access contract",
            ));
        }
        let Type::Path(path) = &field.ty else {
            return Err(syn::Error::new_spanned(&field.ty, "expected safe-mmio field type"));
        };
        let segment = path.path.segments.last().unwrap();
        let kind = segment.ident.to_string();
        let PathArguments::AngleBracketed(args) = &segment.arguments else {
            return Err(syn::Error::new_spanned(&field.ty, "expected a u32 field"));
        };
        if args.args.len() != 1
            || !matches!(args.args.first(), Some(GenericArgument::Type(Type::Path(p))) if p.path.is_ident("u32"))
        {
            return Err(syn::Error::new_spanned(
                &field.ty,
                "experimental backend supports u32 fields only",
            ));
        }
        if !["ReadPure", "ReadOnly", "WriteOnly", "ReadWrite", "ReadPureWrite"].contains(&kind.as_str()) {
            return Err(syn::Error::new_spanned(&field.ty, "unsupported safe-mmio field type"));
        }
        let kind_ident = segment.ident.clone();
        // Canonicalize the field type: do not trust a user-defined alias with a
        // familiar name but different layout or access semantics.
        field.ty = syn::parse_quote!(safe_mmio::fields::#kind_ident<u32>);
        let offset = quote!(core::mem::offset_of!(#block_name, #name));
        let constant = format_ident!("{}_OFFSET", name.to_string().to_uppercase());
        offsets.extend(quote! { #[allow(dead_code)] const #constant: usize = #offset; });
        let label = name.to_string();
        let readable = kind != "WriteOnly";
        let writable = !["ReadOnly", "ReadPure"].contains(&kind.as_str());
        access_rows.push(quote! { (#label, #constant, #readable, #writable) });
        let field_type = &field.ty;
        let shared_field = format_ident!("__read_pure_{}", name);
        for annotation in annotations {
            if annotation.valid_states.is_empty() {
                return Err(syn::Error::new_spanned(&name, "access contract needs a state"));
            }
            for from in &annotation.valid_states {
                check_state(from)?;
                let from = &from.state_name;
                let mut emit = |method: syn::Ident, body: TokenStream| -> syn::Result<()> {
                    if !emitted.insert((from.to_string(), method.to_string())) {
                        return Err(syn::Error::new_spanned(method, "duplicate state operation"));
                    }
                    methods.extend(quote! { impl #register<#from> { #body } });
                    Ok(())
                };
                match &annotation.register_type {
                    RegisterType::StateChange(to, builders, shortname) => {
                        check_state(to)?;
                        if !["WriteOnly", "ReadWrite", "ReadPureWrite"].contains(&kind.as_str()) || builders.len() != 1
                        {
                            return Err(syn::Error::new_spanned(
                                &name,
                                "StateChange needs a writable field and one u32 builder function",
                            ));
                        }
                        let builder = builders.first().unwrap();
                        let to = &to.state_name;
                        let method =
                            format_ident!("into_{}", shortname.as_ref().unwrap_or(to).to_string().to_lowercase());
                        emit(
                            method.clone(),
                            quote! {
                                pub fn #method(mut self) -> #register<#to> {
                                    let word: u32 = #builder();
                                    // SAFETY: generated field offset/type belong to this owned
                                    // block. The temporary exclusive access ends before moving it.
                                    unsafe { self.field_mut::<#field_type>(#offset) }.write(word);
                                    #register { base: self.base, #(#shared_fields: self.#shared_fields,)* state: PhantomData, _not_send_sync: PhantomData }
                                }
                            },
                        )?;
                    }
                    mode => {
                        let read = matches!(mode, RegisterType::ReadOnly | RegisterType::ReadWrite);
                        let write = matches!(mode, RegisterType::WriteOnly | RegisterType::ReadWrite);
                        if (read && kind == "WriteOnly") || (write && ["ReadPure", "ReadOnly"].contains(&kind.as_str()))
                        {
                            return Err(syn::Error::new_spanned(
                                &name,
                                "access contract exceeds the field's access mode",
                            ));
                        }
                        if read {
                            let method = format_ident!("read_{}", name);
                            let body = if kind == "ReadPure" {
                                quote! {
                                    pub fn #method(&self) -> u32 {
                                        self.#shared_field.read()
                                    }
                                }
                            } else if kind == "ReadPureWrite" {
                                needs_transient_shared = true;
                                quote! {
                                    pub fn #method(&self) -> u32 {
                                        // SAFETY: this CPU-writable field must keep its shared
                                        // read handle transient, separate from exclusive writes.
                                        unsafe { self.field_shared::<#field_type>(#offset) }.read()
                                    }
                                }
                            } else {
                                quote! {
                                    pub fn #method(&mut self) -> u32 {
                                        // SAFETY: destructive reads require exclusive access.
                                        unsafe { self.field_mut::<#field_type>(#offset) }.read()
                                    }
                                }
                            };
                            emit(method, body)?;
                        }
                        if write {
                            let method = format_ident!("write_{}", name);
                            emit(
                                method.clone(),
                                quote! {
                                    pub fn #method(&mut self, word: u32) {
                                        // SAFETY: this generated writable field belongs to the
                                        // owned block and is borrowed exclusively for this access.
                                        unsafe { self.field_mut::<#field_type>(#offset) }.write(word);
                                    }
                                },
                            )?;
                        }
                    }
                }
            }
        }
    }
    let (enum_tokens, state_tokens, imports, constructor_impl) = if let Some(initial) = &initial_state {
        // Closed, flat states for hardware-confirmed ownership. No generic
        // constructor, state erasure, or arbitrary state restoration is emitted.
        let names: Vec<_> = input.state_definitions.iter().map(|d| &d.state.state_name).collect();
        (
            quote! {},
            quote! { pub(super) trait State {} #(pub(super) enum #names {} impl State for #names {})* },
            quote! {},
            quote! { impl #register<#initial> },
        )
    } else {
        let (state_tokens, _) = input.generate_states(&register, &state_enum);
        (
            crate::generating::generate_state_enum(&input.state_definitions, &register, &state_enum),
            state_tokens,
            quote! { use abacus_registers::{State, SubState, StateEnum, Reg}; },
            quote! { impl<S: State> #register<S> },
        )
    };
    let transient_shared = needs_transient_shared.then(|| {
        quote! {
            unsafe fn field_shared<T>(&self, offset: usize) -> safe_mmio::SharedMmioPointer<'_, T> {
                let ptr = self.base.as_ptr().wrapping_add(offset).cast::<T>();
                // SAFETY: only generated ReadPureWrite getters call this helper.
                // No retained shared handle covers these CPU-writable registers.
                // The constructor excludes competing access during this temporary
                // unique acquisition; the !Sync owner and synchronous getters keep
                // ordinary safe calls from overlapping it. No handle escapes.
                unsafe { safe_mmio::UniqueMmioPointer::new(core::ptr::NonNull::new_unchecked(ptr)) }.into()
            }
        }
    });
    Ok(quote! {
        #imports
        use core::marker::PhantomData;
        #block
        #offsets
        #[allow(dead_code)]
        pub(super) const REGISTER_ACCESS: &[(&str, usize, bool, bool)] = &[#(#access_rows),*];
        #[allow(dead_code)]
        pub(super) const REGION_LEN: usize = core::mem::size_of::<#block_name>();
        // This owner is not Copy/Clone and does not Deref to device memory.
        // Only never-CPU-written ReadPure registers retain shared handles.
        // Exclusive pointers always cover one other field, never the full block.
        pub struct #register<S: State> {
            base: core::ptr::NonNull<u8>,
            // Lifetime-erased raw handles, not MMIO references. These stay private,
            // move with the owner, and can only be used while its mapping is live.
            // The constructor's runtime validity obligation is not inferred from
            // this internal 'static marker; no static observer is exposed.
            #(#shared_fields: safe_mmio::SharedMmioPointer<'static, #shared_types>,)*
            state: PhantomData<S>,
            _not_send_sync: PhantomData<*mut ()>,
        }
        #constructor_impl {
            /// # Safety
            /// The base must be aligned for the generated layout; all mapped
            /// registers must remain valid for the owner's entire lifetime.
            /// Exclusive CPU ownership of a correctly laid-out live device (or
            /// suitably aligned, exclusively owned test memory) is required.
            /// The entire backing region must remain valid for every subsequent
            /// use of this owner; this raw acquisition does not track its lifetime.
            /// Its actual hardware state must be S and the declared read purity,
            /// access modes and transition words must match the hardware.
            /// Initialization must exclude competing CPU/IRQ/DMA accesses while
            /// each ReadPure field is acquired uniquely and converted to shared.
            /// Afterwards ReadPure fields must have no CPU writer or destructive
            /// reader, and no other accessor may claim exclusive access to them.
            /// External DMA/ISR access to other fields must not overlap a CPU
            /// access to that same field. A ReadPureWrite read still acquires
            /// uniqueness transiently, so it excludes even another pure reader.
            /// Mapping, clocks and power must remain valid until the owner and
            /// any consuming state transitions have finished using the device.
            pub unsafe fn new(base: *mut u8) -> Self {
                let base = core::ptr::NonNull::new(base).expect("null MMIO");
                #(
                    let #shared_fields = {
                        let ptr = base.as_ptr().wrapping_add(#shared_offsets).cast::<#shared_types>();
                        // SAFETY: construction is exclusive, and this generated
                        // pointer covers only its declared never-CPU-written field.
                        // Conversion consumes uniqueness once; no future CPU write
                        // or exclusive acquisition may overlap the retained handle.
                        unsafe { safe_mmio::UniqueMmioPointer::new(core::ptr::NonNull::new_unchecked(ptr)) }.into()
                    };
                )*
                Self { base, #(#shared_fields,)* state: PhantomData, _not_send_sync: PhantomData }
            }
        }
        impl<S: State> #register<S> {
            unsafe fn field_mut<T>(&mut self, offset: usize) -> safe_mmio::UniqueMmioPointer<'_, T> {
                let ptr = self.base.as_ptr().wrapping_add(offset).cast::<T>();
                // SAFETY: only generated, layout-checked field offsets call this
                // helper. The constructor and exclusive borrow provide ownership.
                unsafe { safe_mmio::UniqueMmioPointer::new(core::ptr::NonNull::new_unchecked(ptr)) }
            }
            #transient_shared
        }
        #enum_tokens
        #state_tokens
        #methods
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn restricted_acquisition_and_private_registers_grant_no_extra_authority() {
        let file: syn::File = syn::parse2(
            expand(
                header(),
                quote! {
                    #[mmio_initial_state(Off)]
                    struct Layout {
                        #[mmio_private]
                        control: ReadWrite<u32>,
                        #[RegAttributes([On], ReadOnly)]
                        status: ReadPure<u32>,
                    }
                },
            )
            .unwrap(),
        )
        .unwrap();
        let constructors: Vec<_> = file
            .items
            .iter()
            .filter_map(|item| {
                let syn::Item::Impl(item) = item else { return None };
                item.items
                    .iter()
                    .any(|i| matches!(i, syn::ImplItem::Fn(f) if f.sig.ident == "new"))
                    .then_some(item)
            })
            .collect();
        assert_eq!(constructors.len(), 1);
        assert!(
            constructors[0].generics.params.is_empty(),
            "no constructor for arbitrary states"
        );
        let ty = &constructors[0].self_ty;
        assert_eq!(quote!(#ty).to_string(), "DemoRegisters < Off >");
        assert!(method_bodies(&file, "write_control").is_empty());
        assert!(method_bodies(&file, "read_control").is_empty());
        let text = quote!(#file).to_string();
        assert!(text.contains("_not_send_sync : PhantomData < * mut () >"));
        assert!(!text.contains("StateEnum"), "restricted owner must not erase its state");
    }

    #[test]
    fn rejects_ambiguous_private_and_initial_state_contracts() {
        for item in [
            quote!(
                #[mmio_initial_state(Missing)]
                struct L {
                    #[mmio_private]
                    x: WriteOnly<u32>,
                }
            ),
            quote!(
                #[mmio_initial_state(Off)]
                #[mmio_initial_state(On)]
                struct L {
                    #[mmio_private]
                    x: WriteOnly<u32>,
                }
            ),
            quote!(
                struct L {
                    #[mmio_private]
                    #[RegAttributes([Off], WriteOnly)]
                    x: WriteOnly<u32>,
                }
            ),
            quote!(
                struct L {
                    #[mmio_private(extra)]
                    x: WriteOnly<u32>,
                }
            ),
            quote!(
                struct L {
                    #[mmio_private]
                    #[mmio_private]
                    x: WriteOnly<u32>,
                }
            ),
            quote!(
                struct L {
                    #[mmio_private]
                    _reserved0: [u8; 4],
                }
            ),
        ] {
            assert!(expand(header(), item).is_err());
        }
    }
    fn header() -> TokenStream {
        quote!(peripheral_name = "Demo", register_base_addr = 0, states = [(Off), (On)])
    }
    #[test]
    fn produces_owned_access_not_tock_cells_or_reference_casts() {
        let code = expand(
            header(),
            quote! { struct Layout {
                #[RegAttributes([Off], StateChange(On, [enable]))]
                control: WriteOnly<u32>,
                #[RegAttributes([Off, On], ReadOnly)]
                status: ReadPure<u32>,
                #[RegAttributes([On], ReadOnly)]
                fifo: ReadOnly<u32>,
            }},
        )
        .unwrap()
        .to_string();
        assert!(code.contains("UniqueMmioPointer"));
        assert!(code.contains("SharedMmioPointer"));
        assert!(code.contains("read_fifo (& mut self)"));
        assert!(code.contains("read_status (& self)"));
        assert!(code.contains("impl DemoRegisters < Off >"));
        assert!(!code.contains("transmute"));
        assert!(!code.contains("AbacusStaticRef"));
    }

    fn method_bodies(file: &syn::File, name: &str) -> Vec<String> {
        file.items
            .iter()
            .filter_map(|item| match item {
                syn::Item::Impl(item) => Some(item),
                _ => None,
            })
            .flat_map(|item| &item.items)
            .filter_map(|item| match item {
                syn::ImplItem::Fn(method) if method.sig.ident == name => {
                    let body = &method.block;
                    Some(quote!(#body).to_string())
                }
                _ => None,
            })
            .collect()
    }

    #[test]
    fn read_pure_handles_are_acquired_once_and_preserved_by_every_transition() {
        let file: syn::File = syn::parse2(
            expand(
                header(),
                quote! { struct Layout {
                    #[RegAttributes([Off], StateChange(On, [enable]))]
                    #[RegAttributes([On], StateChange(Off, [disable]))]
                    control: WriteOnly<u32>,
                    #[RegAttributes([Off, On], ReadOnly)]
                    status: ReadPure<u32>,
                    #[RegAttributes([On], ReadOnly)]
                    fifo_count: ReadPure<u32>,
                }},
            )
            .unwrap(),
        )
        .unwrap();
        let owner = file
            .items
            .iter()
            .find_map(|item| match item {
                syn::Item::Struct(item) if item.ident == "DemoRegisters" => Some(item),
                _ => None,
            })
            .unwrap();
        let handles: Vec<_> = owner
            .fields
            .iter()
            .filter(|field| {
                field
                    .ident
                    .as_ref()
                    .is_some_and(|ident| ident.to_string().starts_with("__read_pure_"))
            })
            .collect();
        assert_eq!(handles.len(), 2, "one handle per field, not per allowed state");
        for field in handles {
            assert!(
                matches!(field.vis, syn::Visibility::Inherited),
                "a retained observer must stay private"
            );
            let ty = &field.ty;
            assert!(quote!(#ty).to_string().contains("SharedMmioPointer"));
        }
        let constructors = method_bodies(&file, "new");
        assert_eq!(constructors.len(), 1);
        assert_eq!(constructors[0].matches("UniqueMmioPointer :: new").count(), 2);
        assert_eq!(constructors[0].matches(". into ()").count(), 2);
        for name in ["read_status", "read_fifo_count"] {
            for body in method_bodies(&file, name) {
                assert!(body.contains("self . __read_pure_"));
                assert!(!body.contains("unsafe"));
                assert!(!body.contains("field_shared"));
                assert!(!body.contains("UniqueMmioPointer"));
            }
        }
        for name in ["into_on", "into_off"] {
            let bodies = method_bodies(&file, name);
            assert_eq!(bodies.len(), 1);
            for field in ["__read_pure_status", "__read_pure_fifo_count"] {
                assert!(bodies[0].contains(&format!("{field} : self . {field}")));
            }
            assert!(
                !bodies[0].contains(":: new"),
                "transitions must preserve, not reacquire"
            );
        }
        assert!(
            method_bodies(&file, "field_shared").is_empty(),
            "no transient shared helper without CPU-writable pure reads"
        );
    }

    #[test]
    fn read_pure_write_never_retains_a_shared_handle() {
        let code = expand(
            header(),
            quote! { struct Layout {
                #[RegAttributes([Off, On], ReadWrite)]
                control: ReadPureWrite<u32>,
                #[RegAttributes([On], ReadOnly)]
                readable: ReadPureWrite<u32>,
            }},
        )
        .unwrap();
        assert!(!code.to_string().contains("__read_pure_"));
        let file = syn::parse2(code).unwrap();
        for name in ["read_control", "read_readable"] {
            let bodies = method_bodies(&file, name);
            assert!(!bodies.is_empty());
            assert!(bodies.iter().all(|body| body.contains("self . field_shared")));
        }
        assert!(method_bodies(&file, "write_control")
            .iter()
            .all(|body| body.contains("self . field_mut")));
        assert_eq!(method_bodies(&file, "field_shared").len(), 1);
        assert!(!method_bodies(&file, "new")[0].contains("UniqueMmioPointer"));
    }

    #[test]
    fn multiple_read_contracts_do_not_duplicate_a_retained_handle() {
        let code = expand(
            header(),
            quote! { struct Layout {
                #[RegAttributes([Off], ReadOnly)]
                #[RegAttributes([On], ReadOnly)]
                status: ReadPure<u32>,
            }},
        )
        .unwrap();
        let file = syn::parse2(code).unwrap();
        assert_eq!(method_bodies(&file, "read_status").len(), 2);
        assert_eq!(
            method_bodies(&file, "new")[0]
                .matches("UniqueMmioPointer :: new")
                .count(),
            1
        );
    }
    #[test]
    fn rejects_access_broadening_unknown_states_and_unannotated_fields() {
        for item in [
            quote!(
                struct L {
                    #[RegAttributes([On], WriteOnly)]
                    a: ReadOnly<u32>,
                }
            ),
            quote!(
                struct L {
                    #[RegAttributes([Missing], ReadOnly)]
                    a: ReadPure<u32>,
                }
            ),
            quote!(
                struct L {
                    a: ReadPure<u32>,
                }
            ),
            quote!(
                struct L {
                    #[RegAttributes([], ReadOnly)]
                    a: ReadPure<u32>,
                }
            ),
        ] {
            assert!(expand(header(), item).is_err());
        }
    }
    #[test]
    fn rejects_unsupported_transient_state_instead_of_treating_it_as_stable() {
        let header = quote!(peripheral_name = "Demo", register_base_addr = 0, states = [(On, *T*)]);
        assert!(expand(
            header,
            quote!(
                struct L {
                    _reserved0: [u8; 4],
                }
            )
        )
        .is_err());
    }
    #[test]
    fn rejects_layout_changing_attributes_and_nonbyte_padding() {
        for item in [
            quote!(
                #[repr(C, packed)]
                struct L {
                    _reserved0: [u8; 4],
                }
            ),
            quote!(
                #[cfg(feature = "anything")]
                struct L {
                    _reserved0: [u8; 4],
                }
            ),
            quote!(
                struct L {
                    _reserved0: u32,
                }
            ),
            quote!(
                struct L {
                    _reserved0: [u64; 1],
                }
            ),
            quote!(
                struct L {
                    #[cfg(feature = "anything")]
                    _reserved0: [u8; 4],
                }
            ),
        ] {
            assert!(expand(header(), item).is_err());
        }
        assert!(expand(
            header(),
            quote!(
                #[repr(C)]
                struct L {
                    _reserved0: [u8; 4],
                }
            )
        )
        .is_ok());
    }
}
