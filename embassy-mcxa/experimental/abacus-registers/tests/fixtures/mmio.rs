// Shared by executing host tests and compile-fail documentation tests. This
// exercises real macro expansion, not a handwritten equivalent of its API.
// Exactly one provider is included per executable, including each negative
// doctest: neither a missing nor a duplicate provider may mask a lost rejection.
pub mod demo {
    use abacus_registers::process_mmio_block;

    fn enable_word() -> u32 {
        1
    }
    fn disable_word() -> u32 {
        0
    }

    #[process_mmio_block(
        peripheral_name = "Demo",
        register_base_addr = 0,
        states = [(Off), (On)]
    )]
    #[repr(C)]
    struct Layout {
        #[RegAttributes([Off], StateChange(On, [enable_word], enable))]
        #[RegAttributes([On], StateChange(Off, [disable_word], disable))]
        control: WriteOnly<u32>,
        #[RegAttributes([Off], WriteOnly)]
        timing: WriteOnly<u32>,
        _reserved0: [u8; 4],
        #[RegAttributes([On], ReadOnly)]
        status: ReadPure<u32>,
        #[RegAttributes([On], WriteOnly)]
        tx: WriteOnly<u32>,
        #[RegAttributes([On], ReadOnly)]
        rx: ReadOnly<u32>,
    }

    // Offsets are taken from the macro's actual private layout. The independently
    // reviewed direction expectations do not broaden its generated API.
    pub(super) const REGISTERS: [(usize, &str, bool); 5] = [
        (core::mem::offset_of!(Layout, control), "control", true),
        (core::mem::offset_of!(Layout, timing), "timing", true),
        (core::mem::offset_of!(Layout, status), "status", false),
        (core::mem::offset_of!(Layout, tx), "tx", true),
        (core::mem::offset_of!(Layout, rx), "rx", false),
    ];
}

// The executable fixture is intentionally host-only. It permits no access to
// physical addresses: even pass-through requires a registered, live allocation.
#[allow(dead_code)]
pub mod ram {
    use std::cell::RefCell;
    use std::marker::PhantomData;
    use std::ptr::NonNull;

    #[derive(Debug, Clone, PartialEq, Eq)]
    pub enum Access {
        Read(&'static str, u32),
        Write(&'static str, u32),
    }

    struct Region {
        base: usize,
        log: Vec<Access>,
    }

    std::thread_local! {
        static REGION: RefCell<Option<Region>> = const { RefCell::new(None) };
    }

    /// Owns the host RAM and its thread-local registration. Drop any generated
    /// MMIO owner before finishing this fixture, as required by its unsafe new.
    pub struct Fixture {
        backing: NonNull<[u32; 6]>,
        _not_send: PhantomData<*mut ()>,
    }

    impl Fixture {
        pub fn new(words: [u32; 6]) -> Self {
            REGION.with(|slot| assert!(slot.borrow().is_none(), "nested MMIO fixture"));
            let backing = NonNull::new(Box::into_raw(Box::new(words))).unwrap();
            REGION.with(|slot| {
                *slot.borrow_mut() = Some(Region {
                    base: backing.as_ptr() as usize,
                    log: Vec::new(),
                })
            });
            Self {
                backing,
                _not_send: PhantomData,
            }
        }

        pub fn base(&self) -> *mut u8 {
            self.backing.as_ptr().cast()
        }

        pub fn accesses(&self) -> Vec<Access> {
            REGION.with(|slot| slot.borrow().as_ref().expect("unregistered MMIO fixture").log.clone())
        }

        pub fn finish(self) -> [u32; 6] {
            // SAFETY: this allocation is still live and exclusively owned. The
            // caller of the generated unsafe constructor must end MMIO accesses
            // before inspecting the backing store or dropping its fixture.
            unsafe { self.backing.as_ptr().read() }
        }
    }

    impl Drop for Fixture {
        fn drop(&mut self) {
            // Unregister before freeing: a stale access fails validation before
            // reaching volatile operations, including while another test runs.
            REGION.with(|slot| {
                let region = slot.borrow_mut().take().expect("unregistered MMIO fixture");
                assert_eq!(region.base, self.backing.as_ptr() as usize);
            });
            // SAFETY: exactly the allocation created by Fixture::new; no alias
            // can free it, and generated owners must already be relinquished.
            drop(unsafe { Box::from_raw(self.backing.as_ptr()) });
        }
    }

    /// Validate without touching the pointer. Public so rejection tests can use
    /// impossible addresses without invoking a raw-pointer unsafe API contract.
    pub fn validate(address: usize, width: usize, write: bool) -> &'static str {
        assert_eq!(width, 4, "fixture permits only u32 MMIO");
        assert_eq!(address % 4, 0, "unaligned MMIO access");
        REGION.with(|slot| {
            let region = slot.borrow();
            let region = region.as_ref().expect("unregistered MMIO fixture");
            let offset = address.checked_sub(region.base).expect("MMIO outside fixture");
            assert!(offset < core::mem::size_of::<[u32; 6]>(), "MMIO outside fixture");
            let &(_, name, writable) = super::demo::REGISTERS
                .iter()
                .find(|entry| entry.0 == offset)
                .expect("unknown MMIO register");
            assert_eq!(write, writable, "wrong MMIO direction for {name}");
            name
        })
    }

    fn record(access: Access) {
        REGION.with(|slot| {
            slot.borrow_mut()
                .as_mut()
                .expect("unregistered MMIO fixture")
                .log
                .push(access)
        });
    }

    struct FixtureOps;

    impl safe_mmio::MmioOps for FixtureOps {
        unsafe fn read_u8(src: *const u8) -> u8 {
            validate(src as usize, 1, false);
            unreachable!()
        }
        unsafe fn read_u16(src: *const u16) -> u16 {
            validate(src as usize, 2, false);
            unreachable!()
        }
        unsafe fn read_u64(src: *const u64) -> u64 {
            validate(src as usize, 8, false);
            unreachable!()
        }
        unsafe fn write_u8(dst: *mut u8, _: u8) {
            validate(dst as usize, 1, true);
            unreachable!()
        }
        unsafe fn write_u16(dst: *mut u16, _: u16) {
            validate(dst as usize, 2, true);
            unreachable!()
        }
        unsafe fn write_u64(dst: *mut u64, _: u64) {
            validate(dst as usize, 8, true);
            unreachable!()
        }

        unsafe fn read_u32(src: *const u32) -> u32 {
            let name = validate(src as usize, 4, false);
            // SAFETY: validation established a read-legal aligned u32 inside a
            // registered live allocation on this same thread; no callback runs.
            let value = unsafe { src.read_volatile() };
            record(Access::Read(name, value));
            value
        }

        unsafe fn write_u32(dst: *mut u32, value: u32) {
            let name = validate(dst as usize, 4, true);
            // SAFETY: same registered-allocation proof, with write permission.
            unsafe { dst.write_volatile(value) };
            record(Access::Write(name, value));
        }
    }

    safe_mmio::set_mmio_ops!(FixtureOps);
}
