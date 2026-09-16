#[path = "fixtures/mmio.rs"]
mod fixture;
use fixture::demo::*;
use fixture::ram::{Access, Fixture};

#[test]
fn real_generated_accessors_write_the_expected_words_and_offsets() {
    // Ordinary owned RAM is a test backing store, not a simulated device. No
    // references to it are used until the register owner has been relinquished.
    let backing = Fixture::new([0_u32, 0, 0xfeed, 0x1234, 0, 0x56]);
    // SAFETY: aligned, live exclusive backing matches the six-word layout, Off
    // is represented by control=0, and there are no DMA/interrupt aliases.
    let mut off = unsafe { DemoRegisters::<Off>::new(backing.base()) };
    assert!(
        backing.accesses().is_empty(),
        "retaining handles must not read MMIO at construction"
    );
    off.write_timing(0x789a);
    let mut on = off.into_enable();
    assert_eq!(on.read_status(), 0x1234);
    on.write_tx(0xbc);
    assert_eq!(on.read_rx(), 0x56);
    let state: DemoStateEnum = on.into();
    // Abacus's generated enum conversion cannot fabricate a different state.
    let state = match DemoRegisters::<Off>::try_from(state) {
        Err(actual) => actual,
        Ok(_) => panic!("On unexpectedly converted to Off without MMIO"),
    };
    let on = match DemoRegisters::<On>::try_from(state) {
        Ok(on) => on,
        Err(_) => panic!("On round trip failed"),
    };
    let on = on.into_disable().into_enable();
    assert_eq!(
        on.read_status(),
        0x1234,
        "retained handle survives enum and state round trips"
    );
    let off = on.into_disable();
    drop(off);
    assert_eq!(
        backing.accesses(),
        [
            Access::Write("timing", 0x789a),
            Access::Write("control", 1),
            Access::Read("status", 0x1234),
            Access::Write("tx", 0xbc),
            Access::Read("rx", 0x56),
            Access::Write("control", 0),
            Access::Write("control", 1),
            Access::Read("status", 0x1234),
            Access::Write("control", 0),
        ]
    );
    assert_eq!(backing.finish(), [0, 0x789a, 0xfeed, 0x1234, 0xbc, 0x56]);
}

#[test]
fn state_transition_really_writes_control_before_returning() {
    let backing = Fixture::new([0_u32; 6]);
    // SAFETY: same exclusive, aligned six-word backing contract as above.
    let off = unsafe { DemoRegisters::<Off>::new(backing.base()) };
    let on = off.into_enable();
    drop(on);
    // Inspect only after releasing the owner, not while its invariant is live.
    let backing = backing.finish();
    assert_eq!(backing[0], 1);
}

#[test]
fn owner_size_accounts_for_each_retained_handle() {
    // This fixture has one ReadPure field. The base plus its retained shared
    // handle occupy two pointer words; the state carries no runtime storage.
    assert_eq!(
        core::mem::size_of::<DemoRegisters<Off>>(),
        2 * core::mem::size_of::<usize>()
    );
    assert_eq!(
        core::mem::size_of::<DemoRegisters<On>>(),
        core::mem::size_of::<DemoRegisters<Off>>()
    );
}

#[test]
#[should_panic(expected = "null MMIO")]
fn rejects_null_before_constructing_an_owner() {
    // The constructor rejects the invalid input before any pointer is used.
    let _ = unsafe { DemoRegisters::<Off>::new(core::ptr::null_mut()) };
}

#[test]
fn fixture_rejects_unregistered_outside_padding_width_and_direction() {
    use fixture::ram::validate;
    use std::panic::catch_unwind;
    assert!(catch_unwind(|| validate(0, 4, false)).is_err());
    let ram = Fixture::new([0; 6]);
    let base = ram.base() as usize;
    for (address, width, write) in [
        (base + 24, 4, false),
        (base - 4, 4, false),
        (base + 8, 4, false),
        (base + 12, 1, false),
        (base + 12, 2, false),
        (base + 12, 8, false),
        (base + 13, 4, false),
        (base + 12, 4, true),
        (base, 4, false),
    ] {
        assert!(catch_unwind(|| validate(address, width, write)).is_err());
    }
    assert!(ram.accesses().is_empty());
    drop(ram);
    assert!(catch_unwind(|| validate(base + 12, 4, false)).is_err());
}

#[test]
fn fixture_recovers_after_unwinding_and_rejects_nesting() {
    use std::panic::catch_unwind;
    assert!(catch_unwind(|| {
        let _ram = Fixture::new([0; 6]);
        let _nested = Fixture::new([0; 6]);
    })
    .is_err());
    let ram = Fixture::new([0; 6]);
    assert_eq!(ram.finish(), [0; 6]);
}

#[test]
fn fixture_registration_and_trace_are_isolated_per_thread() {
    std::thread::scope(|scope| {
        for expected in 1..=8 {
            scope.spawn(move || {
                let ram = Fixture::new([0, 0, 0, expected, 0, 0]);
                // SAFETY: this thread retains its exclusive, aligned fixture
                // until after the generated owner is dropped.
                let on = unsafe { DemoRegisters::<Off>::new(ram.base()) }.into_enable();
                assert_eq!(on.read_status(), expected);
                drop(on);
                assert_eq!(
                    ram.accesses(),
                    [Access::Write("control", 1), Access::Read("status", expected)]
                );
            });
        }
    });
}
