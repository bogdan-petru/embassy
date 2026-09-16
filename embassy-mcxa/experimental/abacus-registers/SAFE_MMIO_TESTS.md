Experimental Abacus backend using `safe-mmio` field access.

The fixture is expanded by the real macro in every snippet. The positive test
uses the same fixture as the rejection tests, so a broken import or expansion
cannot masquerade as a passing safety check. Executing host tests separately
verify access offsets and the actual transition writes; these are not hardware
timing tests.

Configuration is available before enable; transfer operations are available
after enable, and disabling consumes the enabled owner:

```
# mod fixture { include!(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/mmio.rs")); }
# use fixture::demo;
fn legal(mut off: demo::DemoRegisters<demo::Off>) {
    off.write_timing(100);
    let mut on = off.into_enable();
    let _status = on.read_status();
    on.write_tx(42);
    let _byte = on.read_rx();
    let _off = on.into_disable();
}
# let ram = fixture::ram::Fixture::new([0, 0, 0, 7, 0, 9]);
# // SAFETY: aligned, exclusively owned RAM remains live until legal returns.
# let off = unsafe { demo::DemoRegisters::<demo::Off>::new(ram.base()) };
# legal(off);
# assert_eq!(ram.accesses().len(), 6);
# assert_eq!(ram.finish(), [0, 100, 0, 7, 42, 9]);
```

A disabled controller cannot transmit:

```compile_fail
# mod fixture { include!(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/mmio.rs")); }
# use fixture::demo;
fn forbidden(mut off: demo::DemoRegisters<demo::Off>) {
    off.write_tx(42);
}
```

An enabled controller cannot rewrite the disabled-only timing configuration:

```compile_fail
# mod fixture { include!(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/mmio.rs")); }
# use fixture::demo;
fn forbidden(mut on: demo::DemoRegisters<demo::On>) {
    on.write_timing(100);
}
```

A state transition moves authority; the old owner is no longer usable:

```compile_fail
# mod fixture { include!(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/mmio.rs")); }
# use fixture::demo;
fn forbidden(off: demo::DemoRegisters<demo::Off>) {
    let _on = off.into_enable();
    let _second = off.into_enable();
}
```

Destructive reads require an exclusive borrow, unlike pure status reads:

```compile_fail
# mod fixture { include!(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/mmio.rs")); }
# use fixture::demo;
fn forbidden(on: &demo::DemoRegisters<demo::On>) -> u32 {
    on.read_rx()
}
```

Write-only registers have no read operation:

```compile_fail
# mod fixture { include!(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/mmio.rs")); }
# use fixture::demo;
fn forbidden(on: &demo::DemoRegisters<demo::On>) -> u32 {
    on.read_tx()
}
```

Read-only registers have no write operation:

```compile_fail
# mod fixture { include!(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/mmio.rs")); }
# use fixture::demo;
fn forbidden(on: &mut demo::DemoRegisters<demo::On>) {
    on.write_rx(0);
}
```

The CPU register owner cannot be copied:

```compile_fail
# mod fixture { include!(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/mmio.rs")); }
# use fixture::demo;
fn forbidden(on: &demo::DemoRegisters<demo::On>) -> demo::DemoRegisters<demo::On> {
    *on
}
```

Code outside the boundary module cannot retrieve its raw base:

```compile_fail
# mod fixture { include!(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/mmio.rs")); }
# use fixture::demo;
fn forbidden(on: &demo::DemoRegisters<demo::On>) {
    let _raw = on.base;
}
```

Initial ownership acquisition is explicitly unsafe:

```compile_fail
# mod fixture { include!(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/mmio.rs")); }
# use fixture::demo;
fn forbidden(base: *mut u8) -> demo::DemoRegisters<demo::Off> {
    demo::DemoRegisters::new(base)
}
```

Shared pure reads do not make the MMIO owner thread-shareable. The retained
handles remain private; the owner deliberately stays non-Sync:

```compile_fail
# mod fixture { include!(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/mmio.rs")); }
# use fixture::demo;
fn forbidden(on: &demo::DemoRegisters<demo::On>) {
    std::thread::scope(|scope| {
        scope.spawn(|| on.read_status());
    });
}
```

Nor may ownership be sent to another thread:

```compile_fail
# mod fixture { include!(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/mmio.rs")); }
# use fixture::demo;
fn forbidden(on: demo::DemoRegisters<demo::On>) {
    std::thread::spawn(move || on.read_status());
}
```

The retained shared observer cannot escape the register owner:

```compile_fail
# mod fixture { include!(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/mmio.rs")); }
# use fixture::demo;
fn forbidden(on: &demo::DemoRegisters<demo::On>) {
    let _observer = on.__read_pure_status;
}
```
