The original runtime is retained only for explicit `legacy-tock` compatibility.
This verifies all four gated exports together with the marker traits and new
backend, which remain available when the compatibility feature is enabled.
It does not audit the original runtime or resolve its soundness TODO.

```
use abacus_registers::{
    process_mmio_block, process_register_block, tock_registers,
    AbacusCell, AbacusStaticRef, StateEnum,
};

struct LegacyState;
impl StateEnum for LegacyState {
    fn sync_state(self) -> Self { self }
}

let cell = AbacusCell::new(LegacyState);
assert_eq!(cell.map(|state| (state, 42)), Some(42));
let _: Option<AbacusStaticRef<tock_registers::registers::ReadOnly<u32>>> = None;
```
