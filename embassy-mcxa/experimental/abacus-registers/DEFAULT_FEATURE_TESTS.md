The default API is the safe-mmio backend and the shared Abacus marker traits.
The upstream Tock runtime is deliberately unavailable without `legacy-tock`.
Each import is checked independently so one missing export cannot hide another.
The real safe-mmio fixture provides the positive default-feature counterpart.

The interior-mutable cell is not exported by default:

```compile_fail,E0432
use abacus_registers::AbacusCell;
```

The shared raw-MMIO reference is not exported by default:

```compile_fail,E0432
use abacus_registers::AbacusStaticRef;
```

The original Tock-backed macro is not exported by default:

```compile_fail,E0432
use abacus_registers::process_register_block;
```

The Tock dependency is not re-exported by default:

```compile_fail,E0432
use abacus_registers::tock_registers;
```
