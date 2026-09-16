#![no_std]
#![cfg_attr(not(feature = "legacy-tock"), doc = include_str!("../DEFAULT_FEATURE_TESTS.md"))]
#![cfg_attr(feature = "legacy-tock", doc = include_str!("../LEGACY_FEATURE_TESTS.md"))]
use core::convert::TryFrom;

// Local experimental backend; see EXPERIMENT.md for scope and provenance.
#[doc = include_str!("../SAFE_MMIO_TESTS.md")]
pub use abacus_registers_dsl::process_mmio_block;
// Preserve the original Tock runtime only for explicit compatibility builds.
// The safe-mmio backend neither needs nor re-exports these APIs by default.
#[cfg(feature = "legacy-tock")]
mod legacy_tock;
#[cfg(feature = "legacy-tock")]
pub use abacus_registers_dsl::process_register_block;
#[cfg(feature = "legacy-tock")]
pub use legacy_tock::{AbacusCell, AbacusStaticRef};
#[cfg(feature = "legacy-tock")]
pub use tock_registers;

/// Trait denoting a register block and specifies
/// the StateEnum associated type (wrapper enum for all
/// states the peripheral can be in).
pub trait Reg
where
    Self: TryFrom<Self::StateEnum, Error = <Self as Reg>::StateEnum>,
{
    type StateEnum: StateEnum;
}

pub trait StableStateReg {}

/// Marker trait for register types in transient states. Requires [`SyncState`]
/// so that transient states can be reconciled to stable states.
pub trait TransientStateReg: SyncState {}

/// User implemented trait that reconciles the state
/// from a transient state.
pub trait SyncState {
    type SyncStateEnum: StateEnum;
    fn sync_state(self) -> Self::SyncStateEnum;
}

/// Marker trait for all state enums.
pub trait StateEnum {
    fn sync_state(self) -> Self;
}

/// Marker trait for a peripheral hardware state (e.g. Off, Reading).
/// Implemented by the macro on generated state types.
pub trait State {}

/// Marker trait for a substate within a composite state.
/// Implemented by the macro on generated substate types (e.g. Any).
pub trait SubState {}
