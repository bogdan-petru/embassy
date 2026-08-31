//! Per-instance I2C state that must not be field-accessible to drivers.
//!
//! This deliberately lives below `i2c` rather than in `i2c/mod.rs`: Rust
//! lets child modules access private fields declared by a parent, so placing
//! it here makes controller and target siblings that must use the narrow
//! accessors below instead of reaching into raw state directly.

use core::sync::atomic::{AtomicBool, Ordering};

use maitake_sync::WaitCell;

use super::controller::ControllerRegisters;
use super::target::TargetRegisters;
use crate::pac;

pub(crate) struct Info {
    regs: pac::lpi2c::Lpi2c,
    wait_cell: WaitCell,
    /// Whether one controller transaction session currently owns this
    /// peripheral. The state machine owns this through intent methods below,
    /// so callers cannot accidentally split recovery ownership with a bare
    /// atomic operation.
    session_open: AtomicBool,
}

impl Info {
    /// Create one instance-owned I2C state block. This is crate-visible only
    /// because the generated peripheral-instance macro expands outside the
    /// `i2c` module; all fields stay private to this module.
    #[doc(hidden)]
    pub(crate) const fn new(regs: pac::lpi2c::Lpi2c) -> Self {
        Self {
            regs,
            wait_cell: WaitCell::new(),
            session_open: AtomicBool::new(false),
        }
    }

    /// Get the controller-specific, opaque register facade for this instance.
    /// Its operations are visible only inside the controller driver tree;
    /// callers cannot turn this back into a raw PAC handle.
    #[inline(always)]
    pub(in crate::i2c) fn controller_registers(&self) -> ControllerRegisters {
        ControllerRegisters::from_pac(self.regs)
    }

    /// Get the target-specific, opaque register facade for this instance.
    /// Its operations are visible only inside the target driver tree; callers
    /// cannot turn this back into a raw PAC handle.
    #[inline(always)]
    pub(in crate::i2c) fn target_registers(&self) -> TargetRegisters {
        TargetRegisters::from_pac(self.regs)
    }

    #[inline(always)]
    pub(in crate::i2c) fn wait_cell(&self) -> &WaitCell {
        &self.wait_cell
    }

    /// Reserve the single live controller-session slot.
    pub(in crate::i2c) fn reserve_session(&self) {
        assert!(
            !self.session_open.swap(true, Ordering::Relaxed),
            "i2c: a transaction started while another session is live"
        );
    }

    /// Release the controller-session slot after normal completion or
    /// recovery. The slot cannot be cleared by a driver with a bare atomic
    /// store.
    pub(in crate::i2c) fn release_session(&self) {
        self.session_open.store(false, Ordering::Relaxed);
    }
}

unsafe impl Sync for Info {}
