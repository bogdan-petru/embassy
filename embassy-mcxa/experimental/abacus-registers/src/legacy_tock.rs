//! Upstream Tock runtime retained for explicit `legacy-tock` compatibility.
//!
//! This module is not part of the safe-mmio backend. Enabling it does not resolve
//! the upstream AbacusCell soundness TODO retained below.
use crate::StateEnum;
use core::cell::Cell;

// (TODO) This is based on Tock's cell types, but soundness
// needs to be thought more about (e.g. around init/uninit)
/// Custom Cell type for Abacus to all a driver to store
/// the current state and access the state using interior mutability.
pub struct AbacusCell<T: StateEnum> {
    val: Cell<Option<T>>,
}

impl<T: StateEnum> AbacusCell<T> {
    pub fn new(val: T) -> Self {
        AbacusCell {
            val: Cell::new(Some(val)),
        }
    }

    /// Accessor to obtain register struct and perform
    /// driver mmio operations.
    #[inline(always)]
    pub fn map<F, R>(&self, closure: F) -> Option<R>
    where
        F: FnOnce(T) -> (T, R),
    {
        if let Some(current) = self.val.take() {
            let (new_val, pass_thru) = closure(current);
            self.val.set(Some(new_val));
            Some(pass_thru)
        } else {
            None
        }
    }
}

// The following code is copied from Tock's StaticRef.
use core::clone::Clone;
use core::fmt::Debug;
use core::marker::Copy;
use core::ops::Deref;
use core::ops::FnOnce;
use core::option::Option;
use core::option::Option::None;
use core::option::Option::Some;
use core::ptr::NonNull;

/// A pointer to statically allocated mutable data such as memory mapped I/O
/// registers.
///
/// Note, this copies Tock's StaticRef but renames it to AbacusStaticRef to avoid
/// potential conflicts with Tock crates.
///
/// This is a simple wrapper around a raw pointer that encapsulates an unsafe
/// dereference in a safe manner. It serve the role of creating a `&'static T`
/// given a raw address and acts similarly to `extern` definitions, except
/// [`AbacusStaticRef`] is subject to module and crate boundaries, while `extern`
/// definitions can be imported anywhere.
///
/// Because this defers the actual dereference, this can be put in a `const`,
/// whereas `const I32_REF: &'static i32 = unsafe { &*(0x1000 as *const i32) };`
/// will always fail to compile since `0x1000` doesn't have an allocation at
/// compile time, even if it's known to be a valid MMIO address.
#[derive(Debug)]
pub struct AbacusStaticRef<T> {
    ptr: NonNull<T>,
}

impl<T> AbacusStaticRef<T> {
    /// Create a new [`AbacusStaticRef`] from a raw pointer
    ///
    /// ## Safety
    ///
    /// - `ptr` must be aligned, non-null, and dereferencable as `T`.
    /// - `*ptr` must be valid for the program duration.
    pub const unsafe fn new(ptr: *const T) -> AbacusStaticRef<T> {
        // SAFETY: `ptr` is non-null as promised by the caller.
        AbacusStaticRef {
            ptr: NonNull::new_unchecked(ptr.cast_mut()),
        }
    }
}

impl<T> Clone for AbacusStaticRef<T> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<T> Copy for AbacusStaticRef<T> {}

impl<T> Deref for AbacusStaticRef<T> {
    type Target = T;
    fn deref(&self) -> &T {
        // SAFETY: `ptr` is aligned and dereferencable for the program duration
        // as promised by the caller of `AbacusStaticRef::new`.
        unsafe { self.ptr.as_ref() }
    }
}
