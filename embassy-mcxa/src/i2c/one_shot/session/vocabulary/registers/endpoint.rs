//! Owned directional FIFO/request pairs, constructed from one I2C singleton.
//!
//! The instance brand exists during acquisition. The non-generic driver then
//! erases that brand, but cannot split, replace, or independently construct the
//! TX and RX endpoint pairs. This is not a claim of branding after erasure.

use core::ptr::NonNull;

use super::Registers;
use crate::Peri;
use crate::dma::DmaRequest;
use crate::i2c::Instance;

/// Keeps both directional request/address pairs tied to the singleton until
/// the same opaque pair is moved into CPU or DMA ownership.
pub(super) struct TxEndpoint<T: Instance> {
    tx: ErasedTxEndpoint,
    rx: ErasedRxEndpoint,
    // Acquisition spends the same singleton that selects both addresses and
    // requests. Keeping it here prevents safe duplicate acquisition before
    // erasure, and prevents pairing one instance's TX with another's RX.
    _peripheral: Peri<'static, T>,
}

/// Both fixed-direction endpoints after the singleton brand is intentionally
/// erased. This remains non-Copy and moves only with the private owner.
pub(in crate::i2c::one_shot) struct ErasedEndpointPair {
    tx: ErasedTxEndpoint,
    rx: ErasedRxEndpoint,
}

/// A non-Copy capability for the MTDR byte destination and its matching
/// controller TX request.
pub(in crate::i2c::one_shot) struct ErasedTxEndpoint {
    // u8 fixes the eDMA destination width: MTDR's low data byte, not a freely
    // supplied transfer width or a pointer to an arbitrary caller buffer.
    address: NonNull<u8>,
    request: DmaRequest,
}

/// A non-Copy capability for the MRDR byte source and its matching controller
/// RX request. The source can never be used as a writable FIFO endpoint.
pub(in crate::i2c::one_shot) struct ErasedRxEndpoint {
    // u8 fixes the eDMA source width to MRDR's byte data path. The pointer is
    // carried only to the trusted DMA setup call and has no public accessor.
    address: NonNull<u8>,
    request: DmaRequest,
}

impl<T: Instance> TxEndpoint<T> {
    /// Acquire the register mapping and both FIFO/request directions as one
    /// instance pair.
    ///
    /// # Safety
    /// The caller must establish a live, clocked, exclusive mapping and issue
    /// the peripheral reset required by `Info::one_shot_registers`. This does
    /// not claim MEN/reset readback has already been confirmed; that is the
    /// acquired register owner's next obligation. The singleton is consumed
    /// here; no PAC handle, address, or request number can be supplied separately.
    pub(super) unsafe fn acquire(peripheral: Peri<'static, T>) -> (Registers, Self) {
        // SAFETY: the caller establishes the live exclusive mapping; T selects
        // the same sealed instance definition for its register map and both
        // directional DMA requests.
        let registers = unsafe { T::info().one_shot_registers() };
        let endpoint = Self {
            tx: ErasedTxEndpoint {
                address: NonNull::new(registers.tx_fifo).expect("I2C TX register is non-null"),
                request: T::TX_DMA_REQUEST,
            },
            rx: ErasedRxEndpoint {
                address: NonNull::new(registers.rx_fifo.cast_mut()).expect("I2C RX register is non-null"),
                request: T::RX_DMA_REQUEST,
            },
            _peripheral: peripheral,
        };
        (registers, endpoint)
    }

    pub(super) fn erase(self) -> ErasedEndpointPair {
        // The singleton token is spent, not returned to any caller. After this
        // point the two fixed-direction pairs stay opaque, but their instance
        // type is no longer known.
        ErasedEndpointPair {
            tx: self.tx,
            rx: self.rx,
        }
    }
}

impl ErasedEndpointPair {
    pub(super) fn tx(&self) -> &ErasedTxEndpoint {
        &self.tx
    }

    pub(super) fn rx(&self) -> &ErasedRxEndpoint {
        &self.rx
    }

    /// Construct only the recorded host fixture's fixed directional pair.
    ///
    /// # Safety
    /// The two non-null addresses must designate the fixture's live MTDR and
    /// MRDR cells. Their allocation must outlive every CPU or DMA owner that
    /// retains this pair. This hook never exists in firmware and must never be
    /// used to arm real DMA.
    #[cfg(all(test, feature = "one-shot-host-tests"))]
    pub(super) unsafe fn from_test_addresses(tx: *mut u8, rx: *const u8) -> Self {
        Self {
            // SAFETY: forwarded fixture-address contract above.
            tx: unsafe { ErasedTxEndpoint::from_test_address(tx) },
            rx: ErasedRxEndpoint {
                address: NonNull::new(rx.cast_mut()).expect("test RX register is non-null"),
                request: DmaRequest::Lpi2C0Rx,
            },
        }
    }
}

impl ErasedTxEndpoint {
    pub(super) fn address(&self) -> *mut u8 {
        self.address.as_ptr()
    }

    pub(super) fn request(&self) -> DmaRequest {
        self.request
    }

    /// Construct only the recorded host fixture's fixed TX request/address pair.
    ///
    /// # Safety
    /// A non-null address must designate the fixture's live TX cell, whose
    /// allocation outlives this endpoint and every test owner retaining it.
    /// Null is rejected before a capability is constructed. This hook never
    /// exists in firmware and must never be used to arm real DMA.
    #[cfg(all(test, feature = "one-shot-host-tests"))]
    pub(super) unsafe fn from_test_address(address: *mut u8) -> Self {
        Self {
            address: NonNull::new(address).expect("test TX register is non-null"),
            request: DmaRequest::Lpi2C0Tx,
        }
    }
}

impl ErasedRxEndpoint {
    pub(super) fn address(&self) -> *const u8 {
        self.address.as_ptr().cast_const()
    }

    pub(super) fn request(&self) -> DmaRequest {
        self.request
    }
}

#[cfg(all(test, feature = "one-shot-host-tests"))]
mod tests {
    use super::*;

    #[test]
    fn instance_brand_is_retained_until_explicit_erasure() {
        // This body needs no singleton acquisition or physical MMIO access.
        // Changing its input to TxEndpoint<LPI2C1> must reject the returned
        // endpoint as E0308; it cannot silently become an LPI2C0 capability.
        fn retain_instance(endpoint: TxEndpoint<crate::peripherals::LPI2C0>) -> TxEndpoint<crate::peripherals::LPI2C0> {
            endpoint
        }
        // Infer the function item so the wrong-input mutation has one precise
        // return-type diagnostic, not a second function-pointer mismatch.
        let _same_instance = retain_instance;
    }

    #[test]
    fn fixture_endpoint_keeps_its_byte_destination_and_fixed_request_together() {
        let mut tx = 0_u32;
        let address = core::ptr::addr_of_mut!(tx).cast::<u8>();
        // SAFETY: this owned, aligned host cell outlives the endpoint, and no
        // DMA is armed. Moving the capability performs no memory access.
        let endpoint = unsafe { ErasedTxEndpoint::from_test_address(address) };
        assert_eq!(endpoint.address(), address);
        assert_eq!(endpoint.request(), DmaRequest::Lpi2C0Tx);
        let moved = endpoint;
        assert_eq!(moved.address(), address);
        assert_eq!(moved.request(), DmaRequest::Lpi2C0Tx);
    }

    #[test]
    fn fixture_pair_keeps_directional_addresses_and_requests_together() {
        let mut tx = 0_u32;
        let mut rx = 0_u32;
        let tx_address = core::ptr::addr_of_mut!(tx).cast::<u8>();
        let rx_address = core::ptr::addr_of_mut!(rx).cast::<u8>();
        // SAFETY: both host cells are live, non-null, and no DMA is armed.
        let endpoints = unsafe { ErasedEndpointPair::from_test_addresses(tx_address, rx_address) };
        assert_eq!(endpoints.tx().address(), tx_address);
        assert_eq!(endpoints.tx().request(), DmaRequest::Lpi2C0Tx);
        assert_eq!(endpoints.rx().address(), rx_address.cast_const());
        assert_eq!(endpoints.rx().request(), DmaRequest::Lpi2C0Rx);
    }

    #[test]
    #[should_panic(expected = "test TX register is non-null")]
    fn fixture_endpoint_rejects_null() {
        // The constructor rejects null before making a capability; no access
        // is attempted and no invalid endpoint can escape this test.
        let _ = unsafe { ErasedTxEndpoint::from_test_address(core::ptr::null_mut()) };
    }

    #[test]
    #[should_panic(expected = "test RX register is non-null")]
    fn fixture_pair_rejects_null_rx() {
        let mut tx = 0_u32;
        // SAFETY: this deliberately supplies an invalid RX address to verify
        // rejection before any pair is constructed or DMA is configured.
        let _ =
            unsafe { ErasedEndpointPair::from_test_addresses(core::ptr::addr_of_mut!(tx).cast(), core::ptr::null()) };
    }
}
