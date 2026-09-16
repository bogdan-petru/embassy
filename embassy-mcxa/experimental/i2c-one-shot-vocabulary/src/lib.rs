//! Layer 3: named I2C operations over opaque, safe layer-2 capabilities.
//! This library cannot acquire raw hardware or implement the unsafe Device
//! contract. Layer 2 owns DMA preparation, request ordering and all cleanup.
#![no_std]
#![forbid(unsafe_code)]

#[forbid(unsafe_code)]
mod types;
#[forbid(unsafe_code)]
mod vocabulary;

use core::task::{Context, Poll};

pub use i2c_one_shot_interface::{Error, Status, WakeOn};
pub use types::{DmaPoll, ReceivePoll};
pub use vocabulary::Hardware;

/// Protocol service consumed by layer 4, not an unsafe memory-safety contract.
/// A wrong implementation can give wrong protocol results; no unsafe code may
/// rely on these answers to release DMA memory or recover raw CPU authority.
/// The production Hardware uses only layer 2's owned capabilities. Safe mocks
/// can test protocol decisions, but do not qualify that physical binding.
pub trait Backend: Sized {
    type Streaming;
    type Receiving;
    fn start(self, address: u8) -> Result<Self, Error>;
    fn arm(self, bytes: &'static [u8]) -> Result<Self::Streaming, Error>;
    fn observe(&mut self, cx: &mut Context<'_>, wake: WakeOn) -> Status;
    fn observe_streaming(lease: &mut Self::Streaming, cx: &mut Context<'_>, wake: WakeOn) -> Status;
    fn poll_dma(lease: Self::Streaming, cx: &mut Context<'_>) -> DmaPoll<Self>;
    fn try_stop(&mut self) -> Poll<Result<(), Error>>;
    fn read_ready(&mut self, address: u8, length: u8) -> Poll<Result<(), Error>>;
    fn arm_receive(self, address: u8, bytes: &'static mut [u8]) -> Result<Self::Receiving, Error>;
    fn observe_receive(lease: &mut Self::Receiving, cx: &mut Context<'_>, wake: WakeOn) -> Status;
    fn poll_receive(lease: Self::Receiving, cx: &mut Context<'_>) -> ReceivePoll<Self>;
}
