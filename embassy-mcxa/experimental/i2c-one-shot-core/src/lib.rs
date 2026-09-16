//! Ownership and protocol decisions, with no PAC, MMIO, HAL or timer dependency.
//! The vocabulary consumes safe lower capabilities; their physical binding
//! remains audited. This crate cannot verify silicon behavior. Completion closes the bus
//! transaction, not the addressed device's application-level work.
//! Every module in this library forbids unsafe code, including declarations.
#![no_std]
#![forbid(unsafe_code)]

#[forbid(unsafe_code)]
mod protocol;

pub use protocol::{
    Idle, Receiving, RxDmaQuiescent, StartQueued, Stopping, Streaming, TxDmaQuiescent, checked, validate_transfer,
};
// The safe protocol-service interface is distinct from layer 2's unsafe Device
// contract. Existing application imports and the transaction sequence remain.
pub use i2c_one_shot_vocabulary::{Backend, DmaPoll, Error, ReceivePoll, Status, WakeOn};
