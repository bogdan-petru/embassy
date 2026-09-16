//! Layer 4 transaction recipes and the single whole-transaction deadline.
//! Each moved owner carries the same hardware cleanup guard through every await.

mod vocabulary;

use core::future::Future;
use embassy_futures::select::{Either, select};
use i2c_one_shot_core::{Backend, Idle};
pub use vocabulary::InterruptHandler;
pub(in crate::i2c) use vocabulary::Registers;
pub use vocabulary::{ClockConfig, McxaI2cPort};
pub(super) use vocabulary::{Hardware, new_hardware};

use super::Error;

pub(super) async fn write_transfer<B: Backend>(idle: Idle<B>, address: u8, bytes: &'static [u8]) -> Result<(), Error> {
    let started = idle.start(address)?;
    let streaming = started.transmit(bytes)?;
    let quiescent = streaming.drain().await?;
    let stopping = quiescent.stop().await?;
    stopping.complete().await
}

// The RX reference moves into the lower DMA lease. A timeout drops that lease
// and quiesces it; forgetting this entire future retains exclusive static
// storage forever. Neither path lends memory back before DMA has stopped.
pub(super) async fn write_read_transfer<B: Backend>(
    idle: Idle<B>,
    address: u8,
    write: &'static [u8],
    read: &'static mut [u8],
) -> Result<&'static mut [u8], Error> {
    let started = idle.start(address)?;
    let streaming = started.transmit(write)?;
    let quiescent = streaming.drain().await?;
    let receiving = quiescent.receive(address, read).await?;
    let received = receiving.drain().await?;
    let stopping = received.stop().await?;
    stopping.complete().await
}

// Both recipes and the host tests use this one whole-transaction deadline.
// select polls the transfer first, including when both futures become ready.
// Timeout drops the owned transfer; synchronous DMA drain is not preemptible.
pub(super) async fn with_deadline<T>(
    transfer: impl Future<Output = Result<T, Error>>,
    deadline: impl Future<Output = ()>,
) -> Result<T, Error> {
    match select(transfer, deadline).await {
        Either::First(result) => result,
        Either::Second(()) => Err(Error::Timeout),
    }
}

#[cfg(all(test, feature = "one-shot-host-tests"))]
mod deadline_tests;
