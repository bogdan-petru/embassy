//! Board-specific decoding for the existing MCXA target driver. No fabricated
//! termination evidence: unknown status variants fail in the shared checker.
#![forbid(unsafe_code)]

use embassy_mcxa::i2c::{Dma, target};
use nxp_shared_examples::i2c::{ReadEnd, Request, TargetPeer, WriteEnd};

pub struct Peer(pub target::I2c<'static, Dma<'static>>);

impl TargetPeer for Peer {
    type Error = target::IOError;

    async fn listen(&mut self) -> Result<Request, Self::Error> {
        Ok(match self.0.async_listen().await? {
            target::Request::Write(address) => Request::Write(address),
            target::Request::Read(address) => Request::Read(address),
            _ => Request::Other,
        })
    }

    async fn receive<'a>(&'a mut self, buffer: &'a mut [u8]) -> Result<WriteEnd, Self::Error> {
        Ok(match self.0.async_respond_to_write(buffer).await? {
            target::WriteStatus::Stopped(count) => WriteEnd::Stopped(count),
            target::WriteStatus::Restarted(count) => WriteEnd::Restarted(count),
            target::WriteStatus::BufferFull(count) => WriteEnd::BufferFull(count),
            _ => WriteEnd::Other,
        })
    }

    async fn reply<'a>(&'a mut self, bytes: &'a [u8]) -> Result<ReadEnd, Self::Error> {
        let outcome = self.0.async_respond_to_read(bytes).await?;
        defmt::info!("WRITE-READ TARGET response outcome={:?}", outcome);
        Ok(match outcome {
            target::ReadStatus::Complete(count) => ReadEnd::Complete(count),
            target::ReadStatus::NeedMore(count) => ReadEnd::NeedMore(count),
            target::ReadStatus::EarlyStop(count) => ReadEnd::EarlyStop(count),
            _ => ReadEnd::Other,
        })
    }
}
