//! Existing short write and combined write/read examples, independent of SoC.
//! Operations keep the driver's deadline and ownership contract. There are no
//! retries, new timeouts, bus-clear operations or transfer-size extensions here.

use core::future::Future;

pub const ADDRESS: u8 = 0x2a;
const MESSAGE: [u8; 16] = *b"Hello from DMA!\n";
const REGISTER: [u8; 1] = [0x10];
const RESPONSE: [u8; 4] = [0x31, 0x82, 0xc7, 0x59];
const TARGET_RESPONSE: [u8; 8] = [0x31, 0x82, 0xc7, 0x59, 0xa1, 0xb2, 0xc3, 0xd4];

/// Populate the board's static DMA storage and invoke its consuming write once.
pub async fn write<E, F, Fut>(source: &'static mut [u8; 16], operation: F) -> Result<(), E>
where
    F: FnOnce(u8, &'static [u8]) -> Fut,
    Fut: Future<Output = Result<(), E>>,
{
    *source = MESSAGE;
    operation(ADDRESS, source).await
}

/// Write register 0x10 and read four bytes using ONE combined operation. Only
/// the RX reference returned after the driver's quiescence/STOP is inspected.
pub async fn write_read<E, F, Fut>(receive: &'static mut [u8; 4], operation: F) -> Result<(), CheckError<E>>
where
    F: FnOnce(u8, &'static [u8], &'static mut [u8]) -> Fut,
    Fut: Future<Output = Result<&'static mut [u8], E>>,
{
    let received = operation(ADDRESS, &REGISTER, receive)
        .await
        .map_err(CheckError::Driver)?;
    if received != RESPONSE {
        return Err(CheckError::DataMismatch);
    }
    Ok(())
}

/// Example-level observations, decoded by the board's independent target HAL.
/// Unknown events/outcomes are explicit and cannot become a successful check.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Request {
    Write(u16),
    Read(u16),
    Other,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WriteEnd {
    Stopped(usize),
    Restarted(usize),
    BufferFull(usize),
    Other,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReadEnd {
    Complete(usize),
    NeedMore(usize),
    EarlyStop(usize),
    Other,
}

#[derive(Debug, PartialEq, Eq)]
pub enum CheckError<E> {
    Driver(E),
    Request(Request),
    WriteEnd(WriteEnd),
    ReadEnd(ReadEnd),
    DataMismatch,
}

/// A narrow interface for the EXISTING test peer, not another driver. Methods
/// preserve pending futures and exact bus observations from the board's HAL.
/// In particular, BufferFull must never be translated into STOP/repeated START.
pub trait TargetPeer {
    type Error;
    fn listen(&mut self) -> impl Future<Output = Result<Request, Self::Error>>;
    fn receive<'a>(&'a mut self, buffer: &'a mut [u8]) -> impl Future<Output = Result<WriteEnd, Self::Error>> + 'a;
    fn reply<'a>(&'a mut self, bytes: &'a [u8]) -> impl Future<Output = Result<ReadEnd, Self::Error>> + 'a;
}

async fn require_request<P: TargetPeer>(peer: &mut P, expected: Request) -> Result<(), CheckError<P::Error>> {
    let actual = peer.listen().await.map_err(CheckError::Driver)?;
    if actual != expected {
        return Err(CheckError::Request(actual));
    }
    Ok(())
}

/// One spare byte makes the peer wait for STOP, not merely DMA DONE/fullness.
pub async fn receive_write<P: TargetPeer>(peer: &mut P) -> Result<(), CheckError<P::Error>> {
    require_request(peer, Request::Write(u16::from(ADDRESS))).await?;
    let mut message = [0; 17];
    let end = peer.receive(&mut message).await.map_err(CheckError::Driver)?;
    if end != WriteEnd::Stopped(16) {
        return Err(CheckError::WriteEnd(end));
    }
    if message[..16] != MESSAGE {
        return Err(CheckError::DataMismatch);
    }
    Ok(())
}

/// Assert actual repeated START after the one-byte register address: STOP is
/// a failure even if all bytes match. The reply's spare suffix keeps DMA short
/// of DONE. EarlyStop(4|5) counts STDR-queued bytes, NOT exact wire progress;
/// the controller separately checks exactly four received bytes and STOP.
pub async fn serve_write_read<P: TargetPeer>(peer: &mut P) -> Result<(), CheckError<P::Error>> {
    require_request(peer, Request::Write(u16::from(ADDRESS))).await?;
    let mut register = [0; 2];
    let end = peer.receive(&mut register).await.map_err(CheckError::Driver)?;
    if end != WriteEnd::Restarted(1) {
        return Err(CheckError::WriteEnd(end));
    }
    if register[0] != REGISTER[0] {
        return Err(CheckError::DataMismatch);
    }
    require_request(peer, Request::Read(u16::from(ADDRESS))).await?;
    let end = peer.reply(&TARGET_RESPONSE).await.map_err(CheckError::Driver)?;
    if !matches!(end, ReadEnd::EarlyStop(4 | 5)) {
        return Err(CheckError::ReadEnd(end));
    }
    Ok(())
}
