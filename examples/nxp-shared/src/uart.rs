//! One short UART transfer with board-independent payload and acceptance rules.

use core::future::Future;

pub static PAYLOAD: [u8; 16] = *b"UART pilot DMA!\n";
pub const RECEIVE_TIMEOUT_US: u64 = 20_000_000;
pub const QUIET_WINDOW_US: u64 = 20_000;

/// Invoke exactly one consuming driver operation, without retries or another
/// deadline. The supplied future retains the driver's ownership and cleanup
/// behavior; success means whatever the driver's documented service promises.
pub async fn send<E, F, Fut>(write: F) -> Result<(), E>
where
    F: FnOnce(&'static [u8]) -> Fut,
    Fut: Future<Output = Result<(), E>>,
{
    write(&PAYLOAD).await
}

#[derive(Debug, PartialEq, Eq)]
pub enum ReceiveError<E> {
    Timeout { received: usize },
    Mismatch { index: usize, expected: u8, actual: u8 },
    ExtraByte(u8),
    Peripheral(E),
}

/// Require the exact payload within 20 s, then no extra byte or receive error
/// for 20 ms. `Ok(None)` alone means an empty receive FIFO; every error fails.
///
/// `read` must be nonblocking. `now_us` must be a continuously advancing,
/// monotonic microsecond clock (wrapping `u64` is supported). The shared body
/// does not configure hardware, suppress errors or retry an entire transfer.
/// These cooperative deadlines cannot bound a callback that never returns.
pub fn verify_receiver<E>(
    mut read: impl FnMut() -> Result<Option<u8>, E>,
    mut now_us: impl FnMut() -> u64,
) -> Result<(), ReceiveError<E>> {
    let start = now_us();
    for (index, expected) in PAYLOAD.iter().copied().enumerate() {
        let actual = loop {
            if now_us().wrapping_sub(start) >= RECEIVE_TIMEOUT_US {
                return Err(ReceiveError::Timeout { received: index });
            }
            match read().map_err(ReceiveError::Peripheral)? {
                Some(byte) => break byte,
                None => {}
            }
        };
        if actual != expected {
            return Err(ReceiveError::Mismatch {
                index,
                expected,
                actual,
            });
        }
    }

    let quiet_start = now_us();
    while now_us().wrapping_sub(quiet_start) < QUIET_WINDOW_US {
        if let Some(byte) = read().map_err(ReceiveError::Peripheral)? {
            return Err(ReceiveError::ExtraByte(byte));
        }
    }
    Ok(())
}
