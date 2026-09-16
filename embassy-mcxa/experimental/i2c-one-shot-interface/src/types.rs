use crate::Device;

/// Shared errors. The HAL originates Clock and Timeout in the driver flow;
/// public enum variants are not constructor-restricted to that crate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    InvalidInput,
    Clock,
    Busy,
    Nack,
    ArbitrationLoss,
    Fifo,
    PinLowTimeout,
    Dma,
    Timeout,
    UnexpectedStop,
}

/// Decoded facts from ONE status read; no protocol priority or reusable permit.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct Status {
    pub pin_low_timeout: bool,
    pub nack: bool,
    pub arbitration_loss: bool,
    pub fifo_error: bool,
    pub stop_seen: bool,
    pub controller_idle: bool,
    pub bus_idle: bool,
}

/// Restricted command vocabulary, not an arbitrary register address or word.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Command {
    Start(u8),
    Stop,
}

/// Input to interrupt arming, not an output claiming durable FIFO readiness.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WakeOn {
    Completion,
    TxSpace,
}

/// Ownership is returned on every nonterminal poll. There is no detached
/// completion receipt that could authorize an unrelated channel or transfer.
pub enum Step<L, C> {
    Pending(L),
    Complete(C),
    Failed(Error),
}

impl<L, C> Step<L, C> {
    /// Map the retained or completed owner without discarding either on Pending.
    /// Failed preserves its error; cleanup belongs to the consumed lower owner.
    pub fn map<N, R>(self, pending: impl FnOnce(L) -> N, complete: impl FnOnce(C) -> R) -> Step<N, R> {
        match self {
            Self::Pending(lease) => Step::Pending(pending(lease)),
            Self::Complete(owner) => Step::Complete(complete(owner)),
            Self::Failed(error) => Step::Failed(error),
        }
    }
}

pub type DmaResult<D> = Step<<D as Device>::InFlight, D>;

/// Receive memory leaves the lower lease only after the channel is quiescent.
pub type ReceiveResult<D> = Step<<D as Device>::Receiving, (D, &'static mut [u8])>;
