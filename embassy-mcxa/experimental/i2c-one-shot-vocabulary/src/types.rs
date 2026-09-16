use i2c_one_shot_interface::Step;

use crate::Backend;

/// Protocol result. It cannot unwrap a layer-2 capability or mint raw authority.
pub type DmaPoll<B> = Step<<B as Backend>::Streaming, B>;

/// These bytes came from the lower owned capability only after quiescence.
pub type ReceivePoll<B> = Step<<B as Backend>::Receiving, (B, &'static mut [u8])>;
