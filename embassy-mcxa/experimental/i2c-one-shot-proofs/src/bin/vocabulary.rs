//! Dedicated proof executable: real L3 over a proof-only modeled L2 binding.
//! Not a firmware dependency. Existing L4 proof library remains unsafe-free.
#![deny(unsafe_code)]

#[cfg(any(kani, test))]
#[path = "vocabulary/model.rs"]
mod model;
#[cfg(any(kani, test))]
#[forbid(unsafe_code)]
#[path = "vocabulary/properties.rs"]
mod properties;

fn main() {}

#[cfg(kani)]
#[forbid(unsafe_code)]
mod proofs {
    use super::{
        model::{Decision, Path},
        properties,
    };
    use i2c_one_shot_interface::{Error, Status, WakeOn};

    fn status() -> Status {
        Status {
            pin_low_timeout: kani::any(),
            nack: kani::any(),
            arbitration_loss: kani::any(),
            fifo_error: kani::any(),
            stop_seen: kani::any(),
            controller_idle: kani::any(),
            bus_idle: kani::any(),
        }
    }

    fn error() -> Error {
        match kani::any::<u8>() % 10 {
            0 => Error::InvalidInput,
            1 => Error::Clock,
            2 => Error::Busy,
            3 => Error::Nack,
            4 => Error::ArbitrationLoss,
            5 => Error::Fifo,
            6 => Error::PinLowTimeout,
            7 => Error::Dma,
            8 => Error::Timeout,
            _ => Error::UnexpectedStop,
        }
    }

    fn reject() -> Option<Error> {
        if kani::any() { Some(error()) } else { None }
    }
    fn decision() -> Decision {
        if kani::any() {
            Decision::Pending
        } else if kani::any() {
            Decision::Complete
        } else {
            Decision::Failed(error())
        }
    }

    #[kani::proof]
    #[kani::unwind(40)]
    fn start_admission_preserves_address() {
        properties::start(status(), kani::any());
    }

    #[kani::proof]
    #[kani::unwind(40)]
    fn stop_uses_current_fifo_state() {
        properties::stop(kani::any());
    }

    #[kani::proof]
    #[kani::unwind(40)]
    fn read_readiness_is_pure() {
        properties::read_ready(kani::any(), kani::any(), kani::any());
    }

    #[kani::proof]
    #[kani::unwind(40)]
    fn observations_preserve_status_and_wake() {
        let path = match kani::any::<u8>() % 3 {
            0 => Path::Cpu,
            1 => Path::Tx,
            _ => Path::Rx,
        };
        let wake = if kani::any() {
            WakeOn::Completion
        } else {
            WakeOn::TxSpace
        };
        properties::observe(status(), wake, path);
    }

    #[kani::proof]
    #[kani::unwind(40)]
    fn tx_arm_preserves_input_and_error() {
        properties::tx_arm(usize::from(kani::any::<u8>() % 32) + 1, reject());
    }

    #[kani::proof]
    #[kani::unwind(40)]
    fn rx_arm_preserves_input_and_error() {
        properties::rx_arm(
            8 + kani::any::<u8>() % 112,
            usize::from(kani::any::<u8>() % 4) + 1,
            kani::any(),
            reject(),
        );
    }

    #[kani::proof]
    #[kani::unwind(40)]
    fn tx_poll_preserves_owner_and_outcome() {
        properties::tx_polls([decision(), decision(), decision()], kani::any());
    }

    #[kani::proof]
    #[kani::unwind(40)]
    fn rx_poll_preserves_owner_and_outcome() {
        properties::rx_polls(
            [decision(), decision(), decision()],
            kani::any(),
            kani::any(),
            usize::from(kani::any::<u8>() % 4) + 1,
        );
    }

    #[kani::proof]
    #[kani::unwind(40)]
    fn forgetting_pending_owners_does_not_complete() {
        properties::forget(kani::any());
    }
}

#[cfg(test)]
#[forbid(unsafe_code)]
mod tests {
    use super::{
        model::{Decision, Path},
        properties,
    };
    use i2c_one_shot_interface::{Error, Status, WakeOn};

    fn status(bits: u8) -> Status {
        Status {
            pin_low_timeout: bits & 1 != 0,
            nack: bits & 2 != 0,
            arbitration_loss: bits & 4 != 0,
            fifo_error: bits & 8 != 0,
            stop_seen: bits & 16 != 0,
            controller_idle: bits & 32 != 0,
            bus_idle: bits & 64 != 0,
        }
    }

    const ERRORS: [Error; 10] = [
        Error::InvalidInput,
        Error::Clock,
        Error::Busy,
        Error::Nack,
        Error::ArbitrationLoss,
        Error::Fifo,
        Error::PinLowTimeout,
        Error::Dma,
        Error::Timeout,
        Error::UnexpectedStop,
    ];

    #[test]
    fn all_start_statuses_and_addresses() {
        for bits in 0..128 {
            for address in 0..=255 {
                properties::start(status(bits), address);
            }
        }
    }
    #[test]
    fn all_three_fifo_sample_sequences() {
        for bits in 0..8 {
            properties::stop([bits & 1 != 0, bits & 2 != 0, bits & 4 != 0]);
        }
    }
    #[test]
    fn all_readiness_inputs() {
        for address in 0..=255 {
            for length in 0..=255 {
                for empty in [false, true] {
                    properties::read_ready(address, length, empty);
                }
            }
        }
    }
    #[test]
    fn all_status_bits_and_wake_paths() {
        for bits in 0..128 {
            for path in [Path::Cpu, Path::Tx, Path::Rx] {
                for wake in [WakeOn::Completion, WakeOn::TxSpace] {
                    properties::observe(status(bits), wake, path);
                }
            }
        }
    }
    #[test]
    fn arm_success_and_every_error() {
        for reject in core::iter::once(None).chain(ERRORS.map(Some)) {
            for length in 1..=32 {
                properties::tx_arm(length, reject);
            }
            for length in 1..=4 {
                for empty in [false, true] {
                    properties::rx_arm(0x2a, length, empty, reject);
                }
            }
        }
    }
    #[test]
    fn bounded_polls_and_cancellation() {
        let decisions: Vec<_> = [Decision::Pending, Decision::Complete]
            .into_iter()
            .chain(ERRORS.map(Decision::Failed))
            .collect();
        for a in &decisions {
            for b in &decisions {
                for c in &decisions {
                    for mask in 0..8 {
                        let cancel = [mask & 1 != 0, mask & 2 != 0, mask & 4 != 0];
                        properties::tx_polls([*a, *b, *c], cancel);
                        properties::rx_polls([*a, *b, *c], cancel, [0x10, 0x28, 0xe3, 0x96], 4);
                    }
                }
            }
        }
    }
    #[test]
    fn forgotten_tx_and_rx_do_not_complete() {
        properties::forget(false);
        properties::forget(true);
    }
}
