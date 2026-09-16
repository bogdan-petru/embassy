//! Proof harnesses compile the real synchronous decisions, not a copy of them.
//! This package is never a firmware dependency. Kani supplies its own compiler.
#![forbid(unsafe_code)]

#[cfg(kani)]
mod proofs {
    use i2c_one_shot_core::{Error, Status, checked, validate_transfer};

    fn observation() -> Status {
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

    #[kani::proof]
    fn fault_priority_for_every_snapshot() {
        let status = observation();
        let want_stop: bool = kani::any();
        let answer = checked(status, want_stop);
        let expected = if status.pin_low_timeout {
            Some(Error::PinLowTimeout)
        } else if status.nack {
            Some(Error::Nack)
        } else if status.arbitration_loss {
            Some(Error::ArbitrationLoss)
        } else if status.fifo_error {
            Some(Error::Fifo)
        } else if status.stop_seen && !want_stop {
            Some(Error::UnexpectedStop)
        } else {
            None
        };
        match expected {
            Some(error) => assert_eq!(answer, Err(error)),
            None => assert_eq!(answer, Ok(status)),
        }
    }

    #[kani::proof]
    fn completion_observation_never_swallows_a_fault() {
        let status = observation();
        let accepted = checked(status, true).is_ok();
        assert_eq!(
            accepted,
            !(status.pin_low_timeout || status.nack || status.arbitration_loss || status.fifo_error)
        );
        // This proves fault selection only, NOT the async STOP wait or silicon.
    }

    #[kani::proof]
    fn all_input_sizes_are_rejected_or_fit_one_receive_command() {
        let address: u8 = kani::any();
        let length: usize = kani::any();
        let valid = validate_transfer(address, length).is_ok();
        assert_eq!(valid, address >= 8 && address <= 0x77 && length >= 1 && length <= 32);
        if valid {
            let receive_count = (length - 1) as u8;
            assert_eq!(usize::from(receive_count) + 1, length);
        }
    }
}
