//! Shape-level ownership checks. Physical quiescence is the Device's contract,
//! not something this generic result or its map operation can establish.
use std::cell::Cell;

use i2c_one_shot_interface::{Error, Step};

struct Owner<'a>(&'a Cell<usize>);
impl Drop for Owner<'_> {
    fn drop(&mut self) {
        self.0.set(self.0.get() + 1);
    }
}

#[test]
fn pending_maps_only_the_retained_owner_and_does_not_drop_it() {
    let drops = Cell::new(0);
    let calls = Cell::new(0);
    let step: Step<_, ()> = Step::Pending(Owner(&drops));
    let Step::Pending(owner) = step.map(
        |owner| {
            calls.set(calls.get() + 1);
            owner
        },
        |_| panic!("Pending cannot become completion"),
    ) else {
        panic!("pending owner was lost")
    };
    assert_eq!(calls.get(), 1);
    assert_eq!(drops.get(), 0);
    drop(owner);
    assert_eq!(drops.get(), 1);
}

#[test]
fn complete_maps_only_the_completed_owner_without_early_drop() {
    let drops = Cell::new(0);
    let calls = Cell::new(0);
    let step: Step<(), _> = Step::Complete(Owner(&drops));
    let Step::Complete(owner) = step.map(
        |_| panic!("Complete cannot become Pending"),
        |owner| {
            calls.set(calls.get() + 1);
            owner
        },
    ) else {
        panic!("completed owner was lost")
    };
    assert_eq!(calls.get(), 1);
    assert_eq!(drops.get(), 0);
    drop(owner);
    assert_eq!(drops.get(), 1);
}

#[test]
fn failure_keeps_its_error_without_minting_an_owner() {
    let step: Step<(), ()> = Step::Failed(Error::Dma);
    assert!(matches!(
        step.map(
            |_| panic!("Failed has no pending owner"),
            |_| panic!("Failed has no completed owner")
        ),
        Step::Failed(Error::Dma)
    ));
}
