//! Final SoC-resource ordering below the owner, not a proof of eDMA internals.
#![forbid(unsafe_code)]

pub(super) trait ReleaseIo {
    fn disable_irq(&mut self);
    fn quiesce_dma(&mut self);
    fn unpend_irq(&mut self);
    fn disable_pins(&mut self);
    fn release_clock_power(&mut self);
}

pub(super) fn release(io: &mut impl ReleaseIo) {
    io.disable_irq();
    io.quiesce_dma();
    io.unpend_irq();
    io.disable_pins();
    io.release_clock_power();
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use std::vec::Vec;

    #[test]
    fn resource_release_never_drops_clock_power_before_dma_quiescence() {
        struct Recorder(Vec<&'static str>);
        impl ReleaseIo for Recorder {
            fn disable_irq(&mut self) {
                self.0.push("irq-off");
            }
            fn quiesce_dma(&mut self) {
                self.0.push("dma-quiescent");
            }
            fn unpend_irq(&mut self) {
                self.0.push("irq-unpend");
            }
            fn disable_pins(&mut self) {
                self.0.push("pins-off");
            }
            fn release_clock_power(&mut self) {
                self.0.push("clock-power-release");
            }
        }
        let mut recorder = Recorder(Vec::new());
        release(&mut recorder);
        assert_eq!(
            recorder.0,
            [
                "irq-off",
                "dma-quiescent",
                "irq-unpend",
                "pins-off",
                "clock-power-release"
            ]
        );
    }
}
