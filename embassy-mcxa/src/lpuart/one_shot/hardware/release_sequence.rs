//! SoC resource-release effects, below the owner and above the HAL calls.
//! The test checks this actual ordering, not the internals of eDMA quiescence.
#![forbid(unsafe_code)]

pub(super) trait ReleaseIo {
    fn disable_irq(&mut self);
    fn quiesce_dma(&mut self);
    fn unpend_irq(&mut self);
    fn disable_pin(&mut self);
    fn release_clock(&mut self);
}

pub(super) fn release(io: &mut impl ReleaseIo) {
    io.disable_irq();
    io.quiesce_dma();
    io.unpend_irq();
    io.disable_pin();
    io.release_clock();
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use std::vec::Vec;

    #[test]
    fn resource_release_never_gates_uart_before_dma_quiescence() {
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
            fn disable_pin(&mut self) {
                self.0.push("pin-off");
            }
            fn release_clock(&mut self) {
                self.0.push("clock-off");
            }
        }
        let mut recorder = Recorder(Vec::new());
        release(&mut recorder);
        assert_eq!(
            recorder.0,
            ["irq-off", "dma-quiescent", "irq-unpend", "pin-off", "clock-off"]
        );
    }
}
