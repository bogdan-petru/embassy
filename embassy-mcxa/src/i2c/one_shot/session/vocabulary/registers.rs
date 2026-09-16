//! Layer 2: the small trusted register/resource capability implementation.
//! The checked direct generator emits state-specific accessors backed by
//! field-sized safe-mmio handles.
//! The PAC supplies offsets/bit meanings; Embassy supplies clocks, pins and eDMA.
//! It owns raw resources and grants opaque CPU/DMA capabilities to layer 3.

mod binding;
mod effects;
mod endpoint;
mod mmio;
mod release_sequence;

pub use binding::InterruptHandler;
pub(super) use binding::RealEffects;
pub use binding::{ClockConfig, McxaI2cPort};
pub(super) use effects::Effects;
use endpoint::ErasedEndpointPair;

#[cfg(all(test, feature = "one-shot-host-tests"))]
pub(super) mod test_support;

use core::task::{Context, Poll};

use i2c_one_shot_interface::{Command, Device, DmaResult, Error, ReceiveResult, Status, WakeOn};
use i2c_one_shot_vocabulary::Hardware as L3Hardware;

use crate::pac;
use crate::pac::lpi2c::{Alf, Cmd, Mbf, Mccr0, Mcfgr1, Mier, Msr, MsrBbf, MsrFef, MsrSdf, Mtdr, Ndf, Pltf, Prescale};

pub(in crate::i2c) struct Registers {
    owned: mmio::Lpi2cRegisters<mmio::Unconfirmed>,
    tx_fifo: *mut u8,
    rx_fifo: *const u8,
}

impl Registers {
    /// # Safety
    /// Caller owns the singleton and its live clocked mapping. No other CPU
    /// accessor or DMA channel may use the register block.
    /// No hardware-state claim is made until reset_confirmed succeeds.
    pub(in crate::i2c) unsafe fn from_pac(pac: pac::lpi2c::Lpi2c) -> Self {
        mmio::check_layout(pac);
        Self {
            // SAFETY: forwarded exclusive ownership and reset-state contract.
            owned: unsafe { mmio::Lpi2cRegisters::new(pac.as_ptr().cast()) },
            // Address calculation only, never a PAC register access.
            tx_fifo: pac.mtdr().as_ptr().cast(),
            // MRDR is a fixed DMA source; the pointer never becomes a CPU
            // FIFO accessor or a writable endpoint.
            rx_fifo: pac.mrdr().as_ptr().cast(),
        }
    }
}

/// The concrete, trusted `Device` implementation. It owns the paired CPU
/// register/channel authority; upper layers receive only `Hardware<Self>`.
pub(in crate::i2c::one_shot) struct DeviceHardware<E: Effects = RealEffects> {
    // Taken only by a consuming arm/completion or synchronous shutdown. The
    // register owner and channel effects move together, never independently.
    parts: Option<CpuParts<E>>,
}

struct CpuParts<E: Effects> {
    registers: mmio::Lpi2cRegisters<mmio::Enabled>,
    endpoints: ErasedEndpointPair,
    effects: E,
}

/// One private lifecycle implementation; the concrete buffer type keeps TX and
/// RX authority distinct. There is no raw-parts or pre-quiescence buffer access.
pub(in crate::i2c::one_shot) struct DmaLease<E: Effects, B: 'static> {
    parts: Option<DmaParts<E, B>>,
}

struct DmaParts<E: Effects, B: 'static> {
    registers: mmio::Lpi2cRegisters<mmio::Streaming>,
    endpoints: ErasedEndpointPair,
    effects: E,
    // The actual reference moves with the channel, including when forgotten.
    // RX retains exclusive storage; TX retains immutable static source storage.
    buffer: B,
}

pub(in crate::i2c::one_shot) type StreamingLease<E = RealEffects> = DmaLease<E, &'static [u8]>;
pub(in crate::i2c::one_shot) type ReceivingLease<E = RealEffects> = DmaLease<E, &'static mut [u8]>;

impl<E: Effects, B: 'static> DmaLease<E, B> {
    fn observe(&mut self, cx: &mut Context<'_>, wake: WakeOn) -> Status {
        let parts = self.parts.as_mut().expect("DMA pair exists until consumed");
        observe(&mut parts.registers, &parts.effects, cx, wake)
    }

    // Only successful quiescence takes the pair out. Pending/errors leave Drop
    // armed, and no safe caller can reach the returned buffer before this exit.
    fn poll(&mut self, cx: &mut Context<'_>) -> Poll<Result<(DeviceHardware<E>, B), Error>> {
        let parts = self.parts.as_mut().expect("DMA pair exists until consumed");
        while effects::register_wait(parts.effects.dma_wait(), cx, |ready| {
            parts.effects.dma_wait_observed(ready)
        })
        .is_ready()
        {}
        if parts.effects.dma_error() {
            return Poll::Ready(Err(Error::Dma));
        }
        if !parts.effects.dma_done() {
            return Poll::Pending;
        }
        let DmaParts {
            registers,
            endpoints,
            effects,
            buffer,
        } = self.parts.take().expect("one DMA pair");
        // MDER-off -> exact channel quiescence -> acquire fence; DONE alone
        // cannot return either CPU authority or RX storage, nor prove STOP.
        let (registers, effects) = registers.quiesce_with(effects);
        Poll::Ready(Ok((DeviceHardware::from_owned(registers, endpoints, effects), buffer)))
    }
}

/// Safe upper-layer owner. Its `Backend` implementation lives in the
/// vocabulary crate and is parameterized by this private Device.
pub(in crate::i2c::one_shot) type Hardware<E = RealEffects> = L3Hardware<DeviceHardware<E>>;

impl DeviceHardware {
    pub(in crate::i2c::one_shot) fn new(port: McxaI2cPort) -> Result<Self, Error> {
        let McxaI2cPort {
            registers,
            endpoints,
            effects,
        } = port;
        let mut owned = registers
            .reset_confirmed(|| effects.now_us())
            .expect("one-shot reset confirmation failed");
        owned.disable_requests();
        owned.write_mier(0);
        let mut config = Mcfgr1(0);
        config.set_prescale(Prescale::DivideBy1);
        owned.write_mcfgr1(config.0);
        // The port enforces the unchanged 12 MHz functional clock:
        // 12 MHz / (62 + 54 + 2 + 2 synchronizer cycles) = 100 kHz.
        let mut timing = Mccr0(0);
        timing.set_clklo(62);
        timing.set_clkhi(54);
        timing.set_sethold(59);
        timing.set_datavd(29);
        owned.write_mccr0(timing.0);
        let registers = owned
            .enable_confirmed(|| effects.now_us())
            .expect("one-shot enable confirmation failed");
        effects.irq_unpend();
        effects.irq_enable();
        Ok(Self::from_owned(registers, endpoints, effects))
    }
}

impl<E: Effects> DeviceHardware<E> {
    fn from_owned(registers: mmio::Lpi2cRegisters<mmio::Enabled>, endpoints: ErasedEndpointPair, effects: E) -> Self {
        Self {
            parts: Some(CpuParts {
                registers,
                endpoints,
                effects,
            }),
        }
    }

    fn parts(&mut self) -> &mut CpuParts<E> {
        self.parts.as_mut().expect("CPU pair exists until consumed")
    }

    fn parts_ref(&self) -> &CpuParts<E> {
        self.parts.as_ref().expect("CPU pair exists until consumed")
    }
}

impl<E: Effects> CpuParts<E> {
    fn command(&mut self, command: Cmd, data: u8) {
        let mut word = Mtdr(0);
        word.set_cmd(command);
        word.set_data(data);
        self.registers.write_mtdr(word.0);
    }
}

// A narrow internal observation surface shared by CPU and DMA register states.
// It does not grant writes to MTDR, arbitrary MDER, or MCR in Streaming.
trait ObservableRegisters {
    fn status(&self) -> u32;
    fn interrupt_mask(&mut self, word: u32);
}

macro_rules! observable_state {
    ($state:ty) => {
        impl ObservableRegisters for mmio::Lpi2cRegisters<$state> {
            fn status(&self) -> u32 {
                self.read_msr()
            }
            fn interrupt_mask(&mut self, word: u32) {
                self.write_mier(word);
            }
        }
    };
}
observable_state!(mmio::Enabled);
observable_state!(mmio::Streaming);

fn observe<R: ObservableRegisters, E: Effects>(
    registers: &mut R,
    effects: &E,
    cx: &mut Context<'_>,
    wake: WakeOn,
) -> Status {
    while effects::register_wait(effects.i2c_wait(), cx, |ready| effects.i2c_wait_observed(ready)).is_ready() {}
    let mut interrupts = Mier(0);
    interrupts.set_ndie(true);
    interrupts.set_alie(true);
    interrupts.set_feie(true);
    interrupts.set_pltie(true);
    interrupts.set_sdie(true);
    interrupts.set_tdie(matches!(wake, WakeOn::TxSpace));
    registers.interrupt_mask(interrupts.0);
    effects.irq_enable();
    decode(Msr(registers.status()))
}

fn decode(snapshot: Msr) -> Status {
    // One MSR read, all raised faults preserved. The upper protocol core, not
    // this trusted Device, decides fault priority and completion policy.
    Status {
        pin_low_timeout: snapshot.pltf() == Pltf::IntYes,
        nack: snapshot.ndf() == Ndf::IntYes,
        arbitration_loss: snapshot.alf() == Alf::IntYes,
        fifo_error: snapshot.fef() == MsrFef::IntYes,
        stop_seen: snapshot.sdf() == MsrSdf::IntYes,
        controller_idle: snapshot.mbf() == Mbf::Idle,
        bus_idle: snapshot.bbf() == MsrBbf::Idle,
    }
}

// SAFETY: private construction pairs the singleton's register map and both
// directional endpoint/request pairs with one owned E (RealEffects owns its
// DmaChannel). No channel or register capability is exposed by safe Device
// calls. TX arm moves the whole pair and static source into StreamingLease;
// RX arm moves the whole pair and exclusive static destination into
// ReceivingLease. Only their consuming quiescence paths restore CPU authority,
// and only RX's path returns storage after request-off, quiescence, and fence.
// Pending retains that pair; errors and ordinary drops shut requests off and
// quiesce before resetting/releasing resources.
// Observations decode the current owned controller's single MSR snapshot; DONE
// is accepted only without DMA error, then requests are disabled and the same
// owned channel quiesced. This remains memory-safe even if the safe upper
// vocabulary composes Device calls in an invalid protocol order (it can fail or
// misoperate the bus). E's private implementations and DMA/power contracts
// remain reviewed trust.
unsafe impl<E: Effects> Device for DeviceHardware<E> {
    type InFlight = StreamingLease<E>;
    type Receiving = ReceivingLease<E>;

    fn snapshot(&self) -> Status {
        decode(Msr(self.parts_ref().registers.read_msr()))
    }

    fn command(&mut self, command: Command) {
        match command {
            Command::Start(address) => self.parts().command(Cmd::START, address << 1),
            Command::Stop => self.parts().command(Cmd::STOP, 0),
        }
    }

    fn fifo_empty(&self) -> bool {
        pac::lpi2c::Mfsr(self.parts_ref().registers.read_mfsr()).txcount() == 0
    }

    fn arm(mut self, bytes: &'static [u8]) -> Result<Self::InFlight, Error> {
        let parts = self.parts();
        // Keep CPU Drop responsibility through the fallible preparation. The
        // remaining arming operations are synchronous: no cancellation gap.
        parts.effects.prepare(bytes, parts.endpoints.tx())?;
        parts.effects.select_request(parts.endpoints.tx());
        parts.effects.dma_interrupt_enable();
        parts.effects.dma_request_enable();
        // From this take to the lease constructor, the sole operation is the
        // generated fixed-word volatile MDER write: no allocation, await,
        // layout check, fallible builder, or firmware callback. The firmware
        // graph gate excludes custom-mmio. A deliberately panicking host MMIO
        // script is not a promise of full engine cleanup during this handover.
        let CpuParts {
            registers,
            endpoints,
            effects,
        } = self.parts.take().expect("one CPU pair");
        let registers = registers.into_arm_dma();
        Ok(StreamingLease {
            parts: Some(DmaParts {
                registers,
                endpoints,
                effects,
                buffer: bytes,
            }),
        })
    }

    fn observe(&mut self, cx: &mut Context<'_>, wake: WakeOn) -> Status {
        let parts = self.parts();
        observe(&mut parts.registers, &parts.effects, cx, wake)
    }

    fn observe_dma(lease: &mut Self::InFlight, cx: &mut Context<'_>, wake: WakeOn) -> Status {
        lease.observe(cx, wake)
    }

    fn poll_dma(mut lease: Self::InFlight, cx: &mut Context<'_>) -> DmaResult<Self> {
        match lease.poll(cx) {
            Poll::Pending => DmaResult::Pending(lease),
            Poll::Ready(Err(error)) => DmaResult::Failed(error),
            Poll::Ready(Ok((cpu, _source))) => DmaResult::Complete(cpu),
        }
    }

    fn arm_receive(mut self, address: u8, bytes: &'static mut [u8]) -> Result<Self::Receiving, Error> {
        // Keep this lower boundary independently defensive: callers normally
        // preflight these values, but only this consuming operation can make
        // the FIFO check and DMA arm adjacent to the generated RX commands.
        if !(0x08..=0x77).contains(&address) || !(1..=32).contains(&bytes.len()) {
            return Err(Error::InvalidInput);
        }
        let count_minus_one = (bytes.len() - 1) as u8;
        // A readiness observation is not a permit. Re-read the live FIFO
        // immediately before effects; this exclusive owner retains no other
        // command producer between the check and the fixed read queue.
        if !self.fifo_empty() {
            return Err(Error::Busy);
        }
        let parts = self.parts();
        // The lower lease, not this temporary effect call, retains the static
        // mutable reference. A failed setup consumes it with this one-shot
        // operation; a successful setup cannot expose it while DMA may write.
        parts.effects.prepare_receive(&mut *bytes, parts.endpoints.rx())?;
        parts.effects.select_receive_request(parts.endpoints.rx());
        parts.effects.dma_interrupt_enable();
        parts.effects.dma_request_enable();
        // Once CPU authority is consumed, the generator performs the reviewed
        // synchronous ordered sequence: fixed RDDE, then START-read, then
        // RECEIVE. No CPU FIFO/reset/command access exists on the resulting
        // Streaming owner.
        let CpuParts {
            registers,
            endpoints,
            effects,
        } = self.parts.take().expect("one CPU pair");
        let registers = registers.into_arm_rx_dma(address, count_minus_one);
        Ok(ReceivingLease {
            parts: Some(DmaParts {
                registers,
                endpoints,
                effects,
                buffer: bytes,
            }),
        })
    }

    fn observe_receive(lease: &mut Self::Receiving, cx: &mut Context<'_>, wake: WakeOn) -> Status {
        lease.observe(cx, wake)
    }

    fn poll_receive(mut lease: Self::Receiving, cx: &mut Context<'_>) -> ReceiveResult<Self> {
        match lease.poll(cx) {
            Poll::Pending => ReceiveResult::Pending(lease),
            Poll::Ready(Err(error)) => ReceiveResult::Failed(error),
            Poll::Ready(Ok((cpu, bytes))) => ReceiveResult::Complete((cpu, bytes)),
        }
    }
}

impl<E: Effects> Drop for DeviceHardware<E> {
    fn drop(&mut self) {
        let Some(CpuParts {
            mut registers,
            endpoints: _,
            mut effects,
        }) = self.parts.take()
        else {
            return;
        };
        effects.irq_disable();
        effects.irq_unpend();
        registers.write_mier(0);
        registers.disable_requests();
        effects.quiesce();
        let _disabled = registers
            .reset_confirmed(|| effects.now_us())
            .expect("one-shot shutdown reset confirmation failed");
        effects.release_pins();
        // No promise of peer recovery: MEN-off is not a physical bus-clear.
    }
}

impl<E: Effects, B: 'static> Drop for DmaLease<E, B> {
    fn drop(&mut self) {
        let Some(DmaParts {
            mut registers,
            endpoints: _,
            effects,
            buffer,
        }) = self.parts.take()
        else {
            return;
        };
        effects.irq_disable();
        effects.irq_unpend();
        registers.write_mier(0);
        // This is the same consuming paired-resource exit as successful DMA,
        // but cancellation/fault never yields successful protocol evidence.
        let (registers, mut effects) = registers.quiesce_with(effects);
        // Keep the actual TX/RX reference through shutdown. In particular, RX
        // exclusivity cannot end before RDDE-off, ACTIVE drain and the fence.
        let _buffer_held_until_shutdown = buffer;
        let _disabled = registers
            .reset_confirmed(|| effects.now_us())
            .expect("one-shot DMA shutdown reset confirmation failed");
        effects.release_pins();
    }
}

// Compile-only anchors name the actual generated HAL owner; the test command
// also links the real host recorder.
#[cfg(test)]
#[path = "registers/direct_map_fixture.rs"]
mod direct_map_fixture;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn one_shot_completion_requires_stop_and_idle_not_just_fifo_progress() {
        let mut snapshot = Msr(0);
        assert!(!decode(snapshot).stop_seen);
        assert!(decode(snapshot).controller_idle);
        assert!(decode(snapshot).bus_idle);
        snapshot.set_sdf(MsrSdf::IntYes);
        snapshot.set_mbf(Mbf::Busy);
        assert!(decode(snapshot).stop_seen);
        assert!(!decode(snapshot).controller_idle);
        snapshot.set_mbf(Mbf::Idle);
        assert!(decode(snapshot).stop_seen && decode(snapshot).controller_idle);
        snapshot.set_bbf(MsrBbf::Busy);
        assert!(!decode(snapshot).bus_idle);
    }

    #[test]
    fn one_shot_decoder_preserves_every_co_latched_fault_without_priority() {
        for bits in 0_u8..16 {
            let mut snapshot = Msr(0);
            if bits & 1 != 0 {
                snapshot.set_pltf(Pltf::IntYes);
            }
            if bits & 2 != 0 {
                snapshot.set_ndf(Ndf::IntYes);
            }
            if bits & 4 != 0 {
                snapshot.set_alf(Alf::IntYes);
            }
            if bits & 8 != 0 {
                snapshot.set_fef(MsrFef::IntYes);
            }
            // Completion indicators remain independent even with all faults.
            snapshot.set_sdf(MsrSdf::IntYes);
            let observed = decode(snapshot);
            assert_eq!(observed.pin_low_timeout, bits & 1 != 0);
            assert_eq!(observed.nack, bits & 2 != 0);
            assert_eq!(observed.arbitration_loss, bits & 4 != 0);
            assert_eq!(observed.fifo_error, bits & 8 != 0);
            assert!(observed.stop_seen && observed.controller_idle && observed.bus_idle);
        }
    }
}
