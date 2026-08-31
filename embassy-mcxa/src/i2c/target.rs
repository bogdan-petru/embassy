//! LPI2C Target Driver
//!
//! This module provides an implementation of an I2C target (slave)
//! driver. It supports both blocking and asynchronous modes of
//! operation, as well as DMA-based transfers. The driver allows the
//! target device to respond to requests from an I2C controller
//! (master), including reading and writing data, handling general
//! calls, and responding to SMBus alerts.
//!
//! ## Example
//!
//! ```rust,no_run
//! #![no_std]
//! #![no_main]
//!
//! # extern crate panic_halt;
//! # extern crate embassy_mcxa;
//! # extern crate embassy_executor;
//! # use panic_halt as _;
//! use embassy_executor::Spawner;
//! use embassy_mcxa::clocks::config::Div8;
//! use embassy_mcxa::config::Config;
//! use embassy_mcxa::i2c::target;
//!
//! #[embassy_executor::main]
//! async fn main(_spawner: Spawner) {
//!     let mut config = Config::default();
//!     config.clock_cfg.sirc.fro_lf_div = Div8::from_divisor(1);
//!
//!     let p = embassy_mcxa::init(config);
//!
//!     let mut config = target::Config::default();
//!     config.address = target::Address::Dual(0x2a, 0x31);
//!     let mut i2c = target::I2c::new_blocking(p.LPI2C3, p.P3_27, p.P3_28, config).unwrap();
//!     let mut buf = [0u8; 32];
//!
//!     loop {
//!         let request = i2c.blocking_listen().unwrap();
//!         match request {
//!             target::Request::Read(addr) => {
//!                 // Controller wants to read from us at `addr`
//!                 buf.fill(0x55);
//!                 let _status = i2c.blocking_respond_to_read(&buf).unwrap();
//!             }
//!             target::Request::Write(_addr) => {
//!                 // Controller wants to write to us at `addr`
//!                 let _status = i2c.blocking_respond_to_write(&mut buf).unwrap();
//!             }
//!             target::Request::Stop(_addr) => {
//!                 // Controller issued a STOP condition for `addr`
//!             }
//!             target::Request::GeneralCall => {
//!                 // Controller issued a General Call (broadcast write
//!                 // to address 0x00). Drain the payload via the
//!                 // normal write-response path.
//!                 let _status = i2c.blocking_respond_to_write(&mut buf).unwrap();
//!             }
//!             target::Request::SmbusAlert => {
//!                 // Controller issued an SMBus Alert
//!             }
//!             _ => {}
//!         }
//!     }
//! }
//! ```

use core::future::poll_fn;
use core::marker::PhantomData;
use core::ops::Range;
use core::task::Poll;

use embassy_hal_internal::Peri;
use embassy_hal_internal::drop::OnDrop;
use maitake_sync::WaitCell;

use super::{Async, AsyncMode, Blocking, Dma, Info, Instance, Mode, SclPin, SdaPin};
pub use crate::clocks::PoweredClock;
pub use crate::clocks::periph_helpers::{Div4, Lpi2cClockSel, Lpi2cConfig};
use crate::clocks::{ClockError, WakeGuard, enable_and_reset};
use crate::dma::{Channel, DMA_MAX_TRANSFER_SIZE, DmaChannel, DmaRequest};
use crate::gpio::{AnyPin, SealedPin};
use crate::interrupt;
use crate::interrupt::typelevel::Interrupt;
use registers::{ChunkEnd, ListenEvent, RxChunkEnd, TargetFault, TargetRxEvent, TargetTxStep};

// Target protocol MMIO is private to this driver tree. The controller driver
// has its own facade, so the two modes cannot name or operate each other's
// protocol events, DMA leases, or raw Tock cells.
#[path = "target_registers.rs"]
mod registers;
// Only the opaque facade name crosses the I2C sibling boundary so `Info`
// can construct it. Its operational methods remain `pub(super)` inside this
// target tree; controller code cannot operate a target facade.
pub(in crate::i2c) use registers::TargetRegisters;

/// Errors exclusive to hardware Initialization
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum SetupError {
    /// Clock configuration error.
    ClockSetup(ClockError),
    /// The selected LPI2C source provides no functional clock.
    ///
    /// This rejects `Lpi2cClockSel::None` before the constructor changes
    /// DMA, interrupt, pin, or target-controller state.
    NoFunctionalClock,
    /// Address is out of range, mixes address widths, or describes an empty
    /// or reversed range.
    InvalidAddress,
    /// Other internal errors or unexpected state.
    Other,
}

/// Errors exclusive to I/O
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum IOError {
    /// Busy Busy
    BusBusy,
    /// Target Busy
    TargetBusy,
    /// FIFO Error
    FifoError,
    /// Bit Error
    BitError,
    /// Other internal errors or unexpected state.
    Other,
}

impl From<crate::dma::InvalidParameters> for IOError {
    fn from(_value: crate::dma::InvalidParameters) -> Self {
        IOError::Other
    }
}

impl From<TargetFault> for IOError {
    fn from(value: TargetFault) -> Self {
        match value {
            TargetFault::Bit => IOError::BitError,
            TargetFault::Fifo => IOError::FifoError,
        }
    }
}

/// Outcome of a `respond_to_read` call.
///
/// The `usize` in every variant counts bytes **queued** for transmission
/// — consumed from the supplied buffer and written into the transmit
/// register.
///
/// # It is not a count of bytes that reached the bus
///
/// When a transfer terminates, `STDR` may hold one byte that was queued
/// but never clocked out (the target has no transmit FIFO beyond that
/// single register), and it is discarded — measured on FRDM-MCXA577:
/// the next transaction never transmits it stale. The count therefore
/// overshoots the bytes the controller actually took by up to one per
/// terminated transfer. All three implementations behave this way — the
/// blocking and interrupt paths count each write to `STDR`, the DMA
/// path counts bytes the engine moved into it.
///
/// A correction was prototyped and measured rather than assumed: TDF
/// sampled in the same status snapshot that detects the termination
/// identifies the stranded byte (SSR reads `SDF` with `TDF` clear,
/// 48 of 49 sampled terminations). But it is raceable, not reliable:
/// if servicing the termination is delayed past the *next*
/// transaction's address phase — back-to-back transfers plus interrupt
/// latency are enough — the first observable snapshot already reads
/// `SDF|AVF|TDF|BBF`, where TDF belongs to the new transfer and the
/// stranded byte's fate is unrecoverable from any register (1 of 49:
/// a silent off-by-one). The ambiguous case is *detectable* (AVF/BBF
/// alongside SDF) but not *resolvable*, and this PAC has no target
/// FIFO status register to resolve it with.
///
/// So the count is buffer bookkeeping only: it says how much of the
/// supplied buffer the driver consumed, nothing more. Resuming a
/// follow-up transmission at this offset silently omits up to one
/// stranded byte per terminated transfer from the stream the
/// controller receives — whether that is tolerable is a protocol
/// decision, not a default. And the count must **not** be used to
/// advance a device-side position, a cursor, or any other model of
/// what the peer received — doing that skips a byte per terminated
/// transfer. Exact transmission progress is not available through
/// this API on this IP.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ReadStatus {
    /// Controller terminated the read with NACK + STOP exactly when the
    /// supplied buffer was exhausted.
    Complete(usize),
    /// Buffer was fully consumed but the controller is still asking for
    /// more bytes (it ACKed the last byte). Caller should call
    /// `respond_to_read` again with additional bytes, or accept that the
    /// bus will clock-stretch (with TXDSTALL enabled) until something
    /// else terminates the transfer.
    ///
    /// This is the one variant whose count is exact: the transfer has
    /// not terminated, so nothing has been discarded.
    NeedMore(usize),
    /// Controller issued an early STOP or repeated START before the
    /// buffer was exhausted.
    EarlyStop(usize),
}

/// Outcome of a `respond_to_write` call.
///
/// The `usize` in every variant counts bytes written into the supplied
/// buffer, i.e. bytes the target ACKed.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum WriteStatus {
    /// Controller issued STOP.
    Stopped(usize),
    /// Controller issued a repeated START. The next `listen` call will
    /// report the direction/address of the new sub-transaction.
    Restarted(usize),
    /// The supplied buffer filled before the controller terminated the
    /// transfer. Caller should call `respond_to_write` again with more
    /// buffer space, or accept that the bus will clock-stretch (with
    /// RXSTALL enabled) until something else terminates the transfer.
    BufferFull(usize),
}

/// I2C interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        T::PERF_INT_INCR();
        let registers = T::info().target_registers();
        if registers.disable_interrupts_if_enabled() {
            T::PERF_INT_WAKE_INCR();
            T::info().wait_cell().wake();
        }
    }
}

/// I2C target addresses.
#[derive(Clone)]
pub enum Address {
    /// One 7-bit (`0x000..=0x07f`) or 10-bit (`0x080..=0x3ff`) address.
    Single(u16),
    /// Two addresses of the same width within `0x000..=0x3ff`.
    Dual(u16, u16),
    /// End-exclusive range of addresses.
    ///
    /// The start and final included address (`end - 1`) must have the same
    /// width. `end` may be one past a width boundary, so `0x20..0x30` is a
    /// 7-bit range, `0x00..0x80` covers every 7-bit address, and
    /// `0x80..0x400` covers the representable 10-bit range.
    Range(Range<u16>),
}

/// A target address plan that has passed the public API's validation rules.
///
/// Only [`Address::validate`] can construct this. The register facade accepts
/// this proof rather than raw `u16`s, so a future setup path cannot silently
/// rely on the PAC setters' truncating masks or reintroduce range-end
/// arithmetic at an MMIO write site.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ValidatedAddress {
    Single7(u16),
    Single10(u16),
    Dual7(u16, u16),
    Dual10(u16, u16),
    Range7 { start: u16, last: u16 },
    Range10 { start: u16, last: u16 },
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum AddressWidth {
    SevenBit,
    TenBit,
}

impl Address {
    /// Validate this public address description before any peripheral,
    /// interrupt, DMA, or pin state is changed.
    fn validate(&self) -> Result<ValidatedAddress, SetupError> {
        fn width(address: u16) -> Option<AddressWidth> {
            match address {
                0x000..=0x07f => Some(AddressWidth::SevenBit),
                0x080..=0x3ff => Some(AddressWidth::TenBit),
                _ => None,
            }
        }

        match self {
            Address::Single(address) => match width(*address) {
                Some(AddressWidth::SevenBit) => Ok(ValidatedAddress::Single7(*address)),
                Some(AddressWidth::TenBit) => Ok(ValidatedAddress::Single10(*address)),
                None => Err(SetupError::InvalidAddress),
            },
            Address::Dual(first, second) => match (width(*first), width(*second)) {
                (Some(AddressWidth::SevenBit), Some(AddressWidth::SevenBit)) => {
                    Ok(ValidatedAddress::Dual7(*first, *second))
                }
                (Some(AddressWidth::TenBit), Some(AddressWidth::TenBit)) => {
                    Ok(ValidatedAddress::Dual10(*first, *second))
                }
                _ => Err(SetupError::InvalidAddress),
            },
            Address::Range(range) => {
                // `Range` is end-exclusive. Derive `last` only after an
                // empty/reversed range has been rejected, so no release-mode
                // wrap can turn `0..0` into a catch-all address match.
                let Some(last) = range.end.checked_sub(1) else {
                    return Err(SetupError::InvalidAddress);
                };
                if range.start > last {
                    return Err(SetupError::InvalidAddress);
                }

                match (width(range.start), width(last)) {
                    (Some(AddressWidth::SevenBit), Some(AddressWidth::SevenBit)) => Ok(ValidatedAddress::Range7 {
                        start: range.start,
                        last,
                    }),
                    (Some(AddressWidth::TenBit), Some(AddressWidth::TenBit)) => Ok(ValidatedAddress::Range10 {
                        start: range.start,
                        last,
                    }),
                    _ => Err(SetupError::InvalidAddress),
                }
            }
        }
    }
}

impl Default for Address {
    fn default() -> Self {
        Self::Single(0x2a)
    }
}

/// Enable or disable feature
#[derive(Copy, Clone, Default)]
pub enum Status {
    #[default]
    Disabled,
    Enabled,
}

impl From<Status> for bool {
    fn from(value: Status) -> Self {
        match value {
            Status::Disabled => false,
            Status::Enabled => true,
        }
    }
}

/// I2C target configuration
#[derive(Clone, Default)]
#[non_exhaustive]
pub struct Config {
    /// Addresses to respond to
    pub address: Address,

    /// Enable SMBus alert
    pub smbus_alert: Status,

    /// Enable general call support
    pub general_call: Status,

    /// Clock configuration
    pub clock_config: ClockConfig,
}

/// I2C target clock configuration
#[derive(Clone)]
#[non_exhaustive]
pub struct ClockConfig {
    /// Powered clock configuration
    pub power: PoweredClock,
    /// LPI2C clock source
    pub source: Lpi2cClockSel,
    /// LPI2C pre-divider
    pub div: Div4,
}

impl Default for ClockConfig {
    fn default() -> Self {
        Self {
            power: PoweredClock::NormalEnabledDeepSleepDisabled,
            source: Lpi2cClockSel::FroLfDiv,
            div: const { Div4::no_div() },
        }
    }
}

/// I2C target events
#[derive(Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Request {
    /// Controller wants to write data to this Target
    Write(u16),
    /// Controller wants to read data from this Target
    Read(u16),
    /// Controller issued Stop condition for this Target
    Stop(u16),
    /// Controller issued a General Call
    GeneralCall,
    /// Controller issued SMBUS Alert
    SmbusAlert,
}

/// I2C target events
#[derive(Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Event {
    SmbusAlert,
    GeneralCall,
    Address0Match(u16),
    Address1Match(u16),
    Stop(u16),
    RepeatedStart(u16),
    TransmitAck,
    AddressValid(u16),
    ReceiveData,
    TransmitData,
}

/// I2C Target Driver.
pub struct I2c<'d, M: Mode> {
    info: &'static Info,
    /// Peripheral input clock frequency in Hz, captured at construction.
    freq: u32,
    _scl: Peri<'d, AnyPin>,
    _sda: Peri<'d, AnyPin>,
    smbus_alert: Status,
    general_call: Status,
    mode: M,
    _wg: Option<WakeGuard>,
}

/// Validated target clock input prepared before constructor side effects.
///
/// The target only derives its data-valid delay from this frequency, but a
/// zero clock would otherwise be silently clamped into a plausible register
/// value and leave a nonfunctional target on the bus.
#[derive(Clone, Copy)]
struct TargetSetup {
    frequency: u32,
}

impl TargetSetup {
    fn new(frequency: u32) -> Result<Self, SetupError> {
        if frequency == 0 {
            return Err(SetupError::NoFunctionalClock);
        }
        Ok(Self { frequency })
    }
}

impl<'d, M: Mode> I2c<'d, M> {
    /// Validate the complete target clock configuration without programming
    /// MRCC or touching the peripheral. Async/DMA constructors call this
    /// before claiming channels or enabling their NVIC lines.
    fn construction_setup<T: Instance>(config: &Config) -> Result<TargetSetup, SetupError> {
        let ClockConfig { power, source, div } = config.clock_config;
        let clock = Lpi2cConfig {
            power,
            source,
            div,
            instance: T::CLOCK_INSTANCE,
        };
        let frequency = crate::clocks::with_clocks(|clocks| clock.functional_clock_hz(clocks))
            .ok_or(SetupError::ClockSetup(ClockError::NeverInitialized))?
            .map_err(SetupError::ClockSetup)?;
        TargetSetup::new(frequency)
    }

    fn new_inner<T: Instance>(
        _peri: Peri<'d, T>,
        scl: Peri<'d, impl SclPin<T>>,
        sda: Peri<'d, impl SdaPin<T>>,
        config: Config,
        address: ValidatedAddress,
        setup: TargetSetup,
        mode: M,
    ) -> Result<Self, SetupError> {
        let ClockConfig { power, source, div } = config.clock_config;

        // Enable clocks
        let conf = Lpi2cConfig {
            power,
            source,
            div,
            instance: T::CLOCK_INSTANCE,
        };

        let parts = unsafe { enable_and_reset::<T>(&conf).map_err(SetupError::ClockSetup)? };

        scl.mux();
        sda.mux();

        let _scl = scl.into();
        let _sda = sda.into();

        let inst = Self {
            info: T::info(),
            freq: setup.frequency,
            _scl,
            _sda,
            smbus_alert: config.smbus_alert.clone(),
            general_call: config.general_call.clone(),
            mode,
            _wg: parts.wake_guard,
        };

        inst.set_configuration(&config, address)?;

        Ok(inst)
    }

    fn set_configuration(&self, config: &Config, address: ValidatedAddress) -> Result<(), SetupError> {
        let datavd = (self.freq / 1_000_000).clamp(1, 63) as u8;
        self.registers()
            .configure(address, config.general_call.into(), config.smbus_alert.into(), datavd)
    }

    /// Resets both TX and RX FIFOs dropping their contents.
    #[inline(always)]
    fn registers(&self) -> TargetRegisters {
        self.info.target_registers()
    }

    fn reset_fifos(&self) {
        self.registers().reset_fifos();
    }

    /// Unconditionally clear the W1C event flags. Only for the entry to
    /// `listen`, where everything still latched is by definition stale
    /// (the previous transaction's lifecycle is complete). Anywhere a
    /// snapshot is being classified, clear through the snapshot instead
    /// — see `status`.
    fn clear_status(&self) {
        self.registers().clear_stale_events();
    }

    /// Take and classify one listen event through the register
    /// facade ([`TargetRegisters::take_listen_event`], which owns the
    /// same-snapshot W1C clear and the SASR consumption), mapping it
    /// to the driver's `Event`/`IOError` surface.
    fn status(&self) -> Result<Event, IOError> {
        match self.registers().take_listen_event() {
            ListenEvent::Fault(f) => Err(f.into()),
            ListenEvent::AddressValid(addr) => Ok(Event::AddressValid(addr)),
            ListenEvent::GeneralCall => Ok(Event::GeneralCall),
            ListenEvent::SmbusAlert => Ok(Event::SmbusAlert),
            ListenEvent::TransmitAck => Ok(Event::TransmitAck),
            ListenEvent::RepeatedStart(addr) => Ok(Event::RepeatedStart(addr)),
            ListenEvent::Stop(addr) => Ok(Event::Stop(addr)),
            ListenEvent::None => Err(IOError::Other),
        }
    }

    // Public API: Blocking

    /// Block waiting for new events.
    ///
    /// This function blocks the caller until a new I2C event is received. It returns the
    /// type of request made by the I2C controller.
    ///
    /// # Returns
    ///
    /// - `Ok(Request)` on success.
    /// - `Err(IOError)` if an error occurs.
    pub fn blocking_listen(&mut self) -> Result<Request, IOError> {
        self.clear_status();

        // Wait for an address match, a STOP, or a fault (see
        // `listen_ready`; the status read below classifies and clears).
        while !self.registers().listen_ready() {}

        let event = self.status()?;

        match event {
            Event::SmbusAlert => Ok(Request::SmbusAlert),
            Event::GeneralCall => Ok(Request::GeneralCall),
            Event::Stop(addr) => Ok(Request::Stop(addr >> 1)),
            Event::RepeatedStart(addr) | Event::AddressValid(addr) => {
                if addr & 1 != 0 {
                    Ok(Request::Read(addr >> 1))
                } else {
                    Ok(Request::Write(addr >> 1))
                }
            }
            _ => Err(IOError::Other),
        }
    }

    /// Transmit data to the I2C controller.
    ///
    /// Sends the contents of the provided buffer to the I2C controller. The
    /// call services the transfer to a clean termination point (STOP,
    /// repeated START, or buffer exhausted) before returning.
    ///
    /// # Parameters
    ///
    /// - `buf`: The buffer containing the data to transmit.
    ///
    /// # Returns
    ///
    /// - `Ok(ReadStatus)` describing how the transfer ended and how many
    ///   bytes were queued for transmission — see [`ReadStatus`] for why
    ///   that is not the same as bytes the controller ACKed.
    /// - `Err(IOError)` if an error occurs.
    pub fn blocking_respond_to_read(&mut self, buf: &[u8]) -> Result<ReadStatus, IOError> {
        let mut count = 0;

        // NOTE: deliberately no entry `clear_status()` here. The
        // `listen` that announced this transaction already cleared
        // the stale flags via `status()`, so anything latched now
        // belongs to *this* transfer — including a STOP or repeated
        // START that arrived between `listen` returning and this
        // call. Clearing it would erase the only signal that the
        // transfer is over and leave the wait below hanging.

        for byte in buf.iter() {
            // Wait until we can send data, honoring termination first
            // (the wrapper's tx_step encodes that order).
            loop {
                match self.registers().tx_step() {
                    Some(TargetTxStep::Fault(f)) => {
                        self.reset_fifos();
                        return Err(f.into());
                    }
                    Some(TargetTxStep::Ended) => {
                        #[cfg(feature = "defmt")]
                        defmt::trace!("Early stop of Target Send routine. STOP or Repeated-start received");
                        return Ok(ReadStatus::EarlyStop(count));
                    }
                    Some(TargetTxStep::Room) => {
                        self.registers().push_tx(*byte);
                        count += 1;
                        break;
                    }
                    None => {}
                }
            }
        }

        // All caller bytes pushed. Wait briefly to determine whether the
        // controller is done (NACK + STOP/RSTART) or whether it wants more.
        let ended = loop {
            match self.registers().tx_step() {
                Some(TargetTxStep::Fault(f)) => {
                    self.reset_fifos();
                    return Err(f.into());
                }
                Some(TargetTxStep::Ended) => break true,
                Some(TargetTxStep::Room) => break false,
                None => {}
            }
        };

        if ended {
            Ok(ReadStatus::Complete(count))
        } else {
            // Room: TX empty during a transmit transfer means the
            // controller is still clocking and wants another byte.
            Ok(ReadStatus::NeedMore(count))
        }
    }

    /// Receive data from the I2C controller.
    ///
    /// Reads bytes the controller writes into the provided buffer. The call
    /// services the transfer to a clean termination point (STOP, repeated
    /// START, or buffer filled) before returning.
    ///
    /// # Parameters
    ///
    /// - `buf`: The buffer to store the received data.
    ///
    /// # Returns
    ///
    /// - `Ok(WriteStatus)` describing how the transfer ended and how many
    ///   bytes the target received.
    /// - `Err(IOError)` if an error occurs.
    pub fn blocking_respond_to_write(&mut self, buf: &mut [u8]) -> Result<WriteStatus, IOError> {
        let mut count = 0;

        // NOTE: deliberately no entry `clear_status()` here. The
        // `listen` that announced this transaction already cleared
        // the stale flags via `status()`, so anything latched now
        // belongs to *this* transfer — including a STOP or repeated
        // START that arrived between `listen` returning and this
        // call. Clearing it would erase the only signal that the
        // transfer is over and leave the wait below hanging.

        for byte in buf.iter_mut() {
            // Wait for one receive event. The wrapper's rx_event drains
            // pending data before honoring any end-of-transfer flag, so
            // bytes that arrived before the STOP cannot be dropped.
            loop {
                match self.registers().rx_event() {
                    Some(TargetRxEvent::Byte(b)) => {
                        *byte = b;
                        count += 1;
                        break;
                    }
                    Some(TargetRxEvent::Fault(f)) => {
                        self.reset_fifos();
                        return Err(f.into());
                    }
                    Some(TargetRxEvent::Stopped) => {
                        #[cfg(feature = "defmt")]
                        defmt::trace!("Early stop of Target Receive routine. STOP received");
                        return Ok(WriteStatus::Stopped(count));
                    }
                    Some(TargetRxEvent::Restarted) => {
                        #[cfg(feature = "defmt")]
                        defmt::trace!("Early stop of Target Receive routine. Repeated-start received");
                        return Ok(WriteStatus::Restarted(count));
                    }
                    None => {}
                }
            }
        }

        Ok(WriteStatus::BufferFull(count))
    }
}

impl<'d> I2c<'d, Blocking> {
    /// Create a new blocking instance of the I2C Target bus driver.
    ///
    /// This function initializes the I2C target driver in blocking mode. It configures the
    /// I2C peripheral, sets up the clock, and prepares the pins for operation. Any external
    /// pin will be placed into the Disabled state upon `Drop`.
    ///
    /// # Parameters
    ///
    /// - `peri`: The I2C peripheral instance.
    /// - `scl`: The SCL pin.
    /// - `sda`: The SDA pin.
    /// - `config`: The configuration for the I2C target.
    ///
    /// # Returns
    ///
    /// - `Ok(Self)` on success.
    /// - `Err(SetupError)` if initialization fails.
    pub fn new_blocking<T: Instance>(
        peri: Peri<'d, T>,
        scl: Peri<'d, impl SclPin<T>>,
        sda: Peri<'d, impl SdaPin<T>>,
        config: Config,
    ) -> Result<Self, SetupError> {
        let address = config.address.validate()?;
        let setup = Self::construction_setup::<T>(&config)?;
        Self::new_inner(peri, scl, sda, config, address, setup, Blocking)
    }
}

impl<'d> I2c<'d, Async> {
    /// Create a new asynchronous instance of the I2C Target bus driver.
    ///
    /// This function initializes the I2C target driver in asynchronous mode. It configures the
    /// I2C peripheral, sets up the clock, and prepares the pins for operation. Any external
    /// pin will be placed into the Disabled state upon `Drop`.
    ///
    /// # Parameters
    ///
    /// - `peri`: The I2C peripheral instance.
    /// - `scl`: The SCL pin.
    /// - `sda`: The SDA pin.
    /// - `_irq`: The interrupt binding for the I2C peripheral.
    /// - `config`: The configuration for the I2C target.
    ///
    /// # Returns
    ///
    /// - `Ok(Self)` on success.
    /// - `Err(SetupError)` if initialization fails.
    pub fn new_async<T: Instance>(
        peri: Peri<'d, T>,
        scl: Peri<'d, impl SclPin<T>>,
        sda: Peri<'d, impl SdaPin<T>>,
        _irq: impl crate::interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Result<Self, SetupError> {
        let address = config.address.validate()?;
        let setup = Self::construction_setup::<T>(&config)?;
        let inst = Self::new_inner(peri, scl, sda, config, address, setup, Async)?;

        T::Interrupt::unpend();

        // Safety: `_irq` ensures an Interrupt Handler exists.
        unsafe { T::Interrupt::enable() };

        Ok(inst)
    }
}

/// Internal outcome of a single DMA TX chunk transfer (target -> controller).
#[derive(Copy, Clone)]
enum TxChunkOutcome {
    /// Controller issued STOP. `usize` is bytes transferred from this chunk.
    Stopped(usize),
    /// Controller issued repeated START. `usize` is bytes transferred from
    /// this chunk.
    Restarted(usize),
    /// DMA exhausted the chunk and the controller is still asking for more
    /// bytes. `usize` equals the chunk length.
    NeedMore(usize),
}

/// Internal outcome of a single DMA RX chunk transfer (controller -> target).
#[derive(Copy, Clone)]
enum RxChunkOutcome {
    /// Controller issued STOP. `usize` is bytes received into this chunk.
    Stopped(usize),
    /// Controller issued repeated START. `usize` is bytes received into
    /// this chunk.
    Restarted(usize),
    /// DMA filled the chunk before the controller terminated the transfer.
    /// `usize` equals the chunk length.
    Filled(usize),
}

/// The fixed RX channel/request pair for this target instance.
///
/// The constructor is private to this module, so the register facade never
/// accepts an interchangeable channel/request tuple at an arm site.
#[must_use]
struct TargetRxDma<'a, 'd> {
    owner: usize,
    registers: TargetRegisters,
    wait_cell: &'a WaitCell,
    channel: &'a DmaChannel<'d>,
    request: DmaRequest,
    _target: PhantomData<&'a mut I2c<'d, Dma<'d>>>,
}

impl<'a, 'd> TargetRxDma<'a, 'd> {
    fn new(
        registers: TargetRegisters,
        wait_cell: &'a WaitCell,
        channel: &'a DmaChannel<'d>,
        request: DmaRequest,
    ) -> Self {
        Self {
            owner: registers.identity(),
            registers,
            wait_cell,
            channel,
            request,
            _target: PhantomData,
        }
    }

    fn owner(&self) -> usize {
        self.owner
    }

    fn channel(&self) -> &'a DmaChannel<'d> {
        self.channel
    }

    fn request(&self) -> DmaRequest {
        self.request
    }

    fn registers(&self) -> TargetRegisters {
        self.registers
    }

    fn wait_cell(&self) -> &'a WaitCell {
        self.wait_cell
    }
}

/// The fixed TX channel/request pair for this target instance. See
/// [`TargetRxDma`] for why this is not a generic `(channel, request)` pair.
#[must_use]
struct TargetTxDma<'a, 'd> {
    owner: usize,
    registers: TargetRegisters,
    wait_cell: &'a WaitCell,
    channel: &'a DmaChannel<'d>,
    request: DmaRequest,
    _target: PhantomData<&'a mut I2c<'d, Dma<'d>>>,
}

impl<'a, 'd> TargetTxDma<'a, 'd> {
    fn new(
        registers: TargetRegisters,
        wait_cell: &'a WaitCell,
        channel: &'a DmaChannel<'d>,
        request: DmaRequest,
    ) -> Self {
        Self {
            owner: registers.identity(),
            registers,
            wait_cell,
            channel,
            request,
            _target: PhantomData,
        }
    }

    fn owner(&self) -> usize {
        self.owner
    }

    fn channel(&self) -> &'a DmaChannel<'d> {
        self.channel
    }

    fn request(&self) -> DmaRequest {
        self.request
    }

    fn registers(&self) -> TargetRegisters {
        self.registers
    }

    fn wait_cell(&self) -> &'a WaitCell {
        self.wait_cell
    }
}

impl<'d> I2c<'d, Dma<'d>> {
    /// Create a new asynchronous instance of the I2C Target bus driver with DMA support.
    ///
    /// This function initializes the I2C target driver in asynchronous mode with DMA support.
    /// It configures the I2C peripheral, sets up the clock, and prepares the pins for operation.
    /// Any external pin will be placed into the Disabled state upon `Drop`, and the DMA channels
    /// are also disabled.
    ///
    /// # Parameters
    ///
    /// - `peri`: The I2C peripheral instance.
    /// - `scl`: The SCL pin.
    /// - `sda`: The SDA pin.
    /// - `tx_dma`: The DMA channel for transmitting data.
    /// - `rx_dma`: The DMA channel for receiving data.
    /// - `_irq`: The interrupt binding for the I2C peripheral.
    /// - `config`: The configuration for the I2C target.
    ///
    /// # Returns
    ///
    /// - `Ok(Self)` on success.
    /// - `Err(SetupError)` if initialization fails.
    pub fn new_async_with_dma<T: Instance>(
        peri: Peri<'d, T>,
        scl: Peri<'d, impl SclPin<T>>,
        sda: Peri<'d, impl SdaPin<T>>,
        tx_dma: Peri<'d, impl Channel>,
        rx_dma: Peri<'d, impl Channel>,
        _irq: impl crate::interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Result<Self, SetupError> {
        let address = config.address.validate()?;
        let setup = Self::construction_setup::<T>(&config)?;

        // enable this channel's interrupt
        let tx_dma = DmaChannel::new(tx_dma);
        let rx_dma = DmaChannel::new(rx_dma);

        tx_dma.enable_interrupt();
        rx_dma.enable_interrupt();

        let inst = Self::new_inner(
            peri,
            scl,
            sda,
            config,
            address,
            setup,
            Dma {
                tx_dma,
                rx_dma,
                tx_request: T::TX_DMA_REQUEST,
                rx_request: T::RX_DMA_REQUEST,
            },
        )?;

        T::Interrupt::unpend();

        // Safety: `_irq` ensures an Interrupt Handler exists.
        unsafe { T::Interrupt::enable() };

        Ok(inst)
    }

    /// Brand this target's RX channel/request wiring for the register
    /// facade. The only construction site is here, so an arm call cannot
    /// swap RX and TX plumbing by accident.
    fn target_rx_dma<'a>(&'a mut self) -> TargetRxDma<'a, 'd> {
        let registers = self.registers();
        TargetRxDma::new(
            registers,
            self.info.wait_cell(),
            &self.mode.rx_dma,
            self.mode.rx_request,
        )
    }

    /// Brand this target's TX channel/request wiring. See
    /// [`Self::target_rx_dma`] for the pairing guarantee.
    fn target_tx_dma<'a>(&'a mut self) -> TargetTxDma<'a, 'd> {
        let registers = self.registers();
        TargetTxDma::new(
            registers,
            self.info.wait_cell(),
            &self.mode.tx_dma,
            self.mode.tx_request,
        )
    }

    // A chunk takes `&mut self`: its directional DMA lease retains an
    // exclusive target-operation borrow alongside the channel and caller
    // buffer until normal completion or Drop.
    async fn read_dma_chunk(&mut self, data: &mut [u8]) -> Result<RxChunkOutcome, IOError> {
        let chunk_len = data.len();
        let registers = self.registers();

        // NOTE: deliberately no entry `clear_status()` here. The
        // `listen` that announced this transaction already cleared
        // the stale flags via `status()`, so anything latched now
        // belongs to *this* transfer — including a STOP or repeated
        // START that arrived between `listen` returning and this
        // call. Clearing it would erase the only signal that the
        // transfer is over and leave the wait below hanging.

        let lease = registers.arm_rx_dma(self.target_rx_dma(), data)?;

        // Wait for any of:
        //  - I2C end-of-transfer flag (sdf, rsf) -> controller terminated
        //  - I2C error flag (fef, bef) -> bus problem
        //  - DMA channel completion -> chunk filled before controller stopped
        //
        // The DMA done interrupt wakes the DMA's wait_cell; I2C status
        // changes wake the I2C wait_cell. Register on both.
        poll_fn(|cx| {
            // Arm and check both the I2C and exact DMA wait sources through
            // the lease. No target operation is available while it is live.
            if lease.poll_wake(cx) {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;

        // Consuming the lease first shuts down RDDE and quiesces the exact
        // channel before `data` is touched again. It returns the count only
        // after DONE/CITER are stable; Drop performs the same cleanup if
        // this future is cancelled at any earlier await.
        let moved = lease.finish();

        // The termination wake races the final byte's DMA handoff: the
        // controller can deliver a byte (RDF set, DMA request asserted)
        // and STOP before the eDMA arbitrates the request, and shutting
        // the request path down above rescinds it — marooning the byte
        // in the FIFO while the count reads one short. `drain_rx`
        // collects what the FIFO still holds through the vocabulary,
        // whose order (data before termination) is exactly what the
        // interrupt path's `rx_event` already enforces and what the
        // reference driver does. Nothing new can arrive mid-drain: the
        // transaction has terminated, and ADRSTALL stretches the next
        // address phase until `listen` services it, so every drained
        // byte belongs to this transfer. (Residue the chunk has no
        // room for is handled by the residue check below.)
        let bytes = registers.drain_rx(&mut data[..chunk_len], moved);
        #[cfg(feature = "defmt")]
        if bytes > moved {
            defmt::debug!(
                "i2c target rx: drained {} residue byte(s) at termination",
                bytes - moved
            );
        }

        match registers.rx_chunk_end(bytes == chunk_len) {
            RxChunkEnd::Fault(f) => {
                // Parity with the interrupt paths' fault arms: the
                // error discards this transfer's accounting, so
                // whatever the FIFOs still hold must not survive into
                // the next transaction as its first bytes.
                registers.reset_fifos();
                Err(f.into())
            }
            RxChunkEnd::ResiduePending => {
                // Data is still pending with no room left in this
                // chunk — even if a termination flag is also latched.
                // Mirror the interrupt engine's data-before-
                // termination contract: report the chunk full so the
                // caller collects the residue (next chunk, or
                // `BufferFull` and a follow-up respond — SDF/RSF stay
                // latched for it, so the follow-up's entry wait
                // completes immediately, drains the residue, and only
                // then reports the termination). Affirming `Stopped`
                // here would maroon the residue in the FIFO to be
                // DMA'd into the NEXT transaction's first bytes.
                #[cfg(feature = "defmt")]
                defmt::debug!("i2c target rx: residue beyond chunk; deferring to follow-up");
                Ok(RxChunkOutcome::Filled(chunk_len))
            }
            RxChunkEnd::Stopped => Ok(RxChunkOutcome::Stopped(bytes)),
            RxChunkEnd::Restarted => Ok(RxChunkOutcome::Restarted(bytes)),
            // DMA done with no end-of-transfer flag: chunk filled,
            // controller may want to write more bytes.
            RxChunkEnd::Continue => Ok(RxChunkOutcome::Filled(chunk_len)),
        }
    }

    // Takes `&mut self` — see `read_dma_chunk`.
    async fn write_dma_chunk(&mut self, data: &[u8]) -> Result<TxChunkOutcome, IOError> {
        let chunk_len = data.len();
        let registers = self.registers();

        // NOTE: deliberately no entry `clear_status()` here. The
        // `listen` that announced this transaction already cleared
        // the stale flags via `status()`, so anything latched now
        // belongs to *this* transfer — including a STOP or repeated
        // START that arrived between `listen` returning and this
        // call. Clearing it would erase the only signal that the
        // transfer is over and leave the wait below hanging.

        let lease = registers.arm_tx_dma(self.target_tx_dma(), data)?;

        // Wait for any of:
        //  - I2C end-of-transfer flag (sdf, rsf) -> controller terminated
        //  - I2C error flag (fef, bef) -> bus problem
        //  - DMA channel completion -> chunk exhausted; if controller still
        //    clocking, caller may want to call again (NeedMore)
        poll_fn(|cx| {
            // See `read_dma_chunk`: the lease owns both wake sources while
            // it holds the target operation exclusively.
            if lease.poll_wake(cx) {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;

        // Consume the lease before inspecting status or ending the source
        // borrow. Its Drop path covers cancellation while DMA remains live.
        let bytes = lease.finish();

        match registers.chunk_end() {
            ChunkEnd::Fault(f) => {
                // Parity with the interrupt paths' fault arms — see
                // `read_dma_chunk`.
                registers.reset_fifos();
                Err(f.into())
            }
            ChunkEnd::Stopped => Ok(TxChunkOutcome::Stopped(bytes)),
            ChunkEnd::Restarted => Ok(TxChunkOutcome::Restarted(bytes)),
            ChunkEnd::Continue => {
                // DMA exhaustion proves only that the final byte
                // entered STDR — NOT that the controller wants more.
                // The interrupt path decides NeedMore-vs-done only
                // after the next observable event; settle the same
                // way through the vocabulary (`tx_ready` couples TDF
                // with faults and termination, so this cannot spin on
                // a dead transfer): TDF means the controller took the
                // byte and clocks on; a termination means the read
                // ended exactly at the chunk. Without this, an
                // exact-length read returns NeedMore here where the
                // interrupt engine returns Complete.
                self.info
                    .wait_cell()
                    .wait_for(|| registers.tx_wake())
                    .await
                    .map_err(|_| IOError::Other)?;

                match registers.chunk_end() {
                    ChunkEnd::Fault(f) => {
                        registers.reset_fifos();
                        Err(f.into())
                    }
                    ChunkEnd::Stopped => Ok(TxChunkOutcome::Stopped(bytes)),
                    ChunkEnd::Restarted => Ok(TxChunkOutcome::Restarted(bytes)),
                    // TDF: chunk exhausted and the controller still
                    // clocks — it really does expect more bytes.
                    ChunkEnd::Continue => Ok(TxChunkOutcome::NeedMore(chunk_len)),
                }
            }
        }
    }
}

#[allow(private_bounds)]
impl<'d, M: AsyncMode> I2c<'d, M>
where
    Self: AsyncEngine,
{
    // Public API: Async

    /// Asynchronously wait for new events.
    ///
    /// This function waits asynchronously for a new I2C event and returns the type of
    /// request made by the I2C controller.
    ///
    /// # Returns
    ///
    /// - `Ok(Request)` on success.
    /// - `Err(IOError)` if an error occurs.
    pub async fn async_listen(&mut self) -> Result<Request, IOError> {
        self.clear_status();
        let general_call = self.general_call.into();
        let smbus_alert = self.smbus_alert.into();

        self.info
            .wait_cell()
            .wait_for(|| self.registers().listen_wake(general_call, smbus_alert))
            .await
            .map_err(|_| IOError::Other)?;

        let event = self.status()?;

        match event {
            Event::SmbusAlert => Ok(Request::SmbusAlert),
            Event::GeneralCall => Ok(Request::GeneralCall),
            Event::Stop(addr) => Ok(Request::Stop(addr >> 1)),
            Event::RepeatedStart(addr) | Event::AddressValid(addr) => {
                if addr & 1 != 0 {
                    Ok(Request::Read(addr >> 1))
                } else {
                    Ok(Request::Write(addr >> 1))
                }
            }
            _ => Err(IOError::Other),
        }
    }

    /// Explicitly abandon a target response that still owns the bus.
    ///
    /// A response result that reports a full caller buffer —
    /// [`ReadStatus::NeedMore`] or [`WriteStatus::BufferFull`] — is a
    /// continuation boundary, not an idle target. The caller must either
    /// provide another buffer with the corresponding response method or call
    /// this method before returning to unrelated work. Merely dropping the
    /// status leaves `TXDSTALL` or `RXSTALL` allowed to hold SCL while the
    /// controller waits for service.
    ///
    /// This masks target requests, releases a possible clock stretch, discards
    /// the unfinished transfer's FIFO residue, and re-arms the configured
    /// target for a later [`Self::async_listen`]. A live response future holds
    /// `&mut self`, so Rust prevents this operation from racing its DMA lease
    /// or register state.
    pub fn abort_response(&mut self) {
        self.registers().abort_active_response();
    }

    /// Asynchronously transmit data to the I2C controller.
    ///
    /// Sends the contents of the provided buffer to the I2C controller.
    /// The future services the transfer to a clean termination point
    /// (STOP, repeated START, or buffer exhausted) before resolving.
    ///
    /// If the controller continues clocking after the buffer has been
    /// fully transmitted (for example, an I2C-HID host that reads a fixed
    /// block size larger than the prepared response), this call resolves
    /// with [`ReadStatus::NeedMore`] so the caller can decide what to do:
    /// call `async_respond_to_read` again with more bytes (or fill data),
    /// or call [`Self::abort_response`] to deliberately abandon the open
    /// transfer. Letting the bus clock-stretch is valid only while the
    /// application intentionally waits for the controller to terminate it.
    ///
    /// # Parameters
    ///
    /// - `buf`: The buffer containing the data to transmit.
    ///
    /// # Returns
    ///
    /// - `Ok(ReadStatus)` describing how the transfer ended and how many
    ///   bytes were queued for transmission — see [`ReadStatus`] for why
    ///   that is not the same as bytes the controller ACKed.
    /// - `Err(IOError)` if an error occurs.
    pub fn async_respond_to_read<'a>(
        &'a mut self,
        buf: &'a [u8],
    ) -> impl Future<Output = Result<ReadStatus, IOError>> + 'a {
        let registers = self.registers();
        async move {
            // Declare the response future AFTER this guard. Local values drop
            // in reverse declaration order, so cancellation first quiesces
            // any DMA lease that still borrows `buf`, then drops SEN to
            // release TXDSTALL/RXSTALL and restore a listening target.
            let abort = OnDrop::new(move || registers.abort_active_response());
            let response = <Self as AsyncEngine>::async_respond_to_read_internal(self, buf);
            let result = response.await;
            abort.defuse();
            result
        }
    }

    /// Asynchronously receive data from the I2C controller.
    ///
    /// Reads bytes the controller writes into the provided buffer. The
    /// future services the transfer to a clean termination point (STOP,
    /// repeated START, or buffer filled) before resolving.
    ///
    /// # Parameters
    ///
    /// - `buf`: The buffer to store the received data.
    ///
    /// # Returns
    ///
    /// - `Ok(WriteStatus)` describing how the transfer ended and how many
    ///   bytes the target received.
    /// - `Err(IOError)` if an error occurs.
    ///
    /// If this returns [`WriteStatus::BufferFull`] and no follow-up receive
    /// buffer will be supplied, call [`Self::abort_response`] to release the
    /// target's receive-side clock stretch.
    pub fn async_respond_to_write<'a>(
        &'a mut self,
        buf: &'a mut [u8],
    ) -> impl Future<Output = Result<WriteStatus, IOError>> + 'a {
        let registers = self.registers();
        async move {
            // See `async_respond_to_read`: the response future is declared
            // after this guard so a live DMA lease is quiesced before target
            // hardware is reset on cancellation.
            let abort = OnDrop::new(move || registers.abort_active_response());
            let response = <Self as AsyncEngine>::async_respond_to_write_internal(self, buf);
            let result = response.await;
            abort.defuse();
            result
        }
    }
}

trait AsyncEngine {
    fn async_respond_to_read_internal<'a>(
        &'a mut self,
        buf: &'a [u8],
    ) -> impl Future<Output = Result<ReadStatus, IOError>> + 'a;

    fn async_respond_to_write_internal<'a>(
        &'a mut self,
        buf: &'a mut [u8],
    ) -> impl Future<Output = Result<WriteStatus, IOError>> + 'a;
}

impl<'d> AsyncEngine for I2c<'d, Async> {
    async fn async_respond_to_read_internal(&mut self, buf: &[u8]) -> Result<ReadStatus, IOError> {
        let mut count = 0;

        // NOTE: deliberately no entry `clear_status()` here. The
        // `listen` that announced this transaction already cleared
        // the stale flags via `status()`, so anything latched now
        // belongs to *this* transfer — including a STOP or repeated
        // START that arrived between `listen` returning and this
        // call. Clearing it would erase the only signal that the
        // transfer is over and leave the wait below hanging.

        for byte in buf.iter() {
            // Wait until we can send data, honoring termination first
            // (the wrapper's tx_step encodes that order).
            loop {
                self.info
                    .wait_cell()
                    .wait_for(|| self.registers().tx_wake())
                    .await
                    .map_err(|_| IOError::Other)?;

                match self.registers().tx_step() {
                    Some(TargetTxStep::Fault(f)) => {
                        self.reset_fifos();
                        return Err(f.into());
                    }
                    Some(TargetTxStep::Ended) => {
                        #[cfg(feature = "defmt")]
                        defmt::trace!("Early stop of Target Send routine. STOP or Repeated-start received");
                        self.reset_fifos();
                        return Ok(ReadStatus::EarlyStop(count));
                    }
                    Some(TargetTxStep::Room) => {
                        self.registers().push_tx(*byte);
                        count += 1;
                        break;
                    }
                    None => {}
                }
            }
        }

        // All caller bytes pushed. Wait briefly to determine whether the
        // controller is done (NACK + STOP/RSTART) or whether it wants more.
        // We do NOT auto-pad here: doing so blocks the firmware for the
        // duration of the controller's extra reads, which causes us to fall
        // behind on subsequent back-to-back transactions. The caller
        // receives ReadStatus::NeedMore and decides how to proceed.
        let ended = loop {
            self.info
                .wait_cell()
                .wait_for(|| self.registers().tx_wake())
                .await
                .map_err(|_| IOError::Other)?;

            match self.registers().tx_step() {
                Some(TargetTxStep::Fault(f)) => {
                    self.reset_fifos();
                    return Err(f.into());
                }
                Some(TargetTxStep::Ended) => break true,
                Some(TargetTxStep::Room) => break false,
                None => {}
            }
        };

        if ended {
            self.reset_fifos();
            Ok(ReadStatus::Complete(count))
        } else {
            // Room: TX empty during a transmit transfer means the
            // controller is still clocking and wants another byte.
            Ok(ReadStatus::NeedMore(count))
        }
    }

    async fn async_respond_to_write_internal(&mut self, buf: &mut [u8]) -> Result<WriteStatus, IOError> {
        let mut count = 0;

        // NOTE: deliberately no entry `clear_status()` here. The
        // `listen` that announced this transaction already cleared
        // the stale flags via `status()`, so anything latched now
        // belongs to *this* transfer — including a STOP or repeated
        // START that arrived between `listen` returning and this
        // call. Clearing it would erase the only signal that the
        // transfer is over and leave the wait below hanging.

        for byte in buf.iter_mut() {
            // Wait for one receive event. The wrapper's rx_event drains
            // pending data before honoring any end-of-transfer flag:
            // when firmware enters late (delayed ISR entry on a busy
            // system), the controller may already have issued the STOP
            // while received bytes still sit in the FIFO, and honoring
            // SDF first would silently drop them.
            loop {
                self.info
                    .wait_cell()
                    .wait_for(|| self.registers().rx_wake())
                    .await
                    .map_err(|_| IOError::Other)?;

                match self.registers().rx_event() {
                    Some(TargetRxEvent::Byte(b)) => {
                        *byte = b;
                        count += 1;
                        break;
                    }
                    Some(TargetRxEvent::Fault(f)) => {
                        self.reset_fifos();
                        return Err(f.into());
                    }
                    Some(TargetRxEvent::Stopped) => {
                        #[cfg(feature = "defmt")]
                        defmt::trace!("Early stop of Target Receive routine. STOP received");
                        self.reset_fifos();
                        return Ok(WriteStatus::Stopped(count));
                    }
                    Some(TargetRxEvent::Restarted) => {
                        #[cfg(feature = "defmt")]
                        defmt::trace!("Early stop of Target Receive routine. Repeated-start received");
                        self.reset_fifos();
                        return Ok(WriteStatus::Restarted(count));
                    }
                    None => {}
                }
            }
        }

        Ok(WriteStatus::BufferFull(count))
    }
}

impl<'d> AsyncEngine for I2c<'d, Dma<'d>> {
    async fn async_respond_to_read_internal(&mut self, buf: &[u8]) -> Result<ReadStatus, IOError> {
        let mut count = 0;

        // NOTE: deliberately no entry `clear_status()` here. The
        // `listen` that announced this transaction already cleared
        // the stale flags via `status()`, so anything latched now
        // belongs to *this* transfer — including a STOP or repeated
        // START that arrived between `listen` returning and this
        // call. Clearing it would erase the only signal that the
        // transfer is over and leave the wait below hanging.

        let total = buf.len();
        let mut chunks = buf.chunks(DMA_MAX_TRANSFER_SIZE).peekable();
        while let Some(chunk) = chunks.next() {
            let is_last = chunks.peek().is_none();
            match self.write_dma_chunk(chunk).await? {
                TxChunkOutcome::Stopped(n) => {
                    count += n;
                    return Ok(if is_last && count == total {
                        ReadStatus::Complete(count)
                    } else {
                        ReadStatus::EarlyStop(count)
                    });
                }
                TxChunkOutcome::Restarted(n) => {
                    count += n;
                    return Ok(if is_last && count == total {
                        ReadStatus::Complete(count)
                    } else {
                        ReadStatus::EarlyStop(count)
                    });
                }
                TxChunkOutcome::NeedMore(n) => {
                    count += n;
                    if is_last {
                        return Ok(ReadStatus::NeedMore(count));
                    }
                    // Non-last chunk completed normally: proceed to next
                    // chunk. The bus will clock-stretch briefly between
                    // chunks while we reprogram the TCD.
                }
            }
        }

        // Reached only when buf was empty.
        Ok(ReadStatus::NeedMore(count))
    }

    async fn async_respond_to_write_internal<'a>(&'a mut self, buf: &'a mut [u8]) -> Result<WriteStatus, IOError> {
        let mut count = 0;

        // NOTE: deliberately no entry `clear_status()` here. The
        // `listen` that announced this transaction already cleared
        // the stale flags via `status()`, so anything latched now
        // belongs to *this* transfer — including a STOP or repeated
        // START that arrived between `listen` returning and this
        // call. Clearing it would erase the only signal that the
        // transfer is over and leave the wait below hanging.

        let mut chunks = buf.chunks_mut(DMA_MAX_TRANSFER_SIZE).peekable();
        while let Some(chunk) = chunks.next() {
            let is_last = chunks.peek().is_none();
            match self.read_dma_chunk(chunk).await? {
                RxChunkOutcome::Stopped(n) => {
                    count += n;
                    return Ok(WriteStatus::Stopped(count));
                }
                RxChunkOutcome::Restarted(n) => {
                    count += n;
                    return Ok(WriteStatus::Restarted(count));
                }
                RxChunkOutcome::Filled(n) => {
                    count += n;
                    if is_last {
                        return Ok(WriteStatus::BufferFull(count));
                    }
                    // Non-last chunk filled: proceed to next chunk.
                }
            }
        }

        // Reached only when buf was empty.
        Ok(WriteStatus::BufferFull(count))
    }
}

impl<'d, M: Mode> Drop for I2c<'d, M> {
    fn drop(&mut self) {
        self._scl.set_as_disabled();
        self._sda.set_as_disabled();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn target_setup_rejects_a_zero_functional_clock() {
        assert!(matches!(TargetSetup::new(0), Err(SetupError::NoFunctionalClock)));
    }

    fn is_valid(address: Address) -> bool {
        address.validate().is_ok()
    }

    #[test]
    fn target_address_validation_accepts_only_representable_same_width_plans() {
        assert!(is_valid(Address::Single(0x00)));
        assert!(is_valid(Address::Single(0x7f)));
        assert!(is_valid(Address::Single(0x80)));
        assert!(is_valid(Address::Single(0x3ff)));
        assert!(!is_valid(Address::Single(0x400)));

        assert!(is_valid(Address::Dual(0x20, 0x30)));
        assert!(is_valid(Address::Dual(0x80, 0x3ff)));
        assert!(!is_valid(Address::Dual(0x7f, 0x80)));
        assert!(!is_valid(Address::Dual(0x400, 0x401)));

        assert!(is_valid(Address::Range(0x20..0x30)));
        assert!(is_valid(Address::Range(0x7f..0x80)));
        assert!(is_valid(Address::Range(0x00..0x80)));
        assert!(is_valid(Address::Range(0x80..0x400)));
        assert!(!is_valid(Address::Range(0x00..0x00)));
        assert!(!is_valid(Address::Range(0x31..0x30)));
        assert!(!is_valid(Address::Range(0x7f..0x81)));
        assert!(!is_valid(Address::Range(0x3ff..0x401)));
    }
}
