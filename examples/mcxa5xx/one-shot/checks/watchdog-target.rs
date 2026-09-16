//! Hold an accepted write for 150 ms without CPU/DMA receive service.
//! Fault cases abort ONLY after the hold. The 200 ms control instead receives
//! the exact payload and STOP, demonstrating genuine resumption, not an abort.
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_mcxa::clocks::config::Div8;
use embassy_mcxa::i2c::target::{self, Address, InterruptHandler, Request, WriteStatus};
use embassy_mcxa::{bind_interrupts, dma, pac, peripherals};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use {defmt_rtt as _, panic_probe as _};

mod scl_observer;

bind_interrupts!(struct Irqs {
    LPI2C3 => InterruptHandler<peripherals::LPI2C3>;
    DMA0_CH0 => dma::InterruptHandler<peripherals::DMA0_CH0>;
    DMA0_CH1 => dma::InterruptHandler<peripherals::DMA0_CH1>;
});

fn require(condition: bool, why: &'static str) -> Result<(), &'static str> {
    if condition { Ok(()) } else { Err(why) }
}

fn unserviced_rx() -> Result<(), &'static str> {
    let status = pac::LPI2C3.ssr().read();
    require(status.rdf() && !status.avf(), "RX absent or address unconsumed")?;
    require(
        !status.sdf() && !status.rsf() && !status.fef() && !status.bef(),
        "RX fault or termination",
    )?;
    require(pac::LPI2C3.scfgr1().read().rxstall(), "RXSTALL disabled")?;
    require(!pac::LPI2C3.sder().read().rdde(), "RDDE unexpectedly enabled")?;
    let csr = pac::EDMA_0_TCD.tcd(1).ch_csr().read();
    require(!csr.erq() && !csr.active(), "RX DMA enabled or active")?;
    require(!scl_observer::high(), "SCL not low at hold sample")
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let requested_ms: u64 = match option_env!("I2C_ONE_SHOT_WATCHDOG") {
        Some("120") => 120,
        Some("140") => 140,
        Some("200") => 200,
        _ => panic!("Build with I2C_ONE_SHOT_WATCHDOG=120, 140 or 200"),
    };
    let repair = match option_env!("I2C_ONE_SHOT_RECOVERY") {
        None | Some("none") => None,
        Some("wait") => Some("wait"),
        Some("reset") => Some("reset"),
        Some("abrupt") => Some("abrupt"),
        Some("abrupt_idle") => Some("abrupt_idle"),
        _ => panic!("Unknown I2C_ONE_SHOT_RECOVERY"),
    };
    assert!(repair.is_none() || requested_ms != 200, "repair requires a fault case");
    let mut board = embassy_mcxa::config::Config::default();
    board.clock_cfg.sirc.fro_lf_div = Div8::from_divisor(1);
    let p = embassy_mcxa::init(board);
    let mut config = target::Config::default();
    config.address = Address::Single(0x2a);
    let mut i2c =
        target::I2c::new_async_with_dma(p.LPI2C3, p.P3_21, p.P3_20, p.DMA0_CH0, p.DMA0_CH1, Irqs, config).unwrap();
    let outcome: Result<(), &'static str> = async {
        scl_observer::log_configuration();
        scl_observer::check_read_path()?;
        require(scl_observer::high(), "SCL not idle-high")?;
        // Preserve complete observer pad/output state; this probe never writes
        // any of it. No second GPIO owner, reset or clock change is performed.
        let pad = pac::PORT3.pcr(21).read().0;
        let direction = pac::GPIO3.pddr().read().0;
        let output = pac::GPIO3.pdor().read().0;
        let input_disable = pac::GPIO3.pidr().read().0;
        defmt::info!("WATCHDOG TARGET: SCL sampled idle-high");
        defmt::info!("WATCHDOG TARGET READY: {}", requested_ms);
        if let Some(mode) = repair {
            defmt::info!("WATCHDOG REPAIR: mode={}", mode);
        }
        let request = with_timeout(Duration::from_secs(25), i2c.async_listen())
            .await
            .map_err(|_| "listen timeout")?
            .map_err(|_| "listen fault")?;
        require(matches!(request, Request::Write(0x2a)), "wrong request")?;
        // Address has been consumed, so ADRSTALL cannot be our hold witness.
        with_timeout(Duration::from_millis(5), async {
            while !pac::LPI2C3.ssr().read().rdf() || scl_observer::high() {
                Timer::after_micros(20).await;
            }
        })
        .await
        .map_err(|_| "no RX/SCL hold within 5 ms")?;
        let started = Instant::now();
        unserviced_rx()?;
        let mut samples = 1_u32;
        while started.elapsed() < Duration::from_millis(150) {
            Timer::after_millis(1).await;
            unserviced_rx()?;
            samples += 1;
        }
        let hold_us = started.elapsed().as_micros();
        require(hold_us <= 175_000, "target hold exceeded characterization window")?;
        defmt::info!(
            "WATCHDOG TARGET: unserviced-RDF samples={}, elapsed_us={}, RDDE/ERQ/ACTIVE off, SCL sampled=true",
            samples,
            hold_us,
        );
        if requested_ms == 200 {
            let mut received = [0_u8; 17];
            let status = with_timeout(Duration::from_millis(50), i2c.async_respond_to_write(&mut received))
                .await
                .map_err(|_| "resume timeout")?
                .map_err(|_| "resume fault")?;
            require(matches!(status, WriteStatus::Stopped(16)), "missing exact length/STOP")?;
            require(&received[..16] == b"Hello from DMA!\n", "payload mismatch")?;
        } else {
            // A separate TARGET release, not evidence of controller bus-clear.
            i2c.abort_response();
        }
        let csr = pac::EDMA_0_TCD.tcd(1).ch_csr().read();
        require(!csr.erq() && !csr.active(), "RX DMA active after release")?;
        require(!pac::LPI2C3.sder().read().rdde(), "RDDE after release")?;
        require(!pac::LPI2C3.ssr().read().rdf(), "RX residue after release")?;
        require(pac::LPI2C3.scr().read().sen(), "target not enabled after release")?;
        Timer::after_micros(100).await;
        scl_observer::check_read_path()?;
        require(scl_observer::high(), "SCL not high after release")?;
        require(pac::PORT3.pcr(21).read().0 == pad, "pad changed")?;
        require(pac::GPIO3.pddr().read().0 == direction, "GPIO direction changed")?;
        require(pac::GPIO3.pdor().read().0 == output, "GPIO output changed")?;
        require(pac::GPIO3.pidr().read().0 == input_disable, "GPIO input gate changed")?;
        defmt::info!("WATCHDOG TARGET: SCL sampled high after release");
        if repair.is_some() {
            // No additional target reset/repair: retain the same release path
            // as the baseline. Require a new address, exact new bytes, STOP.
            let request = with_timeout(Duration::from_millis(750), i2c.async_listen())
                .await
                .map_err(|_| "fresh listen timeout")?
                .map_err(|_| "fresh listen fault")?;
            require(matches!(request, Request::Write(0x2a)), "wrong fresh request")?;
            let mut received = [0_u8; 17];
            let status = with_timeout(Duration::from_millis(100), i2c.async_respond_to_write(&mut received))
                .await
                .map_err(|_| "fresh receive timeout")?
                .map_err(|_| "fresh receive fault")?;
            require(matches!(status, WriteStatus::Stopped(16)), "fresh length/STOP mismatch")?;
            require(&received[..16] == b"Recovered DMA!!\n", "fresh payload mismatch")?;
            require(scl_observer::high(), "fresh transfer left SCL low")?;
            defmt::info!("WATCHDOG REPAIR: fresh exact 16 bytes and STOP");
        }
        Ok(())
    }
    .await;
    if let Err(why) = outcome {
        i2c.abort_response();
        panic!("watchdog target probe failed: {}", why);
    }
    if requested_ms == 200 {
        defmt::info!("WATCHDOG 200 TARGET PASS: exact 16 bytes and STOP");
    } else {
        defmt::info!(
            "WATCHDOG {} TARGET PASS: held RX and explicit target abort",
            requested_ms
        );
    }
    cortex_m_semihosting::debug::exit(cortex_m_semihosting::debug::EXIT_SUCCESS);
    core::mem::forget(i2c);
    loop {
        cortex_m::asm::wfi();
    }
}
