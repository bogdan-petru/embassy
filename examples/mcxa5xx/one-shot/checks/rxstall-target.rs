//! Withhold RX service after address acceptance, before any RX DMA is armed.
//! Timer/interrupts keep running: this is not an artificial CPU blackout.
//! Status samples prove unserviced RX, not a continuous electrical SCL trace.
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_mcxa::clocks::config::Div8;
use embassy_mcxa::i2c::target::{self, Address, InterruptHandler, Request, WriteStatus};
use embassy_mcxa::{bind_interrupts, dma, pac, peripherals};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use {defmt_rtt as _, panic_probe as _};

mod scl_observer;
use scl_observer::{check_read_path as check_scl_sampling, high as scl_high};

bind_interrupts!(struct Irqs {
    LPI2C3 => InterruptHandler<peripherals::LPI2C3>;
    DMA0_CH0 => dma::InterruptHandler<peripherals::DMA0_CH0>;
    DMA0_CH1 => dma::InterruptHandler<peripherals::DMA0_CH1>;
});

fn require(condition: bool, why: &'static str) -> Result<(), &'static str> {
    if condition { Ok(()) } else { Err(why) }
}

fn check_unserviced_rx(sample_scl: bool) -> Result<(), &'static str> {
    // These are non-destructive PAC reads in diagnostic firmware. Never read
    // SRDR here: popping the byte would service the very condition being tested.
    let status = pac::LPI2C3.ssr().read();
    require(status.rdf() && !status.avf(), "RX data absent or address unconsumed")?;
    require(
        !status.sdf() && !status.rsf() && !status.bef() && !status.fef(),
        "RX fault or termination during hold",
    )?;
    require(pac::LPI2C3.scfgr1().read().rxstall(), "RXSTALL disabled")?;
    require(!pac::LPI2C3.sder().read().rdde(), "RX DMA request unexpectedly enabled")?;
    // The constructor takes TX channel 0 followed by RX channel 1.
    let rx = pac::EDMA_0_TCD.tcd(1).ch_csr().read();
    require(!rx.erq() && !rx.active(), "RX DMA channel unexpectedly enabled/active")?;
    if sample_scl {
        require(!scl_high(), "SCL not low at RX hold sample")?;
    }
    Ok(())
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let long = match option_env!("I2C_ONE_SHOT_RXSTALL") {
        Some("short") => false,
        Some("long") => true,
        _ => panic!("Build with I2C_ONE_SHOT_RXSTALL=short or long"),
    };
    let mut config = embassy_mcxa::config::Config::default();
    config.clock_cfg.sirc.fro_lf_div = Div8::from_divisor(1);
    let p = embassy_mcxa::init(config);
    let mut config = target::Config::default();
    config.address = Address::Single(0x2a);
    let mut i2c =
        target::I2c::new_async_with_dma(p.LPI2C3, p.P3_21, p.P3_20, p.DMA0_CH0, p.DMA0_CH1, Irqs, config).unwrap();
    let outcome: Result<(), &'static str> = async {
        scl_observer::log_configuration();
        let sample_scl = match check_scl_sampling() {
            Ok(()) => {
                require(scl_high(), "SCL not idle-high before probe")?;
                defmt::info!("RXSTALL TARGET: configured SCL input sampled idle-high");
                true
            }
            Err(why) => {
                // Neither a clock write nor pad reconfiguration is allowed to
                // make this read-only observer pass. Automatic gating is now
                // recognized; remaining failures stay explicitly unavailable.
                defmt::warn!(
                    "RXSTALL TARGET: SCL observer unavailable: {}; no electrical verdict",
                    why
                );
                false
            }
        };
        if long {
            defmt::info!("RXSTALL TARGET READY: long");
        } else {
            defmt::info!("RXSTALL TARGET READY: short");
        }
        let request = with_timeout(Duration::from_secs(12), i2c.async_listen())
            .await
            .map_err(|_| "listen timeout")?
            .map_err(|_| "listen fault")?;
        require(matches!(request, Request::Write(0x2a)), "wrong address/direction")?;
        // listen consumed SASR: ADRSTALL is released. Wait for an actual data byte
        // and, when available, a low SCL sample without assuming RDF is
        // synchronized with that edge. RDF alone is not electrical proof.
        with_timeout(Duration::from_millis(5), async {
            while !pac::LPI2C3.ssr().read().rdf() || (sample_scl && scl_high()) {
                Timer::after_micros(20).await;
            }
        })
        .await
        .map_err(|_| "RX hold witness not acquired within 5 ms of address acceptance")?;
        let started = Instant::now();
        check_unserviced_rx(sample_scl)?;
        let hold_ms = if long { 150 } else { 5 };
        defmt::info!("RXSTALL TARGET: withholding RX service for at least {} ms", hold_ms);
        let mut samples = 1_u32;
        while started.elapsed() < Duration::from_millis(hold_ms) {
            Timer::after_millis(1).await;
            check_unserviced_rx(sample_scl)?;
            samples += 1;
        }
        defmt::info!(
            "RXSTALL TARGET: unserviced-RDF samples={}, elapsed_us={}, RDDE/ERQ/ACTIVE off, SCL sampled={}",
            samples,
            started.elapsed().as_micros(),
            sample_scl
        );
        if long {
            // The controller may have disabled MEN without STOP. Do not await a
            // response that can never finish; this separate TARGET action releases
            // its own stretch and discards residue. It is not controller bus-clear.
            i2c.abort_response();
            require(!pac::LPI2C3.ssr().read().rdf(), "RX residue after target abort")?;
            require(!pac::LPI2C3.sder().read().rdde(), "RDDE after target abort")?;
            require(pac::LPI2C3.scr().read().sen(), "target not re-enabled after abort")?;
            let rx = pac::EDMA_0_TCD.tcd(1).ch_csr().read();
            require(!rx.erq() && !rx.active(), "RX DMA active after target abort")?;
        } else {
            let mut message = [0_u8; 17];
            let received = with_timeout(Duration::from_millis(50), i2c.async_respond_to_write(&mut message))
                .await
                .map_err(|_| "short-control receive timeout")?
                .map_err(|_| "short-control receive fault")?;
            require(
                matches!(received, WriteStatus::Stopped(16)),
                "wrong receive length or missing STOP",
            )?;
            require(&message[..16] == b"Hello from DMA!\n", "payload mismatch")?;
        }
        if sample_scl {
            Timer::after_micros(100).await;
            check_scl_sampling()?;
            require(scl_high(), "SCL not sampled high after release")?;
            defmt::info!("RXSTALL TARGET: SCL sampled high after release; not a continuous waveform");
        }
        Ok(())
    }
    .await;
    if let Err(why) = outcome {
        // All bounded-wait/observation failures release the target before the
        // fatal verdict. The host runner also restores both normal firmwares.
        i2c.abort_response();
        panic!("RXSTALL target probe failed: {}", why);
    }
    if long {
        defmt::info!("RXSTALL LONG TARGET PASS: withheld RX service and explicit target abort");
    } else {
        defmt::info!("RXSTALL SHORT TARGET PASS: exact 16 bytes and STOP");
    }
    cortex_m_semihosting::debug::exit(cortex_m_semihosting::debug::EXIT_SUCCESS);
}
