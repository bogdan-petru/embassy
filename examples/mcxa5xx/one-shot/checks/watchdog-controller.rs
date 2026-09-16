//! Separate full-driver characterization, NOT the one-shot's 100 ms test.
//! SCL-only hardware thresholds qualify the peripheral's input observations,
//! not arbitrary analog glitches, exact edge timestamps, or oscillator margins.
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_mcxa::clocks::config::Div8;
use embassy_mcxa::i2c::controller::{Config, I2c, IOError, InterruptHandler, PinLowTimeout};
use embassy_mcxa::interrupt::typelevel::{Interrupt, LPI2C3};
use embassy_mcxa::{bind_interrupts, dma, pac, peripherals};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use {defmt_rtt as _, panic_probe as _};

mod scl_observer;
mod watchdog_repair;

bind_interrupts!(struct Irqs {
    LPI2C3 => InterruptHandler<peripherals::LPI2C3>;
    DMA0_CH0 => dma::InterruptHandler<peripherals::DMA0_CH0>;
    DMA0_CH1 => dma::InterruptHandler<peripherals::DMA0_CH1>;
});

fn set_master_enabled(enabled: bool) -> u64 {
    let started = Instant::now();
    pac::LPI2C3.mcr().modify(|w| w.set_men(enabled));
    // Bench readback initially retained the previous MEN value immediately
    // after disabling. Await the actual state, not a guessed CPU-cycle delay.
    // No configuration write is allowed if this bounded check fails.
    while pac::LPI2C3.mcr().read().men() != enabled {
        assert!(
            started.elapsed() < Duration::from_millis(1),
            "MEN readback did not settle"
        );
    }
    started.elapsed().as_micros()
}

fn record_recovery(stage: &'static str, expect_idle: bool) {
    let mcr = pac::LPI2C3.mcr().read();
    let msr = pac::LPI2C3.msr().read();
    let fifo = pac::LPI2C3.mfsr().read();
    defmt::info!(
        "WATCHDOG RECOVERY: stage={}, mcr={}, msr={}, mfsr={}, scl_high={}",
        stage,
        mcr.0,
        msr.0,
        fifo.0,
        scl_observer::high(),
    );
    assert!(mcr.men());
    assert_eq!(fifo.txcount(), 0);
    assert_eq!(fifo.rxcount(), 0);
    if expect_idle {
        assert_eq!(msr.mbf(), pac::lpi2c::Mbf::Idle);
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let requested_ms: u64 = match option_env!("I2C_ONE_SHOT_WATCHDOG") {
        Some("120") => 120,
        Some("140") => 140,
        Some("200") => 200,
        _ => panic!("Build with I2C_ONE_SHOT_WATCHDOG=120, 140 or 200"),
    };
    let repair = watchdog_repair::selected();
    assert!(repair.is_none() || requested_ms != 200, "repair requires a fault case");
    let mut board = embassy_mcxa::config::Config::default();
    board.clock_cfg.sirc.fro_lf_div = Div8::from_divisor(4);
    let p = embassy_mcxa::init(board);
    defmt::info!("WATCHDOG CONTROLLER: 20s quiet window");
    Timer::after_secs(20).await;

    let mut config = Config::default();
    config.transfer_timeout = Duration::from_millis(500);
    config.pin_low_timeout = PinLowTimeout::Enabled(Duration::from_millis(requested_ms));
    let mut i2c = I2c::new_async_with_dma(p.LPI2C3, p.P3_21, p.P3_20, p.DMA0_CH0, p.DMA0_CH1, Irqs, config).unwrap();

    scl_observer::log_configuration();
    scl_observer::check_read_path().unwrap();
    assert!(scl_observer::high());
    // The public driver intentionally watches BOTH lines. This diagnostic-only
    // setup selects SCL without adding a production API or reconfiguring a live
    // session. The constructor just reset/configured the engine; no transfer,
    // DMA lease or future exists. Disable the bound IRQ and MEN before the
    // MCFGR1 write (RM requires disabled master configuration), then read back.
    // This is an explicitly reviewed PAC escape in test firmware, not evidence
    // that safe driver clients can or should bypass the register facade.
    let (disable_us, enable_us) = critical_section::with(|_| {
        LPI2C3::disable();
        assert_eq!(pac::LPI2C3.mier().read().0, 0);
        assert_eq!(pac::LPI2C3.mder().read().0, 0);
        assert_eq!(pac::LPI2C3.msr().read().mbf(), pac::lpi2c::Mbf::Idle);
        let disable_us = set_master_enabled(false);
        assert!(!pac::LPI2C3.mcr().read().men());
        pac::LPI2C3
            .mcfgr1()
            .modify(|w| w.set_timecfg(pac::lpi2c::Timecfg::IfSclLow));
        let enable_us = set_master_enabled(true);
        LPI2C3::unpend();
        // SAFETY: Irqs binds this instance's handler; no transfer exists yet.
        unsafe { LPI2C3::enable() };
        (disable_us, enable_us)
    });
    defmt::info!(
        "WATCHDOG SETUP: MEN disable_wait_us={}, enable_wait_us={}",
        disable_us,
        enable_us
    );

    let input_hz = embassy_mcxa::clocks::with_clocks(|c| c.fro_lf_div.as_ref().unwrap().frequency).unwrap();
    let cfg1 = pac::LPI2C3.mcfgr1().read();
    let pinlow = pac::LPI2C3.mcfgr3().read().pinlow();
    let expected = (requested_ms * 3_000_000).div_ceil(1000 * 256) as u16;
    assert_eq!(input_hz, 3_000_000);
    assert_eq!(pac::MRCC0.mrcc_lpi2c3_clksel().read().mux() as u8, 0);
    assert_eq!(pac::MRCC0.mrcc_lpi2c3_clkdiv().read().div(), 0);
    assert_eq!(cfg1.prescale(), pac::lpi2c::Prescale::DivideBy1);
    assert_eq!(cfg1.timecfg(), pac::lpi2c::Timecfg::IfSclLow);
    assert_eq!(pinlow, expected);
    assert_eq!(pac::LPI2C3.mcfgr2().read().filtscl(), 0);
    let initial = pac::LPI2C3.msr().read();
    assert_eq!(initial.pltf(), pac::lpi2c::Pltf::IntNo);
    assert_eq!(initial.mbf(), pac::lpi2c::Mbf::Idle);
    defmt::info!(
        "WATCHDOG CONFIG: requested_ms={}, input_hz={}, prescale={}, pinlow={}, timecfg={}, filtscl={}, software_timeout_ms=500",
        requested_ms,
        input_hz,
        cfg1.prescale() as u8,
        pinlow,
        cfg1.timecfg() as u8,
        pac::LPI2C3.mcfgr2().read().filtscl(),
    );

    let started = Instant::now();
    // Independent whole-call backstop, also comfortably beyond the 200 ms
    // control. Neither software timeout is an accepted substitute for PLTF.
    let result = with_timeout(Duration::from_millis(750), i2c.async_write(0x2a, b"Hello from DMA!\n"))
        .await
        .expect("watchdog outer deadline expired");
    let elapsed_us = started.elapsed().as_micros();
    defmt::info!("WATCHDOG CONTROLLER result={:?}, elapsed_us={}", result, elapsed_us);
    let window = match requested_ms {
        120 => 110_000..=135_000,
        140 => 130_000..=155_000,
        _ => 145_000..=190_000,
    };
    assert!(window.contains(&elapsed_us));
    assert_eq!(
        result,
        if requested_ms == 200 {
            Ok(())
        } else {
            Err(IOError::PinLowTimeout)
        }
    );
    // Check cleanup BEFORE any repair or target-release wait. Full-driver
    // terminal recovery re-enables MEN; unlike one-shot it retains the driver.
    for channel in 0..=1 {
        let csr = pac::EDMA_0_TCD.tcd(channel).ch_csr().read();
        assert!(!csr.erq() && !csr.active());
    }
    let requests = pac::LPI2C3.mder().read();
    assert!(!requests.tdde() && !requests.rdde());
    // Production recovery now abruptly resets the local engine rather than
    // undoing a pending graceful MEN-off. Require MBF idle even while the peer
    // still holds SCL; BBF and physical release remain distinct. Historical
    // opt-in repair modes retain their pre-repair observation semantics.
    record_recovery("return", requested_ms == 200 || repair.is_none());
    let returned = Instant::now();
    if let Some(mode) = repair {
        watchdog_repair::run(mode, returned).await;
    } else {
        Timer::after_millis(100).await;
    }
    record_recovery("after_wait", true);
    if repair == Some("abrupt_idle") {
        watchdog_repair::require_released_bus();
    }
    scl_observer::check_read_path().unwrap();
    assert!(scl_observer::high(), "SCL still low after target-release wait");
    if pac::LPI2C3.msr().read().mbf() != pac::lpi2c::Mbf::Idle {
        defmt::warn!("WATCHDOG: controller remains busy after peer release; terminal recovery not qualified");
    }
    if repair.is_some() {
        // Different bytes from the stalled transfer; no retry can conceal a
        // replayed suffix, lost address, or still-open transaction.
        with_timeout(Duration::from_millis(750), i2c.async_write(0x2a, b"Recovered DMA!!\n"))
            .await
            .expect("fresh transfer deadline")
            .expect("fresh transfer fault");
        assert_eq!(pac::LPI2C3.msr().read().mbf(), pac::lpi2c::Mbf::Idle);
        assert!(scl_observer::high());
        defmt::info!("WATCHDOG REPAIR: fresh write PASS");
    }
    defmt::info!("WATCHDOG {} CONTROLLER PASS", requested_ms);
    // Keep i2c alive: dropping it disables/remuxes the pads and would alter
    // the peer's still-running 150 ms hold. The runner resets both endpoints.
    cortex_m_semihosting::debug::exit(cortex_m_semihosting::debug::EXIT_SUCCESS);
    core::mem::forget(i2c);
    loop {
        cortex_m::asm::wfi();
    }
}
