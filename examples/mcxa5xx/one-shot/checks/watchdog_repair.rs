//! Opt-in post-return repair comparison. This is diagnostic PAC access, not
//! the production recovery implementation or proof of the original cause.

use embassy_mcxa::interrupt::typelevel::{Interrupt, LPI2C3};
use embassy_mcxa::pac::{self, lpi2c};
use embassy_time::{Duration, Instant, Timer};

use super::scl_observer;

pub fn selected() -> Option<&'static str> {
    match option_env!("I2C_ONE_SHOT_RECOVERY") {
        None | Some("none") => None,
        Some("wait") => Some("wait"),
        Some("reset") => Some("reset"),
        Some("abrupt") => Some("abrupt"),
        Some("abrupt_idle") => Some("abrupt_idle"),
        _ => panic!("Unknown I2C_ONE_SHOT_RECOVERY"),
    }
}

fn configuration() -> [u32; 8] {
    let r = pac::LPI2C3;
    [
        r.mcfgr0().read().0,
        r.mcfgr1().read().0,
        r.mcfgr2().read().0,
        r.mcfgr3().read().0,
        r.mdmr().read().0,
        r.mccr0().read().0,
        r.mccr1().read().0,
        r.mfcr().read().0,
    ]
}

async fn confirm_control(enabled: bool, reset: bool, budget_ms: u64) -> u64 {
    let started = Instant::now();
    loop {
        let control = pac::LPI2C3.mcr().read();
        if control.men() == enabled && control.rst() == reset {
            return started.elapsed().as_micros();
        }
        assert!(
            started.elapsed() < Duration::from_millis(budget_ms),
            "MCR readback deadline"
        );
        // The peripheral IRQ alone is masked. CPU, OSTIMER and executor stay
        // live: this bound must never become a 300 ms critical-section spin.
        Timer::after_micros(1).await;
    }
}

pub async fn run(mode: &str, returned: Instant) {
    defmt::info!("WATCHDOG REPAIR: mode={}", mode);
    let r = pac::LPI2C3;
    LPI2C3::disable();
    assert_eq!(r.mder().read().0, 0);
    r.mier().write_value(lpi2c::Mier(0));
    r.mder().write_value(lpi2c::Mder(0));
    let mut saved = configuration();
    assert!(!lpi2c::Mcfgr0(saved[0]).abort(), "cannot replay an ABORT command");
    let old_control = r.mcr().read();
    // Copy only persistent modes. Never recopy MEN, RST, or FIFO-reset strobes
    // from delayed readback into a subsequent control write.
    let mut control = lpi2c::Mcr(0);
    control.set_dozen(old_control.dozen());
    control.set_dbgen(old_control.dbgen());
    let abrupt = mode == "abrupt" || mode == "abrupt_idle";
    let reset_started = Instant::now();
    if abrupt {
        // Unlike the matched graceful arms, this tests a production-shaped
        // local abort: RM p1904 explicitly permits RST before graceful MEN-off
        // completes. Do not wait for the peer before asserting reset.
        let mut reset = control;
        reset.set_rst(true);
        r.mcr().write_value(reset);
        confirm_control(false, true, 1).await;
    }
    r.mcr().write_value(control);
    let disable_us = confirm_control(false, false, if abrupt { 1 } else { 300 }).await;
    if abrupt {
        let reset_us = reset_started.elapsed().as_micros();
        assert!(reset_us <= 1000, "abrupt reset exceeded local handshake budget");
        defmt::info!("WATCHDOG REPAIR: reset_wait_us={}", reset_us);
    } else {
        defmt::info!("WATCHDOG REPAIR: disable_wait_us={}", disable_us);
    }
    if mode != "wait" {
        if !abrupt {
            let mut reset = control;
            reset.set_rst(true);
            r.mcr().write_value(reset);
            r.mcr().write_value(control);
            confirm_control(false, false, 300).await;
        }
        // RM: RST is immediate, with no minimum asserted interval. It resets
        // ALL master registers except MCR; restore setup, never IRQ/DMA/status.
        r.mcfgr0().write_value(lpi2c::Mcfgr0(saved[0]));
        r.mcfgr1().write_value(lpi2c::Mcfgr1(saved[1]));
        r.mcfgr2().write_value(lpi2c::Mcfgr2(saved[2]));
        r.mcfgr3().write_value(lpi2c::Mcfgr3(saved[3]));
        r.mdmr().write_value(lpi2c::Mdmr(saved[4]));
        r.mccr0().write_value(lpi2c::Mccr0(saved[5]));
        r.mccr1().write_value(lpi2c::Mccr1(saved[6]));
        r.mfcr().write_value(lpi2c::Mfcr(saved[7]));
    }
    if mode == "abrupt_idle" {
        // An explicit diagnostic policy change, not unchanged configuration:
        // enable the RM's both-lines-high idle detector. Without a STOP and
        // BUSIDLE=0, BBF can stay set even after an abrupt local reset's peer
        // later releases the pads. Never use RELAX/ignore-busy as a substitute.
        let timing = lpi2c::Mccr0(saved[5]);
        let busidle = 2 * (u16::from(timing.clklo()) + u16::from(timing.sethold()) + 2);
        assert!(busidle > u16::from(timing.clkhi()) + 1 && busidle <= 4095);
        let mut cfg2 = lpi2c::Mcfgr2(saved[2]);
        assert_eq!(cfg2.busidle(), 0, "expected disabled detector baseline");
        cfg2.set_busidle(busidle);
        r.mcfgr2().write_value(cfg2);
        saved[2] = cfg2.0;
        defmt::info!(
            "WATCHDOG REPAIR: idle_detector busidle={}, clklo={}, sethold={}, clkhi={}",
            busidle,
            timing.clklo(),
            timing.sethold(),
            timing.clkhi(),
        );
    }
    assert_eq!(configuration(), saved);
    assert_eq!(r.mier().read().0, 0);
    assert_eq!(r.mder().read().0, 0);
    defmt::info!("WATCHDOG REPAIR: configuration restored=true");
    // Matched dwell in BOTH arms. A repair that misses this checkpoint is not
    // a valid matched comparison, even if its longer readback bound succeeds.
    let checkpoint = returned + Duration::from_millis(100);
    assert!(Instant::now() < checkpoint, "repair missed matched checkpoint");
    if abrupt {
        // Re-enable immediately, not after the peer's release. Prove the local
        // engine is idle while sampled SCL is STILL LOW, then leave both CPUs
        // running through the original target abort and fresh transfer.
        control.set_men(true);
        r.mcr().write_value(control);
        confirm_control(true, false, 1).await;
        let mcr = r.mcr().read();
        let msr = r.msr().read();
        let fifo = r.mfsr().read();
        defmt::info!(
            "WATCHDOG REPAIR: stage=reset_complete, mcr={}, msr={}, mfsr={}, scl_high={}",
            mcr.0,
            msr.0,
            fifo.0,
            scl_observer::high(),
        );
        assert!(mcr.men() && !mcr.rst());
        assert_eq!(msr.mbf(), lpi2c::Mbf::Idle);
        assert_eq!(fifo.txcount(), 0);
        assert_eq!(fifo.rxcount(), 0);
        assert!(!scl_observer::high(), "peer released before abrupt-reset witness");
        LPI2C3::unpend();
        // SAFETY: main's bound handler, no live transfer/lease, DMA quiescent.
        unsafe { LPI2C3::enable() };
        Timer::at(checkpoint).await;
        assert_eq!(configuration(), saved);
        return;
    }
    Timer::at(checkpoint).await;
    let mcr = r.mcr().read();
    let msr = r.msr().read();
    let fifo = r.mfsr().read();
    defmt::info!(
        "WATCHDOG REPAIR: stage=disabled, mcr={}, msr={}, mfsr={}, scl_high={}",
        mcr.0,
        msr.0,
        fifo.0,
        scl_observer::high(),
    );
    assert!(!mcr.men() && !mcr.rst());
    assert_eq!(msr.mbf(), lpi2c::Mbf::Idle);
    assert_eq!(fifo.txcount(), 0);
    assert_eq!(fifo.rxcount(), 0);
    assert!(scl_observer::high(), "repair did not release SCL");
    // Clear only observed W1C flags after local idle and peer release, equally
    // in both arms. No pending transaction or lease exists at this boundary.
    r.msr().write_value(msr);
    control.set_men(true);
    r.mcr().write_value(control);
    confirm_control(true, false, 300).await;
    assert_eq!(configuration(), saved);
    assert_eq!(r.msr().read().pltf(), lpi2c::Pltf::IntNo);
    LPI2C3::unpend();
    // SAFETY: main's Irqs binding owns this handler, and DMA is quiescent.
    unsafe { LPI2C3::enable() };
}

pub fn require_released_bus() {
    scl_observer::check_read_path().unwrap();
    let sda = pac::PORT3.pcr(20).read();
    assert_eq!(sda.mux(), pac::port::Mux::Mux2);
    assert_eq!(sda.ibe(), pac::port::Ibe::Ibe1);
    assert_eq!(sda.inv(), pac::port::Inv::Inv0);
    assert_eq!(pac::GPIO3.pidr().read().pid(20), pac::gpio::Pid::Pid0);
    // Same clock/reset domain as the checked SCL observer. One passive PDIR
    // snapshot samples BOTH lines, without remuxing or driving either pad.
    let pads = pac::GPIO3.pdir().read();
    let busy = pac::LPI2C3.msr().read().bbf() != lpi2c::MsrBbf::Idle;
    defmt::info!(
        "WATCHDOG REPAIR: lines scl_high={}, sda_high={}, bus_busy={}",
        pads.pdi(21),
        pads.pdi(20),
        busy,
    );
    assert!(pads.pdi(21) && pads.pdi(20), "bus pads not both high");
    assert!(!busy, "bus idle detector did not recognize release");
}
