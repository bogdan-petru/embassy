//! Read-only observation of the pad already owned/muxed by LPI2C3.
//! An enabled automatic gate is valid too: CC alone is not an enable oracle.
use embassy_mcxa::pac;

pub fn log_configuration() {
    defmt::info!(
        "SCL OBSERVER: port_cc={}, port_acc={}, gpio_cc={}, gpio_acc={}, port_reset_released={}, gpio_reset_released={}",
        pac::MRCC0.mrcc_glb_cc1().read().port3(),
        pac::MRCC0.mrcc_glb_acc1().read().port3(),
        pac::MRCC0.mrcc_glb_cc3().read().gpio3(),
        pac::MRCC0.mrcc_glb_acc3().read().gpio3(),
        pac::MRCC0.mrcc_glb_rst1().read().port3(),
        pac::MRCC0.mrcc_glb_rst3().read().gpio3(),
    );
}

pub fn check_read_path() -> Result<(), &'static str> {
    // MRCC supports software-controlled CC and automatic ACC clock gating.
    // HAL init uses ACC for GPIO3. Do not write either gate, reset the port,
    // acquire a second pin owner, or touch GPIO's output path to observe SCL.
    if !(pac::MRCC0.mrcc_glb_cc1().read().port3() || pac::MRCC0.mrcc_glb_acc1().read().port3()) {
        return Err("PORT3 has no enabled clock gate");
    }
    if !(pac::MRCC0.mrcc_glb_cc3().read().gpio3() || pac::MRCC0.mrcc_glb_acc3().read().gpio3()) {
        return Err("GPIO3 has no enabled clock gate");
    }
    if !pac::MRCC0.mrcc_glb_rst1().read().port3() || !pac::MRCC0.mrcc_glb_rst3().read().gpio3() {
        return Err("SCL observer block held in reset");
    }
    let pin = pac::PORT3.pcr(21).read();
    if pin.mux() != pac::port::Mux::Mux2 {
        return Err("SCL mux changed");
    }
    if pin.ibe() != pac::port::Ibe::Ibe1 || pin.inv() != pac::port::Inv::Inv0 {
        return Err("SCL input disabled or inverted");
    }
    if pac::GPIO3.pidr().read().pid(21) != pac::gpio::Pid::Pid0 {
        return Err("SCL GPIO input disabled");
    }
    Ok(())
}

pub fn high() -> bool {
    // Only call after check_read_path succeeds. A high/low/high control is
    // still required: configuration alone is not evidence of the pin voltage.
    pac::GPIO3.pdir().read().pdi(21)
}
