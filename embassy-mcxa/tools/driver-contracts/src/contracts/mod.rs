mod i2c;
mod uart;

pub fn get(name: &str) -> Result<crate::Contract, String> {
    match name {
        "i2c" => Ok(i2c::contract()),
        "uart" => Ok(uart::contract()),
        _ => Err(format!("unknown IP {name}")),
    }
}
