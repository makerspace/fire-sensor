use esp_idf_svc::{
    hal::gpio::{self, PinDriver},
    sys::EspError,
};
use log::info;

pub fn check_is_wokwi() -> Result<bool, EspError> {
    let wokwi_pin = unsafe { gpio::Gpio5::new() };
    let mut driver = PinDriver::input(wokwi_pin)?;
    driver.set_pull(gpio::Pull::Up)?;

    let is_wokwi_simulator = driver.is_low();
    if is_wokwi_simulator {
        info!("Running on Wokwi simulator");
    } else {
        info!("Running on real hardware");
    }

    Ok(is_wokwi_simulator)
}
