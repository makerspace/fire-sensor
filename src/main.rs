#![deny(clippy::future_not_send)]
mod esp;
mod wifi;
mod wokwi;

use brevduva::{channel::SerializationFormat, ReadWriteMode, SyncStorage};
use embedded_hal::delay::DelayNs;
use esp::init_esp;
use esp_idf_svc::{
    eventloop::EspSystemEventLoop,
    hal::{
        adc::{
            attenuation::DB_11,
            oneshot::{config::AdcChannelConfig, AdcChannelDriver},
        },
        gpio::{GpioError, PinDriver, Pull},
        ledc::{config::TimerConfig, LedcDriver, LedcTimerDriver},
        prelude::*,
    },
    http::{client::EspHttpConnection, Method},
    nvs::EspDefaultNvsPartition,
    ota::EspOta,
    sntp::EspSntp,
    sys::EspError,
};
use log::{error, info, warn};
use onewire::DS18B20;
use onewire::{DeviceSearch, OneWire};
use ordered_float::NotNan;
use std::{
    fmt::Debug,
    time::{Duration, Instant},
};
use wifi::start_wifi;
use wokwi::check_is_wokwi;

const MQTT_HOST: &str = "mqtt://arongranberg.com:1883";
const MQTT_CLIENT_ID: &str = "fire_sensor";
const MQTT_USERNAME: &str = "wakeup_alarm";
const MQTT_PASSWORD: &str = "xafzz25nomehasff";

const PRESSURE_SENSOR_TRIGGERED_URL: &str =
    "https://api.makerspace.se/events/pressure_sensor_triggered";
const PRESSURE_SENSOR_RESET_URL: &str = "https://api.makerspace.se/events/pressure_sensor_reset";

fn main() {
    init_esp();

    warn!(
        "ESP max log level={:?} log crate max level={:?}",
        esp_idf_svc::log::EspLogger::default().get_max_level(),
        log::STATIC_MAX_LEVEL
    );
    esp_idf_svc::log::set_target_level("*", log::LevelFilter::Info).unwrap();
    esp_idf_svc::log::set_target_level("brevduva", log::LevelFilter::Trace).unwrap();
    esp_idf_svc::log::set_target_level("fire-sensor", log::LevelFilter::Trace).unwrap();

    tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .unwrap()
        .block_on(async_main())
        .unwrap();
}

struct DebugLed {
    driver: LedcDriver<'static>,
}

impl DebugLed {
    fn new(driver: LedcDriver<'static>) -> Self {
        Self { driver }
    }

    pub fn resolution(&self) -> u32 {
        self.driver.get_max_duty()
    }

    fn set_duty_raw(&mut self, duty: u32) -> Result<(), EspError> {
        self.driver.set_duty(duty)
    }

    fn set_duty(&mut self, duty: f32) -> Result<(), EspError> {
        self.set_duty_raw((duty * self.resolution() as f32).round() as u32)
    }

    async fn blink(&mut self, times: usize, period: Duration) -> Result<(), EspError> {
        for _ in 0..times {
            self.set_duty(1.0)?;
            tokio::time::sleep(period).await;
            self.set_duty(0.0)?;
            tokio::time::sleep(period).await;
        }
        Ok(())
    }
}

fn successful_boot() {
    let mut ota = EspOta::new().expect("obtain OTA instance");
    ota.mark_running_slot_valid().expect("mark app as valid");
}

fn find_onewire_device<
    T: embedded_hal::digital::OutputPin<Error = GpioError>
        + embedded_hal::digital::InputPin<Error = GpioError>,
>(
    wire: &mut OneWire<T>,
    family_code: u8,
    delay: &mut impl DelayNs,
) -> Option<onewire::Device> {
    let mut search = DeviceSearch::new();
    while let Some(device) = wire.search_next(&mut search, delay).unwrap() {
        if device.address[0] == family_code {
            return Some(device);
        } else {
            // Not the right family code, continue searching
        }
    }
    None
}

struct TemperatureSensor<
    T: embedded_hal::digital::OutputPin<Error = GpioError>
        + embedded_hal::digital::InputPin<Error = GpioError>,
> {
    sensor: DS18B20,
    wire: OneWire<T>,
    delay: esp_idf_svc::hal::delay::Ets,
}

impl<
        T: embedded_hal::digital::OutputPin<Error = GpioError>
            + embedded_hal::digital::InputPin<Error = GpioError>,
    > TemperatureSensor<T>
{
    fn new(one: T) -> Result<Self, &'static str> {
        let mut wire = OneWire::new(one, false);

        let mut delay = esp_idf_svc::hal::delay::Ets;
        if wire.reset(&mut delay).is_err() {
            // missing pullup or error on line
            return Err(
                "Failed to initialize temperature sensor. Check wiring and pull-up resistor.",
            );
        }

        // Search for devices
        let device = find_onewire_device(&mut wire, onewire::ds18b20::FAMILY_CODE, &mut delay)
            .ok_or("No DS18B20 temperature sensor found on the OneWire bus.")?;

        let ds18b20 = onewire::DS18B20::new(device).unwrap();
        return Ok(TemperatureSensor {
            sensor: ds18b20,
            wire,
            delay,
        });
    }

    fn measure(&mut self) -> Option<u16> {
        // request sensor to measure temperature
        let resolution = self
            .sensor
            .measure_temperature(&mut self.wire, &mut self.delay)
            .unwrap();

        // wait for compeltion, depends on resolution
        self.delay.delay_ms(resolution.time_ms() as u32);

        // read temperature
        let temperature = self
            .sensor
            .read_temperature(&mut self.wire, &mut self.delay)
            .unwrap();
        Some(temperature)
    }
}

enum PostError {
    EspError(EspError),
    Status(u16),
}

impl From<EspError> for PostError {
    fn from(error: EspError) -> Self {
        PostError::EspError(error)
    }
}

impl Debug for PostError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PostError::EspError(e) => write!(f, "EspError: {}", e),
            PostError::Status(s) => write!(f, "HTTP Status Error: {}", s),
        }
    }
}

async fn post_url(url: &'static str) -> Result<(), PostError> {
    tokio::task::spawn_blocking(move || {
        // Send POST request using esp-idf API
        let mut client = EspHttpConnection::new(&esp_idf_svc::http::client::Configuration {
            crt_bundle_attach: Some(esp_idf_svc::sys::esp_crt_bundle_attach),
            buffer_size: Some(32 * 1024),
            ..Default::default()
        })
        .map_err(PostError::EspError)?;

        let headers = [("Content-Type", "application/json")];
        let body = b"{}"; // Empty JSON body

        client.initiate_request(Method::Post, url, &headers)?;
        client.write(body).map_err(PostError::EspError)?;
        client.initiate_response().map_err(PostError::EspError)?;

        if client.status() == 200 {
            info!("Post message sent successfully to {url}");
        } else {
            error!(
                "Failed to send post message to {url}. Status: {}",
                client.status()
            );
            return Err(PostError::Status(client.status()));
        }

        Ok::<(), PostError>(())
    })
    .await
    .unwrap()
}

async fn async_main() -> Result<(), EspError> {
    // Temperature sensor: DS18B20
    // GND
    // VCC (3.3V)
    // Data: GPIO4 with 4.7k pull-up to VCC
    //
    // Dust level sensor: relay
    // 24V
    // GPIO5, with pull-down to GND, via logic-level converter from 24V
    //
    // Pressure sensor (for when fire sensor triggers): relay
    // 24V
    // GPIO18, with pull-down to GND, via logic-level converter from 24V

    let peripherals = Peripherals::take()?;
    let sys_loop = EspSystemEventLoop::take()?;
    let nvs = EspDefaultNvsPartition::take()?;
    let timer_service = esp_idf_svc::timer::EspTaskTimerService::new()?;

    let is_wokwi_simulator = check_is_wokwi()?;

    let ledc_driver = LedcTimerDriver::new(
        peripherals.ledc.timer2,
        &TimerConfig::default()
            .frequency(610.Hz().into())
            .resolution(esp_idf_svc::hal::ledc::Resolution::Bits17),
    )?;

    let pressure_sensor_pin = peripherals.pins.gpio16;
    let dust_sensor_pin = peripherals.pins.gpio13;
    let temperature_sensor_pin = peripherals.pins.gpio23;
    let dust_sensor_analog_pin = peripherals.pins.gpio27;

    let mut temperature_sensor =
        TemperatureSensor::new(PinDriver::input_output_od(temperature_sensor_pin)?).unwrap();

    let mut pressure_sensor = PinDriver::input(pressure_sensor_pin)?;
    pressure_sensor.set_pull(Pull::Up)?;

    let mut dust_sensor_switch = PinDriver::input(dust_sensor_pin)?;
    dust_sensor_switch.set_pull(Pull::Up)?;

    let adc = esp_idf_svc::hal::adc::oneshot::AdcDriver::new(peripherals.adc2)?;

    let mut dust_sensor_analog = AdcChannelDriver::new(
        &adc,
        dust_sensor_analog_pin,
        &AdcChannelConfig {
            attenuation: DB_11, // Can sense values between 150 mV and 2450 mV
            ..Default::default()
        },
    )?;

    let mut debug_led = DebugLed::new(LedcDriver::new(
        peripherals.ledc.channel1,
        &ledc_driver,
        peripherals.pins.gpio2,
    )?);
    debug_led.blink(4, Duration::from_millis(50)).await?;

    let mac = start_wifi(
        peripherals.modem,
        sys_loop.clone(),
        nvs,
        timer_service.clone(),
        is_wokwi_simulator,
    )
    .await;

    // convert mac to string
    let mac_str = format!(
        "{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]
    );

    let ntp = EspSntp::new_default().unwrap();

    let device_id = format!("{MQTT_CLIENT_ID} {mac_str}");
    info!("Device ID: {}", device_id);

    // Initialize brevduva storage
    // This will connect to an MQTT broker
    // and store sensor data in the cloud
    let storage = SyncStorage::new(
        &device_id,
        MQTT_HOST,
        MQTT_USERNAME,
        MQTT_PASSWORD,
        brevduva::SessionPersistance::Persistent,
    )
    .await;

    let temperature_container = storage
        .add_container_with_mode(
            &format!("fire_sensor/{device_id}/temperature"),
            Option::<NotNan<f32>>::None,
            SerializationFormat::Json,
            ReadWriteMode::Driven,
        )
        .await
        .unwrap();

    let water_pressure_sensor_container = storage
        .add_container_with_mode(
            &format!("fire_sensor/{device_id}/water_pressure"),
            Option::<bool>::None,
            SerializationFormat::Json,
            ReadWriteMode::Driven,
        )
        .await
        .unwrap();

    let dust_sensor_switch_container = storage
        .add_container_with_mode(
            &format!("fire_sensor/{device_id}/dust_level_switch"),
            Option::<bool>::None,
            SerializationFormat::Json,
            ReadWriteMode::Driven,
        )
        .await
        .unwrap();

    let dust_sensor_level_container = storage
        .add_container_with_mode(
            &format!("fire_sensor/{device_id}/dust_level"),
            Option::<u16>::None,
            SerializationFormat::Json,
            ReadWriteMode::Driven,
        )
        .await
        .unwrap();

    ota_flasher::downloader::initialize_ota(&storage, &device_id, env!("BUILD_ID")).await;

    successful_boot();

    debug_led.blink(1, Duration::from_millis(100)).await?;

    // Wait until we have current time from network
    while ntp.get_sync_status() != esp_idf_svc::sntp::SyncStatus::Completed {
        tokio::time::sleep(Duration::from_millis(10)).await;
    }

    debug_led.blink(2, Duration::from_millis(40)).await?;

    storage.wait_for_sync().await;

    debug_led.blink(10, Duration::from_millis(20)).await?;

    let mut last_time_pressure_sensor_value_unchanged = Instant::now();
    let mut pressure_sensor_last = pressure_sensor.is_high();

    loop {
        let t = temperature_sensor.measure().unwrap() as f32 / 16.0;
        const M: f32 = 1.0; // Example slope
        const B: f32 = 0.0; // Example intercept
        let temperature = M * t + B;

        info!("Temperature: {:.2} °C", temperature);

        temperature_container
            .set(Some(NotNan::new(temperature).unwrap()))
            .await;

        let pressure_sensor_value = pressure_sensor.is_low();
        water_pressure_sensor_container
            .set(Some(pressure_sensor_value))
            .await;

        let dust_sensor_value = dust_sensor_switch.is_low();
        dust_sensor_switch_container
            .set(Some(dust_sensor_value))
            .await;

        let dust_sensor_level: u16 = dust_sensor_analog.read().unwrap_or(0);
        dust_sensor_level_container
            .set(Some(dust_sensor_level))
            .await;

        if pressure_sensor_value != pressure_sensor_last {
            // Delay to avoid multiple triggers due to sensor bouncing
            if last_time_pressure_sensor_value_unchanged.elapsed() > Duration::from_millis(500) {
                pressure_sensor_last = pressure_sensor_value;
                last_time_pressure_sensor_value_unchanged = Instant::now();

                if pressure_sensor_value {
                    warn!(
                        "Pressure sensor triggered! Temperature: {:.2} °C",
                        temperature
                    );

                    match post_url(PRESSURE_SENSOR_TRIGGERED_URL).await {
                        Ok(_) => info!("Pressure sensor triggered event sent successfully"),
                        Err(e) => warn!("Failed to send pressure sensor triggered event: {:?}", e),
                    }
                } else {
                    match post_url(PRESSURE_SENSOR_RESET_URL).await {
                        Ok(_) => info!("Pressure sensor reset event sent successfully"),
                        Err(e) => warn!("Failed to send pressure sensor reset event: {:?}", e),
                    }
                }
            }
        } else {
            last_time_pressure_sensor_value_unchanged = Instant::now();
        }

        tokio::time::sleep(Duration::from_millis(500)).await;
    }
}
