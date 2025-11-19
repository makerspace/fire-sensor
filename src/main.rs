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
        gpio::{GpioError, IOPin, InputPin, Level, OutputPin, Pin, PinDriver, Pull},
        ledc::{config::TimerConfig, LedcDriver, LedcTimerDriver},
        peripheral::Peripheral,
        prelude::*,
    },
    http::{client::EspHttpConnection, Method},
    nvs::EspDefaultNvsPartition,
    ota::EspOta,
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
const MQTT_CLIENT_ID: &str = "dust_collector";
const MQTT_USERNAME: &str = "wakeup_alarm";
const MQTT_PASSWORD: &str = "xafzz25nomehasff";

const PRESSURE_SENSOR_TRIGGERED_URL: &str =
    "https://api.makerspace.se/alerts/pressure_sensor_triggered";
const PRESSURE_SENSOR_RESET_URL: &str = "https://api.makerspace.se/alerts/pressure_sensor_reset";

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
    // This will fail when flashing the app the normal way (not OTA), so we ignore the result.
    ota.mark_running_slot_valid().ok();
}

enum OneWireError {
    DeviceNotFound,
    Onewire(onewire::Error<GpioError>),
}

impl Debug for OneWireError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            OneWireError::DeviceNotFound => write!(f, "OneWire device not found"),
            OneWireError::Onewire(e) => write!(f, "OneWire error: {:?}", e),
        }
    }
}

impl From<onewire::Error<GpioError>> for OneWireError {
    fn from(error: onewire::Error<GpioError>) -> Self {
        OneWireError::Onewire(error)
    }
}

fn find_onewire_device<
    T: embedded_hal::digital::OutputPin<Error = GpioError>
        + embedded_hal::digital::InputPin<Error = GpioError>,
>(
    wire: &mut OneWire<T>,
    family_code: u8,
    delay: &mut impl DelayNs,
) -> Result<onewire::Device, OneWireError> {
    let mut search = DeviceSearch::new();
    while let Some(device) = wire.search_next(&mut search, delay)? {
        if device.address[0] == family_code {
            return Ok(device);
        } else {
            // Not the right family code, continue searching
        }
    }
    Err(OneWireError::DeviceNotFound)
}

/// Driver for a DS18B20 temperature sensor on a OneWire bus
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
    fn new(one: T) -> Result<Self, OneWireError> {
        let mut wire = OneWire::new(one, false);

        let mut delay = esp_idf_svc::hal::delay::Ets;
        wire.reset(&mut delay)?;

        // Search for devices
        let device = find_onewire_device(&mut wire, onewire::ds18b20::FAMILY_CODE, &mut delay)?;

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

/// Driver for a 16-channel digital multiplexer
/// Like the cd74hc4067
struct Multiplexer16<T: embedded_hal::digital::OutputPin, I: embedded_hal::digital::InputPin> {
    selector_pins: [T; 4],
    data_pin: I,
    delay: esp_idf_svc::hal::delay::Ets,
}

impl<
        T: embedded_hal::digital::OutputPin<Error = E>,
        I: embedded_hal::digital::InputPin<Error = E>,
        E,
    > Multiplexer16<T, I>
{
    fn new(selector_pins: [T; 4], data_pin: I, delay: esp_idf_svc::hal::delay::Ets) -> Self {
        Self {
            selector_pins,
            data_pin,
            delay,
        }
    }

    fn select_channel(&mut self, channel: u8) -> Result<(), E> {
        for (i, pin) in self.selector_pins.iter_mut().enumerate() {
            if (channel & (1 << i)) != 0 {
                pin.set_high()?;
            } else {
                pin.set_low()?;
            }
        }
        Ok(())
    }

    fn read_data(&mut self) -> Result<bool, E> {
        self.data_pin.is_high()
    }

    fn channel_is_high(&mut self, channel: u8) -> Result<bool, E> {
        self.select_channel(channel)?;
        // The multiplexer should settle in 6ns or so, which is smaller than even the ESP32's instruction latency.
        // But just because we're paranoid, add a tiny delay here.
        self.delay.delay_us(1);
        self.read_data().map_err(|e| e.into())
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
    let peripherals = Peripherals::take()?;
    let sys_loop = EspSystemEventLoop::take()?;
    let nvs = EspDefaultNvsPartition::take()?;
    let timer_service = esp_idf_svc::timer::EspTaskTimerService::new()?;

    let is_wokwi_simulator = check_is_wokwi()?;

    // Pin assignments
    let temperature_sensor_pin = peripherals.pins.gpio23;
    let dust_sensor_analog_pin = peripherals.pins.gpio13;
    let power_controller_pin = peripherals.pins.gpio12;
    let debug_led_pin = peripherals.pins.gpio2;
    let multiplexer_data_pin = peripherals.pins.gpio33;
    let multiplexer_selector_pin_0 = peripherals.pins.gpio14;
    let multiplexer_selector_pin_1 = peripherals.pins.gpio27;
    let multiplexer_selector_pin_2 = peripherals.pins.gpio26;
    let multiplexer_selector_pin_3 = peripherals.pins.gpio25;

    // Multiplexer channel assignments
    let low_pressure_signal_channel = 0;
    let dust_warning_signal_1_channel = 1;
    let dust_warning_signal_2_channel = 2;
    let main_power_relay_channel = 3;
    let dust_port_channels = 4..16;

    let mut temperature_sensor =
        match TemperatureSensor::new(PinDriver::input_output_od(temperature_sensor_pin)?) {
            Ok(sensor) => Some(sensor),
            Err(e) => {
                error!("Failed to initialize temperature sensor: {:?}", e);
                None
            }
        };

    let adc = esp_idf_svc::hal::adc::oneshot::AdcDriver::new(peripherals.adc2)?;

    let mut dust_sensor_analog = AdcChannelDriver::new(
        &adc,
        dust_sensor_analog_pin,
        &AdcChannelConfig {
            attenuation: DB_11, // Can sense values between 150 mV and 2450 mV
            ..Default::default()
        },
    )?;

    let mut multiplexer_data_driver = PinDriver::input(multiplexer_data_pin)?;

    // Gate signals are GND / Floating, so we need a pull-up on data pin.
    // The multiplexer also reads various relay signals that are active when high, but they all have stronger external pull-downs.
    // The ESP32 internal pull-up is weak (around 10k-100k), but the external pull-downs are stronger (1.5k).
    multiplexer_data_driver.set_pull(Pull::Up)?;

    let mut multiplexer = Multiplexer16::new(
        [
            PinDriver::output(multiplexer_selector_pin_0.downgrade_output())?,
            PinDriver::output(multiplexer_selector_pin_1.downgrade_output())?,
            PinDriver::output(multiplexer_selector_pin_2.downgrade_output())?,
            PinDriver::output(multiplexer_selector_pin_3.downgrade_output())?,
        ],
        multiplexer_data_driver,
        esp_idf_svc::hal::delay::Ets,
    );

    // Main power relay control pin.
    // Low or Floating  = Machine Off
    // High = Machine On (unless some other safety relay prevents it from running)
    // Note: Has external pull-down to GND
    let mut power_controller_driver = PinDriver::output(power_controller_pin)?;

    let ledc_driver = LedcTimerDriver::new(
        peripherals.ledc.timer2,
        &TimerConfig::default()
            .frequency(610.Hz().into())
            .resolution(esp_idf_svc::hal::ledc::Resolution::Bits17),
    )?;

    let mut debug_led = DebugLed::new(LedcDriver::new(
        peripherals.ledc.channel1,
        &ledc_driver,
        debug_led_pin,
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

    // Convert mac to string
    let mac_str = format!(
        "{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]
    );

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
            &format!("dust_collector/{device_id}/temperature"),
            Option::<NotNan<f32>>::None,
            SerializationFormat::Json,
            ReadWriteMode::Driven,
        )
        .await
        .unwrap();

    let temperature_warning_container = storage
        .add_container_with_mode(
            &format!("dust_collector/{device_id}/temperature_warning"),
            Option::<bool>::None,
            SerializationFormat::Json,
            ReadWriteMode::Driven,
        )
        .await
        .unwrap();

    let water_pressure_sensor_container = storage
        .add_container_with_mode(
            &format!("dust_collector/{device_id}/water_pressure"),
            Option::<bool>::None,
            SerializationFormat::Json,
            ReadWriteMode::Driven,
        )
        .await
        .unwrap();

    let dust_sensor_warning_1_container = storage
        .add_container_with_mode(
            &format!("dust_collector/{device_id}/dust_level_warning_1"),
            Option::<bool>::None,
            SerializationFormat::Json,
            ReadWriteMode::Driven,
        )
        .await
        .unwrap();

    let dust_sensor_warning_2_container = storage
        .add_container_with_mode(
            &format!("dust_collector/{device_id}/dust_level_warning_2"),
            Option::<bool>::None,
            SerializationFormat::Json,
            ReadWriteMode::Driven,
        )
        .await
        .unwrap();

    let main_power_relay_container = storage
        .add_container_with_mode(
            &format!("dust_collector/{device_id}/machine_running"),
            Option::<bool>::None,
            SerializationFormat::Json,
            ReadWriteMode::Driven,
        )
        .await
        .unwrap();

    let dust_sensor_level_container = storage
        .add_container_with_mode(
            &format!("dust_collector/{device_id}/dust_level"),
            Option::<u16>::None,
            SerializationFormat::Json,
            ReadWriteMode::Driven,
        )
        .await
        .unwrap();

    let powered_on_too_long_container = storage
        .add_container_with_mode(
            &format!("dust_collector/{device_id}/powered_on_too_long"),
            Option::<bool>::None,
            SerializationFormat::Json,
            ReadWriteMode::Driven,
        )
        .await
        .unwrap();

    let mut gate_containers: Vec<std::sync::Arc<brevduva::SyncedContainer<Option<bool>>>> = vec![];
    for (i, _) in dust_port_channels.clone().enumerate() {
        gate_containers.push(
            storage
                .add_container_with_mode(
                    &format!("dust_collector/{device_id}/gates/{i}/open"),
                    Option::<bool>::None,
                    SerializationFormat::Json,
                    ReadWriteMode::Driven,
                )
                .await
                .unwrap(),
        );
    }

    // Allow over-the-air updates
    // It's very convenient to be able to update the firmware without plugging in a cable.
    // You will need to be on the same network as the device to perform the update, however.
    ota_flasher::downloader::initialize_ota(&storage, &device_id, env!("BUILD_ID")).await;

    // Mark the current boot as successful
    // If the device fails to reach this point during the first boot after flashing OTA, it will revert to the previous firmware.
    successful_boot();

    debug_led.blink(1, Duration::from_millis(100)).await?;

    // This is good to do, but won't actually block us as all our containers are Driven.
    storage.wait_for_sync().await;

    debug_led.blink(10, Duration::from_millis(20)).await?;

    let mut last_time_pressure_sensor_value_unchanged = Instant::now();
    let mut last_time_all_gates_closed = Instant::now();
    let mut pressure_sensor_last = multiplexer
        .channel_is_high(low_pressure_signal_channel)
        .unwrap();

    let mut in_force_powered_off_state = false;

    loop {
        let temperature = temperature_sensor
            .as_mut()
            .and_then(|s| s.measure().map(|v| v as f32 / 16.0));

        match temperature {
            Some(t) => info!("Temperature: {:.2} °C", t),
            None => info!("Temperature: N/A"),
        }

        // Read multiplexer channels
        let pressure_low = multiplexer
            .channel_is_high(low_pressure_signal_channel)
            .unwrap();
        let dust_sensor_warning_1 = multiplexer
            .channel_is_high(dust_warning_signal_1_channel)
            .unwrap();
        let dust_sensor_warning_2 = multiplexer
            .channel_is_high(dust_warning_signal_2_channel)
            .unwrap();
        let main_power_relay = multiplexer
            .channel_is_high(main_power_relay_channel)
            .unwrap();

        // Gates are open when low. The input pin has an internal pull-up resistor.
        let gates_open = dust_port_channels
            .clone()
            .map(|channel| !multiplexer.channel_is_high(channel).unwrap())
            .collect::<Vec<_>>();

        // Read dust sensor analog value. This is an approximate distance value. A low value means more dust.
        let dust_sensor_level: u16 = dust_sensor_analog.read().unwrap_or(0);

        let any_gate_open = gates_open.iter().any(|&open| open);
        if !any_gate_open {
            last_time_all_gates_closed = Instant::now();
        }

        // High temperature safety cut-off
        // The temperature sensor is right above the dust bin, below the filters.
        let temperature_too_high = temperature.map(|t| t > 60.0);

        // If the machine has been powered on for a long time, cut the power automatically.
        // This will require closing all gates to reset the system (or a power cycle).
        let powered_on_too_long =
            last_time_all_gates_closed.elapsed() > Duration::from_secs(30 * 60);

        let force_powered_off_internal =
            temperature_too_high.unwrap_or(false) || powered_on_too_long;

        // Note: dust_sensor_warning_2 and pressure_low already use external relays that will cut the power if needed.
        // But we still want to track the forced powered off state here, and will also keep the machine powered off
        // until all gates have been closed, even if the external relays have reset.
        let force_powered_off_external = dust_sensor_warning_2 || pressure_low;
        let force_powered_off = force_powered_off_internal || force_powered_off_external;

        in_force_powered_off_state |= force_powered_off;

        if in_force_powered_off_state && !force_powered_off && !any_gate_open {
            // Reset the forced powered off state when all conditions are cleared and all gates are closed.
            in_force_powered_off_state = false;
            info!("Exiting forced powered off state");
        }

        // Update brevduva containers
        temperature_container
            .set(temperature.map(|t| NotNan::new(t).unwrap()))
            .await;

        temperature_warning_container
            .set(temperature_too_high)
            .await;

        water_pressure_sensor_container
            .set(Some(!pressure_low))
            .await;

        dust_sensor_warning_1_container
            .set(Some(dust_sensor_warning_1))
            .await;

        dust_sensor_warning_2_container
            .set(Some(dust_sensor_warning_2))
            .await;

        main_power_relay_container.set(Some(main_power_relay)).await;

        dust_sensor_level_container
            .set(Some(dust_sensor_level))
            .await;

        powered_on_too_long_container
            .set(Some(powered_on_too_long))
            .await;

        for (gate_container, &open) in gate_containers.iter().zip(gates_open.iter()) {
            gate_container.set(Some(open)).await;
        }

        // Control power to the machine.
        power_controller_driver.set_level(if in_force_powered_off_state {
            Level::Low
        } else if any_gate_open {
            Level::High
        } else {
            Level::Low
        })?;

        // Check for pressure sensor state change
        if pressure_low != pressure_sensor_last {
            // Delay to avoid multiple triggers due to sensor bouncing
            if last_time_pressure_sensor_value_unchanged.elapsed() > Duration::from_millis(500) {
                pressure_sensor_last = pressure_low;
                last_time_pressure_sensor_value_unchanged = Instant::now();

                if pressure_low {
                    warn!(
                        "Pressure sensor triggered! Temperature: {:.2} °C",
                        temperature.map(|t| t).unwrap_or(f32::NAN)
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

        tokio::time::sleep(Duration::from_millis(200)).await;
    }
}
