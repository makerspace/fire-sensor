#![deny(clippy::future_not_send)]
mod esp;
mod wifi;
mod wokwi;

use brevduva::{
    channel::{Channel, SerializationFormat},
    ReadWriteMode, SyncStorage,
};
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
        reset::ResetReason,
    },
    http::{client::EspHttpConnection, Method},
    nvs::EspDefaultNvsPartition,
    ota::EspOta,
    sys::{multi_heap_info_t, EspError},
};
use log::{error, info, warn};
use onewire::DS18B20;
use onewire::{DeviceSearch, OneWire};
use ordered_float::NotNan;
use std::{
    fmt::Debug,
    time::{Duration, Instant},
};
use tokio::join;
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
    esp_idf_svc::log::set_target_level("*", log::LevelFilter::Trace).unwrap();
    esp_idf_svc::log::set_target_level("brevduva", log::LevelFilter::Trace).unwrap();
    esp_idf_svc::log::set_target_level("fire-sensor", log::LevelFilter::Trace).unwrap();

    tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .max_blocking_threads(2)
        .thread_stack_size(30000)
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

/// Driver for an 8-channel digital multiplexer
/// Like the CD4051BM96
struct Multiplexer8<T: embedded_hal::digital::OutputPin, I: embedded_hal::digital::InputPin> {
    selector_pins: [T; 3],
    data_pin: I,
    delay: esp_idf_svc::hal::delay::Ets,
}

impl<
        T: embedded_hal::digital::OutputPin<Error = E>,
        I: embedded_hal::digital::InputPin<Error = E>,
        E,
    > Multiplexer8<T, I>
{
    fn new(selector_pins: [T; 3], data_pin: I, delay: esp_idf_svc::hal::delay::Ets) -> Self {
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
        // However, the wires are long and it takes some time for the signal to stabilize.
        // The ESP32 also has some capacitance on its input pins
        // which, according to my calculations takes around 200us to charge/discharge through
        // the built-in pull-up/pull-down resistors.
        self.delay.delay_us(200);

        // Even though we delay some time, we still get occasional unstable readings.
        // So we read the value multiple times and use majority voting.
        // This seems to eliminate all noise.
        let mut high_count = 0;
        for _ in 0..4 {
            if self.read_data().map_err(|e| e.into())? {
                high_count += 1;
            }
            self.delay.delay_us(20);
        }
        if high_count > 2 {
            Ok(true)
        } else {
            Ok(false)
        }

        // self.read_data().map_err(|e| e.into())
    }
}

/// Driver for a 16-channel digital multiplexer composed of two 8-channel multiplexers
struct Multiplexer16<
    T: embedded_hal::digital::OutputPin,
    I1: embedded_hal::digital::InputPin,
    I2: embedded_hal::digital::InputPin,
> {
    mux1: Multiplexer8<T, I1>,
    mux2: Multiplexer8<T, I2>,
}

impl<
        T: embedded_hal::digital::OutputPin<Error = E>,
        I1: embedded_hal::digital::InputPin<Error = E>,
        I2: embedded_hal::digital::InputPin<Error = E>,
        E,
    > Multiplexer16<T, I1, I2>
{
    fn new(mux1: Multiplexer8<T, I1>, mux2: Multiplexer8<T, I2>) -> Self {
        Self { mux1, mux2 }
    }

    fn channel_is_high(&mut self, channel: u8) -> Result<bool, E> {
        if channel < 8 {
            self.mux1.channel_is_high(channel)
        } else {
            self.mux2.channel_is_high(channel - 8)
        }
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
            timeout: Some(Duration::from_secs(10)),
            ..Default::default()
        })
        .map_err(PostError::EspError)?;

        let headers = [("Content-Type", "application/json")];
        let body = b"{}"; // Empty JSON body

        client.initiate_request(Method::Post, url, &headers)?;
        client.write_all(body).map_err(PostError::EspError)?;
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

async fn maybe_enter_safe_mode(debug_led: &mut DebugLed, status_channel: &Channel<String>) {
    match ResetReason::get() {
        ResetReason::Panic | ResetReason::TaskWatchdog | ResetReason::CPULockup => {
            error!(
                "Previous reset was due to panic or watchdog. Entering safe mode for 30 seconds."
            );
            status_channel
                .send(
                    "Restart was due to panic or watchdog. Entering safe mode for 30 seconds."
                        .to_string(),
                )
                .await;
            // Sleep
            debug_led
                .blink(30, Duration::from_millis(500))
                .await
                .unwrap();
            status_channel.send("Exiting safe mode.".to_string()).await;
            info!("Exiting safe mode.");
        }
        r => {
            info!("Reset reason: {:?}", r);
        }
    }
}
async fn async_main() -> Result<(), EspError> {
    let peripherals = Peripherals::take()?;
    let sys_loop = EspSystemEventLoop::take()?;
    let nvs = EspDefaultNvsPartition::take()?;
    let timer_service = esp_idf_svc::timer::EspTaskTimerService::new()?;

    let is_wokwi_simulator = check_is_wokwi()?;

    // Pin assignments
    let temperature_sensor_pin = peripherals.pins.gpio32;
    let dust_sensor_analog_pin = peripherals.pins.gpio34;
    let power_controller_pin = peripherals.pins.gpio27;
    let dimmer2_pin = peripherals.pins.gpio26;
    let debug_led_pin = peripherals.pins.gpio25;

    let indicator_red_pin = peripherals.pins.gpio13;
    let indicator_yellow_pin = peripherals.pins.gpio12;
    let indicator_green_pin = peripherals.pins.gpio14;
    let indicator_sound_pin = peripherals.pins.gpio2;

    let multiplexer1_data_pin = peripherals.pins.gpio21;
    let multiplexer1_selector_pin_0 = peripherals.pins.gpio5;
    let multiplexer1_selector_pin_1 = peripherals.pins.gpio18;
    let multiplexer1_selector_pin_2 = peripherals.pins.gpio19;

    let multiplexer2_data_pin = peripherals.pins.gpio17;
    let multiplexer2_selector_pin_0 = peripherals.pins.gpio0;
    let multiplexer2_selector_pin_1 = peripherals.pins.gpio4;
    let multiplexer2_selector_pin_2 = peripherals.pins.gpio16;

    // Multiplexer channel assignments
    let main_power_relay_channel = 5;
    let low_pressure_signal_channel = 6;
    let high_pressure_signal_channel = 7;
    let dust_port_channels = &[
        3,
        0,
        1,
        2,
        8 + 5,
        8 + 7,
        8 + 6,
        8 + 4,
        8 + 3,
        8 + 0,
        8 + 1,
        8 + 2,
    ];

    let mut temperature_sensor =
        match TemperatureSensor::new(PinDriver::input_output_od(temperature_sensor_pin)?) {
            Ok(sensor) => Some(sensor),
            Err(e) => {
                error!("Failed to initialize temperature sensor: {:?}", e);
                None
            }
        };

    let adc = esp_idf_svc::hal::adc::oneshot::AdcDriver::new(peripherals.adc1)?;

    let mut dust_sensor_analog = AdcChannelDriver::new(
        &adc,
        dust_sensor_analog_pin,
        &AdcChannelConfig {
            attenuation: DB_11, // Can sense values between 150 mV and 2450 mV
            ..Default::default()
        },
    )?;

    let multiplexer1_data_driver = PinDriver::input(multiplexer1_data_pin)?;

    let multiplexer1 = Multiplexer8::new(
        [
            PinDriver::output(multiplexer1_selector_pin_0.downgrade_output())?,
            PinDriver::output(multiplexer1_selector_pin_1.downgrade_output())?,
            PinDriver::output(multiplexer1_selector_pin_2.downgrade_output())?,
        ],
        multiplexer1_data_driver,
        esp_idf_svc::hal::delay::Ets,
    );

    let multiplexer2_data_driver = PinDriver::input(multiplexer2_data_pin)?;

    let multiplexer2 = Multiplexer8::new(
        [
            PinDriver::output(multiplexer2_selector_pin_0.downgrade_output())?,
            PinDriver::output(multiplexer2_selector_pin_1.downgrade_output())?,
            PinDriver::output(multiplexer2_selector_pin_2.downgrade_output())?,
        ],
        multiplexer2_data_driver,
        esp_idf_svc::hal::delay::Ets,
    );

    let mut multiplexer = Multiplexer16::new(multiplexer1, multiplexer2);

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

    let mut indicator_red = DebugLed::new(LedcDriver::new(
        peripherals.ledc.channel0,
        &ledc_driver,
        indicator_red_pin,
    )?);
    let mut indicator_yellow = DebugLed::new(LedcDriver::new(
        peripherals.ledc.channel3,
        &ledc_driver,
        indicator_yellow_pin,
    )?);
    let mut indicator_green = DebugLed::new(LedcDriver::new(
        peripherals.ledc.channel2,
        &ledc_driver,
        indicator_green_pin,
    )?);
    let mut indicator_sound = DebugLed::new(LedcDriver::new(
        peripherals.ledc.channel4,
        &ledc_driver,
        indicator_sound_pin,
    )?);

    debug_led.blink(4, Duration::from_millis(50)).await?;
    indicator_red.blink(2, Duration::from_millis(100)).await?;
    indicator_yellow
        .blink(2, Duration::from_millis(100))
        .await?;
    indicator_green.blink(2, Duration::from_millis(100)).await?;
    indicator_sound.blink(2, Duration::from_millis(100)).await?;

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

    let (status_channel, _) = storage
        .add_channel::<String>(
            &format!("dust_collector/{device_id}/status"),
            SerializationFormat::String,
        )
        .await
        .unwrap();

    // Allow over-the-air updates
    // It's very convenient to be able to update the firmware without plugging in a cable.
    // You will need to be on the same network as the device to perform the update, however.
    ota_flasher::downloader::initialize_ota(&storage, &device_id, env!("BUILD_ID")).await;

    // Mark the current boot as successful
    // If the device fails to reach this point during the first boot after flashing OTA, it will revert to the previous firmware.
    successful_boot();

    maybe_enter_safe_mode(&mut debug_led, &status_channel).await;

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
    for (i, _) in dust_port_channels.iter().enumerate() {
        gate_containers.push(
            storage
                .add_container_with_mode(
                    &format!("dust_collector/{device_id}/gates/{}/open", i + 1),
                    Option::<bool>::None,
                    SerializationFormat::Json,
                    ReadWriteMode::Driven,
                )
                .await
                .unwrap(),
        );
    }

    debug_led.blink(1, Duration::from_millis(100)).await?;

    info!("Waiting for storage to sync...");

    // This is good to do, but won't actually block us as all our containers are Driven.
    storage.wait_for_sync().await;

    info!("Storage synced.");

    debug_led.blink(10, Duration::from_millis(20)).await?;

    let mut last_time_all_gates_closed = Instant::now();
    let mut pressure_high_last = !multiplexer
        .channel_is_high(low_pressure_signal_channel)
        .unwrap();

    info!(
        "Initial pressure sensor state: {}",
        if pressure_high_last {
            "High pressure"
        } else {
            "Low pressure"
        }
    );

    let mut last_t = Instant::now();

    let send_alart_messages = {
        let mut last_time_pressure_sensor_value_unchanged = Instant::now();
        let water_pressure_sensor_container = water_pressure_sensor_container.clone();
        let temperature_container = temperature_container.clone();
        async move {
            loop {
                info!("Checking pressure sensor for state changes...");
                let pressure_high = water_pressure_sensor_container.get().unwrap();

                // Check for pressure sensor state change
                match pressure_high {
                    None => {
                        // Sensor not available
                        tokio::time::sleep(Duration::from_millis(200)).await;
                        continue;
                    }
                    Some(pressure_high) => {
                        if pressure_high == pressure_high_last {
                            last_time_pressure_sensor_value_unchanged = Instant::now();
                        } else {
                            // State changed
                            // Delay to avoid multiple triggers due to sensor bouncing
                            if last_time_pressure_sensor_value_unchanged.elapsed()
                                > Duration::from_millis(500)
                            {
                                pressure_high_last = pressure_high;
                                last_time_pressure_sensor_value_unchanged = Instant::now();
                                let temperature =
                                    temperature_container.get().unwrap().map(|t| t.into_inner());

                                if !pressure_high {
                                    warn!(
                                        "Pressure sensor triggered! Temperature: {:.2} °C",
                                        temperature.map(|t| t).unwrap_or(f32::NAN)
                                    );

                                    match post_url(PRESSURE_SENSOR_TRIGGERED_URL).await {
                                        Ok(_) => info!(
                                            "Pressure sensor triggered event sent successfully"
                                        ),
                                        Err(e) => warn!(
                                            "Failed to send pressure sensor triggered event: {:?}",
                                            e
                                        ),
                                    }
                                } else {
                                    warn!(
                                        "Pressure sensor reset. Temperature: {:.2} °C",
                                        temperature.map(|t| t).unwrap_or(f32::NAN)
                                    );
                                    match post_url(PRESSURE_SENSOR_RESET_URL).await {
                                        Ok(_) => {
                                            info!("Pressure sensor reset event sent successfully")
                                        }
                                        Err(e) => warn!(
                                            "Failed to send pressure sensor reset event: {:?}",
                                            e
                                        ),
                                    }
                                }
                            }
                        }
                    }
                }

                tokio::time::sleep(Duration::from_millis(200)).await;
            }

            #[allow(unreachable_code)]
            Result::<(), EspError>::Ok(())
        }
    };

    let monitor_sensors_and_control_power = async move {
        // True if a dust port is currently being ignored (i.e., treated as always closed).
        // It will need to be closed to reset the ignored state.
        let mut dust_port_ignored = dust_port_channels.iter().map(|_| false).collect::<Vec<_>>();

        // Track the last time each dust port was closed.
        let mut dust_port_last_time_closed = dust_port_channels
            .iter()
            .map(|_| Instant::now())
            .collect::<Vec<_>>();

        loop {
            let t = Instant::now();
            let dt = t.duration_since(last_t);
            last_t = t;
            info!("Loop dt: {:?}", dt);
            info!("Reading sensors...");
            let temperature = temperature_sensor
                .as_mut()
                .and_then(|s| s.measure().map(|v| v as f32 / 16.0));

            match temperature {
                Some(t) => info!("Temperature: {:.2} °C", t),
                None => info!("Temperature: N/A"),
            }

            // Read multiplexer channels
            let pressure_high = multiplexer
                .channel_is_high(high_pressure_signal_channel)
                .unwrap();
            // Hopefully doesn't contradict
            let pressure_low = multiplexer
                .channel_is_high(low_pressure_signal_channel)
                .unwrap();
            let main_power_relay_positive = multiplexer
                .channel_is_high(main_power_relay_channel)
                .unwrap();

            // Gates are open when high
            let gates_open = dust_port_channels
                .iter()
                .map(|&channel| multiplexer.channel_is_high(channel).unwrap())
                .collect::<Vec<_>>();

            // Read dust sensor analog value. This is an approximate distance value. A low value means more dust.
            let dust_sensor_level: u16 = dust_sensor_analog.read().unwrap_or(0);
            let mut powered_on_too_long = false;

            for ((last_time_closed, &open), ignored) in dust_port_last_time_closed
                .iter_mut()
                .zip(gates_open.iter())
                .zip(dust_port_ignored.iter_mut())
            {
                if !open {
                    *last_time_closed = Instant::now();
                    *ignored = false;
                } else if last_time_closed.elapsed() > Duration::from_secs(30 * 60) {
                    // If a gate is open too long, start ignoring the gate automatically.
                    // This will require closing the gate to reset the system and allow the gate to work again (or a power cycle).
                    *ignored = true;
                    powered_on_too_long = true;
                }
            }

            // High temperature safety cut-off
            // The temperature sensor is right above the dust bin, below the filters.
            let temperature_too_high = temperature.map(|t| t > 60.0);

            let force_powered_off_internal = temperature_too_high.unwrap_or(false);

            // Will also keep the machine powered off until all gates have been closed, even if the external relays have reset.
            let force_powered_off_external = !pressure_high;
            let force_powered_off = force_powered_off_internal || force_powered_off_external;

            if force_powered_off {
                for (ignored, &open) in dust_port_ignored.iter_mut().zip(gates_open.iter()) {
                    if open {
                        if !*ignored {
                            warn!("Forcing gate to be ignored, as it was open when the machine was powered off due to a safety relay.");
                        }
                        *ignored = true;
                    }
                }
            }

            let any_gate_open = gates_open
                .iter()
                .zip(dust_port_ignored.iter())
                .any(|(&open, &ignored)| open && !ignored);

            if !any_gate_open {
                last_time_all_gates_closed = Instant::now();
            }

            for (i, &gate) in gates_open.iter().enumerate() {
                info!("Gate {}: {}", i + 1, if gate { "open" } else { "closed" });
            }

            // Delay turning on the machine after opening gates.
            // This furhter reduces noise from unstable gate readings (not sure why they happen, but they do).
            let any_gate_open_delayed =
                last_time_all_gates_closed.elapsed() > Duration::from_millis(250);

            let level = if force_powered_off {
                Level::Low
            } else if any_gate_open_delayed {
                Level::High
            } else {
                Level::Low
            };

            indicator_red.set_duty(if force_powered_off { 1.0 } else { 0.0 })?;
            indicator_green.set_duty(if level == Level::High { 1.0 } else { 0.0 })?;

            // Update brevduva containers
            temperature_container
                .set(temperature.map(|t| NotNan::new(t).unwrap()))
                .await;

            temperature_warning_container
                .set(temperature_too_high)
                .await;

            water_pressure_sensor_container
                .set(Some(pressure_high))
                .await;

            // We control the negative side, and we read the positive side of the main power relay.
            // If both are enabled, the machine is running.
            main_power_relay_container
                .set(Some(level == Level::High && main_power_relay_positive))
                .await;

            dust_sensor_level_container
                .set(Some(dust_sensor_level))
                .await;

            powered_on_too_long_container
                .set(Some(powered_on_too_long && !force_powered_off))
                .await;

            for (gate_container, &open) in gate_containers.iter().zip(gates_open.iter()) {
                gate_container.set(Some(open)).await;
            }

            // Control power to the machine.
            info!("Setting power controller to {:?}", level);
            power_controller_driver.set_level(level)?;
            debug_led.set_duty(if level == Level::High { 1.0 } else { 0.0 })?;

            tokio::time::sleep(Duration::from_millis(50)).await;
        }

        #[allow(unreachable_code)]
        Ok(())
    };

    // Run both tasks concurrently (but not in parallel)
    // This allows the machine's power to still be controlled even
    // if sending alert messages takes a long time due to spotty wifi or similar.
    tokio::try_join!(send_alart_messages, monitor_sensors_and_control_power,).unwrap();
    Ok(())
}
