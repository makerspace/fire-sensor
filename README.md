# Fire sensor for Stockholm Makerspace

This ESP32-based sensor monitors the fire-safety system of the [big dust collector](https://wiki.makerspace.se/Sp%C3%A5nsug,_Felder_RL_125) in the [Stockholm Makerspace](https://makerspace.se) Wood Workshop.
If a fire is detected, a POST request is sent to a URL, which will (presumably) then notify people that something bad is happening.

It monitors:

* A temperature sensor.
* A relay for the pressure sensor on the fire-safety system's water tank.
* A distance sensor, used for measuring the amount of dust in the dust bin.
* A relay for the distance sensor.

## Installation

### Pre-requisites

```bash
# Use Linux. It might be compatible with macOS, but likely not.

# Install rust
open https://rustup.rs/

# Install espflash
cargo install espflash
```

### Flashing the ESP

The ESP supports both flashing via USB, and updates over wifi.

To flash via a connected USB cable, use
```bash
make flash
```

To flash to a powered-on ESP32 module on the same network, use:
```bash
make ota
```

Note: The MAC-address of the ESP32 module is hardcoded in the Makefile.

Note: The ESP32 will connect to the *Stockholm Makerspace Wifi* on startup. You must be on this network to be able to flash it wirelessly.

If the flash failed and the ESP32 cannot start properly, it will automatically revert to the previous firmware.

## Data logging

The ESP32 can connect to an MQTT broker (see config details in `main.rs`) to continuously log all sensor values.
This can also be used to send alerts, or even alerts for if the ESP32 loses power or breaks for some reason.

It will log the following data to the given MQTT topics:

```
sync/devices/{device_id}/status: JSON string, "online" | "offline". Uses MQTT last-will-and-testament to set the status to "offline" if the device goes down.
sync/devices/{device_id}/version: JSON string, "#{git hash} {timestamp}". The git hash of the commit the firmware was last flashed with, and the timestamp of the last build.
sync/dust_collector/{device_id}/temperature: Option<f32>, the temperature in degrees Celcius as given by the attached temperature sensor.
sync/dust_collector/{device_id}/water_pressure: Option<bool>, true when there's positive pressure.
sync/dust_collector/{device_id}/dust_level: Option<u16>, sensor value from the ultrasonic distance sensor.
sync/dust_collector/{device_id}/dust_level_warning_1: Option<bool>, true when the dust level is too high.
sync/dust_collector/{device_id}/dust_level_warning_2: Option<bool>, true when the dust level is very high and the machine will be turned off.
sync/dust_collector/{device_id}/machine_running: Option<bool>, true when the machine is running.
sync/dust_collector/{device_id}/powered_on_too_long: Option<bool>, true when the gates have been opened for too long, and the machine has been turned off automatically.
sync/dust_collector/{device_id}/gates/N/open: Option<bool>, true when the gate is open
```

Where `device_id` is `fire_sensor {MAC ADDRESS OF ESP32}`.
