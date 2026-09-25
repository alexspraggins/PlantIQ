# PlantIQ / FloraSense Firmware

PlantIQ, also referred to as FloraSense, is an ESP32-based plant-care controller. The firmware combines sensor readings, automatic watering, grow-light control, BLE provisioning, Wi-Fi connectivity, backend configuration, and telemetry.

This repository is a cleaned-up and organized version of the original file structure. The original folder contained dated experiments, duplicate sketches, test programs, and embedded credentials. The code has been grouped by purpose, converted from Arduino `.ino` sketches to normal `.cpp` files, and stripped of real Wi-Fi passwords and device keys.

This is a historical development snapshot, not a production-ready release.

## Main firmware

The primary firmware candidate is [`firmware/main/PlantIQ_Controller/PlantIQ_Controller.cpp`](firmware/main/PlantIQ_Controller/PlantIQ_Controller.cpp).

It provides:

- BLE device discovery and provisioning
- Wi-Fi credential and device-key storage in ESP32 NVS
- backend assignment-status checks
- plant-specific care-configuration retrieval
- VH400-style soil-moisture measurement and automatic watering
- BH1750 light measurement and closed-loop LED control
- BME280 temperature, humidity, and pressure sensing
- telemetry posting after a valid plant assignment and care configuration are available
- safety limits for watering, stale sensors, and invalid control conditions

The controller still uses Arduino `setup()` and `loop()` entry points. It should be built as an ESP32 Arduino-framework C++ source file using PlatformIO, Arduino CLI, or an equivalent build system.

## Hardware assumptions

The current main firmware assumes an ESP32 with:

| Function | GPIO / interface |
| --- | --- |
| Soil-moisture ADC | GPIO 34 |
| Pump control | GPIO 27 |
| White LED PWM | GPIO 25 |
| Red/blue LED PWM | GPIO 26 |
| I2C SDA | GPIO 21 |
| I2C SCL | GPIO 22 |
| BH1750 | I2C |
| BME280 | I2C, address `0x76` or `0x77` |

Confirm the wiring and electrical limits before powering hardware. Pin assignments and control constants are defined near the relevant subsystem sections in the main source file.

## Software requirements

The main controller expects:

- ESP32 Arduino core 2.0.17-era APIs
- ArduinoJson
- hp_BH1750
- Adafruit BME280 Library
- ESP32 Wi-Fi, BLE, Preferences, Wire, HTTP, and mbedTLS support

The archived code targets multiple ESP32 API generations. Some files use the older `ledcSetup()` / `ledcAttachPin()` API, while newer experiments use `ledcAttach()`.

## Provisioning and runtime flow

At startup, the device advertises as a PlantIQ BLE device. A companion client provisions the device with Wi-Fi credentials and a device key. The firmware stores those values locally, connects to Wi-Fi, and checks the backend for an assignment.

Control is intentionally gated until the device has both:

1. a valid plant assignment, and
2. valid plant-specific care parameters.

When the gate is closed, the pump and lights are placed in a safe state and plant-specific climate control is disabled. Once the gate opens, the controller resumes sensing, watering, lighting, and telemetry.

The public repository does not contain working Wi-Fi passwords, backend device keys, or the AES key used by encrypted BLE provisioning. Configure those values locally and never commit them. The encrypted provisioning code currently contains an intentionally blank AES-key placeholder and will not work until a matching local key is supplied.

## Repository layout

```text
firmware/main/       Current merged controller candidate
firmware/reference/  Standalone moisture and light controllers
tests/sensors/       Sensor and moisture experiments
tests/actuators/     LED and pump experiments
tests/backend/       Wi-Fi, BLE, and backend experiments
tests/integration/   Cross-subsystem experiments
archive/             Older control-logic revisions and duplicate prototypes
```

Each test or reference controller is a standalone source file. They are preserved for development history and comparison; they are not all intended to be compiled together.

## Build notes

The repository does not yet include a complete PlatformIO or Arduino CLI build configuration. Before compiling, select an ESP32 board, install the libraries listed above, and configure the local backend/provisioning values required by the specific source file.

Because the original `.ino` files were converted to `.cpp`, Arduino IDE users should use an appropriate C++ project layout or restore a local sketch wrapper if needed. Do not rename files back in the public repository unless required by the chosen build tool.

## Security note

Credentials were present in the original development files. Rotate any Wi-Fi passwords, device keys, backend tokens, or provisioning keys that were used with those files before publishing or reusing the system.

The cleanup removes sensitive values from the current working tree, but it cannot revoke credentials that may have been copied elsewhere.

## Current status

This cleanup organizes the historical code and makes it suitable for repository review. It does not claim that the firmware has been compiled against the current toolchain, electrically tested, or validated on a complete PlantIQ/FloraSense hardware assembly.
