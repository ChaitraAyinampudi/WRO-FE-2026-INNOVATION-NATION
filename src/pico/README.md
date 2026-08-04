# Raspberry Pi Pico Sensor System

This folder contains the MicroPython code used by the Raspberry Pi Pico sensor board.

## Purpose

The Pico reads the WT901 gyroscope and the VL53L0X Time-of-Flight sensors, then sends the newest sensor values to the main Raspberry Pi through USB serial.

The Raspberry Pi uses this data for gyro angle tracking, wall detection, wall following, and parking alignment.

## Hardware

- Raspberry Pi Pico
- WT901 gyroscope sensor
- VL53L0X Time-of-Flight sensors
- USB connection to the Raspberry Pi

## WT901 Wiring

| WT901 pin | Pico pin |
| --------- | -------- |
| VCC       | 3V3(OUT) |
| GND       | GND      |
| SDA       | GP4      |
| SCL       | GP5      |

WT901 settings:

| Setting | Value |
| ------- | ----- |
| I2C bus | I2C0  |
| Address | `0x50` |
| Rate    | About 100 Hz |

## VL53L0X Wiring

| Sensor | XSHUT pin | Address | Status |
| ------ | --------- | ------- | ------ |
| A      | GP18      | `0x2A` | Active |
| B      | GP19      | `0x2B` | Active |
| C      | GP20      | `0x2C` | Active |
| D      | GP21      | Off    | Disabled |

The ToF sensors share:

- I2C1
- SDA: GP2
- SCL: GP3
- Frequency: 400 kHz

## Code

### `main.py`

The main Pico program:

- Starts the WT901 gyro reader on core1
- Reads ToF sensors A, B, and C on core0
- Keeps sensor D turned off
- Blinks the Pico LED while the program is alive
- Accepts restart and gyro-reset commands from the Raspberry Pi
- Sends one combined sensor line through USB serial

Output format:

```text
Angle=X,A=X,AS=X,B=X,BS=X,C=X,CS=X
```

Example output:

```text
Angle=12.50,A=35.2,AS=0,B=100.0,BS=0,C=82.4,CS=0
```

## Sensor Status Values

| Status | Meaning |
| ------ | ------- |
| `0`    | Valid reading |
| `9`    | Invalid, out of range, read error, or sensor missing |

Invalid ToF readings are sent as:

```text
-1.0
```

## USB Commands

The Raspberry Pi can send these commands to the Pico:

| Command | Action |
| ------- | ------ |
| `restart main` | Reboots the Pico |
| `restart_main` | Reboots the Pico |
| `restart` | Reboots the Pico |
| `reboot` | Reboots the Pico |
| `reset gyro` | Resets and recalibrates the gyro |
| `reset_gyro` | Resets and recalibrates the gyro |

## Required Files

```text
pico/
├── README.md
├── main.py
└── vl53l0x.py
```

`vl53l0x.py` is the MicroPython driver used to communicate with the VL53L0X sensors.
