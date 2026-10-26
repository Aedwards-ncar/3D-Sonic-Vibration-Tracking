# AC001 Tower Accelerometer / Gyro Node

MicroPython firmware for a **Raspberry Pi Pico 2** that reads a **Bosch BNO055** (raw accelerometer + gyroscope) and streams the data as plain ASCII over **RS-485** at 120 Hz. Built for unattended mounting on a tower: it has hardware and software watchdogs, I2C fault recovery, automatic calibration, and a remote command interface so it can be reset, recalibrated, or paused from the far end of the cable.

<!-- Add wiring diagram here -->
<!-- ![Wiring diagram](docs/wiring.png) -->

## Features

- 120 Hz streaming of 3-axis acceleration (m/s²) and 3-axis angular rate (rad/s)
- Half-duplex RS-485 (SP3485) with remote command interface (reset, recalibrate, start/stop stream)
- Automatic accel + gyro calibration at boot and on command, with the last good calibration saved to flash
- Hardware watchdog (WDT) plus software watchdogs for I2C failures, frozen sensor data, and the BNO055 dropping out of its operating mode
- I2C bus-recovery routine (clocks out a stuck slave) and full sensor re-initialisation
- Optional sensor power-cycle or hardware-reset pin for faults software can't clear
- Optional XOR checksum on every line
- Mirrors the stream to USB serial when a terminal is open (never blocks if nothing is listening)
- Onboard LED heartbeat (one short blink every 5 s)
- BOOTSEL button: tap to pause/resume the stream, hold to enter programming mode
- PC viewer script (`ac001_viewer.py`) with live gyro and accel plots

## Hardware

| Part | Notes |
|---|---|
| Raspberry Pi Pico 2 (or Pico 2 W) | MicroPython for RP2350 |
| Adafruit BNO055 breakout | Used in AMG (raw) mode, see below |
| SparkFun SP3485 RS-485 breakout | 3.3 V transceiver |
| MPM3610 5 V buck breakout | 12 V in, 5 V out to Pico VSYS |
| 10 kΩ resistor | Pull-down on GP2 so the RS-485 driver stays off during boot/reset |
| 120 Ω resistor | Bus termination at the far end of the cable |

### Pin map

| Pico 2 pin | Signal | Connects to |
|---|---|---|
| GP4 (pin 6) | I2C0 SDA | BNO055 SDA |
| GP5 (pin 7) | I2C0 SCL | BNO055 SCL |
| GP0 (pin 1) | UART0 TX | SP3485 DI |
| GP1 (pin 2) | UART0 RX | SP3485 RO |
| GP2 (pin 4) | Direction | SP3485 DE + RE (tied together), 10 kΩ pull-down to GND |
| 3V3 OUT (pin 36) | 3.3 V | BNO055 VIN, SP3485 VCC |
| VSYS (pin 39) | 5 V in | MPM3610 5 V output |
| GND | Ground | All grounds common |
| GP6 (pin 9) *(optional)* | Sensor power | BNO055 VIN, instead of 3V3 (`SENSOR_PWR_PIN = 6`) |
| GP7 (pin 10) *(optional)* | Sensor reset | BNO055 RST (`SENSOR_RST_PIN = 7`) |

BNO055 I2C address is `0x28` with the ADR pin unconnected (`0x29` if pulled high; set `BNO_ADDR`).

### RS-485 notes

- **Polarity:** on the SP3485, **A = non-inverting (D+/Tx+/Rx+)** and **B = inverting (D−/Tx−/Rx−)**. Letter labelling is inconsistent between manufacturers, so match **+ to +** and **− to −** rather than A to A. If you see no data or garbage on first power-up, swap the pair.
- **Half-duplex:** one twisted pair carries both directions.
- **Termination:** 120 Ω across the pair at the far end.
- **Biasing:** at the host end, add bias resistors (about 560 Ω to 1 kΩ) pulling the + line up to 3.3 V and the − line down to GND so an idle bus reads as logic high. Many USB-RS485 adapters already include this.
- **Ground:** run a ground reference wire along with the pair if possible.
- **Surge protection:** at height, use RS-485 TVS surge protection at both ends and proper grounding. Consider an isolated transceiver if the ground reference can't be guaranteed.

## Serial output

**Settings:** 230400 baud, 8N1 (change with `BAUD`). Line ending is `\r\n`.

### Data line

```
AC001,ax,ay,az,gx,gy,gz
```

Example:

```
AC001,0.012,-0.008,0.021,0.0003,-0.0001,0.0002
```

| Field | Unit |
|---|---|
| `ax, ay, az` | m/s² |
| `gx, gy, gz` | rad/s |

`AC001` is the sensor ID: `AC` = sensor type, `0` = generation, `01` = sensor number. Change it with `SENSOR_ID`. **This also changes the command prefix**, so commands become `<SENSOR_ID>,RESET` and so on.

If `CHECKSUM = True`, each line ends with `*XX`, where `XX` is the hex XOR of every character before the `*`:

```
AC001,0.012,-0.008,0.021,0.0003,-0.0001,0.0002*5A
```

### Status lines

Lines that aren't six numbers are status messages. A parser should ignore anything it doesn't recognise.

| Message | Meaning |
|---|---|
| `AC001,BOOT,PWR` / `WDT` / `OTHER` | Sent at start-up, with the reset cause |
| `AC001,CAL,OK` / `AC001,CAL,FAIL` | Result of a calibration |
| `AC001,WARN,NOCAL` | Boot calibration rejected (moving) and no saved calibration exists |
| `AC001,WARN,NOCAL,USING_SAVED` | Boot calibration rejected, saved calibration loaded instead |
| `AC001,STREAM,ON` / `OFF` | Stream resumed or paused |
| `AC001,PROGRAM` | Entering programming mode (BOOTSEL hold) |
| `AC001,RESETTING` | Commanded reset in progress |
| `AC001,ERR,I2C` | I2C read failures |
| `AC001,ERR,STUCK` | Sensor data frozen for 2 s |
| `AC001,ERR,MODE` | BNO055 dropped out of its operating mode |
| `AC001,ERR,NOSENSOR` | BNO055 not answering at start-up |
| `AC001,ERR,RESETTING` | Repeated re-init failures, rebooting |

Error lines are rate-limited to one per second.

## Command interface (RS-485)

Send ASCII commands to the Pico over the same pair, ending with `\n` or `\r\n`. Matching is **exact and case-sensitive**, so line noise won't trigger anything.

| Command | Action |
|---|---|
| `AC001,RESET` | Replies `AC001,RESETTING`, then reboots the Pico |
| `AC001,CAL` | Recalibrates accel + gyro (sensor must be still). Replies `AC001,CAL,OK` or `AC001,CAL,FAIL`. Data pauses about 1 s |
| `AC001,STOP` | Pauses the data stream. Commands and status messages still work |
| `AC001,START` | Resumes the data stream |

**Half-duplex timing:** the Pico can only hear commands between its own transmissions, so a command that overlaps a data line is lost. Send each command **repeatedly** (10 or more times over about a second) from the host. Repeating `RESET` or `STOP` is harmless. A repeated `CAL` will just recalibrate again.

Commands entered over the USB serial port are not processed.

## Calibration

The BNO055 runs in **AMG mode** (raw accelerometer, magnetometer, gyro), which allows sensor rates above the 100 Hz limit of its fusion modes. In this mode the chip's own automatic calibration isn't active, so the firmware does its own.

**What gets calibrated**

- **Gyro:** the average rate at rest is subtracted from every reading (zero-rate offset).
- **Accel:** depends on `ACCEL_CAL_MODE`:

| Mode | Behaviour |
|---|---|
| `"ZERO"` *(default)* | Subtracts the at-rest vector, **including gravity**. All three axes read about 0 when still. Works at any mounting angle, but later tilting appears as slow acceleration |
| `"SCALE"` | Keeps gravity. Corrects the gain so the total magnitude at rest is exactly 9.80665 m/s². Can't remove a per-axis offset |
| `"OFF"` | Raw accelerometer data |

**When it runs**

1. **At boot:** automatically, if `CALIBRATE_GYRO = True`. The sensor must stay still for `CAL_SECONDS`.
2. **On command:** `AC001,CAL`.

**Rejecting bad calibrations:** if the sensor moves during the window (gyro peak-to-peak above `CAL_MAX_MOTION_DPS`, or accel above `CAL_MAX_ACCEL`), the run is rejected instead of baking motion into the zero. In `"SCALE"` mode, a magnitude far from 1 g is also rejected as a sensor fault.

**Saved calibration:** each good calibration is written to `cal.txt` on the Pico's flash (only when the values change meaningfully, to spare the flash). If a boot-time calibration is rejected, such as when the mast is swaying, the last saved one is loaded and `AC001,WARN,NOCAL,USING_SAVED` is sent.

> **Remounting the sensor?** A saved `ZERO` calibration is only valid for the orientation it was taken in. Run `AC001,CAL` during calm conditions after any remount, or delete `cal.txt`.

## Watchdogs and fault handling

| Mechanism | Behaviour |
|---|---|
| Hardware watchdog | Resets the Pico if the main loop stalls for `WDT_TIMEOUT_MS` (3 s) |
| I2C read failure | After `REINIT_AFTER` (12) consecutive failures: free the bus, reset and re-initialise the BNO055 |
| Frozen data | Identical raw data for `STUCK_LIMIT` samples (2 s) triggers re-init |
| Mode check | Once a second, confirms the BNO055 is still in AMG mode. If not, re-initialises it |
| Repeated failure | After `MAX_REINIT_FAILS` (5) failed re-inits in a row, the Pico reboots itself |
| Optional power / reset line | Re-init power-cycles or hard-resets the BNO055 if `SENSOR_PWR_PIN` or `SENSOR_RST_PIN` is set |
| Driver safety | The RS-485 driver defaults to listen mode (GP2 low, with a pull-down) so a booting or crashed Pico can't hold the bus |

A full re-init takes 1 to 2 seconds with no data output. The hardware watchdog is fed throughout.

## Button and LED

| Action | Result |
|---|---|
| LED, one short blink every 5 s | Normal heartbeat |
| LED solid on | Stream paused |
| **Tap BOOTSEL** | Pause / resume the stream |
| **Hold BOOTSEL for 3 s** | Programming mode: sends `AC001,PROGRAM`, flashes the LED, reboots, and idles with the watchdog **not** armed |

**Programming mode** is how to reprogram the board with the watchdog enabled. After the reboot, the LED blinks fast and the script idles. Press **Stop** in Thonny to get the REPL and upload new files. The mode flag is consumed on that boot, so the next reset or power cycle runs normally.

While developing, set `USE_WDT = False` in `main.py`, because an armed watchdog can't be stopped. Reading BOOTSEL briefly pauses flash access, so the firmware samples it about 30 times a second. If your MicroPython build lacks `rp2.bootsel_button()`, the button is disabled and everything else works.

## Configuration

All settings are at the top of `main.py`.

| Variable | Default | Description |
|---|---|---|
| `SENSOR_ID` | `"AC001"` | Prefix on every line and command |
| `SAMPLE_HZ` | `120` | Output rate |
| `BAUD` | `230400` | RS-485 baud rate. `115200` also works; the host must match |
| `EOL` | `"\r\n"` | Line ending |
| `CHECKSUM` | `False` | Append `*XX` XOR checksum |
| `USE_WDT` | `True` | Hardware watchdog. Set `False` while developing |
| `WDT_TIMEOUT_MS` | `3000` | Watchdog timeout |
| `BNO_ADDR` | `0x28` | BNO055 I2C address (`0x29` if ADR is high) |
| `USE_EXT_CRYSTAL` | `True` | Use the breakout's 32 kHz crystal |
| `CALIBRATE_GYRO` | `True` | Auto-calibrate at boot and on command (covers accel too) |
| `ACCEL_CAL_MODE` | `"ZERO"` | `"ZERO"`, `"SCALE"`, or `"OFF"` |
| `CAL_SECONDS` | `1.0` | How long the sensor must be still to calibrate |
| `CAL_MAX_MOTION_DPS` | `3.0` | Gyro peak-to-peak limit during calibration |
| `CAL_MAX_ACCEL` | `0.5` | Accel peak-to-peak limit (m/s²) during calibration |
| `CAL_FILE` | `"cal.txt"` | Saved calibration file |
| `SENSOR_PWR_PIN` | `None` | GPIO that powers the BNO055 (enables power-cycle recovery) |
| `SENSOR_RST_PIN` | `None` | GPIO wired to BNO055 RST (enables hardware reset) |
| `USB_OUTPUT` | `True` | Mirror the stream to USB serial when a terminal is open |
| `LED_PERIOD_MS` / `LED_ON_MS` | `5000` / `100` | Heartbeat blink timing |
| `USE_BUTTON` | `True` | Enable the BOOTSEL button |
| `PROGRAM_HOLD_MS` | `3000` | Hold time for programming mode |
| `SDA_PIN`, `SCL_PIN` | `4`, `5` | I2C0 pins |
| `DE_RE_PIN` | `2` | RS-485 direction pin |
| `STUCK_LIMIT` | `SAMPLE_HZ * 2` | Identical samples before a frozen sensor is declared |
| `REINIT_AFTER` | `12` | Consecutive I2C failures before re-init |
| `MAX_REINIT_FAILS` | `5` | Failed re-inits before a full reboot |

### Sensor ranges and filtering

The sensor ranges and filter bandwidth are set in `start_sensor()`, not in the settings block:

| Setting | Value |
|---|---|
| Accelerometer | ±4 g, 125 Hz bandwidth (`REG_ACC_CONFIG = 0x11`) |
| Gyroscope | ±500 °/s, 64 Hz bandwidth (`REG_GYR_CONFIG_0 = 0x32`) |

The output units are fixed at m/s² and rad/s via `UNIT_SEL`. If you change a range, check the BNO055 datasheet for the matching register value. The scale constants (`ACCEL_SCALE`, `GYRO_SCALE`) apply to the unit selection, so they don't change with range.

## Installation

1. Flash MicroPython for the **Pico 2** from [micropython.org](https://micropython.org/download/RPI_PICO2/) (hold BOOTSEL, plug in USB, drag the UF2 onto the drive).
2. Copy `main.py` to the Pico with Thonny. It runs automatically at every power-up.
3. Connect the hardware as in the pin map above.
4. Open the Pico's USB serial port in a terminal (any baud rate), or connect an RS-485 adapter at the other end of the cable at 230400 8N1.

## PC viewer

`ac001_viewer.py` plots gyro (left) and accel (right) live.

```
pip install pyserial matplotlib numpy
python ac001_viewer.py                                     # auto-detect Pico on USB
python ac001_viewer.py --port COM5 --baud 230400           # RS-485 adapter on Windows
python ac001_viewer.py --port /dev/ttyUSB0 --baud 230400   # RS-485 adapter on Linux/macOS
python ac001_viewer.py --demo                              # synthetic data, no hardware
```

Keys: **space** pauses, **c** clears. The viewer verifies checksums if enabled, shows status messages and the measured sample rate, and reconnects automatically if the port drops. The viewer matches the ID `AC001`, so edit `SENSOR_ID` at the top of the script if you change the firmware's ID.

## Limitations

- Raw accelerometer and gyro only. The BNO055's fused orientation output is not used, because its fusion modes are limited to 100 Hz.
- The USB serial port is output-only. Commands are accepted over RS-485 only.
- A command can be missed if it arrives while the Pico is transmitting (half-duplex), so repeat commands.
- Calibration assumes the mast is still. Swaying at boot falls back to the saved calibration.
