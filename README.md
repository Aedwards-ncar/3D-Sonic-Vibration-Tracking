# AC001 Tower Accelerometer / Gyro Node

MicroPython firmware for a **Raspberry Pi Pico 2** that reads a **Bosch BNO055** and streams ASCII data over **RS-485**. It has two switchable output profiles, changed with a command: a **60 Hz chip profile** (the default, straight from the BNO055 with no Pico calibration) and a **120 Hz raw profile** (raw sensor data with Pico calibration). Built for unattended mounting on a tower: it has hardware and software watchdogs, I2C fault recovery, automatic calibration, and a remote command interface so it can be reset, recalibrated, or paused from the far end of the cable.

<!-- Add wiring diagram here -->
<!-- ![Wiring diagram](docs/wiring.png) -->

## Features

- Two output profiles, switched by an RS-485 command: **60 Hz chip** (default) and **120 Hz raw**
- 3-axis acceleration (m/s²) and 3-axis angular rate (rad/s) in both profiles
- Baud rate capped at 115200 in firmware
- Half-duplex RS-485 (SP3485) with remote command interface (switch profile, reset, recalibrate, start/stop stream)
- Automatic accel + gyro calibration in the 120 Hz raw profile, at switch-over and on command, with the last good calibration saved to flash
- Hardware watchdog (WDT) plus software watchdogs for I2C failures, frozen sensor data, and the BNO055 dropping out of its operating mode
- I2C bus-recovery routine (clocks out a stuck slave) and full sensor re-initialisation
- Optional sensor power-cycle or hardware-reset pin for faults software can't clear
- Optional XOR checksum on every line
- Mirrors the stream to USB serial when a terminal is open (never blocks if nothing is listening)
- Onboard LED heartbeat (one short blink every 5 s)
- BOOTSEL button: tap to pause/resume the stream, hold to enter programming mode
- PC viewer script (`ac001_viewer.py`) with live gyro and accel plots for either profile

## Enclosure (CAD files)

CAD files for an enclosure that mounts this sensor to the sonic anemometer are shared in this repository.

- **Location:** `[path to CAD folder, e.g. /cad]`
- **Formats:** `[e.g. STEP and STL]`
- **Fits:** `[sonic anemometer model / mounting interface]`
- **Printing notes:** `[material, orientation, infill, anything that matters outdoors]`

Mounting notes:

- Mount the enclosure **rigidly**. A loose sensor adds its own motion to the data.
- Record how the sensor's X, Y, and Z axes line up with the sonic's axes: `[axis mapping]`.
- **Recalibrate after mounting.** Once the enclosure is on the sonic and the mast is calm, send `AC001,CAL` (see [Calibration](#calibration)) so the stored zero matches the final orientation.
- Seal the enclosure and use cable glands for the RS-485 and power wiring.

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

**Settings:** **115200 baud**, 8N1, line ending `\r\n`. The baud rate is capped at 115200 in firmware: `BAUD` is clamped to `MAX_BAUD`, so a higher value is ignored.

### Profiles

Only one profile is active at a time. The Pico starts in the **60 Hz chip profile** and switches with a command (see [Command interface](#command-interface-rs-485)).

| | 60 Hz chip profile *(default)* | 120 Hz raw profile |
|---|---|---|
| Rate | 60 Hz | 120 Hz |
| Data line tag | `C` | `R` |
| BNO055 mode | Fusion (`IMU`, accel + gyro) | `AMG` (raw sensors) |
| Accel | Chip **linear acceleration** (gravity removed by the chip) | Raw accel register |
| Gyro | Chip gyro register | Raw gyro register |
| Pico calibration | **None.** Values are exactly what the chip reports | **Applied** by default (`RAW_APPLY_PICO_CAL`) |
| Chip status line | `AC001,STAT,...` every 10 s | None |

Set `CHIP_ACCEL = "ACC"` to make the 60 Hz profile's accel the chip's accel register (gravity included) instead of linear acceleration.

The BNO055 can only run one operating mode at a time, so changing profile re-initialises the chip in the other mode. **Expect no data for about 1 to 2 seconds** while it switches, and the 120 Hz profile then spends about another second on its still-sensor calibration.

### Data lines

```
AC001,C,ax,ay,az,gx,gy,gz      60 Hz chip profile
AC001,R,ax,ay,az,gx,gy,gz      120 Hz raw profile
```

Example:

```
AC001,C,0.00,-0.01,0.00,0.001,0.001,-0.001
```

| Field | Unit |
|---|---|
| `ax, ay, az` | m/s² (2 decimals) |
| `gx, gy, gz` | rad/s (3 decimals) |

`AC001` is the sensor ID: `AC` = sensor type, `0` = generation, `01` = sensor number. Change it with `SENSOR_ID`. **This also changes the command prefix**, so commands become `<SENSOR_ID>,RESET` and so on.

If `CHECKSUM = True`, each data line ends with `*XX`, where `XX` is the hex XOR of every character before the `*`.

### Notes on the chip profile

- In fusion mode the BNO055 fixes its own settings: accelerometer range ±4 g, bandwidth chosen by the chip, and data updating at **100 Hz**. The 60 Hz profile samples that at 60 Hz.
- The chip's calibration profile is lost at power-up. Its gyro calibration settles quickly while the sensor is still. Accelerometer calibration normally needs the sensor held in several orientations, which isn't practical on a mast, so the accel part may stay partly uncalibrated. Watch the `STAT` lines (below).
- The firmware doesn't save, restore, or modify the chip's calibration.

### Notes on the raw profile

- In `AMG` mode the ranges and bandwidth are set by the firmware (see [Sensor ranges and filtering](#sensor-ranges-and-filtering)) and the sensor updates faster than 120 Hz, so each line is a fresh sample.
- Pico calibration (zero or scale for accel, bias for gyro) is applied. See [Calibration](#calibration).

### Status lines

Any line that isn't a `C` or `R` data line is a status message. A parser should only treat `AC001,C,...` and `AC001,R,...` as data and ignore anything else.

| Message | Meaning |
|---|---|
| `AC001,BOOT,PWR` / `WDT` / `OTHER` | Sent at start-up, with the reset cause |
| `AC001,PROFILE,60` / `120` | Active profile. Sent at boot, after a profile change, and in reply to a query |
| `AC001,STAT,sys,gyr,acc,mag` | BNO055 internal calibration status, each 0 (none) to 3 (full). 60 Hz profile only, every `CHIP_STATUS_S` seconds. Describes the chip's own calibration |
| `AC001,CAL,OK` / `AC001,CAL,FAIL` | Result of a Pico calibration (120 Hz profile) |
| `AC001,CAL,NA` | `CAL` was sent while in the 60 Hz profile, where Pico calibration isn't used |
| `AC001,WARN,NOCAL` | Boot calibration rejected (moving) and no saved calibration exists |
| `AC001,WARN,NOCAL,USING_SAVED` | Boot calibration rejected, saved calibration loaded instead |
| `AC001,STREAM,ON` / `OFF` | Stream resumed or paused |
| `AC001,PROGRAM` | Entering programming mode (BOOTSEL hold) |
| `AC001,RESETTING` | Commanded reset in progress |
| `AC001,ERR,I2C` | I2C read failures |
| `AC001,ERR,STUCK` | Sensor data frozen for 2 s |
| `AC001,ERR,MODE` | BNO055 dropped out of its configured mode |
| `AC001,ERR,NOSENSOR` | BNO055 not answering at start-up |
| `AC001,ERR,RESETTING` | Repeated re-init failures, rebooting |

Error lines are rate-limited to one per second.

## Command interface (RS-485)

Send ASCII commands to the Pico over the same pair, ending with `\n` or `\r\n`. Matching is **exact and case-sensitive**, so line noise won't trigger anything.

| Command | Action |
|---|---|
| `AC001,PROFILE,60` | Switch to the 60 Hz chip profile. Replies `AC001,PROFILE,60`. Data pauses 1 to 2 s |
| `AC001,PROFILE,120` | Switch to the 120 Hz raw profile and run its calibration. Replies `AC001,PROFILE,120`. Data pauses 2 to 3 s |
| `AC001,PROFILE,?` | Reports the active profile |
| `AC001,RESET` | Replies `AC001,RESETTING`, then reboots the Pico |
| `AC001,CAL` | Recalibrates accel + gyro (sensor must be still). Replies `AC001,CAL,OK` or `AC001,CAL,FAIL`. Only meaningful in the 120 Hz profile. In the 60 Hz profile it replies `AC001,CAL,NA` |
| `AC001,STOP` | Pauses the data stream. Commands and status messages still work |
| `AC001,START` | Resumes the data stream |

**Half-duplex timing:** the Pico can only hear commands between its own transmissions, so a command that overlaps a data line is lost. Send each command **repeatedly** (every 10 to 20 ms for about 1 second, a little longer in the 120 Hz profile, where the bus is busier) from the host. Repeating `PROFILE`, `RESET`, `STOP`, or `START` is harmless. A repeated `CAL` will just recalibrate again.

After a `PROFILE` change the Pico is busy re-initialising the sensor for a second or two, and any repeats that arrive meanwhile are handled only after it finishes. They are harmless: the same profile is simply confirmed again.

The profile resets to `DEFAULT_PROFILE` (60 Hz) at every boot, including a watchdog reset, unless you set `PERSIST_PROFILE = True`, in which case the last commanded profile is saved to flash and restored.

Commands entered over the USB serial port are not processed.

## Calibration

There are two separate calibrations:

- **Pico calibration** (this section) corrects the **120 Hz raw profile** in software, controlled by `RAW_APPLY_PICO_CAL`. It is not used in the 60 Hz profile.
- **Chip calibration** is the BNO055's own, which feeds the **60 Hz chip profile**. The firmware does not touch or correct it, and only reports its status in the `STAT` lines.

The rest of this section describes the Pico calibration.

**What gets calibrated**

- **Gyro:** the average rate at rest is subtracted from every reading (zero-rate offset).
- **Accel:** depends on `ACCEL_CAL_MODE`:

| Mode | Behaviour |
|---|---|
| `"ZERO"` *(default)* | Subtracts the at-rest vector, **including gravity**. All three axes read about 0 when still. Works at any mounting angle, but later tilting appears as slow acceleration |
| `"SCALE"` | Keeps gravity. Corrects the gain so the total magnitude at rest is exactly 9.80665 m/s². Can't remove a per-axis offset |
| `"OFF"` | Raw accelerometer data |

**When it runs**

1. **When the 120 Hz profile starts**, whether at boot (if `DEFAULT_PROFILE = 120`) or after a `PROFILE,120` command. The sensor must stay still for `CAL_SECONDS`.
2. **On command:** `AC001,CAL`, while in the 120 Hz profile.

**Rejecting bad calibrations:** if the sensor moves during the window (gyro peak-to-peak above `CAL_MAX_MOTION_DPS`, or accel above `CAL_MAX_ACCEL`), the run is rejected instead of baking motion into the zero. In `"SCALE"` mode, a magnitude far from 1 g is also rejected as a sensor fault.

**Saved calibration:** each good calibration is written to `cal.txt` on the Pico's flash (only when the values change meaningfully, to spare the flash). If a calibration is rejected, such as when the mast is swaying when you switch to the 120 Hz profile, the last saved one is loaded and `AC001,WARN,NOCAL,USING_SAVED` is sent. If there's no saved file yet, the profile runs uncalibrated and sends `AC001,WARN,NOCAL`.

> **Remounting the sensor?** A saved `ZERO` calibration is only valid for the orientation it was taken in. Run `AC001,CAL` in the 120 Hz profile during calm conditions after any remount, or delete `cal.txt`.

## Watchdogs and fault handling

| Mechanism | Behaviour |
|---|---|
| Hardware watchdog | Resets the Pico if the main loop stalls for `WDT_TIMEOUT_MS` (3 s) |
| I2C read failure | After `REINIT_AFTER` (12) consecutive failures: free the bus, reset and re-initialise the BNO055 |
| Frozen data | Identical raw data for `STUCK_SECONDS` (2 s) triggers re-init |
| Mode check | Once a second, confirms the BNO055 is still in its configured mode. If not, re-initialises it |
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
| `DEFAULT_PROFILE` | `60` | Profile at boot: `60` (chip) or `120` (raw) |
| `PERSIST_PROFILE` | `False` | Save the commanded profile to flash and restore it at boot |
| `MAX_BAUD` | `115200` | Hard ceiling on the baud rate |
| `BAUD` | `115200` | RS-485 baud rate, clamped to `MAX_BAUD`. The host must match |
| `FUSION_MODE` | `"IMU"` | Fusion mode for the 60 Hz profile: `"IMU"` (accel + gyro) or `"NDOF"` (adds magnetometer, poor choice near steel or electronics) |
| `CHIP_ACCEL` | `"LIA"` | 60 Hz profile accel: `"LIA"` (gravity removed by chip) or `"ACC"` (gravity included) |
| `RAW_APPLY_PICO_CAL` | `True` | Apply Pico calibration in the 120 Hz profile. `False` sends it uncorrected. The 60 Hz profile never gets it |
| `ACCEL_DECIMALS` / `GYRO_DECIMALS` | `2` / `3` | Decimals printed. These match the chip's resolution, and fewer digits keep the lines short enough for 115200 baud |
| `CHIP_STATUS_S` | `10` | 60 Hz profile: seconds between `STAT` lines. `0` turns them off |
| `TAG_RAW` / `TAG_CHIP` | `"R"` / `"C"` | Data line tags for the 120 Hz / 60 Hz profiles |
| `EOL` | `"\r\n"` | Line ending |
| `CHECKSUM` | `False` | Append `*XX` XOR checksum |
| `USE_WDT` | `True` | Hardware watchdog. Set `False` while developing |
| `WDT_TIMEOUT_MS` | `3000` | Watchdog timeout |
| `BNO_ADDR` | `0x28` | BNO055 I2C address (`0x29` if ADR is high) |
| `USE_EXT_CRYSTAL` | `True` | Use the breakout's 32 kHz crystal |
| `CALIBRATE_GYRO` | `True` | Auto-calibrate at boot and on command (covers accel too) |
| `ACCEL_CAL_MODE` | `"ZERO"` | `"ZERO"`, `"SCALE"`, or `"OFF"` |
| `CAL_SECONDS` | `1.0` | How long the sensor must be still for the Pico calibration |
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
| `STUCK_SECONDS` | `2` | Seconds of identical samples before a frozen sensor is declared |
| `REINIT_AFTER` | `12` | Consecutive I2C failures before re-init |
| `MAX_REINIT_FAILS` | `5` | Failed re-inits before a full reboot |

### Sensor ranges and filtering

In the 120 Hz raw profile (`AMG` mode) the ranges and bandwidths are set in `start_sensor()`:

| Setting | Value |
|---|---|
| Accelerometer | ±4 g, 125 Hz bandwidth (`REG_ACC_CONFIG = 0x11`) |
| Gyroscope | ±500 °/s, 64 Hz bandwidth (`REG_GYR_CONFIG_0 = 0x32`) |

In the 60 Hz chip profile (fusion mode) the chip sets its own ranges and bandwidth, and these writes are skipped. The output units are fixed at m/s² and rad/s via `UNIT_SEL`, so `ACCEL_SCALE` and `GYRO_SCALE` don't change with range.

## Installation

1. Flash MicroPython for the **Pico 2** from [micropython.org](https://micropython.org/download/RPI_PICO2/) (hold BOOTSEL, plug in USB, drag the UF2 onto the drive).
2. Copy `main.py` to the Pico with Thonny. It runs automatically at every power-up.
3. Connect the hardware as in the pin map above.
4. Open the Pico's USB serial port in a terminal (any baud rate), or connect an RS-485 adapter at the other end of the cable at 115200 8N1.

## PC viewer

`ac001_viewer.py` plots live data in a 2×2 grid: the top row is the 60 Hz chip profile and the bottom row is the 120 Hz raw profile, with gyro on the left and accel on the right. Only the active profile's row updates, and the other says "not the active profile". The viewer only listens. To change profile, send the `PROFILE` command from a separate terminal or tool on the RS-485 adapter.

```
pip install pyserial matplotlib numpy
python ac001_viewer.py                                     # auto-detect Pico on USB
python ac001_viewer.py --port COM5                         # RS-485 adapter on Windows (115200 default)
python ac001_viewer.py --port /dev/ttyUSB0                 # RS-485 adapter on Linux/macOS
python ac001_viewer.py --demo                              # synthetic data, no hardware
```

Keys: **space** pauses, **c** clears. The viewer verifies checksums if enabled, shows status messages and the measured rate of each profile, and reconnects automatically if the port drops. The viewer matches the ID `AC001`, so edit `SENSOR_ID` at the top of the script if you change the firmware's ID.

## Limitations

- Only accel and gyro are streamed. Orientation (Euler/quaternion) is not output.
- The BNO055 can only run one mode at a time, so the profiles can't run together, and switching interrupts the data for 1 to 3 seconds.
- In the 60 Hz profile the chip's data updates at 100 Hz internally and is sampled at 60 Hz.
- The chip's own calibration is not saved or restored by this firmware. It restarts at every power-up.
- The USB serial port is output-only. Commands are accepted over RS-485 only.
- A command can be missed if it arrives while the Pico is transmitting (half-duplex), so repeat commands.
- Pico calibration assumes the mast is still. If it's swaying when the 120 Hz profile starts, the saved calibration is used instead.
