# AC001 Tower Accelerometer / Gyro Node

MicroPython firmware for a **Raspberry Pi Pico 2** that reads a **Bosch BNO055** and streams ASCII data over **RS-485**. It has two switchable output profiles, changed with a command: a **60 Hz chip profile** (the default, straight from the BNO055 with no Pico calibration) and a **120 Hz raw profile** (raw sensor data with Pico calibration). Built for unattended mounting on a tower: it has hardware and software watchdogs, I2C fault recovery, automatic calibration, and a remote command interface so it can be reset, recalibrated, or paused from the far end of the cable.

**[Wiring diagram (PDF)](Wiring_Diagram.pdf)**

## Features

- Two output profiles, switched by an RS-485 command: **60 Hz chip** (default) and **120 Hz raw**
- 3-axis acceleration (m/s²) and 3-axis angular rate (rad/s) in both profiles
- Baud rate capped at 115200 in firmware
- Half-duplex RS-485 (SP3485) with a short-form command interface (`SID`, `TYP`, `MFD`, `SSN`, `BAU`, `PRF`, `CAL`, `TXO`, ...) and a **command mode** that keeps the bus free so commands always get through
- Identity and settings are saved to flash: sensor type and number (e.g. `AC001`), manufacture date, serial/name, baud rate, profile, calibration on/off, send-only mode
- Automatic accel + gyro calibration in the 120 Hz raw profile, at switch-over and on command, with the last good calibration saved to flash
- Hardware watchdog (WDT) plus software watchdogs for I2C failures, frozen sensor data, and the BNO055 dropping out of its operating mode
- I2C bus-recovery routine (clocks out a stuck slave) and full sensor re-initialisation
- Optional sensor power-cycle or hardware-reset pin for faults software can't clear
- Optional XOR checksum on every line
- USB serial output, **off by default**, switched on and off with a command (never blocks if nothing is listening). The same commands also work when typed over USB
- Onboard LED heartbeat (one short blink every 5 s)
- BOOTSEL button: tap to pause/resume the stream, hold to enter programming mode
- PC viewer script (`ac001_viewer.py`) with live gyro and accel plots for either profile

## Enclosure (CAD files)

CAD files for an enclosure that mounts this sensor to the sonic anemometer are shared in this repository.

- **Location:** `[path to CAD folder, e.g. /cad]`
- **Formats:** `Step files and 3mf for polymaker ASA on the Core one + gen2 (prusa slicer 3.0)`
- **Fits:** `Sonic anemometer CAST 3A, `
- **Printing notes:** `[material, orientation, infill, anything that matters outdoors]`

Mounting notes:

- Mount the enclosure **rigidly**. A loose sensor adds its own motion to the data.
- Record how the sensor's X, Y, and Z axes line up with the sonic's axes: `[axis mapping]`.
- **Recalibrate after mounting.** Once the enclosure is on the sonic and the mast is calm, send `CAL NOW` (see [Calibration](#calibration)) so the stored zero matches the final orientation.
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
- **Troubleshooting garbled data:** run `rs485_test.py` from Thonny. It holds the driver on and sends a fixed line at 60 Hz with no sensor involved. If the receiver still shows garbage, the problem is the bus, wiring, baud, or the receiver's port settings. If it's clean, use `TX_ONLY = True` or the timed driver release in `main.py`.
- **Surge protection:** at height, use RS-485 TVS surge protection at both ends and proper grounding. Consider an isolated transceiver if the ground reference can't be guaranteed.

## Serial output

**Settings:** **115200 baud** by default, 8N1, line ending `\r\n`. Change it with the `BAU` command (9600, 19200, 38400, 57600 or 115200). The firmware never goes above 115200, and refuses a rate that can't carry the current profile.

### Profiles

Only one profile is active at a time. The Pico starts in the **60 Hz chip profile** and switches with a command (see [Command interface](#command-interface-rs-485)).

| | 60 Hz chip profile *(default)* | 120 Hz raw profile |
|---|---|---|
| Rate | 60 Hz | 120 Hz |
| Data line tag | `C` | `R` |
| BNO055 mode | Fusion (`NDOF`: accel + gyro + magnetometer) | `AMG` (raw sensors) |
| Accel | Chip **linear acceleration** (gravity removed by the chip) | Raw accel register |
| Gyro | Chip gyro register | Raw gyro register |
| Pico calibration | **None.** Values are exactly what the chip reports | **Applied** while calibration is on (`CAL ON`, the default) |
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

`AC001` is the sensor ID: `AC` = sensor type, `0` = generation, `01` = sensor number. Change them with the commands `TYP` (the two letters) and `SID` (the three digits), for example `TYP GS` and `SID 002` give `GS002`. The new ID is used on every data and status line straight away. (The factory values come from `DEFAULT_TYPE` and `DEFAULT_SID` in `main.py`.) **If your NIDAS catalog entry spells out `AC001,C,` in its `scanfFormat`, it will stop parsing when the ID changes. Use `%*[^,],C,%f,...` instead so any ID is accepted.**

If `CHECKSUM = True`, each data line ends with `*XX`, where `XX` is the hex XOR of every character before the `*`.

### Notes on the chip profile

- The default fusion mode is **`NDOF`**, which adds the magnetometer so the chip knows magnetic north. That is what makes the `DIR` compass possible (see [Compass and tilt](#compass-and-tilt-dir)). Set `FUSION_MODE = "IMU"` for accel + gyro only; `DIR` then gives pitch and roll but no heading.
- In fusion mode the BNO055 fixes its own settings: accelerometer range ±4 g, bandwidth chosen by the chip, and data updating at **100 Hz**. The 60 Hz profile samples that at 60 Hz.
- The chip's calibration profile is lost at power-up. Its gyro calibration settles quickly while the sensor is still. Accelerometer calibration normally needs the sensor held in several orientations, which isn't practical on a mast, so the accel part may stay partly uncalibrated. Watch the `STAT` lines (below).
- The firmware doesn't save, restore, or modify the chip's calibration.

### Notes on the raw profile

- In `AMG` mode the ranges and bandwidth are set by the firmware (see [Sensor ranges and filtering](#sensor-ranges-and-filtering)) and the sensor updates faster than 120 Hz, so each line is a fresh sample.
- Pico calibration (zero or scale for accel, bias for gyro) is applied. See [Calibration](#calibration).

### Status lines

Any line that isn't a `C` or `R` data line is a status or reply line. Status lines start with the sensor ID (`AC001,...`). Replies to commands are short and start with the command name (`SID 002`). A parser should only treat `AC001,C,...` and `AC001,R,...` as data and ignore anything else.

| Message | Meaning |
|---|---|
| `AC001,BOOT,PWR` / `WDT` / `OTHER` | Sent at start-up, with the reset cause |
| `AC001,PROFILE,60` / `120` | Active profile. Sent at boot, after a profile change, and in reply to a query |
| `AC001,USB,ON` / `OFF` | USB output state. Sent in reply to a `USB` command |
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

The ID-prefixed lines above are what the boot sequence and the older-style commands use. The short-form commands reply in the short form (`PRF 120`, `CAL OK`, `USB ON`, ...).

## Command interface

Commands are short ASCII lines, **case-insensitive**, ended with Enter (CR or LF). They work the same over **RS-485** and over the Pico's **USB** serial port, and a command typed on USB is answered on USB.

- A command with **nothing after it** reports the current value, for example `SID` replies `SID 001`.
- A command **with a value** sets it and replies with the new value, for example `SID 002` replies `SID 002`.
- A value that isn't accepted replies `<CMD> ERR` (for example `SID ERR`). Unknown commands are ignored, except in command mode.

| Command | What it does | Reply |
|---|---|---|
| `CMD` (or **Ctrl+U**) | **Enter command mode** (see below). Ctrl+U is a single byte (0x15), so no Enter is needed | `CMD ON` |
| `RUN` (also `EXIT` or `CMD OFF`) | Leave command mode and resume streaming | `CMD OFF` |
| `ECH [ON\|OFF]` | Command-mode echo and prompt (on at the start of every command session) | `ECH ON` |
| `ID` | Full sensor ID | `ID AC001` |
| `INF` | Everything at once | `INF AC001 SID=001 TYP=AC MFD=- SSN=BNO055 BAU=115200 PRF=60 CAL=ON TXO=OFF` |
| `DIR` | **Direction the front (x+) face points, in degrees and the nearest compass point, plus pitch and roll** (see below). `DIR RAW` adds a diagnostic field | `DIR HDG=273.4 W PIT=-1.2 ROL=0.4 MAG=3 PRO=1` |
| `MAG [SAVE\|CLR]` | Compass calibration. `MAG` reports the chip's calibration levels, `MAG SAVE` saves the calibration to flash, `MAG CLR` erases the saved one. See [Saving the compass calibration](#saving-the-compass-calibration) | `MAG SYS=3 GYR=3 ACC=3 MAG=3 PRO=1`, `MAG SAVED` |
| `SID [nnn]` | Sensor number, 1 to 3 digits, zero-padded. `SID 2` gives `002`, so `AC001` becomes `AC002` | `SID 002` |
| `TYP [xx]` | Two-character sensor type. `TYP GS` turns `AC001` into `GS001` | `TYP GS` |
| `MFD [date]` | Date of manufacture. Slashes are fine: `MFD 10/30/2026`. `-` and `.` are also accepted and stored with slashes | `MFD 10/30/2026` |
| `SSN [text]` | Accelerometer name or number, up to 24 characters: `SSN BNO055` | `SSN BNO055` |
| `BAU [rate]` (also `BAUD`) | Baud rate: 9600, 19200, 38400, 57600, or 115200. A rate too slow for the current profile is refused (57600 and below can't carry 120 Hz) | `BAU 57600` |
| `PRF [60\|120]` | Output profile: 60 Hz chip or 120 Hz raw | `PRF 120` |
| `CAL [ON\|OFF\|NOW]` | Pico calibration (120 Hz profile): `ON`/`OFF` enable or disable it, `NOW` runs one with the sensor still | `CAL ON`, `CAL OK`, `CAL FAIL` |
| `TXO [ON\|OFF]` | **Send only** (`ON`: driver held on, the Pico stops listening) or **send and receive** (`OFF`, the default) | `TXO ON` |
| `STR [ON\|OFF]` | Pause or resume the data stream | `STR OFF` |
| `USB [ON\|OFF]` | Mirror the stream to the USB serial port (off by default) | `USB ON` |
| `RST` | Reboot | `RST` |
| `PGM` | Reboot into programming mode (no data, watchdog off) so you can upload code | `PGM` |
| `DEF` | Erase saved settings (back to factory values) and reboot | `DEF` |

The older ID-prefixed commands (`AC001,PROFILE,60`, `AC001,RESET`, `AC001,CAL`, ...) are still accepted for the current ID.

### Compass and tilt (`DIR`)

`DIR` reports which way the sensor is facing and how far it is tilted from level:

```
DIR HDG=273.4 W PIT=-1.2 ROL=0.4 MAG=3 PRO=1
```

**Axes** (the same as the accel and gyro data): **x+ is the front face** (the direction the sensor is "facing"), **y+ is left**, **z+ is up**.

| Field | Meaning |
|---|---|
| `HDG` | Heading of the **front (x+) face**, degrees **clockwise from magnetic north**: 0 = north, 90 = east, 180 = south, 270 = west, followed by the nearest of the 8 compass points (`N NE E SE S SW W NW`) |
| `PIT` | Tilt of the front face above the horizon, degrees. **Positive = front up** |
| `ROL` | Roll about the front axis, degrees. **Positive = right side down** (left side up) |
| `MAG` | Magnetometer calibration level 0 (none) to 3 (full), as the chip measures it right now. `HDG` is best at 3 |
| `PRO` | `1` if a saved calibration (`MAG SAVE`) is in flash and is loaded at every start, `0` if not |

How it works: the chip's own fusion produces an orientation quaternion, and the firmware works out where the x+ axis points in the East/North/Up world frame. Heading, pitch, and roll are all computed from that, so tilt doesn't corrupt the heading.

**What you need for a compass heading**
- The **60 Hz chip profile** with `FUSION_MODE = "NDOF"` (the default). In the **120 Hz raw profile** the chip isn't fusing, so `DIR` gives only pitch and roll (from a low-passed gravity vector) and `HDG=-`. Same with `FUSION_MODE = "IMU"`.
- A calibrated magnetometer. The BNO055 learns its magnetometer calibration from motion (slow figure-8 movements of the whole sensor) and loses it at power-off. Save it once with `MAG SAVE` and the firmware loads it back at every start (next section).
- **Keep steel and currents away from the sensor.** The magnetometer can correct fixed offsets but not distortions: steel in the sonic mount or mast, motors, the buck converter, and wires carrying current all bend the field. Bosch's advice is to place the sensor well away from ferromagnetic parts.

**Magnetic versus true north.** `HDG` is relative to *magnetic* north. Add the local magnetic declination (east positive) with `HDG_OFFSET_DEG` in `main.py` to get *true* north. The same setting corrects a fixed mounting offset.

**Check it on the bench** (the sign and frame conventions were worked out from Bosch's documentation and tested in simulation, not on your hardware):
1. Set the sensor level with its front face pointing at a known direction, for example north from a phone compass. `HDG` should read about 0 (east 90, south 180, west 270). Turning the sensor clockwise as seen from above should make `HDG` increase.
2. Tilt the front face up. `PIT` should go positive. Tip the right side down. `ROL` should go positive.
3. `DIR RAW` also prints `EULH`, the chip's own Euler heading, which is the bearing of the chip's +y (left) axis. Level, `HDG` minus `EULH` should be 90 (mod 360). If it isn't, the quaternion convention is mirrored: set `QUAT_INVERSE = True`.
4. If a sign is backwards for your mounting, flip `PITCH_SIGN` or `ROLL_SIGN`.

Near straight up or down (pitch close to ±90°) the heading of the front face is not well defined.

### Saving the compass calibration

The BNO055 needs its magnetometer calibrated before the heading means much, and it forgets that at power-off. `MAG SAVE` stores the chip's calibration in flash (`bnocal.txt`), and the firmware writes it back into the chip every time the 60 Hz profile starts (power-up, reboot, or switching back from 120 Hz). For a general idea of direction, that's usually enough.

1. **Calibrate with the sensor assembled the way it will be mounted**: in its enclosure, on the sonic if you can, with the cable and power connected. Metal and wires near the sensor shift the field, and this calibration absorbs the fixed part of that.
2. **Move the whole assembly slowly** through a figure-8, and tilt it to a few different angles, for up to a minute.
3. **Watch `MAG`** (or the `MAG=` field in `DIR`). When `MAG=3`, send **`MAG SAVE`**. The reply is `MAG SAVED`.
   - If the magnetometer is not at 3 yet, you get `MAG ERR NOCAL` and nothing is saved. `MAG SAVE FORCE` saves anyway.
   - The data pauses for about 60 ms while the chip reads its offsets.
4. After that `DIR` shows `PRO=1`. After a power cycle the saved offsets are loaded straight away, so the heading is useful immediately even though the chip's own `MAG` level may read lower until it sees some motion.

Details:
- It works in the **60 Hz profile only**. In the 120 Hz raw profile `MAG` replies `MAG NA` and `MAG SAVE` replies `MAG ERR PRF`.
- `MAG CLR` erases the saved calibration. `DEF` (factory reset) does **not** erase it, because it's hard to recreate on a mounted unit.
- The saved calibration describes the magnetic environment it was made in. If the sensor is moved to a different mount or the nearby steel changes, redo it. A fixed offset on a tower is fine; a changing field isn't.

### Command mode

In half-duplex the Pico can only hear a command while it isn't transmitting, and while streaming it is transmitting a good part of the time (about a quarter of the time at 60 Hz, about half at 120 Hz). A command can therefore land on top of a data line and be lost.

Send **`CMD`** to enter command mode. Then:

- All unsolicited output stops (no data, no status lines), so the bus is free and **every command is heard**.
- The Pico answers each command as it arrives. Wait for each reply before sending the next command.
- It behaves like a small terminal: **what you type is echoed back, Backspace works, Enter starts a new line, the reply appears on its own line, and a `> ` prompt shows it is ready for the next command.** For example:

  ```
  > SID 2
  SID 002
  > 
  ```

  Echo is sent a few milliseconds after you stop typing (`ECHO_IDLE_MS`) so the Pico never talks over a sender that is still typing. A script that sends a whole line at once gets the echo after the line has arrived.
- Send `ECH OFF` to turn the echo and prompt off (scripts like `pico_cmd.py` do this). Every new session, started by `CMD` or Ctrl+U, begins with echo on again. If you see every character twice, your terminal has local echo turned on: turn it off, or send `ECH OFF`.
- The same line editing and echo work on the USB port.
- The LED blinks fast.
- **`RUN`** resumes streaming. If you forget, the Pico returns to streaming by itself after `CMD_TIMEOUT_S` seconds (120 by default) without a command, so a forgotten session can't silence a unit on a tower.

**Entering command mode** is itself a message the Pico has to hear, so send it a few times until you see `CMD ON`. Two ways:

- Type `CMD` and press Enter.
- Press **Ctrl+U** (one byte, 0x15, no Enter). In a terminal program connected straight to the serial port (PuTTY, `screen`) just press Ctrl+U. From a shell on the DSM use `printf '\025' > /dev/ttyDSM0`, because a shell's own terminal treats a typed Ctrl+U as "erase line" and never sends it. Ctrl+U typed on the Pico's USB port works too.

Ctrl+U is easier to type, but it is **not more likely to be heard**: in the simulation a single byte and `CMD` + Enter were heard about equally often while streaming (roughly 3 attempts in 4 at 60 Hz and about half at 120 Hz), because the loss comes from the Pico transmitting, not from the length of the command. Keep sending it until you see the reply. If line noise ever enters command mode by accident, set `CMD_KEY_COUNT = 2` so two Ctrl+U in a row are needed, or `CMD_KEY = 0` to turn the key off. Command mode returns to streaming by itself after `CMD_TIMEOUT_S` either way.

In simulation (not measured on hardware), 150 short commands sent while streaming were answered 73% of the time at 60 Hz and 47% of the time at 120 Hz. In command mode all 150 were answered.

### Sending commands from the DSM (over SSH)

The DSM is a Raspberry Pi with a serial card, so you can talk to the sensor from an SSH session. **NIDAS must not have the port open**, so stop it first, the way you normally do (this pauses data collection). The port is `/dev/ttyDSM0` for port 0 (use the number of the port you're on).

**Option 1: `pico_cmd.py` (easiest)**

```
python3 pico_cmd.py "SID 002" "TYP GS" "MFD 10/30/2026" "SSN BNO055-7"
python3 pico_cmd.py INF DIR MAG
python3 pico_cmd.py --port /dev/ttyDSM0 "BAU 57600"
```

It presses Ctrl+U until the sensor answers `CMD ON`, turns the echo off for its own session, sends each command, resending until it gets the reply, prints the replies, and sends `RUN` at the end (`--stay` leaves the sensor in command mode). For `BAU` it follows the sensor to the new baud rate and confirms the change so it is kept; remember to change the baud in the NIDAS catalog to match. It needs pyserial: `sudo apt install python3-serial`.

**Option 2: an interactive terminal**

```
sudo apt install picocom
picocom -b 115200 /dev/ttyDSM0
```

Press **Ctrl+U** until you see `CMD ON` and a `> ` prompt, then type commands and press Enter, and `RUN` when done. In command mode the sensor echoes your typing and starts a new line on Enter itself, so don't turn on the terminal's local echo (you would see every character twice). Leave picocom with Ctrl+A then Ctrl+X. `screen /dev/ttyDSM0 115200` also works (leave with Ctrl+A then K). Ctrl+U passes straight through both of them.

**If the sensor never answers**

- The data from the sensor reaches the DSM (you've seen that), but sending the other way needs the DSM's RS-485 driver switched on while it writes. Some serial boards do that by themselves once NIDAS has put the port in RS-485 half-duplex mode, and some switch it with the RTS line. For the RTS kind, run `pico_cmd.py` with `--rts high` or `--rts low` (the RTS level that enables the driver). NIDAS has the same idea in its catalog as the `rts485` attribute, which your `LUFFT` entry uses.
- Check that NIDAS is really stopped, that the baud rate is 115200, and that the sensor is not in send-only mode (`TXO ON`), which makes it deaf except for the first 30 seconds after power-up.
- Your RS-485 wiring needs to carry the DSM's transmit direction as well: on the half-duplex pair (pins 4 and 5 of the Bulgin connector) that's the same pair.

### Saved settings

`SID`, `TYP`, `MFD`, `SSN`, `BAU`, `PRF`, `CAL`, and `TXO` are saved to `config.json` on the Pico's flash and survive power cycles. The file is written only when a value actually changes. `DEF` erases it. The factory values are the `DEFAULT_...` settings at the top of `main.py`.

**Changing the baud rate safely.** `BAU` replies at the old rate, then switches. If the command came over RS-485, the new rate is only **kept** once the Pico hears a valid command at the new rate (for example a plain `BAU` query) within `BAUD_REVERT_S` seconds (60). Otherwise it goes back to the old rate and sends `BAU ERR REVERT`, so a mistyped rate can't strand a remote unit. A change typed on USB is kept straight away.

**Send-only mode (`TXO ON`).** The default is send and receive. `TXO ON` holds the driver on permanently, which is the cleanest transmit, but the Pico can no longer hear RS-485 commands. It is saved, so it stays on after a power cycle. To avoid locking you out, the Pico **always listens for `TXO_GRACE_S` seconds (30) after every power-up**, and `TXO OFF` sent in that window undoes it. From USB, `TXO OFF` works any time.

**The profile is saved too.** If you send `PRF 120`, the unit comes back in 120 Hz after a reboot, not 60 Hz. `DEF` returns it to 60 Hz.

## Calibration

There are two separate calibrations:

- **Pico calibration** (this section) corrects the **120 Hz raw profile** in software, switched with the command `CAL ON` / `CAL OFF`. It is not used in the 60 Hz profile.
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

1. **When the 120 Hz profile starts**, whether at boot (if the saved profile is 120) or after a `PRF 120` command, as long as calibration is on. The sensor must stay still for `CAL_SECONDS`.
2. **On command:** `CAL NOW`, while in the 120 Hz profile.

**Rejecting bad calibrations:** if the sensor moves during the window (gyro peak-to-peak above `CAL_MAX_MOTION_DPS`, or accel above `CAL_MAX_ACCEL`), the run is rejected instead of baking motion into the zero. In `"SCALE"` mode, a magnitude far from 1 g is also rejected as a sensor fault.

**Saved calibration:** each good calibration is written to `cal.txt` on the Pico's flash (only when the values change meaningfully, to spare the flash). If a calibration is rejected, such as when the mast is swaying when you switch to the 120 Hz profile, the last saved one is loaded and `AC001,WARN,NOCAL,USING_SAVED` is sent. If there's no saved file yet, the profile runs uncalibrated and sends `AC001,WARN,NOCAL`.

> **Remounting the sensor?** A saved `ZERO` calibration is only valid for the orientation it was taken in. Run `CAL NOW` in the 120 Hz profile during calm conditions after any remount, or delete `cal.txt`.

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
| LED fast blink | Command mode (`CMD`) |

**Programming mode** is how to reprogram the board with the watchdog enabled. You can also enter it by sending `PGM` from a plain serial terminal (not Thonny, which interrupts the program and lets the watchdog reset the board). If nobody connects within `PROGRAM_TIMEOUT_S` (15 minutes by default), the Pico reboots back to normal operation, so a stray command can't leave a remote unit silent. After the reboot, the LED blinks fast and the script idles. Press **Stop** in Thonny to get the REPL and upload new files. The mode flag is consumed on that boot, so the next reset or power cycle runs normally.

While developing, set `USE_WDT = False` in `main.py`, because an armed watchdog can't be stopped. Reading BOOTSEL briefly pauses flash access, so the firmware samples it about 30 times a second. If your MicroPython build lacks `rp2.bootsel_button()`, the button is disabled and everything else works.

## Configuration

Two kinds of setting:

1. **Saved by command (factory defaults at the top of `main.py`).** These are the values the unit starts with until you change them with a command. Changes are kept across power cycles and `DEF` restores the defaults.

| Variable | Default | Command | Description |
|---|---|---|---|
| `DEFAULT_TYPE` | `"AC"` | `TYP` | Two-character sensor type |
| `DEFAULT_SID` | `"001"` | `SID` | Three digits: generation + number. ID = type + digits |
| `DEFAULT_MFD` | `""` | `MFD` | Date of manufacture |
| `DEFAULT_SSN` | `"BNO055"` | `SSN` | Accelerometer name or number |
| `DEFAULT_PROFILE` | `60` | `PRF` | `60` (chip) or `120` (raw) |
| `DEFAULT_BAUD` | `115200` | `BAU` | RS-485 baud rate |
| `DEFAULT_CAL` | `True` | `CAL ON` / `CAL OFF` | Pico calibration in the 120 Hz profile |
| `DEFAULT_TX_ONLY` | `False` | `TXO ON` / `TXO OFF` | `False` = send and receive (half-duplex), `True` = send only |

2. **Fixed in the code.**

| Variable | Default | Description |
|---|---|---|
| `ALLOWED_BAUDS` / `MAX_BAUD` | 9600 to 115200 / `115200` | Baud rates `BAU` accepts, and the hard ceiling |
| `BAUD_REVERT_S` | `60` | A `BAU` change over RS-485 is reverted unless a command is heard at the new rate within this time |
| `TXO_GRACE_S` | `30` | After power-up, send-only mode still listens this long so `TXO OFF` can undo it |
| `ECHO_IDLE_MS` | `4` | Command mode over RS-485: typed characters are echoed once the sender has paused this long |
| `CMD_KEY` | `0x15` | Single control byte that enters command mode (0x15 = Ctrl+U). `0` turns it off |
| `CMD_KEY_COUNT` | `1` | How many `CMD_KEY` bytes in a row are needed |
| `CMD_TIMEOUT_S` | `120` | Command mode returns to streaming after this long without a command (`0` = never) |
| `CONFIG_FILE` | `"config.json"` | Where the saved settings live |
| `BNO_CAL_FILE` | `"bnocal.txt"` | Where `MAG SAVE` keeps the chip's compass calibration |
| `EOL` | `"\\r\\n"` | Line ending |
| `CHECKSUM` | `False` | Append `*XX` XOR checksum |
| `USE_WDT` | `True` | Hardware watchdog. Set `False` while developing |
| `WDT_TIMEOUT_MS` | `3000` | Watchdog timeout |
| `FUSION_MODE` | `"NDOF"` | Fusion mode for the 60 Hz profile: `"NDOF"` (adds the magnetometer, needed for the `DIR` compass heading) or `"IMU"` (accel + gyro only, no absolute heading) |
| `HDG_OFFSET_DEG` | `0.0` | Added to the `DIR` heading. Set it to the local magnetic declination (east positive) for true north, or a mounting offset |
| `QUAT_INVERSE` | `False` | `True` if `DIR` heading and tilt come out mirrored (see the bench check) |
| `PITCH_SIGN` / `ROLL_SIGN` | `1` / `1` | `-1` flips the sign of the `DIR` pitch or roll |
| `CHIP_ACCEL` | `"LIA"` | 60 Hz profile accel: `"LIA"` (gravity removed by chip) or `"ACC"` (gravity included) |
| `RAW_APPLY_PICO_CAL` | `True` | Master switch for applying Pico calibration in the 120 Hz profile. `False` never applies it |
| `ACCEL_DECIMALS` / `GYRO_DECIMALS` | `2` / `3` | Decimals printed. These match the chip's resolution, and fewer digits keep the lines short enough for 115200 baud |
| `CHIP_STATUS_S` | `10` | 60 Hz profile: seconds between `STAT` lines. `0` turns them off |
| `TAG_RAW` / `TAG_CHIP` | `"R"` / `"C"` | Data line tags for the 120 Hz / 60 Hz profiles |
| `BNO_ADDR` | `0x28` | BNO055 I2C address (`0x29` if ADR is high) |
| `USE_EXT_CRYSTAL` | `True` | Use the breakout's 32 kHz crystal |
| `ACCEL_CAL_MODE` | `"ZERO"` | `"ZERO"`, `"SCALE"`, or `"OFF"` |
| `CAL_SECONDS` | `1.0` | How long the sensor must be still for the Pico calibration |
| `CAL_MAX_MOTION_DPS` | `3.0` | Gyro peak-to-peak limit during calibration |
| `CAL_MAX_ACCEL` | `0.5` | Accel peak-to-peak limit (m/s²) during calibration |
| `CAL_FILE` | `"cal.txt"` | Saved calibration file |
| `SENSOR_PWR_PIN` | `None` | GPIO that powers the BNO055 (enables power-cycle recovery) |
| `SENSOR_RST_PIN` | `None` | GPIO wired to BNO055 RST (enables hardware reset) |
| `USB_OUTPUT` | `False` | Whether USB output is on at boot. Off by default. Change it at run time with `USB ON` / `USB OFF` |
| `LED_PERIOD_MS` / `LED_ON_MS` | `5000` / `100` | Heartbeat blink timing |
| `USE_BUTTON` | `True` | Enable the BOOTSEL button |
| `PROGRAM_HOLD_MS` | `3000` | Hold time for programming mode |
| `PROGRAM_TIMEOUT_S` | `900` | Programming mode gives up and reboots to normal operation after this long if nobody connects (`0` = never) |
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

## DSM (NIDAS) configuration

How to set up the data system (DSM) that records this sensor. Three files are involved, plus the wiring and the port mode.

| File | What goes in it |
|---|---|
| `sensor_catalog.xml` | The sensor definition: baud rate, port type, how each line is parsed, variable names and units |
| `callab.xml` (the project config that `configs.xml` points to) | Which port the sensor is on, its sensor `id`, and a name suffix |
| `configs.xml` | The plot-range variables the catalog entry refers to |

### 1. Sensor catalog entry (`sensor_catalog.xml`)

Paste this just above the closing `</sensorcatalog>` tag. It is also in `ac001_catalog_entry.xml`.

```xml
    <!-- AC001: Pico 2 + BNO055 motion sensor mounted on the sonic.
         ASCII lines, 115200 8N1, RS-485 half duplex, CRLF terminated. The "AC001" at the start of each line is the sensor's own
         ID (commands TYP and SID change it), so the formats below skip the first field instead of spelling it out.
         Two profiles, only one active at a time (command PRF 60 / PRF 120):
           <ID>,C,ax,ay,az,gx,gy,gz   60 Hz (default): BNO055 chip output, linear accel (gravity removed)
           <ID>,R,ax,ay,az,gx,gy,gz   120 Hz: raw sensor data with Pico calibration
         <ID>,STAT,sys,gyr,acc,mag    every 10 s in the 60 Hz profile: chip calibration levels 0-3
         accel in m/s^2, gyro in rad/s. Boot, reply and other status lines match none of the formats and are ignored. -->
    <serialSensor ID="AC001" class="DSMSerialSensor"
        baud="115200" parity="none" databits="8" stopbits="1" porttype="RS485_HALF">
        <!-- 60 Hz chip profile -->
        <sample id="1" scanfFormat="%*[^,],C,%f,%f,%f,%f,%f,%f" rate="60">
            <variable name="Ax_c" units="m/s^2" longname="AC001 accel X, chip profile" plotrange="$ACC_RANGE"/>
            <variable name="Ay_c" units="m/s^2" longname="AC001 accel Y, chip profile" plotrange="$ACC_RANGE"/>
            <variable name="Az_c" units="m/s^2" longname="AC001 accel Z, chip profile" plotrange="$ACC_RANGE"/>
            <variable name="Gx_c" units="rad/s" longname="AC001 angular rate X, chip profile" plotrange="$GYRO_RANGE"/>
            <variable name="Gy_c" units="rad/s" longname="AC001 angular rate Y, chip profile" plotrange="$GYRO_RANGE"/>
            <variable name="Gz_c" units="rad/s" longname="AC001 angular rate Z, chip profile" plotrange="$GYRO_RANGE"/>
        </sample>
        <!-- 120 Hz raw profile -->
        <sample id="2" scanfFormat="%*[^,],R,%f,%f,%f,%f,%f,%f" rate="120">
            <variable name="Ax_r" units="m/s^2" longname="AC001 accel X, raw profile" plotrange="$ACC_RANGE"/>
            <variable name="Ay_r" units="m/s^2" longname="AC001 accel Y, raw profile" plotrange="$ACC_RANGE"/>
            <variable name="Az_r" units="m/s^2" longname="AC001 accel Z, raw profile" plotrange="$ACC_RANGE"/>
            <variable name="Gx_r" units="rad/s" longname="AC001 angular rate X, raw profile" plotrange="$GYRO_RANGE"/>
            <variable name="Gy_r" units="rad/s" longname="AC001 angular rate Y, raw profile" plotrange="$GYRO_RANGE"/>
            <variable name="Gz_r" units="rad/s" longname="AC001 angular rate Z, raw profile" plotrange="$GYRO_RANGE"/>
        </sample>
        <!-- optional: BNO055 calibration levels (0 none to 3 full), sent every 10 s in the 60 Hz profile -->
        <sample id="3" scanfFormat="%*[^,],STAT,%d,%d,%d,%d" rate="0.1">
            <variable name="CalSys" units="none" longname="AC001 chip calibration, system"/>
            <variable name="CalGyr" units="none" longname="AC001 chip calibration, gyro"/>
            <variable name="CalAcc" units="none" longname="AC001 chip calibration, accel"/>
            <variable name="CalMag" units="none" longname="AC001 chip calibration, magnetometer"/>
        </sample>
        <message separator="\r\n" position="end" length="0"/>
    </serialSensor>
```

Points to know:

- **`porttype="RS485_HALF"`** matches the half-duplex hardware. For a data-only, receive-only hookup on the RS-422 pins, use `porttype="RS422"` instead (see the port section).
- **`baud="115200"`** must match the sensor. If you change it with the `BAU` command, change it here too.
- **The formats start with `%*[^,]`, which skips the sensor's own ID.** That lets you change the ID with `TYP` and `SID` without touching the catalog. The catalog `ID="AC001"` is only the name of this catalog entry, and it doesn't have to match the sensor's ID.
- **Which sample gets data depends on the profile.** The 60 Hz chip profile fills sample 1 (and sample 3 every 10 s). The 120 Hz raw profile fills sample 2. The other stays empty.
- **Boot messages, replies, and `DIR`/`INF` output match none of the formats** and are ignored.
- **Variable names have the port suffix added by NIDAS**, for example `Ax_c.0` on port 0.

### 2. Port assignment (`callab.xml`)

Add the sensor to the DSM's list, with the port it is wired to:

```xml
<serialSensor IDREF="AC001" devicename="/dev/ttyDSM0" suffix=".0" id="100">
</serialSensor>
```

- `IDREF="AC001"` is the catalog entry from step 1.
- `devicename` is the DSM serial port (`/dev/ttyDSM0` is port 0).
- `id` must be unique among the sensors on that DSM. The sample ids NIDAS reports are `id` + the sample number, so with `id="100"` the raw line is `100` and the parsed samples are `101`, `102`, and `103`.
- `suffix` is added to the variable names (`Ax_c.0`).

Example site with only this sensor on port 0 (add the sensors your DSM needs):

```xml
<site name="TONY_TEST" class="isff.GroundStation" suffix="">
    <dsm name="$SITE" id="5" rserialPort="30002">

        <serialSensor IDREF="AC001" devicename="/dev/ttyDSM0" suffix=".0" id="100">
        </serialSensor>

        <output class="RawSampleOutputStream">
            <socket type="server" port="30000" maxIdle="60" block="false"/>
        </output>

        <output class="RawSampleOutputStream">
            <fileset dir="$DATAMNT/projects/${PROJECT}/raw_data"
                    file="${DSM}_%Y%m%d_%H%M%S.dat"
                    length="43200">
                <mount dir="$DATAMNT"/>
            </fileset>
        </output>
    </dsm>
</site>
```

`callab.xml` pulls in the catalog with an `xi:include`, so the edited `sensor_catalog.xml` must be in the same directory. Only one `<site>` runs at a time, and the DSM chooses it by the `$SITE` setting. Don't put two sensors on the same `devicename`.

### 3. Plot ranges (`configs.xml`)

The catalog entry uses `$ACC_RANGE` and `$GYRO_RANGE`. Add these inside the config's list of environment variables, next to the other range variables:

```xml
<envvar name="ACC_RANGE" value="-10 10"/>
<envvar name="GYRO_RANGE" value="-2 2"/>
```

Adjust the numbers to taste. If you keep gravity in the accel (`ACCEL_CAL_MODE = "SCALE"`, or `CHIP_ACCEL = "ACC"`), use a wider accel range such as `-15 15`. This file is ISO-8859-1, so type plain ASCII.

### 4. Wiring and port mode

NCAR/EOL Bulgin connector pins (signals with respect to the DSM) for **RS-485 half-duplex**:

| Pin | Signal | Connect to |
|---|---|---|
| 1 | +12 VDC (DSM to sensor) | Buck converter input |
| 2 | Shield | Connect at one end only |
| 4 | Tx−/Rx− | Breakout **B** (inverting, −) |
| 5 | Tx+/Rx+ | Breakout **A** (non-inverting, +) |
| 7 | Signal ground | Breakout GND |
| 8 | Gnd (power return) | Buck converter ground |
| 3, 6 | Not used in RS-485 half | Leave unconnected |

- **Match polarity by sign, not by letter.** The connector table's `(A)` and `(B)` letters follow the TIA convention (A is −), while the SP3485 breakout's A is the non-inverting (+) line. Connect the breakout's A to a `+` pin and its B to a `−` pin. If the data is garbage, swapping the two wires is harmless.
- **The same pins 4 and 5 are the DSM's transmit pair in RS-422 mode** (and RTS and RxD in RS-232 mode). The DSM only listens there if the port really is in RS-485 half-duplex mode.
- **The XML setting only takes effect on the new FTDI serial boards.** On other boards the mode is set by a jumper on the board. To check what a port is doing, disconnect the sensor and measure DC volts across pins 5 and 4 with the DSM running: several volts means the DSM is driving the pair (RS-422-type mode), while about 0 to 0.5 V is the idle state you want.
- **Data only (no commands from the DSM):** you can instead use the RS-422 receive pair, pins 3 (`Rx−`) and 6 (`Rx+`), with `porttype="RS422"`. The DSM can't send commands to the sensor that way.
- **Sending commands from the DSM** needs half-duplex on pins 4 and 5, with the DSM's driver switched on while it writes. Boards that switch the driver with the RTS line need the `rts485` attribute in the catalog entry (your `LUFFT` entry uses it). See [Sending commands from the DSM](#sending-commands-from-the-dsm-over-ssh).

### 5. Applying and checking

1. Back up the files you will edit: `cp sensor_catalog.xml sensor_catalog.xml.bak` (and the same for `callab.xml` and `configs.xml`).
2. Make the edits, then check each file is still valid XML: `xmllint --noout <file>`.
3. Restart the DSM the way you normally do.
4. See what arrives: `data_stats sock:localhost:30000` lists a line for the sensor with its sample count and rate.
5. Look at the text: `data_dump -i 5,100 -A sock:localhost:30000` (DSM id 5, sensor id 100). Use `5,101` for the parsed 60 Hz samples.
6. To look at the raw port instead, stop NIDAS and run `stty -F /dev/ttyDSM0 115200 cs8 -parenb -cstopb raw -echo`, then `cat /dev/ttyDSM0`. You should see `AC001,C,...` lines.

### 6. Things that can bite

- **Timeouts while the sensor is silent.** In command mode (up to 120 s) or after `STR OFF`, NIDAS receives nothing. If it logs timeouts or reopens the port, raise the sensor's `timeout` attribute.
- **Baud changes.** `BAU` changes the sensor's rate immediately. Update the catalog `baud` to match, then restart the DSM.
- **Stop NIDAS before talking to the sensor by hand** (`pico_cmd.py`, picocom, `cat`). Only one program can use the port.
- **A changed ID is not a problem** for the catalog as long as it uses the `%*[^,]` formats above. The viewer (`ac001_viewer.py`) also accepts any ID.

### 7. Link test scripts

If the DSM isn't seeing clean lines, these scripts isolate the problem. Run them from Thonny on the Pico, with `main.py` not running:

| Script | What it does |
|---|---|
| `rs485_ping.py` | Sends `ping` once a second with the driver held on. The simplest end-to-end test |
| `rs485_test.py` | Sends a fixed data-style line at 60 Hz |
| `rs485_u_test.py` | Sends a continuous stream of `U`, the simplest waveform for a scope |
| `rs485_static_test.py` | Holds the driver at a steady high, then low, so a multimeter can check A minus B (+2 V, then −2 V) |
| `rs485_echo.py` | Listens and echoes every received line, which tests the DSM-to-sensor direction |

## Installation

1. Flash MicroPython for the **Pico 2** from [micropython.org](https://micropython.org/download/RPI_PICO2/) (hold BOOTSEL, plug in USB, drag the UF2 onto the drive).
2. Copy `main.py` to the Pico with Thonny. It runs automatically at every power-up.
3. Connect the hardware as in the pin map above.
4. Connect an RS-485 adapter at the other end of the cable at 115200 8N1. To watch the stream over the Pico's USB port instead, open its serial port in a terminal (any baud rate), type `USB ON` and press Enter. USB output is off until you do.

## PC viewer

`ac001_viewer.py` plots live data in a 2×2 grid: the top row is the 60 Hz chip profile and the bottom row is the 120 Hz raw profile, with gyro on the left and accel on the right. Only the active profile's row updates, and the other says "not the active profile". The viewer only listens. To change profile, send the `PROFILE` command from a separate terminal or tool on the RS-485 adapter.

```
pip install pyserial matplotlib numpy
python ac001_viewer.py --enable-usb                        # Pico on USB (turns its USB output on)
python ac001_viewer.py --port COM5                         # RS-485 adapter on Windows (115200 default)
python ac001_viewer.py --port /dev/ttyUSB0                 # RS-485 adapter on Linux/macOS
python ac001_viewer.py --demo                              # synthetic data, no hardware
```

Keys: **space** pauses, **c** clears. The viewer verifies checksums if enabled, shows status messages and the measured rate of each profile, and reconnects automatically if the port drops. The viewer matches the ID `AC001`, so edit `SENSOR_ID` at the top of the script if you change the firmware's ID.

## Limitations

- The compass (`DIR`) needs the 60 Hz profile with `NDOF`, and its accuracy depends on the magnetometer calibration (`MAG=3`) and on the magnetic environment. Save the calibration with `MAG SAVE` so it survives power cycles.
- Only accel and gyro are streamed. Orientation (Euler/quaternion) is not output.
- The BNO055 can only run one mode at a time, so the profiles can't run together, and switching interrupts the data for 1 to 3 seconds.
- In the 60 Hz profile the chip's data updates at 100 Hz internally and is sampled at 60 Hz.
- The chip's own calibration is not saved or restored by this firmware. It restarts at every power-up.
- USB output is off at every boot unless `USB_OUTPUT = True`. The firmware does not remember a commanded `USB,ON` across a reset.
- A command can be missed if it arrives while the Pico is transmitting (half-duplex), so repeat commands.
- Pico calibration assumes the mast is still. If it's swaying when the 120 Hz profile starts, the saved calibration is used instead.
