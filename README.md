# AC001 Tower Accelerometer / Gyro Node

MicroPython firmware for a **Raspberry Pi Pico 2** that reads a **Bosch BNO055** and streams ASCII accel/gyro data over **RS-485**. Built for unattended tower mounting: hardware and software watchdogs, I2C fault recovery, automatic calibration, and a remote command interface.

**[Wiring diagram (PDF)](Wiring_Diagram.pdf)**

## Features

- Two output profiles, switched by command: **60 Hz chip** (default, BNO055 fusion output) and **120 Hz raw** (raw sensors + Pico calibration)
- Accel (m/s²) and angular rate (rad/s), 3 axes each
- Half-duplex RS-485 with short commands and a **command mode** that frees the bus
- Settings saved to flash (ID, baud, profile, calibration, send-only mode)
- Compass heading, pitch and roll on request (`DIR`), with saveable magnetometer calibration
- Hardware + software watchdogs, I2C bus recovery, optional sensor power/reset pin
- Optional XOR checksum, optional USB output (off by default)
- LED heartbeat; BOOTSEL tap to pause, hold for programming mode
- PC viewer (`ac001_viewer.py`) with live plots

## Hardware

| Part | Notes |
|---|---|
| Raspberry Pi Pico 2 (or 2 W) | MicroPython for RP2350 |
| Adafruit BNO055 breakout | Fusion (`NDOF`) at 60 Hz, `AMG` at 120 Hz |
| SparkFun SP3485 RS-485 breakout | 3.3 V transceiver |
| MPM3610 5 V buck breakout | 12 V in, 5 V to VSYS |
| M2.5 countersunk screws | Stainless preferred |

### Pin map

| Pico 2 pin | Signal | Connects to |
|---|---|---|
| GP4 (6) | I2C0 SDA | BNO055 SDA |
| GP5 (7) | I2C0 SCL | BNO055 SCL |
| GP0 (1) | UART0 TX | SP3485 DI |
| GP1 (2) | UART0 RX | SP3485 RO |
| GP2 (4) | Direction | SP3485 DE + RE tied, 10 kΩ pull-down |
| 3V3 OUT (36) | 3.3 V | BNO055 VIN, SP3485 VCC |
| VSYS (39) | 5 V in | Buck output |
| GND | Ground | All grounds common |
| GP6 (9) *(opt.)* | Sensor power | BNO055 VIN instead of 3V3 (`SENSOR_PWR_PIN = 6`) |
| GP7 (10) *(opt.)* | Sensor reset | BNO055 RST (`SENSOR_RST_PIN = 7`) |

BNO055 address `0x28` (`0x29` with ADR high; set `BNO_ADDR`).

### RS-485 notes

- **Polarity:** SP3485 **A = +**, **B = −**. Manufacturers' letters disagree, so match by sign. Garbage or no data? Swap the pair.
- One twisted pair (half-duplex), 120 Ω termination at the far end, bias resistors (560 Ω–1 kΩ) at the host if the adapter lacks them, and a ground wire if possible.
- Use TVS surge protection at both ends; consider an isolated transceiver if ground can't be guaranteed.
- Garbled data: run `rs485_test.py` (fixed line, no sensor). Still garbled means bus/wiring/baud; clean means try `TXO ON`.

## Enclosure

- **Location:** `[Place Holder]`
- **Formats:** STEP and 3MF for Core One + Gen2 (PrusaSlicer 3.0)
- **Fits:** CAST 3A sonic, ISFS bracket (Campbell mount needs adapting)
- **Printing:** Polymaker PolyLite ASA (white), 0.25 mm nozzle (0.4 mm works but may leak more around screw holes)

Mount rigidly, seal with cable glands, and record the axis mapping to the sonic: `[axis mapping]`.

## Serial output

115200 8N1, `\r\n` line endings. Change baud with `BAU` (9600–115200).

| | 60 Hz chip *(default)* | 120 Hz raw |
|---|---|---|
| Tag | `C` | `R` |
| BNO055 mode | Fusion (`NDOF`) | `AMG` |
| Accel | Linear accel (gravity removed) | Raw register |
| Gyro | Chip register | Raw register |
| Pico calibration | None | Applied (`CAL ON`) |
| `STAT` line | Every 10 s | None |

```
AC001,C,ax,ay,az,gx,gy,gz      e.g. AC001,C,0.00,-0.01,0.00,0.001,0.001,-0.001
AC001,R,ax,ay,az,gx,gy,gz
```

Accel in m/s² (2 dp), gyro in rad/s (3 dp). With `CHECKSUM = True`, lines end `*XX` (hex XOR of everything before `*`).

- **ID:** `AC` = type, `0` = generation, `01` = number. Change with `TYP` and `SID`; takes effect immediately. Parsers should match `%*[^,],C,...` rather than a literal ID.
- **Switching profile** re-initialises the chip: ~1–2 s with no data, plus ~1 s of calibration in 120 Hz.
- **Chip profile:** fusion fixes the accel at ±4 g and updates at 100 Hz (sampled at 60 Hz). Gyro calibration settles quickly at rest; accel may stay partly uncalibrated on a mast (watch `STAT`). `CHIP_ACCEL = "ACC"` keeps gravity.
- **Raw profile:** ranges set by firmware (below); each line is a fresh sample.

### Status lines

Only `<ID>,C,...` and `<ID>,R,...` are data; ignore everything else. Short-form commands reply in short form (`PRF 120`); boot and legacy messages are ID-prefixed. Errors are limited to one per second.

| Message | Meaning |
|---|---|
| `BOOT,PWR` / `WDT` / `OTHER` | Start-up and reset cause |
| `PROFILE,60` / `120` | Active profile |
| `STAT,sys,gyr,acc,mag` | Chip calibration levels 0–3 (60 Hz only) |
| `CAL,OK` / `FAIL` / `NA` | Pico calibration result (`NA` in 60 Hz) |
| `WARN,NOCAL[,USING_SAVED]` | Calibration rejected; saved one loaded if available |
| `STREAM,ON` / `OFF` | Stream resumed / paused |
| `USB,ON` / `OFF` | USB output state |
| `PROGRAM` / `RESETTING` | Entering programming mode / rebooting |
| `ERR,I2C` / `STUCK` / `MODE` | Read failures / frozen data / chip left its mode |
| `ERR,NOSENSOR` / `ERR,RESETTING` | No sensor at boot / repeated re-init failures |

## Commands

Case-insensitive ASCII lines ending CR or LF, over RS-485 or USB (replies go back the same way). No value = query; a value = set and echo; bad value = `<CMD> ERR`.

| Command | Action | Example reply |
|---|---|---|
| `CMD` or **Ctrl+U** | Enter command mode | `CMD ON` |
| `RUN` / `EXIT` / `CMD OFF` | Leave command mode | `CMD OFF` |
| `ECH [ON\|OFF]` | Echo and prompt in command mode | `ECH ON` |
| `ID` / `INF` | Full ID / all settings | `ID AC001` |
| `DIR [RAW]` | Heading, pitch, roll | `DIR HDG=273.4 W PIT=-1.2 ROL=0.4 MAG=3 PRO=1` |
| `MAG [SAVE\|CLR]` | Compass calibration status / save / erase | `MAG SAVED` |
| `SID [nnn]` | Sensor number (zero-padded) | `SID 002` |
| `TYP [xx]` | Two-character type | `TYP GS` |
| `MFD [date]` | Manufacture date (`/`, `-`, `.` accepted) | `MFD 10/30/2026` |
| `SSN [text]` | Sensor name, ≤24 chars | `SSN BNO055` |
| `BAU [rate]` | Baud; refuses rates too slow for the profile (≤57600 can't carry 120 Hz) | `BAU 57600` |
| `PRF [60\|120]` | Output profile | `PRF 120` |
| `CAL [ON\|OFF\|NOW]` | Pico calibration (120 Hz) | `CAL OK` |
| `TXO [ON\|OFF]` | Send-only mode | `TXO ON` |
| `STR [ON\|OFF]` | Pause / resume stream | `STR OFF` |
| `USB [ON\|OFF]` | Mirror stream to USB | `USB ON` |
| `RST` / `PGM` / `DEF` | Reboot / programming mode / factory reset + reboot | `RST` |

Legacy ID-prefixed commands (`AC001,PROFILE,60`, `AC001,RESET`, ...) still work.

### Command mode

While streaming, the Pico is transmitting much of the time (¼ at 60 Hz, ½ at 120 Hz), so commands can be lost. `CMD` stops all output so every command is heard.

- Send `CMD` (or Ctrl+U) repeatedly until `CMD ON` appears. Both are heard about equally often.
- Acts like a terminal: echo, Backspace, `> ` prompt. `ECH OFF` disables echo for scripts. Seeing double characters means your terminal's local echo is on.
- LED blinks fast. `RUN` resumes; it also resumes on its own after `CMD_TIMEOUT_S` (120 s).
- If noise triggers it, set `CMD_KEY_COUNT = 2` or `CMD_KEY = 0`.

In simulation, commands sent while streaming got through 73% (60 Hz) / 47% (120 Hz) of the time; 100% in command mode.

### Compass and tilt (`DIR`)

Axes: **x+ = front**, **y+ = left**, **z+ = up**.

| Field | Meaning |
|---|---|
| `HDG` | Front-face heading, ° clockwise from magnetic north, plus nearest of 8 compass points |
| `PIT` | Positive = front up |
| `ROL` | Positive = right side down |
| `MAG` | Magnetometer calibration 0–3 (best at 3) |
| `PRO` | `1` if a saved calibration is loaded at start |

Computed from the chip's fusion quaternion, so tilt doesn't corrupt heading. Requirements:

- 60 Hz profile with `FUSION_MODE = "NDOF"`. In 120 Hz or `IMU` mode, only pitch and roll are given (`HDG=-`).
- A calibrated magnetometer (see below), away from steel, motors, the buck converter, and current-carrying wires.
- `HDG_OFFSET_DEG` adds magnetic declination (east positive) or a mounting offset.
- Heading is undefined near pitch ±90°.

**Bench check** (conventions verified in simulation only):
1. Level, front pointing north: `HDG` ≈ 0, increasing clockwise.
2. Front up → `PIT` positive; right side down → `ROL` positive.
3. `DIR RAW` adds `EULH`; level, `HDG − EULH` should be 90. If not, set `QUAT_INVERSE = True`.
4. Flip `PITCH_SIGN` / `ROLL_SIGN` if needed.

### Saving the compass calibration

The chip forgets magnetometer calibration at power-off. `MAG SAVE` stores it in `bnocal.txt` and it's reloaded whenever the 60 Hz profile starts.

1. Assemble as mounted (enclosure, cable, power, ideally on the sonic).
2. Move slowly in figure-8s and tilts for up to a minute.
3. When `MAG=3`, send `MAG SAVE` (`MAG ERR NOCAL` if not ready; `MAG SAVE FORCE` overrides). Data pauses ~60 ms.

60 Hz only (`MAG NA` / `MAG ERR PRF` in 120 Hz). `MAG CLR` erases it; `DEF` does not. Redo after moving to a different mount.

### Sending commands from the DSM

Stop NIDAS first (only one program can use the port). Port 0 is `/dev/ttyDSM0`.

```
python3 pico_cmd.py "SID 002" "TYP GS" "MFD 10/30/2026"
python3 pico_cmd.py INF DIR MAG
python3 pico_cmd.py --port /dev/ttyDSM0 "BAU 57600"
```

`pico_cmd.py` (needs `python3-serial`) enters command mode, sends each command until answered, and sends `RUN` (`--stay` to remain). It follows `BAU` changes and confirms them; update the NIDAS catalog to match.

Interactive: `picocom -b 115200 /dev/ttyDSM0` (exit Ctrl+A, Ctrl+X) or `screen /dev/ttyDSM0 115200`. From a shell, send Ctrl+U with `printf '\025' > /dev/ttyDSM0`.

**No answer?** The DSM's driver must be enabled while writing: some boards do this in RS-485 half mode, others via RTS (`pico_cmd.py --rts high|low`, or `rts485` in the catalog). Also check NIDAS is stopped, baud is right, `TXO` is off, and the half-duplex pair (Bulgin pins 4/5) is wired.

### Saved settings

`SID`, `TYP`, `MFD`, `SSN`, `BAU`, `PRF`, `CAL`, `TXO` are saved to `config.json` (written only on change). `DEF` restores the `DEFAULT_...` values.

- **Baud:** `BAU` replies at the old rate, then switches. Over RS-485 the change is kept only if a command is heard at the new rate within `BAUD_REVERT_S` (60 s); otherwise it reverts with `BAU ERR REVERT`. USB changes are kept immediately.
- **Send-only (`TXO ON`):** cleanest transmit, but RS-485 commands are ignored except for `TXO_GRACE_S` (30 s) after power-up. `TXO OFF` works any time over USB.
- **Profile** persists across reboots.

## Calibration

Two independent calibrations:

- **Pico calibration** — 120 Hz profile only, applied in software (`CAL ON`/`OFF`).
- **Chip calibration** — the BNO055's own, feeding the 60 Hz profile. Reported in `STAT`; only the magnetometer part can be saved (`MAG SAVE`).

Pico calibration subtracts the at-rest gyro offset and corrects accel per `ACCEL_CAL_MODE`:

| Mode | Behaviour |
|---|---|
| `"ZERO"` *(default)* | Subtracts the at-rest vector incl. gravity; reads ~0 at rest. Later tilt shows as slow acceleration |
| `"SCALE"` | Keeps gravity; scales magnitude to 9.80665 m/s² |
| `"OFF"` | Raw |

Runs when the 120 Hz profile starts and on `CAL NOW`. The sensor must be still for `CAL_SECONDS`; motion above `CAL_MAX_MOTION_DPS` / `CAL_MAX_ACCEL` rejects it. Good results go to `cal.txt`; on rejection the saved one is used (`WARN,NOCAL,USING_SAVED`) or none (`WARN,NOCAL`).

> **After remounting**, run `CAL NOW` in calm conditions or delete `cal.txt`. A `ZERO` calibration is tied to orientation.

## Watchdogs and fault handling

| Mechanism | Behaviour |
|---|---|
| Hardware WDT | Resets if the loop stalls for 3 s |
| I2C failures | After 12 in a row: bus recovery and sensor re-init |
| Frozen data | 2 s of identical samples: re-init |
| Mode check | Once a second; re-init if the chip left its mode |
| Repeated failure | 5 failed re-inits: reboot |
| Power / reset pin | Re-init power-cycles or resets the BNO055 if configured |
| Driver safety | GP2 pulled low so a crashed Pico can't hold the bus |

Re-init takes 1–2 s with no output.

## Button and LED

| Action | Result |
|---|---|
| Short blink every 5 s | Heartbeat |
| Solid | Stream paused |
| Fast blink | Command mode or programming mode |
| Tap BOOTSEL | Pause / resume |
| Hold BOOTSEL 3 s (or `PGM`) | Programming mode: reboots with watchdog off |

In programming mode, press **Stop** in Thonny to reach the REPL and upload. It reboots to normal after `PROGRAM_TIMEOUT_S` (15 min) if unused, and only lasts one boot. Send `PGM` from a plain terminal, not Thonny. During development set `USE_WDT = False`.

## Configuration

**Saved by command** (factory defaults in `main.py`):

| Variable | Default | Command |
|---|---|---|
| `DEFAULT_TYPE` | `"AC"` | `TYP` |
| `DEFAULT_SID` | `"001"` | `SID` |
| `DEFAULT_MFD` | `""` | `MFD` |
| `DEFAULT_SSN` | `"BNO055"` | `SSN` |
| `DEFAULT_PROFILE` | `60` | `PRF` |
| `DEFAULT_BAUD` | `115200` | `BAU` |
| `DEFAULT_CAL` | `True` | `CAL` |
| `DEFAULT_TX_ONLY` | `False` | `TXO` |

**Fixed in code:**

| Variable | Default | Description |
|---|---|---|
| `ALLOWED_BAUDS` / `MAX_BAUD` | 9600–115200 | Accepted rates / ceiling |
| `BAUD_REVERT_S` | `60` | Revert window for RS-485 baud changes |
| `TXO_GRACE_S` | `30` | Listening window after power-up in send-only mode |
| `ECHO_IDLE_MS` | `4` | Echo delay after typing pauses |
| `CMD_KEY` / `CMD_KEY_COUNT` | `0x15` / `1` | Command-mode key (0 = off) and repeats needed |
| `CMD_TIMEOUT_S` | `120` | Command-mode auto-exit (0 = never) |
| `CONFIG_FILE` / `CAL_FILE` / `BNO_CAL_FILE` | `config.json` / `cal.txt` / `bnocal.txt` | Saved files |
| `EOL` | `"\r\n"` | Line ending |
| `CHECKSUM` | `False` | Append `*XX` |
| `USE_WDT` / `WDT_TIMEOUT_MS` | `True` / `3000` | Hardware watchdog |
| `FUSION_MODE` | `"NDOF"` | `"NDOF"` or `"IMU"` (no heading) |
| `HDG_OFFSET_DEG` | `0.0` | Declination or mounting offset |
| `QUAT_INVERSE` | `False` | Fix mirrored `DIR` output |
| `PITCH_SIGN` / `ROLL_SIGN` | `1` / `1` | Flip `DIR` signs |
| `CHIP_ACCEL` | `"LIA"` | `"LIA"` (no gravity) or `"ACC"` |
| `RAW_APPLY_PICO_CAL` | `True` | Master switch for Pico calibration |
| `ACCEL_DECIMALS` / `GYRO_DECIMALS` | `2` / `3` | Output precision |
| `CHIP_STATUS_S` | `10` | `STAT` interval (0 = off) |
| `TAG_RAW` / `TAG_CHIP` | `"R"` / `"C"` | Data tags |
| `BNO_ADDR` | `0x28` | I2C address |
| `USE_EXT_CRYSTAL` | `True` | Use 32 kHz crystal |
| `ACCEL_CAL_MODE` | `"ZERO"` | `"ZERO"`, `"SCALE"`, `"OFF"` |
| `CAL_SECONDS` | `1.0` | Still time for calibration |
| `CAL_MAX_MOTION_DPS` / `CAL_MAX_ACCEL` | `3.0` / `0.5` | Motion limits during calibration |
| `SENSOR_PWR_PIN` / `SENSOR_RST_PIN` | `None` | Optional recovery pins |
| `USB_OUTPUT` | `False` | USB output at boot |
| `LED_PERIOD_MS` / `LED_ON_MS` | `5000` / `100` | Heartbeat |
| `USE_BUTTON` / `PROGRAM_HOLD_MS` | `True` / `3000` | BOOTSEL |
| `PROGRAM_TIMEOUT_S` | `900` | Programming-mode timeout (0 = never) |
| `SDA_PIN` / `SCL_PIN` / `DE_RE_PIN` | `4` / `5` / `2` | Pins |
| `STUCK_SECONDS` / `REINIT_AFTER` / `MAX_REINIT_FAILS` | `2` / `12` / `5` | Fault thresholds |

**Raw-profile sensor settings** (`start_sensor()`; skipped in fusion mode): accel ±4 g, 125 Hz bandwidth (`0x11`); gyro ±500 °/s, 64 Hz bandwidth (`0x32`). Output units are fixed via `UNIT_SEL`.

## DSM (NIDAS) configuration

| File | Contents |
|---|---|
| `sensor_catalog.xml` | Sensor definition |
| `callab.xml` | Port, sensor `id`, suffix |
| `configs.xml` | Plot-range variables |

### 1. Catalog entry

Paste above `</sensorcatalog>` (also in `ac001_catalog_entry.xml`):

```xml
    <!-- AC001: Pico 2 + BNO055 on the sonic. 115200 8N1, RS-485 half duplex, CRLF.
         <ID>,C,... 60 Hz chip | <ID>,R,... 120 Hz raw | <ID>,STAT,... every 10 s (60 Hz).
         Formats skip the ID field so TYP/SID changes don't break parsing. -->
    <serialSensor ID="AC001" class="DSMSerialSensor"
        baud="115200" parity="none" databits="8" stopbits="1" porttype="RS485_HALF">
        <sample id="1" scanfFormat="%*[^,],C,%f,%f,%f,%f,%f,%f" rate="60">
            <variable name="Ax_c" units="m/s^2" longname="AC001 accel X, chip profile" plotrange="$ACC_RANGE"/>
            <variable name="Ay_c" units="m/s^2" longname="AC001 accel Y, chip profile" plotrange="$ACC_RANGE"/>
            <variable name="Az_c" units="m/s^2" longname="AC001 accel Z, chip profile" plotrange="$ACC_RANGE"/>
            <variable name="Gx_c" units="rad/s" longname="AC001 angular rate X, chip profile" plotrange="$GYRO_RANGE"/>
            <variable name="Gy_c" units="rad/s" longname="AC001 angular rate Y, chip profile" plotrange="$GYRO_RANGE"/>
            <variable name="Gz_c" units="rad/s" longname="AC001 angular rate Z, chip profile" plotrange="$GYRO_RANGE"/>
        </sample>
        <sample id="2" scanfFormat="%*[^,],R,%f,%f,%f,%f,%f,%f" rate="120">
            <variable name="Ax_r" units="m/s^2" longname="AC001 accel X, raw profile" plotrange="$ACC_RANGE"/>
            <variable name="Ay_r" units="m/s^2" longname="AC001 accel Y, raw profile" plotrange="$ACC_RANGE"/>
            <variable name="Az_r" units="m/s^2" longname="AC001 accel Z, raw profile" plotrange="$ACC_RANGE"/>
            <variable name="Gx_r" units="rad/s" longname="AC001 angular rate X, raw profile" plotrange="$GYRO_RANGE"/>
            <variable name="Gy_r" units="rad/s" longname="AC001 angular rate Y, raw profile" plotrange="$GYRO_RANGE"/>
            <variable name="Gz_r" units="rad/s" longname="AC001 angular rate Z, raw profile" plotrange="$GYRO_RANGE"/>
        </sample>
        <sample id="3" scanfFormat="%*[^,],STAT,%d,%d,%d,%d" rate="0.1">
            <variable name="CalSys" units="none" longname="AC001 chip calibration, system"/>
            <variable name="CalGyr" units="none" longname="AC001 chip calibration, gyro"/>
            <variable name="CalAcc" units="none" longname="AC001 chip calibration, accel"/>
            <variable name="CalMag" units="none" longname="AC001 chip calibration, magnetometer"/>
        </sample>
        <message separator="\r\n" position="end" length="0"/>
    </serialSensor>
```

- Only the active profile's sample fills (60 Hz → samples 1 and 3; 120 Hz → sample 2). Replies and boot lines are ignored.
- Keep `baud` in step with `BAU`. Use `porttype="RS422"` for a receive-only hookup.
- The catalog `ID` is just the entry name; it needn't match the sensor ID. NIDAS adds the port suffix (`Ax_c.0`).

### 2. Port assignment (`callab.xml`)

```xml
<serialSensor IDREF="AC001" devicename="/dev/ttyDSM0" suffix=".0" id="100">
</serialSensor>
```

`id` must be unique on the DSM; samples report as `id` + sample number (`101`, `102`, `103`). Minimal site example:

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
                    file="${DSM}_%Y%m%d_%H%M%S.dat" length="43200">
                <mount dir="$DATAMNT"/>
            </fileset>
        </output>
    </dsm>
</site>
```

`sensor_catalog.xml` must sit beside `callab.xml` (it's `xi:include`d). The DSM runs the `<site>` matching `$SITE`.

### 3. Plot ranges (`configs.xml`, ISO-8859-1, plain ASCII)

```xml
<envvar name="ACC_RANGE" value="-10 10"/>
<envvar name="GYRO_RANGE" value="-2 2"/>
```

Widen accel (e.g. `-15 15`) if gravity is included.

### 4. Wiring (Bulgin, RS-485 half-duplex)

| Pin | Signal | Connect to |
|---|---|---|
| 1 | +12 VDC | Buck input |
| 2 | Shield | One end only |
| 4 | Tx−/Rx− | Breakout **B** (−) |
| 5 | Tx+/Rx+ | Breakout **A** (+) |
| 7 | Signal ground | Breakout GND |
| 8 | Power return | Buck ground |
| 3, 6 | — | Unconnected |

- The connector's `(A)`/`(B)` follow TIA (A = −), the opposite of the SP3485. Match by sign; swapping is harmless.
- The XML port mode only applies on new FTDI boards; others use a jumper. Check by measuring pins 5–4 with the sensor unplugged: several volts = DSM driving (wrong mode), ~0–0.5 V = correct idle.
- Data only: use pins 3 (−) and 6 (+) with `porttype="RS422"` (no commands possible).

### 5. Apply and check

1. Back up the three files, edit, validate with `xmllint --noout <file>`, restart the DSM.
2. `data_stats sock:localhost:30000` — sample counts and rates.
3. `data_dump -i 5,100 -A sock:localhost:30000` — raw text (`5,101` for parsed 60 Hz).
4. Raw port (NIDAS stopped): `stty -F /dev/ttyDSM0 115200 cs8 -parenb -cstopb raw -echo; cat /dev/ttyDSM0`.

**Gotchas:** command mode or `STR OFF` silences the sensor (raise the catalog `timeout` if NIDAS complains); update the catalog after `BAU`; stop NIDAS before manual access.

### 6. Link test scripts (run from Thonny, `main.py` stopped)

| Script | Purpose |
|---|---|
| `rs485_ping.py` | `ping` once a second |
| `rs485_test.py` | Fixed data line at 60 Hz |
| `rs485_u_test.py` | Continuous `U` for a scope |
| `rs485_static_test.py` | Steady high then low (A−B ≈ +2 V / −2 V) |
| `rs485_echo.py` | Echoes received lines (tests DSM → sensor) |

## Installation

1. Flash [MicroPython for Pico 2](https://micropython.org/download/RPI_PICO2/) (hold BOOTSEL, plug in, drop the UF2).
2. Copy `main.py` with Thonny.
3. Wire per the pin map.
4. Connect an RS-485 adapter at 115200 8N1, or open the USB port and send `USB ON`.

## PC viewer

Plots both profiles in a 2×2 grid (gyro left, accel right); only the active profile updates. Listen-only — send `PRF` from another tool to switch.

```
pip install pyserial matplotlib numpy
python ac001_viewer.py --enable-usb          # Pico on USB
python ac001_viewer.py --port COM5           # RS-485 adapter (Windows)
python ac001_viewer.py --port /dev/ttyUSB0   # RS-485 adapter (Linux/macOS)
python ac001_viewer.py --demo                # synthetic data
```

**Space** pauses, **c** clears. Verifies checksums, shows status and rates, auto-reconnects, and accepts any sensor ID.

## Limitations

- Heading needs the 60 Hz `NDOF` profile, a good magnetometer calibration, and a clean magnetic environment.
- Orientation (Euler/quaternion) isn't streamed.
- Profiles can't run together; switching drops 1–3 s of data.
- The chip's accel and gyro calibration restart at every power-up (only the magnetometer part can be saved).
- USB output is off at every boot unless `USB_OUTPUT = True`.
- Commands can be lost while streaming; use command mode or repeat them.
- Pico calibration assumes a still mast; otherwise the saved one is used.
