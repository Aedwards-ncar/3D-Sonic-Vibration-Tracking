from machine import I2C, UART, Pin, WDT
import machine
import os
import sys
import json
import select
import struct
import math
import time

# ============================ Settings ============================
# ---- Factory defaults. They apply only until you change them with a command (SID, TYP, MFD, SSN, BAU, PRF, CAL,
# ---- TXO). Changes are saved to flash (CONFIG_FILE) and survive power cycles. Command DEF restores these.
DEFAULT_TYPE = "AC"        # 2-character sensor type                 (command TYP)
DEFAULT_SID = "001"        # 3 digits: generation + number           (command SID).  ID = TYPE + SID, e.g. AC001
DEFAULT_MFD = ""           # date of manufacture, MM/DD/YYYY         (command MFD)
DEFAULT_SSN = "BNO055"     # accelerometer name / number             (command SSN)
DEFAULT_PROFILE = 60       # 60  = chip profile: BNO055 fusion output, no Pico calibration, 60 Hz   (command PRF)
                           # 120 = raw profile: BNO055 raw sensors + Pico calibration, 120 Hz
DEFAULT_BAUD = 115200      # RS-485 baud rate                        (command BAU)
DEFAULT_CAL = True         # Pico calibration enabled                (command CAL ON / CAL OFF)
DEFAULT_TX_ONLY = False    # False = send AND receive (half duplex)  (command TXO ON = send only, TXO OFF = send+receive)
TXO_GRACE_S = 30           # in send-only mode the Pico still listens for this long after power-up, so TXO can be undone
ALLOWED_BAUDS = (9600, 19200, 38400, 57600, 115200)
MAX_BAUD = 115200          # hard ceiling on the RS-485 baud rate
BAUD_REVERT_S = 60         # a BAU change over RS-485 is only kept if a valid command arrives at the new speed within this time
CONFIG_FILE = "config.json"
BNO_CAL_FILE = "bnocal.txt"   # BNO055 chip calibration saved by MAG SAVE, restored at every start in the 60 Hz fusion profile
CMD_KEY = 0x15              # single control byte that enters command mode, no Enter needed: 0x15 = Ctrl+U (0 = off)
CMD_KEY_COUNT = 1          # how many in a row are needed (use 2 if line noise ever triggers it by accident)
ECHO_IDLE_MS = 4            # command mode over RS-485: typed characters are echoed once the sender has paused this long
CMD_TIMEOUT_S = 120        # command mode (CMD) falls back to normal streaming after this long without a command (0 = never)
EOL = "\r\n"               # line ending
CHECKSUM = False           # True appends *XX (XOR of every char before the '*')

USE_WDT = True             # hardware watchdog (set False while developing in Thonny)
WDT_TIMEOUT_MS = 3000

FUSION_MODE = "NDOF"       # fusion mode used by the 60 Hz chip profile: "NDOF" (adds the magnetometer: needed for a compass heading,
                           # command DIR) or "IMU" (accel+gyro only: no absolute heading, DIR gives pitch and roll only)
HDG_OFFSET_DEG = 0.0       # added to the DIR heading: use it for magnetic declination (east positive) to get TRUE north, or a mounting offset
QUAT_INVERSE = False       # set True if DIR heading/tilt come out mirrored (conjugates the chip quaternion)
PITCH_SIGN = 1             # DIR pitch: +1 = front (x+) up is positive. -1 flips it
ROLL_SIGN = 1              # DIR roll:  +1 = right side down (left side up, y+ rising) is positive. -1 flips it
CHIP_ACCEL = "LIA"         # 60 Hz profile accel: "LIA" = chip linear accel (gravity removed by the chip)
                           #                      "ACC" = chip accel register (gravity included)
RAW_APPLY_PICO_CAL = True  # 120 Hz profile gets the Pico zero/scale/gyro-bias calibration (when CAL is ON). False = never
                           # (the 60 Hz profile never gets Pico calibration)
ACCEL_DECIMALS = 2         # printed decimals: chip resolution is 0.01 m/s^2, so 2 loses nothing
GYRO_DECIMALS = 3          # chip resolution is ~0.0011 rad/s, so 3 loses nothing
CHIP_STATUS_S = 10         # 60 Hz profile: seconds between AC001,STAT lines (chip calibration 0-3); 0 = off
TAG_RAW = "R"              # data line tags: AC001,R,... (120 Hz profile) and AC001,C,... (60 Hz profile)
TAG_CHIP = "C"

BNO_ADDR = 0x28            # 0x28 default, 0x29 if the ADR pin is pulled high
USE_EXT_CRYSTAL = True     # Adafruit BNO055 board has a 32 kHz crystal

# Pico calibration (accel + gyro) in the 120 Hz profile is switched with the command CAL ON / CAL OFF.
ACCEL_CAL_MODE = "ZERO"    # "ZERO"  = remove the at-rest vector incl. gravity (reads ~0 when still)
                           # "SCALE" = keep gravity, correct gain so |a| = 9.80665 at rest
                           # "OFF"   = raw accel, no correction
CAL_SECONDS = 1.0          # how long the sensor must sit still while calibrating
CAL_MAX_MOTION_DPS = 3.0   # gyro peak-to-peak above this during cal = moving, reject
CAL_MAX_ACCEL = 0.5        # accel peak-to-peak (m/s^2) above this during cal = moving, reject
CAL_FILE = "cal.txt"       # last good calibration, reused if a boot-time cal is rejected

SENSOR_PWR_PIN = None      # e.g. 6 = power BNO055 VIN from GP6 so firmware can power-cycle it
SENSOR_RST_PIN = None      # e.g. 7 = wire to BNO055 RST pin so firmware can hardware-reset it

USB_OUTPUT = False         # USB serial output at boot: OFF by default. Enable with the command USB ON (RS-485 or USB)
LED_PERIOD_MS = 5000       # onboard LED blinks once every 5 s
LED_ON_MS = 100            # length of each blink

USE_BUTTON = True          # BOOTSEL button: tap = stream on/off, hold = programming mode
PROGRAM_HOLD_MS = 3000     # hold BOOTSEL this long to enter programming mode
PROGRAM_FLAG = "program.flag"
PROGRAM_TIMEOUT_S = 900    # programming mode gives up and reboots to normal after this long (0 = never)

SDA_PIN, SCL_PIN = 4, 5    # I2C0
DE_RE_PIN = 2              # SP3485 DE+RE tied together: high = transmit, low = listen
# UART0: TX = GP0 -> DI, RX = GP1 <- RO

STUCK_SECONDS = 2             # identical raw data for this long = frozen sensor
REINIT_AFTER = 12             # consecutive read failures before re-initialising the sensor
MAX_REINIT_FAILS = 5          # failed re-inits in a row before a full MCU reset
# ==================================================================

# ---------------- Saved configuration ----------------
CMD_NAMES = ("SID", "TYP", "MFD", "SSN", "BAU", "PRF", "CAL", "TXO", "RST", "PGM", "STR", "USB", "INF", "DEF", "ID", "CMD", "RUN", "DIR", "MAG", "ECH")
ALIASES = {"BAUD": "BAU", "EXIT": "RUN"}   # alternative spellings
MAX_LINE = 52                      # longest data line in bytes (used to check a baud rate can carry the stream)

def baud_ok(b, hz):
    """True if b is an allowed baud rate that can carry the stream at hz lines/s with some headroom."""
    return b in ALLOWED_BAUDS and b <= MAX_BAUD and MAX_LINE * hz * 10 <= 0.85 * b

def norm_sid(v):
    if v and len(v) <= 3 and all("0" <= c <= "9" for c in v):
        return ("000" + v)[-3:]
    return None

def norm_typ(v):
    v = v.upper() if v else ""
    if len(v) == 2 and all(("A" <= c <= "Z") or ("0" <= c <= "9") for c in v):
        return v
    return None

def norm_mfd(v):
    if not v:
        return None
    for sep in ("-", "."):
        v = v.replace(sep, "/")
    p = v.split("/")
    if len(p) != 3:
        return None
    try:
        m, d, y = int(p[0]), int(p[1]), int(p[2])
    except ValueError:
        return None
    if y < 100:
        y += 2000
    if not (1 <= m <= 12 and 1 <= d <= 31 and 2000 <= y <= 2099):
        return None
    return "%02d/%02d/%04d" % (m, d, y)

def norm_ssn(v):
    v = v.strip() if v else ""
    return v if 1 <= len(v) <= 24 else None

def cfg_defaults():
    return {"typ": DEFAULT_TYPE, "sid": DEFAULT_SID, "mfd": DEFAULT_MFD, "ssn": DEFAULT_SSN,
            "bau": DEFAULT_BAUD, "prf": 120 if DEFAULT_PROFILE == 120 else 60,
            "cal": 1 if DEFAULT_CAL else 0, "txo": 1 if DEFAULT_TX_ONLY else 0}

def cfg_load():
    c = cfg_defaults()
    try:
        with open(CONFIG_FILE) as f:
            d = json.load(f)
        v = norm_typ(d.get("typ"))
        if v:
            c["typ"] = v
        v = norm_sid(d.get("sid"))
        if v:
            c["sid"] = v
        if d.get("mfd") == "" or norm_mfd(d.get("mfd")):
            c["mfd"] = d.get("mfd")
        v = norm_ssn(d.get("ssn"))
        if v:
            c["ssn"] = v
        if d.get("bau") in ALLOWED_BAUDS:
            c["bau"] = d.get("bau")
        if d.get("prf") in (60, 120):
            c["prf"] = d.get("prf")
        if d.get("cal") in (0, 1):
            c["cal"] = d.get("cal")
        if d.get("txo") in (0, 1):
            c["txo"] = d.get("txo")
    except Exception:
        pass                                    # no file or a damaged one: factory defaults
    if not baud_ok(c["bau"], c["prf"]):
        c["bau"] = 115200
    return c

def cfg_save():
    """Write the settings to flash. Written to a temp file and renamed, so a power cut cannot leave half a file."""
    try:
        tmp = CONFIG_FILE + ".tmp"
        with open(tmp, "w") as f:
            json.dump(cfg, f)
        try:
            os.rename(tmp, CONFIG_FILE)
        except OSError:
            os.remove(CONFIG_FILE)
            os.rename(tmp, CONFIG_FILE)
        return True
    except Exception:
        return False

def cfg_set(key, val):
    """Change one setting and save it, but only touch the flash if the value actually changed."""
    if cfg.get(key) != val:
        cfg[key] = val
        cfg_save()

cfg = cfg_load()
SENSOR_ID = cfg["typ"] + cfg["sid"]
LEGACY = ()                        # prefixed commands such as AC001,RESET (rebuilt whenever the ID changes)
cal_on = bool(cfg["cal"])
boot_ms = time.ticks_ms()

def build_cmds():
    """Recompute the sensor ID and the ID-prefixed (older style) command strings."""
    global SENSOR_ID, LEGACY
    SENSOR_ID = cfg["typ"] + cfg["sid"]
    p = SENSOR_ID + ","
    LEGACY = ((p + "RESET", ("RST", ())), (p + "PROGRAM", ("PGM", ())), (p + "CAL", ("CAL", ("NOW",))),
              (p + "PROFILE,60", ("PRF", ("60",))), (p + "PROFILE,120", ("PRF", ("120",))), (p + "PROFILE,?", ("PRF", ())),
              (p + "STOP", ("STR", ("OFF",))), (p + "START", ("STR", ("ON",))),
              (p + "USB,ON", ("USB", ("ON",))), (p + "USB,OFF", ("USB", ("OFF",))), (p + "USB,?", ("USB", ())))

build_cmds()

# BNO055 registers (page 0 unless noted)
REG_CHIP_ID = 0x00
REG_PAGE_ID = 0x07
REG_ACC_DATA = 0x08        # accel X/Y/Z (6 bytes), then mag (6), then gyro X/Y/Z (6)
REG_UNIT_SEL = 0x3B
REG_OPR_MODE = 0x3D
REG_PWR_MODE = 0x3E
REG_SYS_TRIGGER = 0x3F
REG_ACC_CONFIG = 0x08      # page 1
REG_GYR_CONFIG_0 = 0x0A    # page 1
REG_GYR_CONFIG_1 = 0x0B    # page 1

BNO_CHIP_ID = 0xA0
MODE_CONFIG = 0x00
MODE_AMG = 0x07            # raw accel+mag+gyro, non-fusion: sensor rates >100 Hz
MODE_IMU = 0x08            # fusion: accel + gyro
MODE_NDOF = 0x0C           # fusion: accel + gyro + mag
REG_CALIB_STAT = 0x35      # bits 7-6 sys, 5-4 gyro, 3-2 accel, 1-0 mag (0 = none, 3 = full)
LIA_OFFSET = 0x28 - 0x08   # linear-accel bytes inside the burst read that starts at 0x08
QUA_OFFSET = 0x20 - 0x08   # quaternion (w, x, y, z as int16, 1/16384) inside the same burst
EUL_OFFSET = 0x1A - 0x08   # Euler heading, roll, pitch (int16, 1/16 degree) inside the same burst

FUSION_OP = MODE_NDOF if FUSION_MODE == "NDOF" else MODE_IMU

raw = bytearray(38)                # burst read: accel, mag, gyro, euler, quat, linear-accel registers
rbuf = memoryview(raw)[:18]
profile = 60
sample_hz = 60
FUSION = True                      # True in the 60 Hz profile (chip fusion), False in the 120 Hz profile (raw AMG)
OP_MODE = FUSION_OP
stuck_limit = 120

def apply_profile(n):
    """Set the run-time parameters for profile 60 (chip, fusion) or 120 (raw, AMG)."""
    global profile, sample_hz, FUSION, OP_MODE, stuck_limit, rbuf
    profile = 120 if n == 120 else 60
    sample_hz = profile
    FUSION = profile == 60
    OP_MODE = FUSION_OP if FUSION else MODE_AMG
    stuck_limit = sample_hz * STUCK_SECONDS
    rbuf = memoryview(raw)[:(38 if FUSION else 18)]

apply_profile(cfg["prf"])

ACCEL_SCALE = 1.0 / 100.0  # UNIT_SEL m/s^2: 100 LSB per m/s^2
GYRO_SCALE = 1.0 / 900.0   # UNIT_SEL rad/s: 900 LSB per rad/s

# ---------------- Programming mode gate ----------------
# A long BOOTSEL press writes PROGRAM_FLAG and reboots. On that boot we stop here, BEFORE the
# watchdog is armed, and idle so Thonny can interrupt (Stop button) and upload new code.
# The flag is deleted immediately, so the next reset or power cycle runs normally.
try:
    os.stat(PROGRAM_FLAG)
    _prog = True
except OSError:
    _prog = False
if _prog:
    try:
        os.remove(PROGRAM_FLAG)
    except OSError:
        pass
    try:
        _l = Pin("LED", Pin.OUT)
    except (ValueError, TypeError):
        _l = Pin(25, Pin.OUT)
    _t0 = time.ticks_ms()
    while True:                        # Ctrl-C / Thonny Stop interrupts this and gives you the REPL
        _l.toggle()
        time.sleep_ms(100)
        if PROGRAM_TIMEOUT_S and time.ticks_diff(time.ticks_ms(), _t0) > PROGRAM_TIMEOUT_S * 1000:
            machine.reset()            # nobody connected in time: go back to normal operation

# ---------------- Watchdog ----------------
wdt = WDT(timeout=WDT_TIMEOUT_MS) if USE_WDT else None

def feed():
    if wdt:
        wdt.feed()

# ---------------- RS-485 ----------------
Pin(1, Pin.IN, Pin.PULL_UP)                       # idle-high RX when receiver is disabled
de_re = Pin(DE_RE_PIN, Pin.OUT, value=0)          # default: listening (send-only mode switches it on after TXO_GRACE_S)
uart = None
BAUD = 0
CHAR_US = 0
TURNAROUND_US = 0
TX_MARGIN_US = 0
tx_only_active = False             # True once the driver is held on permanently (send-only mode)
rx = b""

def make_uart(b):
    """(Re)create the RS-485 UART at baud rate b and recompute everything that depends on it."""
    global uart, BAUD, CHAR_US, TURNAROUND_US, TX_MARGIN_US
    if uart is not None:
        uart.deinit()
    BAUD = b
    CHAR_US = 10_000_000 // b                   # time for one 8N1 character on the wire
    TURNAROUND_US = CHAR_US + 30                # extra wait after the last byte before dropping DE
    TX_MARGIN_US = 2 * CHAR_US + 50             # DE is held at least this long beyond the computed end of the data
    uart = UART(0, baudrate=b, tx=Pin(0), rx=Pin(1), bits=8, parity=None, stop=1, txbuf=512, rxbuf=256)

make_uart(cfg["bau"])

# ---------------- Status LED ----------------
try:
    led = Pin("LED", Pin.OUT, value=0)     # Pico 2 and Pico 2 W
except (ValueError, TypeError):
    led = Pin(25, Pin.OUT, value=0)
led_on = False
led_t = time.ticks_ms()
streaming = True
cmd_mode = False                   # True while in command mode: no unsolicited output, the Pico just listens and answers
cmd_deadline = 0

def blink():
    """Non-blocking: LED on for LED_ON_MS at the start of every LED_PERIOD_MS."""
    global led_on, led_t
    if cmd_mode:
        led.value((time.ticks_ms() // 125) & 1)   # fast blink = command mode
        led_on = False
        return
    if not streaming:
        led.value(1)                       # solid on = stream paused
        led_on = False
        return
    now = time.ticks_ms()
    el = time.ticks_diff(now, led_t)
    if not led_on and el >= LED_PERIOD_MS:
        led.value(1)
        led_on = True
        led_t = now
    elif led_on and el >= LED_ON_MS:
        led.value(0)
        led_on = False

# ---------------- Stream control and BOOTSEL button ----------------
try:
    import rp2
    _bootsel = rp2.bootsel_button if USE_BUTTON else None
except (ImportError, AttributeError):
    _bootsel = None

def set_stream(on, announce=True):
    global streaming, led_on, led_t
    if on == streaming:
        return
    streaming = on
    led.value(0)
    led_on = False
    led_t = time.ticks_ms()
    if announce:
        send(SENSOR_ID + ",STREAM," + ("ON" if on else "OFF") + EOL)

def enter_program_mode(msg=None):
    send((msg if msg else SENSOR_ID + ",PROGRAM") + EOL)
    try:
        with open(PROGRAM_FLAG, "w") as f:
            f.write("1")
    except OSError:
        return                             # can't write the flag: stay running rather than lose control
    for _ in range(12):                    # fast flashes = going into programming mode
        led.toggle()
        feed()
        time.sleep_ms(80)
    machine.reset()

btn_t0 = None
btn_fired = False
btn_poll = time.ticks_ms()

def poll_button():
    global btn_t0, btn_fired, btn_poll
    if _bootsel is None:
        return
    now = time.ticks_ms()
    if time.ticks_diff(now, btn_poll) < 30:
        return
    btn_poll = now
    if _bootsel():
        if btn_t0 is None:
            btn_t0 = now
            btn_fired = False
        elif not btn_fired and time.ticks_diff(now, btn_t0) >= PROGRAM_HOLD_MS:
            btn_fired = True
            enter_program_mode()
    else:
        if btn_t0 is not None and not btn_fired:
            held = time.ticks_diff(now, btn_t0)
            if 40 <= held < 1500:
                set_stream(not streaming)
        btn_t0 = None

# ---------------- USB serial ----------------
usb_on = USB_OUTPUT                # run-time state: is the stream mirrored to USB serial?
usb_force = False                  # True while answering a command that was typed on USB: the reply always shows there
_poll = None                       # write-ready check for stdout
_pin = None                        # read-ready check for stdin
try:
    _poll = select.poll()
    _poll.register(sys.stdout, select.POLLOUT)
except Exception:
    _poll = None
try:
    _pin = select.poll()
    _pin.register(sys.stdin, select.POLLIN)
except Exception:
    _pin = None
urx = ""                           # USB command line being typed (kept apart from the RS-485 buffer)
usb_in_t = time.ticks_ms()

def usb_write(text):
    """Write to USB serial only if enabled, a terminal is connected and ready (never blocks)."""
    if not (usb_on or usb_force) or _poll is None:
        return
    try:
        if _poll.poll(0):
            sys.stdout.write(text)
    except Exception:
        pass

def usb_in_poll():
    """Accept the same commands typed on the USB serial port (works even while USB output is off)."""
    global urx, usb_in_t
    if _pin is None:
        return
    now = time.ticks_ms()
    if time.ticks_diff(now, usb_in_t) < 10:
        return
    usb_in_t = now
    try:
        n = 0
        while n < 64 and _pin.poll(0):
            ch = sys.stdin.read(1)
            n += 1
            if not ch:
                break
            if CMD_KEY and ch == chr(CMD_KEY):             # Ctrl+U typed on USB: enter command mode
                if len(cmd_q) < 8:
                    cmd_q.append(("CMD", [], 1, False))
            elif cmd_mode:                                 # command mode: line editing + echo, no waiting needed on USB
                for ln in ed_usb.feed(ch.encode()):
                    handle_line(ln, 1)
                echo_flush(ed_usb, 1)
            elif ch == "\n" or ch == "\r":
                line = urx.strip()
                urx = ""
                if line:
                    handle_line(line.encode(), 1)
            else:
                urx += ch
                if len(urx) > 64:
                    urx = urx[-64:]
    except Exception:
        pass

tx_busy = False
tx_until = 0

def tx_poll():
    """Drop the RS-485 driver only after BOTH the computed end-of-transmission time has passed
    AND the UART reports it is done. (Not trusting txdone() alone: if it reported done early the
    driver would switch off mid-message and the receiver would only see the first few characters.)"""
    global tx_busy, rx
    if tx_busy and time.ticks_diff(time.ticks_us(), tx_until) >= 0 and uart.txdone():
        time.sleep_us(TURNAROUND_US)
        if not tx_only_active:
            de_re.value(0)
            time.sleep_us(CHAR_US)
            uart.read()                          # drop the stray byte(s) the receiver can emit as the driver switches off
            rx = b""
        tx_busy = False

def send_async(text):
    """Start transmitting and return at once, so I2C reads / formatting overlap the wire time."""
    global rx, tx_busy, tx_until
    if not cmd_mode:             # in command mode keep queued input: the host may have sent several commands
        uart.read()              # drop any partial command; it can't complete across a TX
        rx = b""
    now = time.ticks_us()
    base = tx_until if (tx_busy and time.ticks_diff(tx_until, now) > 0) else now
    tx_until = time.ticks_add(base, len(text) * CHAR_US + TX_MARGIN_US)
    de_re.value(1)
    uart.write(text)             # RS-485 first: nothing may delay the samples on the wire
    tx_busy = True
    usb_write(text)              # USB mirror after, so a slow USB host can never hold up RS-485

def send(text):
    """Transmit one string and wait for it to finish, then return to listen mode."""
    global tx_busy
    send_async(text)
    t0 = time.ticks_ms()
    while tx_busy:
        tx_poll()
        if time.ticks_diff(time.ticks_ms(), t0) > 200:
            break
    if tx_busy:
        if not tx_only_active:
            de_re.value(0)
        tx_busy = False

cmd_q = []                         # parsed commands waiting for the main loop: (name, args, source, legacy_style)
cur_legacy = False                 # reply style for the command being processed
loop_reset = False                 # set when a command made the main loop restart its timing
baud_pending = None                # a BAU change that has not been confirmed yet
baud_prev = 0
baud_deadline = 0

echo_on = True                     # command mode: echo what is typed, start a new line on Enter, show a "> " prompt (command ECH)

class LineEd:
    """Tiny line editor used in command mode: collects typed characters, handles backspace, and builds the echo."""
    def __init__(self):
        self.buf = bytearray()
        self.out = bytearray()         # echo text waiting to be sent back
        self.eol = False               # last byte was a CR (so a following LF is the second half of CRLF)
        self.t = 0                     # ticks_ms of the last received byte

    def reset(self):
        self.buf = bytearray()
        self.out = bytearray()
        self.eol = False

    def feed(self, data):
        """Process received bytes. Returns the completed lines (bytes); the echo accumulates in self.out."""
        lines = []
        for b in data:
            if b == 13 or b == 10:
                if self.eol and b == 10:
                    self.eol = False               # LF after CR: same Enter
                    continue
                self.eol = b == 13
                line = bytes(self.buf)
                self.buf = bytearray()
                if echo_on:
                    self.out += b"\r\n"
                    if not line:
                        self.out += b"> "          # Enter on an empty line: just a fresh prompt
                if line:
                    lines.append(line)
            elif b == 8 or b == 127:               # backspace / delete
                self.eol = False
                if self.buf:
                    self.buf.pop()
                    if echo_on:
                        self.out += b"\x08 \x08"
            elif 32 <= b < 127:
                self.eol = False
                if len(self.buf) < 64:
                    self.buf.append(b)
                    if echo_on:
                        self.out.append(b)
        return lines

ed_rs = LineEd()                   # RS-485 side
ed_usb = LineEd()                  # USB side

def echo_flush(ed, src):
    """Send the pending echo text: on the RS-485 bus (src 0) or on the USB port (src 1)."""
    global usb_force
    if not ed.out:
        return
    t = bytes(ed.out).decode()
    ed.out = bytearray()
    if src == 1:
        usb_force = True
        usb_write(t)
        usb_force = False
    else:
        send(t)

def echo_poll():
    """Echo typed characters once the sender has paused (so we never talk over a sender still typing a line)."""
    if cmd_mode and ed_rs.out and time.ticks_diff(time.ticks_ms(), ed_rs.t) >= ECHO_IDLE_MS:
        echo_flush(ed_rs, 0)

def handle_line(line, src=0):
    """Parse one received line (bytes). src 0 = RS-485, 1 = USB. The command is queued for the main loop."""
    # Printable ASCII only, and match tolerantly: when the driver switches off the receiver can emit a stray byte or two
    # in front of a command, and that must not stop it being recognised.
    s = bytes(c for c in line if 32 <= c < 127).decode()
    if not s or len(cmd_q) >= 8:
        return
    for key, cmd in LEGACY:                       # older style: AC001,PROFILE,60
        if s.endswith(key):
            cmd_q.append((cmd[0], list(cmd[1]), src, True))
            return
    toks = s.split()
    t0 = toks[0].upper()
    name = None
    if t0 in ALIASES:
        name = ALIASES[t0]
    elif t0 in CMD_NAMES:
        name = t0
    else:
        for n in CMD_NAMES:
            if len(n) == 3 and len(t0) <= 8 and t0.endswith(n):
                name = n
                break
    if name:
        cmd_q.append((name, toks[1:], src, False))

key_run = 0                        # consecutive CMD_KEY bytes seen so far

def check_command():
    global rx, key_run
    data = uart.read()
    if not data:
        return
    if CMD_KEY and (key_run or CMD_KEY in data):      # Ctrl+U: enter command mode straight away, whatever else is in the buffer
        out = bytearray()
        for b in data:
            if b == CMD_KEY:
                key_run += 1
                if key_run >= CMD_KEY_COUNT:
                    key_run = 0
                    if len(cmd_q) < 8:
                        cmd_q.append(("CMD", [], 0, False))
            else:
                key_run = 0
                out.append(b)
        data = bytes(out)
        if not data:
            return
    if cmd_mode:                                      # command mode: line editing, echo, new line on Enter
        ed_rs.t = time.ticks_ms()
        lines = ed_rs.feed(data)
        for ln in lines:
            handle_line(ln, 0)
        if lines or (ed_rs.out and ed_rs.out.endswith(b"> ")):
            echo_flush(ed_rs, 0)                      # a line was completed: send the new line now, the reply follows it
        return
    rx += data.replace(b"\r", b"\n")
    if len(rx) > 64:
        rx = rx[-64:]
    while True:
        i = rx.find(b"\n")
        if i < 0:
            break
        line = rx[:i].strip()
        rx = rx[i + 1:]
        handle_line(line, 0)

def say(new, old=None):
    """Reply to a command: new style ("SID 002") or, for older style commands, the ID-prefixed form."""
    send((old if (cur_legacy and old) else new) + EOL)

gf = None                          # low-passed gravity vector (raw counts), kept in the 120 Hz profile for pitch/roll

def orientation():
    """Direction the sensor's x+ (front) face points and its tilt, from the chip quaternion (60 Hz fusion profile).
    Sensor frame: x+ front, y+ left, z+ up. Returns (heading_deg clockwise from magnetic north, pitch_deg front-up positive,
    roll_deg right-side-down positive, chip_euler_heading_deg) or None if there is no valid quaternion yet."""
    w, x, y, z = struct.unpack_from("<4h", raw, QUA_OFFSET)
    w = w / 16384.0
    x = x / 16384.0
    y = y / 16384.0
    z = z / 16384.0
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 0.5:
        return None
    w, x, y, z = w / n, x / n, y / n, z / n
    if QUAT_INVERSE:
        x, y, z = -x, -y, -z
    # device axes expressed in the world frame (East, North, Up) = columns of the rotation matrix
    fE = 1 - 2 * (y * y + z * z)               # front (x+)
    fN = 2 * (x * y + z * w)
    fU = 2 * (x * z - y * w)
    lU = 2 * (y * z + x * w)                   # left (y+), up component
    uU = 1 - 2 * (x * x + y * y)               # up (z+), up component
    hdg = (math.degrees(math.atan2(fE, fN)) + HDG_OFFSET_DEG) % 360.0
    pit = PITCH_SIGN * math.degrees(math.asin(max(-1.0, min(1.0, fU))))
    rol = ROLL_SIGN * math.degrees(math.atan2(lU, uU))
    eh = struct.unpack_from("<h", raw, EUL_OFFSET)[0] / 16.0
    return hdg, pit, rol, eh

def dir_line(verbose=False):
    """Reply text for the DIR command."""
    absolute = FUSION and FUSION_OP == MODE_NDOF
    mag = "-"
    if FUSION:
        o = orientation()
        if o is None:
            return "DIR ERR"
        hdg, pit, rol, eh = o
        if absolute:
            try:
                mag = str(read_reg(REG_CALIB_STAT) & 3)     # magnetometer calibration 0 (none) to 3 (full)
            except OSError:
                mag = "?"
    else:                                                    # 120 Hz raw profile: tilt only, from the gravity vector
        if gf is None:
            return "DIR ERR"
        gx, gy, gz = gf
        n = math.sqrt(gx * gx + gy * gy + gz * gz)
        if n < 1:
            return "DIR ERR"
        hdg, eh = None, None
        pit = PITCH_SIGN * math.degrees(math.asin(max(-1.0, min(1.0, gx / n))))
        rol = ROLL_SIGN * math.degrees(math.atan2(gy, gz))
    hs = "-"
    if hdg is not None and absolute:
        hv = (int(round(hdg * 10)) % 3600) / 10.0               # 359.96 prints as 0.0, never 360.0
        hs = "%.1f %s" % (hv, CARD[int(((hv + 22.5) % 360.0) // 45.0)])    # degrees plus the nearest of 8 compass points
    t = "DIR HDG=%s PIT=%.1f ROL=%.1f MAG=%s" % (hs, pit + 0.0, rol + 0.0, mag)      # + 0.0 turns -0.0 into 0.0
    if absolute:
        t += " PRO=%d" % (1 if load_chip_cal() else 0)           # 1 = a saved calibration (MAG SAVE) is in flash
    if verbose and eh is not None:
        t += " EULH=%.1f" % ((int(round(eh * 10)) % 3600) / 10.0)   # chip's own Euler heading (bearing of the chip Y axis), for checking
    return t

def info_line():
    return "INF %s SID=%s TYP=%s MFD=%s SSN=%s BAU=%d PRF=%d CAL=%s TXO=%s" % (
        SENSOR_ID, cfg["sid"], cfg["typ"], cfg["mfd"] or "-", cfg["ssn"], BAUD, profile,
        "ON" if cal_on else "OFF", "ON" if cfg["txo"] else "OFF")

def process_cmd(name, args, src, legacy):
    """Carry out one command and send its reply. Called from the main loop."""
    global cal_on, usb_on, tx_only_active, baud_pending, baud_prev, baud_deadline, loop_reset, cur_legacy, usb_force
    global cmd_mode, cmd_deadline, echo_on
    cur_legacy = legacy
    usb_force = src == 1
    if cmd_mode:                                      # any command keeps command mode alive
        cmd_deadline = time.ticks_add(time.ticks_ms(), CMD_TIMEOUT_S * 1000)
    try:
        if baud_pending is not None and src == 0:     # a valid command arrived at the new speed: keep it
            cfg_set("bau", baud_pending)
            baud_pending = None
        a0 = args[0].upper() if args else ""
        rest = " ".join(args)
        if name == "CMD" and a0 != "OFF":
            cmd_mode = True                           # enter command mode: streaming and status lines stop, Pico only listens
            cmd_deadline = time.ticks_add(time.ticks_ms(), CMD_TIMEOUT_S * 1000)
            echo_on = True                            # every session starts with echo and prompt on
            ed_rs.reset()
            ed_usb.reset()
            say("CMD ON")
        elif name == "RUN" or name == "CMD":
            cmd_mode = False                          # back to normal streaming
            say("CMD OFF")
        elif name == "ID":
            say("ID " + SENSOR_ID)
        elif name == "INF":
            say(info_line())
        elif name == "DIR":
            say(dir_line(a0 == "RAW"))
        elif name == "ECH":
            if a0 == "ON":
                echo_on = True
            elif a0 == "OFF":
                echo_on = False
            elif args:
                say("ECH ERR")
                a0 = "?"
            if a0 != "?":
                say("ECH ON" if echo_on else "ECH OFF")
        elif name == "MAG":
            if a0 == "SAVE":
                say(save_chip_cal(len(args) > 1 and args[1].upper() == "FORCE"))
                loop_reset = True
            elif a0 == "CLR":
                try:
                    os.remove(BNO_CAL_FILE)
                except OSError:
                    pass
                say("MAG CLEARED")
            elif not args:
                say(mag_status())
            else:
                say("MAG ERR")
        elif name == "SID":
            if not args:
                say("SID " + cfg["sid"])
            else:
                v = norm_sid(args[0])
                if v is None:
                    say("SID ERR")
                else:
                    cfg_set("sid", v)
                    build_cmds()
                    say("SID " + v)
        elif name == "TYP":
            if not args:
                say("TYP " + cfg["typ"])
            else:
                v = norm_typ(args[0])
                if v is None:
                    say("TYP ERR")
                else:
                    cfg_set("typ", v)
                    build_cmds()
                    say("TYP " + v)
        elif name == "MFD":
            if not args:
                say("MFD " + (cfg["mfd"] or "-"))
            else:
                v = norm_mfd(rest)
                if v is None:
                    say("MFD ERR")
                else:
                    cfg_set("mfd", v)
                    say("MFD " + v)
        elif name == "SSN":
            if not args:
                say("SSN " + cfg["ssn"])
            else:
                v = norm_ssn(rest)
                if v is None:
                    say("SSN ERR")
                else:
                    cfg_set("ssn", v)
                    say("SSN " + v)
        elif name == "BAU":
            if not args:
                say("BAU %d" % BAUD + (" PENDING" if baud_pending is not None else ""))
            else:
                try:
                    n = int(args[0])
                except ValueError:
                    n = 0
                if not baud_ok(n, sample_hz):
                    say("BAU ERR")
                elif n == BAUD:
                    say("BAU %d" % n)
                else:
                    say("BAU %d" % n)                 # confirm at the OLD speed, then switch
                    baud_prev = BAUD
                    make_uart(n)
                    if src == 1:
                        cfg_set("bau", n)             # typed locally on USB: keep it
                    else:
                        baud_pending = n              # over RS-485: kept only once a command is heard at the new speed
                        baud_deadline = time.ticks_add(time.ticks_ms(), BAUD_REVERT_S * 1000)
        elif name == "PRF":
            if not args:
                say("PRF %d" % profile, SENSOR_ID + ",PROFILE," + str(profile))
            else:
                try:
                    n = int(args[0])
                except ValueError:
                    n = 0
                if n not in (60, 120):
                    say("PRF ERR")
                elif not baud_ok(BAUD, n):
                    say("PRF ERR BAU")                # this baud rate can't carry that profile
                else:
                    if n != profile:
                        apply_profile(n)
                        start_sensor()                # re-initialise the chip in the new mode (1-2 s, no data)
                        profile_cal()
                        loop_reset = True
                    cfg_set("prf", n)
                    say("PRF %d" % profile, SENSOR_ID + ",PROFILE," + str(profile))
        elif name == "CAL":
            if a0 == "ON":
                cfg_set("cal", 1)
                cal_on = True
                if profile == 120:
                    profile_cal()
                    loop_reset = True
                say("CAL ON")
            elif a0 == "OFF":
                cfg_set("cal", 0)
                cal_on = False
                say("CAL OFF")
            elif a0 == "NOW":
                if not cal_on:
                    say("CAL OFF", SENSOR_ID + ",CAL,OFF")
                elif profile != 120:
                    say("CAL NA", SENSOR_ID + ",CAL,NA")      # Pico calibration is only used in the 120 Hz profile
                else:
                    ok = do_cal(False)
                    loop_reset = True
                    say("CAL OK" if ok else "CAL FAIL", SENSOR_ID + (",CAL,OK" if ok else ",CAL,FAIL"))
            elif not args:
                say("CAL ON" if cal_on else "CAL OFF")
            else:
                say("CAL ERR")
        elif name == "TXO":
            if a0 == "ON":
                cfg_set("txo", 1)
                say("TXO ON")
                tx_only_active = True                 # driver held on from now: the Pico can no longer hear RS-485
                de_re.value(1)
            elif a0 == "OFF":
                cfg_set("txo", 0)
                tx_only_active = False
                say("TXO OFF")
                de_re.value(0)
            elif not args:
                say("TXO ON" if cfg["txo"] else "TXO OFF")
            else:
                say("TXO ERR")
        elif name == "STR":
            if a0 == "ON":
                set_stream(True, False)
                say("STR ON", SENSOR_ID + ",STREAM,ON")
            elif a0 == "OFF":
                set_stream(False, False)
                say("STR OFF", SENSOR_ID + ",STREAM,OFF")
            elif not args:
                say("STR ON" if streaming else "STR OFF")
            else:
                say("STR ERR")
        elif name == "USB":
            if a0 == "ON":
                usb_on = True
                say("USB ON", SENSOR_ID + ",USB,ON")
            elif a0 == "OFF":
                say("USB OFF", SENSOR_ID + ",USB,OFF")     # still mirrored to USB, then USB goes quiet
                usb_on = False
            elif not args:
                say("USB ON" if usb_on else "USB OFF")
            else:
                say("USB ERR")
        elif name == "RST":
            say("RST", SENSOR_ID + ",RESETTING")
            time.sleep_ms(20)
            machine.reset()
        elif name == "PGM":
            enter_program_mode(None if legacy else "PGM")
        elif name == "DEF":
            try:
                os.remove(CONFIG_FILE)
            except OSError:
                pass
            say("DEF")
            time.sleep_ms(20)
            machine.reset()
        if cmd_mode and echo_on and name not in ("RST", "PGM", "DEF"):
            send("> ")                                # prompt: ready for the next command
    finally:
        usb_force = False

# ---------------- Sensor ----------------
pwr = None
if SENSOR_PWR_PIN is not None:
    pwr = Pin(SENSOR_PWR_PIN, Pin.OUT, value=1)
rst = None
if SENSOR_RST_PIN is not None:
    rst = Pin(SENSOR_RST_PIN, Pin.OUT, value=1)

i2c = None

def write_reg(reg, val):
    i2c.writeto_mem(BNO_ADDR, reg, bytes([val]))

def read_reg(reg):
    return i2c.readfrom_mem(BNO_ADDR, reg, 1)[0]

def read_raw():
    i2c.readfrom_mem_into(BNO_ADDR, REG_ACC_DATA, rbuf)       # one 18-byte burst
    return struct.unpack_from("<3h", raw, 0) + struct.unpack_from("<3h", raw, 12)  # ax ay az gx gy gz

def bus_recover():
    """Free the I2C bus if a slave is holding SDA low (clock out up to 9 pulses + STOP)."""
    scl = Pin(SCL_PIN, Pin.OPEN_DRAIN, value=1)
    sda = Pin(SDA_PIN, Pin.IN, Pin.PULL_UP)
    for _ in range(9):
        if sda.value():
            break
        scl.value(0)
        time.sleep_us(10)
        scl.value(1)
        time.sleep_us(10)
    scl.value(0)
    sda.init(Pin.OPEN_DRAIN, value=0)
    time.sleep_us(10)
    scl.value(1)
    time.sleep_us(10)
    sda.value(1)
    time.sleep_us(10)

def wait_chip_id(timeout_ms):
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < timeout_ms:
        feed()
        blink()
        try:
            if read_reg(REG_CHIP_ID) == BNO_CHIP_ID:
                return True
        except OSError:
            pass
        time.sleep_ms(20)
    return False

BNO_OFFSET_REG = 0x55              # 22 bytes: accel/mag/gyro offsets + accel/mag radius (read/write in CONFIG mode only)
CARD = ("N", "NE", "E", "SE", "S", "SW", "W", "NW")

def load_chip_cal():
    """The saved BNO055 calibration as 22 bytes, or None if nothing valid is saved."""
    try:
        with open(BNO_CAL_FILE) as f:
            h = f.read().strip()
        if len(h) == 44:
            return bytes(int(h[i:i + 2], 16) for i in range(0, 44, 2))
    except Exception:
        pass
    return None

def restore_chip_cal():
    """Write the saved calibration into the chip. Called inside start_sensor while the chip is in CONFIG mode."""
    if not FUSION:
        return                                   # only the fusion (60 Hz) profile uses the chip's calibration
    d = load_chip_cal()
    if d:
        try:
            i2c.writeto_mem(BNO_ADDR, BNO_OFFSET_REG, d)
        except OSError:
            pass

def save_chip_cal(force=False):
    """Read the chip's calibration offsets and save them to flash. Returns the reply text."""
    if not FUSION:
        return "MAG ERR PRF"                     # needs the 60 Hz fusion profile
    try:
        c = read_reg(REG_CALIB_STAT)
        if (c & 3) < 3 and not force:
            return "MAG ERR NOCAL"               # magnetometer not fully calibrated yet
        write_reg(REG_OPR_MODE, MODE_CONFIG)     # offsets can only be read in CONFIG mode (data pauses ~60 ms)
        time.sleep_ms(30)
        feed()
        d = i2c.readfrom_mem(BNO_ADDR, BNO_OFFSET_REG, 22)
        write_reg(REG_OPR_MODE, OP_MODE)
        time.sleep_ms(30)
        feed()
    except OSError:
        try:
            write_reg(REG_OPR_MODE, OP_MODE)
        except OSError:
            pass
        return "MAG ERR I2C"
    try:
        tmp = BNO_CAL_FILE + ".tmp"
        with open(tmp, "w") as f:
            f.write("".join("%02x" % b for b in d))
        try:
            os.rename(tmp, BNO_CAL_FILE)
        except OSError:
            os.remove(BNO_CAL_FILE)
            os.rename(tmp, BNO_CAL_FILE)
    except Exception:
        return "MAG ERR FLASH"
    return "MAG SAVED"

def mag_status():
    if not FUSION:
        return "MAG NA"
    try:
        c = read_reg(REG_CALIB_STAT)
    except OSError:
        return "MAG ERR I2C"
    return "MAG SYS=%d GYR=%d ACC=%d MAG=%d PRO=%d" % ((c >> 6) & 3, (c >> 4) & 3, (c >> 2) & 3, c & 3,
                                                       1 if load_chip_cal() else 0)

def start_sensor(cycle_power=False):
    """(Re)initialise bus and BNO055 in the active profile mode. Returns True on success."""
    global i2c
    feed()
    if pwr is not None and cycle_power:
        pwr.value(0)
        time.sleep_ms(300)
        pwr.value(1)
        feed()
    elif rst is not None and cycle_power:
        rst.value(0)
        time.sleep_ms(20)
        rst.value(1)
    bus_recover()
    i2c = I2C(0, sda=Pin(SDA_PIN), scl=Pin(SCL_PIN), freq=400_000, timeout=20000)
    try:
        if not wait_chip_id(2000):               # BNO055 needs ~650 ms after power/reset
            return False
        write_reg(REG_OPR_MODE, MODE_CONFIG)
        time.sleep_ms(30)
        try:
            write_reg(REG_SYS_TRIGGER, 0x20)     # software reset (chip may NACK while resetting)
        except OSError:
            pass
        time.sleep_ms(100)
        if not wait_chip_id(2000):
            return False
        write_reg(REG_OPR_MODE, MODE_CONFIG)
        time.sleep_ms(30)
        write_reg(REG_PWR_MODE, 0x00)            # normal power
        write_reg(REG_PAGE_ID, 0)
        write_reg(REG_SYS_TRIGGER, 0x80 if USE_EXT_CRYSTAL else 0x00)
        time.sleep_ms(10)
        write_reg(REG_UNIT_SEL, 0x02)            # accel m/s^2, gyro rad/s
        if not FUSION:                           # fusion modes fix ranges/bandwidth themselves (+/-4 g, 100 Hz)
            write_reg(REG_PAGE_ID, 1)
            write_reg(REG_ACC_CONFIG, 0x11)      # +/-4 g, 125 Hz BW (250 Hz data rate)
            write_reg(REG_GYR_CONFIG_0, 0x32)    # +/-500 dps, 64 Hz BW (200 Hz data rate)
            write_reg(REG_GYR_CONFIG_1, 0x00)    # normal
            write_reg(REG_PAGE_ID, 0)
        restore_chip_cal()                       # saved chip calibration (compass), fusion profile only
        write_reg(REG_OPR_MODE, OP_MODE)
        time.sleep_ms(50)
        feed()
        if read_reg(REG_OPR_MODE) != OP_MODE:
            return False
        read_raw()
        return True
    except OSError:
        feed()
        return False

# Calibration values (raw counts): accel offset x3, accel gain, gyro offset x3
abias = [0.0, 0.0, 0.0]
ascale = 1.0
gbias = [0.0, 0.0, 0.0]

def calibrate():
    """Average accel+gyro while still. Returns (abias, ascale, gbias) or None if moving / I2C error."""
    n = max(20, int(CAL_SECONDS * 200))
    s6 = [0] * 6
    lo = [32767] * 6
    hi = [-32768] * 6
    try:
        for _ in range(n):
            r = read_raw()
            for k in range(6):
                v = r[k]
                s6[k] += v
                if v < lo[k]:
                    lo[k] = v
                if v > hi[k]:
                    hi[k] = v
            time.sleep_ms(5)
            feed()
            blink()
    except OSError:
        return None
    g_lim = math.radians(CAL_MAX_MOTION_DPS) * 900.0
    a_lim = CAL_MAX_ACCEL * 100.0
    for k in range(3):
        if hi[k] - lo[k] > a_lim or hi[3 + k] - lo[3 + k] > g_lim:
            return None
    m = [x / n for x in s6]
    gb = m[3:6]
    if ACCEL_CAL_MODE == "ZERO":
        return (m[0:3], 1.0, gb)
    if ACCEL_CAL_MODE == "SCALE":
        mag = math.sqrt(m[0] * m[0] + m[1] * m[1] + m[2] * m[2])
        if mag < 300 or mag > 1500:        # nowhere near 1 g: sensor fault, don't trust it
            return None
        return ([0.0, 0.0, 0.0], 980.665 / mag, gb)
    return ([0.0, 0.0, 0.0], 1.0, gb)

def load_cal():
    try:
        with open(CAL_FILE) as f:
            v = [float(x) for x in f.read().split(",")]
        if len(v) == 7:
            return (v[0:3], v[3], v[4:7])
    except Exception:
        pass
    return None

def save_cal(c):
    old = load_cal()
    if old:
        a = c[0] + [c[1]] + c[2]
        b = old[0] + [old[1]] + old[2]
        if all(abs(x - y) < 2.0 for x, y in zip(a[:3] + a[4:], b[:3] + b[4:])) and abs(a[3] - b[3]) < 0.002:
            return                          # basically unchanged: spare the flash
    try:
        with open(CAL_FILE, "w") as f:
            f.write(",".join("%.4f" % x for x in c[0] + [c[1]] + c[2]))
    except Exception:
        pass

def do_cal(report=True):
    """Run a calibration; apply it if good, otherwise fall back to the saved one. Returns True if fresh."""
    global abias, ascale, gbias
    c = calibrate() if (cal_on and ACCEL_CAL_MODE is not None) else None
    if c:
        abias, ascale, gbias = c
        save_cal(c)
        if report:
            send(SENSOR_ID + ",CAL,OK" + EOL)
        return True
    if report:
        send(SENSOR_ID + ",CAL,FAIL" + EOL)
    return False

def checksum(s):
    c = 0
    for ch in s:
        c ^= ord(ch)
    return c

# ============================ Startup ============================
cause = machine.reset_cause()
cause_name = "PWR" if cause == machine.PWRON_RESET else ("WDT" if cause == machine.WDT_RESET else "OTHER")
send(SENSOR_ID + ",BOOT," + cause_name + EOL)

attempt = 0
while not start_sensor(attempt > 0):
    attempt += 1
    send(SENSOR_ID + ",ERR,NOSENSOR" + EOL)
    feed()
    time.sleep_ms(500)
    feed()

def profile_cal():
    """120 Hz profile only: Pico calibration (fresh if the sensor is still, else the saved one)."""
    global abias, ascale, gbias
    if not (cal_on and profile == 120):
        return
    if do_cal(report=False):
        send(SENSOR_ID + ",CAL,OK" + EOL)
        return
    saved = load_cal()
    if saved:
        abias, ascale, gbias = saved
        send(SENSOR_ID + ",WARN,NOCAL,USING_SAVED" + EOL)
    else:
        send(SENSOR_ID + ",WARN,NOCAL" + EOL)

send(SENSOR_ID + ",PROFILE," + str(profile) + EOL)
send(info_line() + EOL)
profile_cal()

# ============================ Main loop ============================
period_us = 1_000_000 // sample_hz
next_t = time.ticks_us()
fails = 0
reinit_fails = 0
same = 0
last = None
frame = 0
stat_s = 0
ZERO3 = (0.0, 0.0, 0.0)
FMT = "%s,%s," + ",".join(["%." + str(ACCEL_DECIMALS) + "f"] * 3 + ["%." + str(GYRO_DECIMALS) + "f"] * 3)

def make_line(tag, v):
    p = FMT % ((SENSOR_ID, tag) + tuple(v))
    if CHECKSUM:
        p += "*%02X" % checksum(p)
    return p + EOL

last_err_ms = time.ticks_add(time.ticks_ms(), -1000)

while True:
    feed()
    blink()
    if baud_pending is not None and time.ticks_diff(time.ticks_ms(), baud_deadline) >= 0:
        make_uart(baud_prev)                         # nobody confirmed the new speed in time: back to the saved one
        baud_pending = None
        send("BAU ERR REVERT" + EOL)
    if cfg["txo"] and not tx_only_active and not tx_busy and time.ticks_diff(time.ticks_ms(), boot_ms) >= TXO_GRACE_S * 1000:
        tx_only_active = True                        # send-only mode: hold the driver on from here
        de_re.value(1)
    while cmd_q:
        c = cmd_q.pop(0)
        process_cmd(c[0], c[1], c[2], c[3])
    if cmd_mode and CMD_TIMEOUT_S and time.ticks_diff(time.ticks_ms(), cmd_deadline) >= 0:
        cmd_mode = False                             # nobody finished the session: go back to streaming
        send("CMD OFF" + EOL)
    if loop_reset:
        loop_reset = False
        gf = None
        same = 0
        last = None
        frame = 0
        stat_s = 0
        fails = 0
        period_us = 1_000_000 // sample_hz
        next_t = time.ticks_us()
    line = None
    extra = None
    err = None
    bad_mode = False
    try:
        frame += 1
        if frame >= sample_hz:                   # once a second: confirm BNO055 hasn't dropped out of its mode
            frame = 0
            if read_reg(REG_OPR_MODE) != OP_MODE:
                bad_mode = True
                raise OSError(5)
            stat_s += 1
            if FUSION and CHIP_STATUS_S and stat_s >= CHIP_STATUS_S:
                stat_s = 0
                c = read_reg(REG_CALIB_STAT)
                extra = "%s,STAT,%d,%d,%d,%d%s" % (SENSOR_ID, (c >> 6) & 3, (c >> 4) & 3, (c >> 2) & 3, c & 3, EOL)
        r = read_raw()
        if r == last:
            same += 1
        else:
            same = 0
            last = r
        if same >= stuck_limit:
            raise OSError(5)
        fails = 0
        reinit_fails = 0
        if profile == 120:                       # raw profile: sensor registers + Pico calibration
            if gf is None:
                gf = [float(r[0]), float(r[1]), float(r[2])]
            else:
                gf[0] += (r[0] - gf[0]) * 0.02
                gf[1] += (r[1] - gf[1]) * 0.02
                gf[2] += (r[2] - gf[2]) * 0.02
            if RAW_APPLY_PICO_CAL and cal_on:
                ab, asc, gb = abias, ascale, gbias
            else:
                ab, asc, gb = ZERO3, 1.0, ZERO3
            line = make_line(TAG_RAW, (
                (r[0] - ab[0]) * ACCEL_SCALE * asc,
                (r[1] - ab[1]) * ACCEL_SCALE * asc,
                (r[2] - ab[2]) * ACCEL_SCALE * asc,
                (r[3] - gb[0]) * GYRO_SCALE,
                (r[4] - gb[1]) * GYRO_SCALE,
                (r[5] - gb[2]) * GYRO_SCALE,
            ))
        else:                                    # chip profile: straight from the BNO055, no Pico calibration
            if CHIP_ACCEL == "LIA":
                l = struct.unpack_from("<3h", raw, LIA_OFFSET)
            else:
                l = r
            line = make_line(TAG_CHIP, (
                l[0] * ACCEL_SCALE, l[1] * ACCEL_SCALE, l[2] * ACCEL_SCALE,
                r[3] * GYRO_SCALE, r[4] * GYRO_SCALE, r[5] * GYRO_SCALE,
            ))
    except OSError:
        if same >= stuck_limit:
            err = "STUCK"
            fails = REINIT_AFTER
        elif bad_mode:
            err = "MODE"
            fails = REINIT_AFTER
        else:
            err = "I2C"
            fails += 1
        if fails >= REINIT_AFTER:
            fails = 0
            same = 0
            last = None
            if start_sensor(reinit_fails > 0):
                reinit_fails = 0
            else:
                reinit_fails += 1
                if reinit_fails >= MAX_REINIT_FAILS:
                    send(SENSOR_ID + ",ERR,RESETTING" + EOL)
                    machine.reset()

    check_command()
    if cmd_mode:
        pass                                         # command mode: stay quiet so the bus is free for commands
    elif line and streaming:
        send_async(line)
    elif err and time.ticks_diff(time.ticks_ms(), last_err_ms) >= 1000:
        last_err_ms = time.ticks_ms()
        send(SENSOR_ID + ",ERR," + err + EOL)
    if extra and streaming and not cmd_mode:
        send(extra)

    # Listen for commands until the next frame is due
    next_t = time.ticks_add(next_t, period_us)
    while True:
        remaining = time.ticks_diff(next_t, time.ticks_us())
        if remaining <= 0:
            break
        tx_poll()
        check_command()
        echo_poll()
        usb_in_poll()
        poll_button()
        blink()
        time.sleep_us(remaining if remaining < 300 else 300)
    if time.ticks_diff(time.ticks_us(), next_t) > period_us:
        next_t = time.ticks_us()      # fell behind (e.g. after a re-init), resync
