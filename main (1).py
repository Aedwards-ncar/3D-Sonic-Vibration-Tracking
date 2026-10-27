from machine import I2C, UART, Pin, WDT
import machine
import os
import sys
import select
import struct
import math
import time

# ============================ Settings ============================
SENSOR_ID = "AC001"        # AC = sensor type, 0 = generation, 01 = sensor number
DEFAULT_PROFILE = 60       # 60  = chip profile: BNO055 fusion output, no Pico calibration, 60 Hz  (default)
                           # 120 = raw profile: BNO055 raw sensors + Pico calibration, 120 Hz
                           # Switch at run time with AC001,PROFILE,60 / AC001,PROFILE,120
PERSIST_PROFILE = False    # True = remember the last commanded profile across reboots
PROFILE_FILE = "profile.txt"
MAX_BAUD = 115200          # hard ceiling on the RS-485 baud rate
BAUD = 115200              # RS-485 baud (anything above MAX_BAUD is clamped down to it)
EOL = "\r\n"               # line ending
CHECKSUM = False           # True appends *XX (XOR of every char before the '*')

USE_WDT = True             # hardware watchdog (set False while developing in Thonny)
WDT_TIMEOUT_MS = 3000

FUSION_MODE = "IMU"        # fusion mode used by the 60 Hz chip profile: "IMU" (accel+gyro) or "NDOF" (adds magnetometer)
CHIP_ACCEL = "LIA"         # 60 Hz profile accel: "LIA" = chip linear accel (gravity removed by the chip)
                           #                      "ACC" = chip accel register (gravity included)
RAW_APPLY_PICO_CAL = True  # 120 Hz profile gets the Pico zero/scale/gyro-bias calibration. False = uncorrected
                           # (the 60 Hz profile never gets Pico calibration)
ACCEL_DECIMALS = 2         # printed decimals: chip resolution is 0.01 m/s^2, so 2 loses nothing
GYRO_DECIMALS = 3          # chip resolution is ~0.0011 rad/s, so 3 loses nothing
CHIP_STATUS_S = 10         # 60 Hz profile: seconds between AC001,STAT lines (chip calibration 0-3); 0 = off
TAG_RAW = "R"              # data line tags: AC001,R,... (120 Hz profile) and AC001,C,... (60 Hz profile)
TAG_CHIP = "C"

BNO_ADDR = 0x28            # 0x28 default, 0x29 if the ADR pin is pulled high
USE_EXT_CRYSTAL = True     # Adafruit BNO055 board has a 32 kHz crystal

CALIBRATE_GYRO = True      # auto-calibrate at boot and on command (name kept; covers accel + gyro)
ACCEL_CAL_MODE = "ZERO"    # "ZERO"  = remove the at-rest vector incl. gravity (reads ~0 when still)
                           # "SCALE" = keep gravity, correct gain so |a| = 9.80665 at rest
                           # "OFF"   = raw accel, no correction
CAL_SECONDS = 1.0          # how long the sensor must sit still while calibrating
CAL_MAX_MOTION_DPS = 3.0   # gyro peak-to-peak above this during cal = moving, reject
CAL_MAX_ACCEL = 0.5        # accel peak-to-peak (m/s^2) above this during cal = moving, reject
CAL_FILE = "cal.txt"       # last good calibration, reused if a boot-time cal is rejected

SENSOR_PWR_PIN = None      # e.g. 6 = power BNO055 VIN from GP6 so firmware can power-cycle it
SENSOR_RST_PIN = None      # e.g. 7 = wire to BNO055 RST pin so firmware can hardware-reset it

USB_OUTPUT = True          # also stream every line over USB serial when a terminal is open
LED_PERIOD_MS = 5000       # onboard LED blinks once every 5 s
LED_ON_MS = 100            # length of each blink

USE_BUTTON = True          # BOOTSEL button: tap = stream on/off, hold = programming mode
PROGRAM_HOLD_MS = 3000     # hold BOOTSEL this long to enter programming mode
PROGRAM_FLAG = "program.flag"

SDA_PIN, SCL_PIN = 4, 5    # I2C0
DE_RE_PIN = 2              # SP3485 DE+RE tied together: high = transmit, low = listen
# UART0: TX = GP0 -> DI, RX = GP1 <- RO

RESET_CMD = (SENSOR_ID + ",RESET").encode()   # send "AC001,RESET\r\n" from the host
PROF60_CMD = (SENSOR_ID + ",PROFILE,60").encode()     # switch to the 60 Hz chip profile
PROF120_CMD = (SENSOR_ID + ",PROFILE,120").encode()   # switch to the 120 Hz raw profile
PROFQ_CMD = (SENSOR_ID + ",PROFILE,?").encode()       # report the active profile
STOP_CMD = (SENSOR_ID + ",STOP").encode()     # pause the data stream (commands still work)
START_CMD = (SENSOR_ID + ",START").encode()   # resume the data stream
CAL_CMD = (SENSOR_ID + ",CAL").encode()       # send "AC001,CAL\r\n" to recalibrate (sensor must be still)

STUCK_SECONDS = 2             # identical raw data for this long = frozen sensor
REINIT_AFTER = 12             # consecutive read failures before re-initialising the sensor
MAX_REINIT_FAILS = 5          # failed re-inits in a row before a full MCU reset
# ==================================================================

if BAUD > MAX_BAUD:
    BAUD = MAX_BAUD

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

def load_profile():
    try:
        with open(PROFILE_FILE) as f:
            return 120 if f.read().strip() == "120" else 60
    except Exception:
        return None

def save_profile():
    if PERSIST_PROFILE:
        try:
            with open(PROFILE_FILE, "w") as f:
                f.write(str(profile))
        except Exception:
            pass

_p = load_profile() if PERSIST_PROFILE else None
apply_profile(_p if _p else DEFAULT_PROFILE)

ACCEL_SCALE = 1.0 / 100.0  # UNIT_SEL m/s^2: 100 LSB per m/s^2
GYRO_SCALE = 1.0 / 900.0   # UNIT_SEL rad/s: 900 LSB per rad/s
TURNAROUND_US = (10_000_000 // BAUD) + 30   # one char time + margin before dropping DE

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
    while True:                        # Ctrl-C / Thonny Stop interrupts this and gives you the REPL
        _l.toggle()
        time.sleep_ms(100)

# ---------------- Watchdog ----------------
wdt = WDT(timeout=WDT_TIMEOUT_MS) if USE_WDT else None

def feed():
    if wdt:
        wdt.feed()

# ---------------- RS-485 ----------------
Pin(1, Pin.IN, Pin.PULL_UP)                       # idle-high RX when receiver is disabled
de_re = Pin(DE_RE_PIN, Pin.OUT, value=0)          # default: listening
uart = UART(0, baudrate=BAUD, tx=Pin(0), rx=Pin(1), bits=8, parity=None,
            stop=1, txbuf=512, rxbuf=256)
rx = b""

# ---------------- Status LED ----------------
try:
    led = Pin("LED", Pin.OUT, value=0)     # Pico 2 and Pico 2 W
except (ValueError, TypeError):
    led = Pin(25, Pin.OUT, value=0)
led_on = False
led_t = time.ticks_ms()
streaming = True

def blink():
    """Non-blocking: LED on for LED_ON_MS at the start of every LED_PERIOD_MS."""
    global led_on, led_t
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

def set_stream(on):
    global streaming, led_on, led_t
    if on == streaming:
        return
    streaming = on
    led.value(0)
    led_on = False
    led_t = time.ticks_ms()
    send(SENSOR_ID + ",STREAM," + ("ON" if on else "OFF") + EOL)

def enter_program_mode():
    send(SENSOR_ID + ",PROGRAM" + EOL)
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
_poll = None
if USB_OUTPUT:
    try:
        _poll = select.poll()
        _poll.register(sys.stdout, select.POLLOUT)
    except Exception:
        _poll = None

def usb_write(text):
    """Write to USB serial only if a terminal is connected and ready (never blocks)."""
    if _poll is None:
        return
    try:
        if _poll.poll(0):
            sys.stdout.write(text)
    except Exception:
        pass

tx_busy = False

def tx_poll():
    """Drop the RS-485 driver once the last byte has left the UART. Call this often."""
    global tx_busy
    if tx_busy and uart.txdone():
        time.sleep_us(TURNAROUND_US)
        de_re.value(0)
        tx_busy = False

def send_async(text):
    """Start transmitting and return at once, so I2C reads / formatting overlap the wire time."""
    global rx, tx_busy
    usb_write(text)
    uart.read()                  # drop any partial command; it can't complete across a TX
    rx = b""
    de_re.value(1)
    uart.write(text)
    tx_busy = True

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
        de_re.value(0)
        tx_busy = False

cal_req = False
stream_req = None
profile_req = None

def do_reset():
    send(SENSOR_ID + ",RESETTING" + EOL)
    time.sleep_ms(20)
    machine.reset()

def check_command():
    global rx, cal_req, stream_req, profile_req
    data = uart.read()
    if not data:
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
        if line == RESET_CMD:
            do_reset()
        elif line == CAL_CMD:
            cal_req = True
        elif line == PROF60_CMD:
            profile_req = 60
        elif line == PROF120_CMD:
            profile_req = 120
        elif line == PROFQ_CMD:
            profile_req = 0
        elif line == STOP_CMD:
            stream_req = False
        elif line == START_CMD:
            stream_req = True

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
    c = calibrate() if (CALIBRATE_GYRO and ACCEL_CAL_MODE is not None) else None
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
    if not (CALIBRATE_GYRO and profile == 120):
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
    if stream_req is not None:
        set_stream(stream_req)
        stream_req = None
    if profile_req is not None:
        n = profile_req
        profile_req = None
        if n in (60, 120) and n != profile:
            apply_profile(n)
            save_profile()
            start_sensor()                       # re-initialise the chip in the new mode (1-2 s, no data)
            profile_cal()
            period_us = 1_000_000 // sample_hz
        send(SENSOR_ID + ",PROFILE," + str(profile) + EOL)
        same = 0
        last = None
        frame = 0
        stat_s = 0
        fails = 0
        next_t = time.ticks_us()
    if cal_req:
        cal_req = False
        if profile == 120:
            do_cal()
        else:
            send(SENSOR_ID + ",CAL,NA" + EOL)    # Pico calibration is not used in the 60 Hz chip profile
        same = 0
        last = None
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
            if RAW_APPLY_PICO_CAL:
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
    if line and streaming:
        send_async(line)
    elif err and time.ticks_diff(time.ticks_ms(), last_err_ms) >= 1000:
        last_err_ms = time.ticks_ms()
        send(SENSOR_ID + ",ERR," + err + EOL)
    if extra and streaming:
        send(extra)

    # Listen for commands until the next frame is due
    next_t = time.ticks_add(next_t, period_us)
    while True:
        remaining = time.ticks_diff(next_t, time.ticks_us())
        if remaining <= 0:
            break
        tx_poll()
        check_command()
        poll_button()
        blink()
        time.sleep_us(remaining if remaining < 300 else 300)
    if time.ticks_diff(time.ticks_us(), next_t) > period_us:
        next_t = time.ticks_us()      # fell behind (e.g. after a re-init), resync
