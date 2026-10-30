"""
rs485_test.py - bare-bones RS-485 transmit test for the Pico 2 + SP3485.

Run it from Thonny (Run current script). It does NOT need to be saved on the Pico.
It holds the SP3485 driver ON the whole time and sends one known line at 60 Hz,
115200 8N1, in the same format as the real firmware:

    AC001,C,0.00,-0.01,0.00,0.001,0.001,-0.001

No sensor, no watchdog, no driver switching. If the receiving end (DSM, USB-RS485 adapter,
scope) still sees garbage with this, the problem is the bus, wiring, baud or the receiver's
port settings, not the firmware.

Wiring: GP0 (pin 1) -> SP3485 DI,  GP2 (pin 4) -> SP3485 DE/RE (RTS), 3V3 -> SP3485 VCC.
"""
from machine import UART, Pin
import time

BAUD = 115200
RATE_HZ = 60
LINE = b"AC001,C,0.00,-0.01,0.00,0.001,0.001,-0.001\r\n"

de = Pin(2, Pin.OUT, value=1)            # driver permanently enabled
try:
    led = Pin("LED", Pin.OUT)
except (ValueError, TypeError):
    led = Pin(25, Pin.OUT)
uart = UART(0, baudrate=BAUD, tx=Pin(0), rx=Pin(1), bits=8, parity=None, stop=1, txbuf=256)

period = 1_000_000 // RATE_HZ
nxt = time.ticks_us()
n = 0
while True:
    uart.write(LINE)
    n += 1
    if n % RATE_HZ == 0:
        led.toggle()                      # LED toggles once a second while it runs
    nxt = time.ticks_add(nxt, period)
    d = time.ticks_diff(nxt, time.ticks_us())
    if d > 0:
        time.sleep_us(d)
    else:
        nxt = time.ticks_us()
