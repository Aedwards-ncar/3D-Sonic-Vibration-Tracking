#!/usr/bin/env python3
"""
Live viewer for the AC001 sensor stream.

Data lines:   AC001,C,ax,ay,az,gx,gy,gz[*XX]   60 Hz profile (default): BNO055 chip output, no Pico calibration
              AC001,R,ax,ay,az,gx,gy,gz[*XX]   120 Hz profile: raw sensor + Pico calibration
Units:        accel m/s^2, gyro rad/s
Other lines (BOOT / PROFILE / CAL / STAT / ERR / WARN ...) are shown in the status bar.

Only one profile is active at a time. Top row = 60 Hz chip profile, bottom row = 120 Hz raw profile
(gyro | accel). The inactive row says so. The firmware profile is switched by sending
AC001,PROFILE,60 or AC001,PROFILE,120 to the Pico over RS-485.

Install:   pip install pyserial matplotlib numpy
Run:       python ac001_viewer.py                    (auto-detects the Pico on USB)
           python ac001_viewer.py --port COM5        (RS-485 adapter, 115200 baud)
           python ac001_viewer.py --port /dev/ttyUSB0
           python ac001_viewer.py --demo             (fake data, no hardware needed)

Keys:      space = pause/resume    c = clear
"""
import argparse
import collections
import math
import threading
import time

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

SENSOR_ID = "AC001"
TAGS = ("C", "R")


# ------------------------------------------------------------------ parsing
def parse_line(line):
    """Return ('data', tag, [6 floats]) or ('status', text) or None for junk."""
    line = line.strip()
    if not line.startswith(SENSOR_ID + ","):
        return None
    if "*" in line:                                  # optional XOR checksum
        body, _, cs = line.rpartition("*")
        try:
            want = int(cs, 16)
        except ValueError:
            return None
        got = 0
        for ch in body:
            got ^= ord(ch)
        if got != want:
            return None
        line = body
    parts = line.split(",")
    if len(parts) < 2:
        return None
    if parts[1] in TAGS:
        if len(parts) != 8:
            return None
        try:
            return "data", parts[1], [float(x) for x in parts[2:]]
        except ValueError:
            return None
    if parts[1].isalpha() and parts[1].isupper():    # BOOT, CAL, STAT, ERR, WARN, STREAM, ...
        return "status", ",".join(parts[1:])
    return None


# ------------------------------------------------------------------ data store
class Channel:
    def __init__(self):
        self.t = collections.deque()
        self.v = collections.deque()
        self.stamps = collections.deque(maxlen=240)
        self.total = 0


class Store:
    def __init__(self, window_s):
        self.window = window_s
        self.lock = threading.Lock()
        self.ch = {tag: Channel() for tag in TAGS}
        self.status = "waiting for data..."
        self.status_t = 0.0
        self.bad = 0
        self.paused = False

    def add(self, tag, vals):
        now = time.monotonic()
        with self.lock:
            c = self.ch[tag]
            c.total += 1
            c.stamps.append(now)
            if self.paused:
                return
            c.t.append(now)
            c.v.append(vals)
            cutoff = now - self.window - 1.0
            while c.t and c.t[0] < cutoff:
                c.t.popleft()
                c.v.popleft()

    def set_status(self, text):
        with self.lock:
            self.status = text
            self.status_t = time.monotonic()

    def clear(self):
        with self.lock:
            for c in self.ch.values():
                c.t.clear()
                c.v.clear()

    def snapshot(self, tag):
        with self.lock:
            c = self.ch[tag]
            t = np.array(c.t, dtype=float)
            v = np.array(c.v, dtype=float).reshape(-1, 6)
            rate = 0.0
            if len(c.stamps) > 10:
                span = c.stamps[-1] - c.stamps[0]
                if span > 0:
                    rate = (len(c.stamps) - 1) / span
            return t, v, rate, c.total

    def meta(self):
        with self.lock:
            return self.status, self.status_t, self.bad


# ------------------------------------------------------------------ sources
def serial_reader(port, baud, store, stop):
    import serial
    while not stop.is_set():
        try:
            with serial.Serial(port, baud, timeout=0.2) as ser:
                store.set_status("connected to %s @ %d" % (port, baud))
                buf = b""
                while not stop.is_set():
                    chunk = ser.read(512)
                    if not chunk:
                        continue
                    buf += chunk
                    while b"\n" in buf:
                        raw, buf = buf.split(b"\n", 1)
                        res = parse_line(raw.decode("ascii", "ignore"))
                        if res is None:
                            if raw.strip():
                                store.bad += 1
                        elif res[0] == "data":
                            store.add(res[1], res[2])
                        else:
                            store.set_status(res[1])
                    if len(buf) > 1024:
                        buf = b""
        except Exception as e:
            store.set_status("port error: %s (retrying)" % e)
            time.sleep(1.0)


def demo_reader(store, stop):
    """Synthetic data. Starts in the 60 Hz profile and flips profile every 8 s, like a PROFILE command."""
    t0 = time.monotonic()
    nxt = t0
    while not stop.is_set():
        t = time.monotonic() - t0
        vals = [
            0.8 * math.sin(2 * math.pi * 0.5 * t),
            0.5 * math.sin(2 * math.pi * 1.3 * t + 1),
            0.3 * math.sin(2 * math.pi * 0.2 * t),
            0.4 * math.sin(2 * math.pi * 0.3 * t),
            0.25 * math.sin(2 * math.pi * 0.9 * t + 2),
            0.1 * math.sin(2 * math.pi * 2.0 * t),
        ]
        hz = 60 if int(t // 8) % 2 == 0 else 120
        store.add("C" if hz == 60 else "R", vals)
        nxt += 1.0 / hz
        time.sleep(max(0.0, nxt - time.monotonic()))


def autodetect_port():
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    for p in ports:                                   # Raspberry Pi VID is 0x2E8A
        if p.vid == 0x2E8A:
            return p.device
    for p in ports:
        if any(k in (p.description or "").lower() for k in ("pico", "usb serial", "usb-serial", "ch340", "ftdi", "cp210")):
            return p.device
    return ports[0].device if ports else None


# ------------------------------------------------------------------ GUI
def main():
    ap = argparse.ArgumentParser(description="AC001 live gyro/accel viewer")
    ap.add_argument("--port", help="serial port (default: auto-detect)")
    ap.add_argument("--baud", type=int, default=115200, help="baud rate (ignored for USB CDC)")
    ap.add_argument("--window", type=float, default=10.0, help="seconds of history shown")
    ap.add_argument("--demo", action="store_true", help="synthetic data, no hardware")
    args = ap.parse_args()

    store = Store(args.window)
    stop = threading.Event()

    if args.demo:
        target, targs = demo_reader, (store, stop)
        store.set_status("DEMO MODE (synthetic data)")
    else:
        port = args.port or autodetect_port()
        if not port:
            raise SystemExit("No serial port found. Use --port COMx or --demo.")
        target, targs = serial_reader, (port, args.baud, store, stop)
    threading.Thread(target=target, args=targs, daemon=True).start()

    fig, axes = plt.subplots(2, 2, figsize=(13, 8), sharex=True)
    fig.suptitle("AC001", fontsize=13)
    colors = ("tab:red", "tab:green", "tab:blue")
    rows = {"C": 0, "R": 1}
    row_title = {"C": "60 Hz chip profile (BNO055)", "R": "120 Hz raw profile (Pico-calibrated)"}
    lines, texts, notes = {}, {}, {}
    ylim = {}
    for tag in TAGS:
        ax_g, ax_a = axes[rows[tag]]
        lines[tag] = (
            [ax_g.plot([], [], c, lw=1.2, label=n)[0] for c, n in zip(colors, ("gx", "gy", "gz"))],
            [ax_a.plot([], [], c, lw=1.2, label=n)[0] for c, n in zip(colors, ("ax", "ay", "az"))],
        )
        for ax, title, unit in ((ax_g, "Gyro", "rad/s"), (ax_a, "Accel", "m/s²")):
            ax.set_title("%s: %s" % (row_title[tag], title), fontsize=10)
            ax.set_ylabel(unit)
            ax.set_xlim(-args.window, 0)
            ax.grid(True, alpha=0.3)
            ax.legend(loc="upper left", ncol=3, fontsize=8)
        axes[1][0].set_xlabel("seconds ago")
        axes[1][1].set_xlabel("seconds ago")
        texts[tag] = (
            ax_g.text(0.99, 0.02, "", transform=ax_g.transAxes, ha="right", va="bottom", family="monospace", fontsize=8),
            ax_a.text(0.99, 0.02, "", transform=ax_a.transAxes, ha="right", va="bottom", family="monospace", fontsize=8),
        )
        notes[tag] = ax_g.text(0.5, 0.5, "", transform=ax_g.transAxes, ha="center", va="center", fontsize=10, alpha=0.6)
        ylim[tag] = ([-1.0, 1.0], [-1.0, 1.0])
    info = fig.text(0.5, 0.005, "", ha="center", fontsize=9, family="monospace")
    fig.tight_layout(rect=(0, 0.03, 1, 0.97))

    def fit(ax, cur, data):
        lo, hi = float(data.min()), float(data.max())
        pad = max(0.1 * (hi - lo), 0.05)
        cur[0] += (lo - pad - cur[0]) * 0.2           # smooth autoscale
        cur[1] += (hi + pad - cur[1]) * 0.2
        ax.set_ylim(cur[0], cur[1])

    def update(_):
        rates = {}
        for tag in TAGS:
            t, v, rate, total = store.snapshot(tag)
            rates[tag] = (rate, total)
            ax_g, ax_a = axes[rows[tag]]
            gl, al = lines[tag]
            if len(t):
                notes[tag].set_text("")
                x = t - time.monotonic()
                for i in range(3):
                    al[i].set_data(x, v[:, i])
                    gl[i].set_data(x, v[:, 3 + i])
                vis = x >= -args.window
                if vis.any():
                    fit(ax_a, ylim[tag][1], v[vis, 0:3])
                    fit(ax_g, ylim[tag][0], v[vis, 3:6])
                a, g = v[-1, 0:3], v[-1, 3:6]
                texts[tag][1].set_text("ax %+8.3f\nay %+8.3f\naz %+8.3f\n|a| %7.3f" % (a[0], a[1], a[2], np.linalg.norm(a)))
                texts[tag][0].set_text("gx %+8.4f\ngy %+8.4f\ngz %+8.4f\n|g| %7.4f" % (g[0], g[1], g[2], np.linalg.norm(g)))
            else:
                notes[tag].set_text("not the active profile")
        status, st_t, bad = store.meta()
        stat = status if time.monotonic() - st_t < 30 else status.split(" ")[0]
        info.set_text("%s | 60 Hz profile: %.1f Hz (%d) | 120 Hz profile: %.1f Hz (%d) | %d bad lines%s" % (
            stat, rates["C"][0], rates["C"][1], rates["R"][0], rates["R"][1], bad,
            " | PAUSED" if store.paused else ""))
        return [info]

    def on_key(ev):
        if ev.key == " ":
            store.paused = not store.paused
        elif ev.key == "c":
            store.clear()

    fig.canvas.mpl_connect("key_press_event", on_key)
    anim = FuncAnimation(fig, update, interval=40, cache_frame_data=False)  # noqa: F841
    try:
        plt.show()
    finally:
        stop.set()


if __name__ == "__main__":
    main()
