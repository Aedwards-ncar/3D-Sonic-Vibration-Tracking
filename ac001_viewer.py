#!/usr/bin/env python3
"""
Live viewer for the AC001 sensor stream.

Line format:  AC001,ax,ay,az,gx,gy,gz[*XX]\r\n   (accel m/s^2, gyro rad/s)
Status lines (BOOT / ERR / WARN) are shown in the window and ignored for plotting.

Install:   pip install pyserial matplotlib numpy
Run:       python ac001_viewer.py                    (auto-detects the Pico on USB)
           python ac001_viewer.py --port COM5 --baud 230400
           python ac001_viewer.py --port /dev/ttyUSB0 --baud 230400   (USB-RS485 adapter)
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


# ------------------------------------------------------------------ parsing
def parse_line(line):
    """Return ('data', [6 floats]) or ('status', text) or None for junk."""
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
    if len(parts) == 7:
        try:
            return "data", [float(x) for x in parts[1:]]
        except ValueError:
            return None
    if len(parts) >= 2 and parts[1] in ("BOOT", "ERR", "WARN", "RESETTING"):
        return "status", ",".join(parts[1:])
    return None


# ------------------------------------------------------------------ data store
class Store:
    def __init__(self, window_s):
        self.window = window_s
        self.lock = threading.Lock()
        self.t = collections.deque()
        self.v = collections.deque()
        self.status = "waiting for data..."
        self.status_t = 0.0
        self.stamps = collections.deque(maxlen=240)    # for rate estimate
        self.total = 0
        self.bad = 0
        self.paused = False

    def add(self, vals):
        now = time.monotonic()
        with self.lock:
            self.total += 1
            self.stamps.append(now)
            if self.paused:
                return
            self.t.append(now)
            self.v.append(vals)
            cutoff = now - self.window - 1.0
            while self.t and self.t[0] < cutoff:
                self.t.popleft()
                self.v.popleft()

    def set_status(self, text):
        with self.lock:
            self.status = text
            self.status_t = time.monotonic()

    def clear(self):
        with self.lock:
            self.t.clear()
            self.v.clear()

    def snapshot(self):
        with self.lock:
            t = np.array(self.t, dtype=float)
            v = np.array(self.v, dtype=float).reshape(-1, 6)
            rate = 0.0
            if len(self.stamps) > 10:
                span = self.stamps[-1] - self.stamps[0]
                if span > 0:
                    rate = (len(self.stamps) - 1) / span
            return t, v, rate, self.status, self.status_t, self.total, self.bad


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
                            store.add(res[1])
                        else:
                            store.set_status(res[1])
                    if len(buf) > 1024:
                        buf = b""
        except Exception as e:
            store.set_status("port error: %s (retrying)" % e)
            time.sleep(1.0)


def demo_reader(store, stop, hz=120):
    t0 = time.monotonic()
    nxt = t0
    while not stop.is_set():
        t = time.monotonic() - t0
        vals = [
            0.8 * math.sin(2 * math.pi * 0.5 * t),
            0.5 * math.sin(2 * math.pi * 1.3 * t + 1),
            9.81 + 0.3 * math.sin(2 * math.pi * 0.2 * t),
            0.4 * math.sin(2 * math.pi * 0.3 * t),
            0.25 * math.sin(2 * math.pi * 0.9 * t + 2),
            0.1 * math.sin(2 * math.pi * 2.0 * t),
        ]
        store.add(vals)
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
    ap.add_argument("--baud", type=int, default=230400, help="baud rate (ignored for USB CDC)")
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

    fig, (ax_g, ax_a) = plt.subplots(1, 2, figsize=(13, 5.5), sharex=True)
    fig.suptitle("AC001", fontsize=13)
    colors = ("tab:red", "tab:green", "tab:blue")
    gl = [ax_g.plot([], [], c, lw=1.2, label=n)[0] for c, n in zip(colors, ("gx", "gy", "gz"))]
    al = [ax_a.plot([], [], c, lw=1.2, label=n)[0] for c, n in zip(colors, ("ax", "ay", "az"))]
    for ax, title, unit in ((ax_g, "Gyro", "rad/s"), (ax_a, "Accel", "m/s²")):
        ax.set_title(title)
        ax.set_xlabel("seconds ago")
        ax.set_ylabel(unit)
        ax.set_xlim(-args.window, 0)
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper left", ncol=3)
    g_txt = ax_g.text(0.99, 0.02, "", transform=ax_g.transAxes, ha="right", va="bottom", family="monospace", fontsize=9)
    a_txt = ax_a.text(0.99, 0.02, "", transform=ax_a.transAxes, ha="right", va="bottom", family="monospace", fontsize=9)
    info = fig.text(0.5, 0.01, "", ha="center", fontsize=9, family="monospace")
    fig.tight_layout(rect=(0, 0.04, 1, 0.97))

    ylim = {"g": [-1.0, 1.0], "a": [-1.0, 1.0]}

    def fit(ax, key, data):
        lo, hi = float(data.min()), float(data.max())
        pad = max(0.1 * (hi - lo), 0.05)
        want = [lo - pad, hi + pad]
        cur = ylim[key]
        cur[0] += (want[0] - cur[0]) * 0.2            # smooth autoscale
        cur[1] += (want[1] - cur[1]) * 0.2
        ax.set_ylim(cur[0], cur[1])

    def update(_):
        t, v, rate, status, st_t, total, bad = store.snapshot()
        if len(t):
            x = t - time.monotonic()
            for i in range(3):
                al[i].set_data(x, v[:, i])
                gl[i].set_data(x, v[:, 3 + i])
            vis = x >= -args.window
            if vis.any():
                fit(ax_a, "a", v[vis, 0:3])
                fit(ax_g, "g", v[vis, 3:6])
            a, g = v[-1, 0:3], v[-1, 3:6]
            a_txt.set_text("ax %+8.3f\nay %+8.3f\naz %+8.3f\n|a| %7.3f" % (a[0], a[1], a[2], np.linalg.norm(a)))
            g_txt.set_text("gx %+8.4f\ngy %+8.4f\ngz %+8.4f\n|g| %7.4f" % (g[0], g[1], g[2], np.linalg.norm(g)))
        age = time.monotonic() - st_t
        stat = status if age < 30 else status.split(" ")[0]
        info.set_text("%s | %.1f Hz | %d samples | %d bad lines%s" % (
            stat, rate, total, bad, " | PAUSED" if store.paused else ""))
        return gl + al + [g_txt, a_txt, info]

    def on_key(ev):
        if ev.key == " ":
            store.paused = not store.paused
        elif ev.key == "c":
            store.clear()

    fig.canvas.mpl_connect("key_press_event", on_key)
    anim = FuncAnimation(fig, update, interval=33, cache_frame_data=False)  # noqa: F841
    try:
        plt.show()
    finally:
        stop.set()


if __name__ == "__main__":
    main()
