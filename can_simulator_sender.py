"""
CAN Simulator Sender
====================
Reads the DBC definition, generates realistic random signal
values, encodes them into CAN frames and transmits via a PCAN-USB interface.

Requirements:
    pip install python-can cantools

Usage (GUI — default):
    python can_simulator_sender.py

Usage (headless CLI):
    python can_simulator_sender.py --no-gui [--channel PCAN_USBBUS1] [--bitrate 500000]
"""

import argparse
import datetime
import math
import os
import random
import struct
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import can
import cantools

# ─── DBC DEFINITION (embedded so no file path needed) ───────────────────────

DBC_STRING = """
VERSION ""

NS_ :

BS_:

BU_: Vector__XXX

BO_ 528 RC40VP1: 8 Vector__XXX
 SG_ ACC_PP : 7|8@0+ (0.4,0) [0|102] "%" Vector__XXX
 SG_ BR_PP : 15|8@0+ (0.4,0) [0|102] "%" Vector__XXX
 SG_ AETQ : 23|8@0+ (1,-125) [-125|130] "%" Vector__XXX
 SG_ VS : 31|16@0+ (0.00390625,0) [0|255.99609375] "Kmph" Vector__XXX
 SG_ CG : 47|8@0+ (1,-125) [-125|130] "NA" Vector__XXX
 SG_ SG : 55|8@0+ (1,-125) [-125|130] "NA" Vector__XXX

BO_ 529 RC40VP2: 8 Vector__XXX
 SG_ HRDT : 7|32@0+ (5,0) [0|21474836475] "m" Vector__XXX
 SG_ HRFC : 39|32@0+ (0.001,0) [0|4294967.295] "L" Vector__XXX

BO_ 530 RC40VP3: 8 Vector__XXX
 SG_ AP : 7|8@0+ (0.5,0) [0|127.5] "kPa" Vector__XXX
 SG_ AT : 15|16@0+ (0.03125,-273) [-273|1774.96875] "degC" Vector__XXX
 SG_ ELCS : 31|8@0+ (1,0) [0|255] "%" Vector__XXX
 SG_ ES : 39|16@0+ (0.125,0) [0|8191.875] "RPM" Vector__XXX
 SG_ BRS : 55|2@0+ (1,0) [0|3] "NA" Vector__XXX
 SG_ CLS : 53|2@0+ (1,0) [0|3] "NA" Vector__XXX
 SG_ CCS : 51|2@0+ (1,0) [0|3] "NA" Vector__XXX

BO_ 531 RC40IMU1: 8 Vector__XXX
 SG_ IMU_P : 7|16@0+ (0.00125,-40.96) [-40.96|40.95875] "m/s2" Vector__XXX
 SG_ IMU_PR : 23|16@0+ (0.005,-163.84) [-163.84|163.835] "deg/s" Vector__XXX
 SG_ IMU_R : 39|16@0+ (0.00125,-40.96) [-40.96|40.95875] "m/s2" Vector__XXX
 SG_ IMU_RR : 55|16@0+ (0.005,-163.84) [-163.84|163.835] "deg/s" Vector__XXX

BO_ 532 RC40IMU2: 8 Vector__XXX
 SG_ IMU_Y : 7|16@0+ (0.00125,-40.96) [-40.96|40.95875] "m/s2" Vector__XXX
 SG_ IMU_YR : 23|16@0+ (0.005,-163.84) [-163.84|163.835] "deg/s" Vector__XXX

BO_ 533 RC40LLC1: 8 Vector__XXX
 SG_ AA : 7|16@0+ (0.0491354666,-701.2971142) [-701.2971142|2518.795689431] "deg" Vector__XXX
 SG_ BTC : 23|8@0+ (1,0) [0|255] "NA" Vector__XXX
 SG_ CLC : 31|8@0+ (1,0) [0|255] "NA" Vector__XXX
 SG_ OPM : 39|8@0+ (1,0) [0|255] "NA" Vector__XXX
 SG_ OPR : 47|8@0+ (1,0) [0|255] "NA" Vector__XXX
 SG_ BTS : 48|8@1+ (1,0) [0|255] "NA" Vector__XXX

BO_ 534 RC40LLC2: 8 Vector__XXX
 SG_ TQR : 7|16@0+ (1,0) [0|32767] "%" Vector__XXX
 SG_ CNTL_OP : 23|8@0+ (1,-100) [0|155] "%" Vector__XXX
 SG_ S_CNTL_OP : 31|8@0+ (1,-100) [-100|155] "%" Vector__XXX
 SG_ WSS_VS : 39|8@0+ (1,0) [0|255] "Kmph" Vector__XXX
 SG_ MN_TQ : 40|8@1+ (1,0) [0|255] "%" Vector__XXX

BO_ 535 RC40LLC3: 8 Vector__XXX
 SG_ BT_SOC : 7|16@0+ (0.01,0) [0|655.35] "%" Vector__XXX
 SG_ BT_V : 23|16@0+ (0.1,0) [0|6553.5] "V" Vector__XXX
 SG_ BT_C : 39|16@0+ (0.1,-3276.75) [-327.675|327.675] "A" Vector__XXX

BO_ 536 RC40LLC4: 8 Vector__XXX
 SG_ MTQ_FB : 0|16@1+ (0.125,-4096) [-4096|4095.875] "Nm" Vector__XXX
 SG_ PTQ_AV : 16|15@1+ (0.125,0) [0|4095.875] "Nm" Vector__XXX
 SG_ RTQ_AV : 32|15@1+ (0.125,0) [0|4095.875] "Nm" Vector__XXX

BO_ 537 RC40LLC5: 8 Vector__XXX
 SG_ TP_M : 0|8@1+ (1,-40) [-40|215] "Deg" Vector__XXX
 SG_ TP_AB : 8|8@1+ (1,-40) [-40|215] "Deg" Vector__XXX
 SG_ TP_EMI : 16|8@1+ (1,-40) [-40|215] "Deg" Vector__XXX
 SG_ TP_MCU : 24|8@1+ (1,-40) [-40|215] "Deg" Vector__XXX
 SG_ TP_AUX1 : 32|8@1+ (1,0) [0|255] "Deg" Vector__XXX
 SG_ TP_AUX2 : 40|8@1+ (1,0) [0|255] "Deg" Vector__XXX
 SG_ TP_PCB : 48|8@1+ (1,0) [0|255] "Deg" Vector__XXX
 SG_ TP_Cell : 56|8@1+ (1,0) [0|255] "Deg" Vector__XXX

BO_ 544 RC40WSS1: 8 Vector__XXX
 SG_ WSS1_FR : 7|16@0+ (1,0) [0|65535] "Hz" Vector__XXX
 SG_ WSS2_FR : 23|16@0+ (1,0) [0|65535] "Hz" Vector__XXX
 SG_ Omega_1 : 39|16@0+ (1,0) [0|65535] "rad/sec" Vector__XXX
 SG_ Omega_2 : 55|16@0+ (1,0) [0|65535] "rad/s" Vector__XXX
"""

# ─── REALISTIC VALUE GENERATORS ─────────────────────────────────────────────

class VehicleSimState:
    """Holds correlated simulation state so values are physically plausible."""
    def __init__(self):
        self.speed = 60.0          # km/h
        self.rpm = 2000.0
        self.soc = 75.0            # %
        self.distance = 0.0        # m
        self.fuel = 10.0           # L consumed
        self.gear = 3
        self.acc_pedal = 20.0      # %
        self.brake_pedal = 0.0
        self.motor_temp = 45.0
        self.battery_voltage = 380.0
        self._t = 0.0

    def step(self, dt=0.01):
        self._t += dt
        self.acc_pedal = max(0, min(100, self.acc_pedal + random.gauss(0, 2)))
        self.brake_pedal = max(0, min(100, random.expovariate(1/5) if random.random() < 0.05 else 0))
        self.speed = max(0, min(120, self.speed + random.gauss(0, 0.5)))
        self.rpm = max(800, min(6000, self.speed * 30 + random.gauss(0, 50)))
        self.soc = max(10, min(100, self.soc - 0.0001))
        self.distance += self.speed / 3600 * dt
        self.fuel += self.speed * 0.000002 * dt
        self.gear = max(1, min(6, int(self.speed / 20) + 1))
        self.motor_temp = max(20, min(120, self.motor_temp + random.gauss(0, 0.1)))
        self.battery_voltage = max(300, min(420, self.battery_voltage + random.gauss(0, 0.5)))

    def signal_values(self):
        t = self._t
        return {
            "ACC_PP": self.acc_pedal,
            "BR_PP": self.brake_pedal,
            "AETQ": self.acc_pedal * 1.2 - 10,
            "VS": self.speed,
            "CG": float(self.gear),
            "SG": float(self.gear),
            "HRDT": self.distance,
            "HRFC": self.fuel,
            "AP": 101.3 + random.gauss(0, 0.1),
            "AT": 25.0 + random.gauss(0, 0.2),
            "ELCS": self.acc_pedal * 0.8,
            "ES": self.rpm,
            "BRS": 1.0 if self.brake_pedal > 5 else 0.0,
            "CLS": 0.0,
            "CCS": 0.0,
            "IMU_P": 0.2 * math.sin(t * 0.5) + random.gauss(0, 0.02),
            "IMU_PR": 0.5 * math.sin(t * 1.1) + random.gauss(0, 0.05),
            "IMU_R": 0.15 * math.sin(t * 0.7) + random.gauss(0, 0.02),
            "IMU_RR": 0.3 * math.cos(t * 0.9) + random.gauss(0, 0.05),
            "IMU_Y": 0.1 * math.sin(t * 0.3) + random.gauss(0, 0.01),
            "IMU_YR": 0.8 * math.sin(t * 0.6) + random.gauss(0, 0.05),
            "AA": random.gauss(0, 2),
            "BTC": 1.0,
            "CLC": 1.0,
            "OPM": 2.0,
            "OPR": 2.0,
            "BTS": 1.0,
            "TQR": self.acc_pedal * 100,
            "CNTL_OP": self.acc_pedal - 20,
            "S_CNTL_OP": random.gauss(0, 5),
            "WSS_VS": self.speed,
            "MN_TQ": 50.0,
            "BT_SOC": self.soc,
            "BT_V": self.battery_voltage,
            "BT_C": (self.acc_pedal - 30) * 2.0 + random.gauss(0, 1),
            "MTQ_FB": self.acc_pedal * 30 - 500,
            "PTQ_AV": 3000.0,
            "RTQ_AV": 1500.0,
            "TP_M": self.motor_temp,
            "TP_AB": 25.0 + random.gauss(0, 0.5),
            "TP_EMI": self.motor_temp - 5,
            "TP_MCU": self.motor_temp + 10,
            "TP_AUX1": 40.0,
            "TP_AUX2": 42.0,
            "TP_PCB": 38.0,
            "TP_Cell": self.motor_temp - 10,
            "WSS1_FR": self.speed * 10,
            "WSS2_FR": self.speed * 10,
            "Omega_1": self.speed * 2,
            "Omega_2": self.speed * 2,
        }


def clamp_to_signal(sig, value):
    """Clamp a physical value to the signal's defined min/max."""
    lo, hi = sig.minimum, sig.maximum
    if lo is not None and value < lo:
        value = lo
    if hi is not None and value > hi:
        value = hi
    return value


# ─── COLORS / THEME ─────────────────────────────────────────────────────────

BG       = "#0d1117"
PANEL    = "#161b22"
CARD     = "#1c2128"
BORDER   = "#30363d"
ACCENT   = "#00d4aa"
ACCENT2  = "#ff6b35"
TEXT_PRI = "#e6edf3"
TEXT_SEC = "#8b949e"
TEXT_VAL = "#ffffff"
RED      = "#f85149"
GREEN    = "#3fb950"
YELLOW   = "#d29922"


# ─── WINDOW / HEADER ICON ───────────────────────────────────────────────────

def _draw_can_icon(size: int = 32, tx: bool = False) -> tk.PhotoImage:
    """
    Programmatically draw a CAN bus topology icon.
      tx=False  →  teal down-arrow above centre node  (receive / dashboard)
      tx=True   →  orange up-arrow above centre node  (transmit / simulator)
    """
    BG, TEAL, ORG = "#0d1117", "#00d4aa", "#ff6b35"

    img = tk.PhotoImage(width=size, height=size)
    bg_row = "{" + (" ".join([BG] * size)) + "}"
    for y in range(size):
        img.put(bg_row, (0, y))

    def px(x, y, c):
        if 0 <= x < size and 0 <= y < size:
            img.put(c, to=(x, y, x + 1, y + 1))

    bus_y   = int(size * 0.68)
    node_r  = max(3, size // 10)
    node_xs = [int(size * 0.18), int(size * 0.50), int(size * 0.82)]
    node_cy = bus_y - node_r - 4

    # Horizontal bus line
    for x in range(3, size - 3):
        px(x, bus_y,     TEAL)
        px(x, bus_y + 1, TEAL)

    # Nodes + vertical stubs
    for nx in node_xs:
        for y in range(node_cy + node_r, bus_y + 2):
            px(nx,     y, TEAL)
            px(nx + 1, y, TEAL)
        for dy in range(-node_r, node_r + 1):
            for dx in range(-node_r, node_r + 1):
                if dx * dx + dy * dy <= node_r * node_r:
                    px(nx + dx, node_cy + dy, ORG)

    # Direction arrow above centre node
    cx  = int(size * 0.50)
    tip = node_cy - node_r - 2
    col = ORG if tx else TEAL
    if tx:
        for i in range(5):                           # shaft up
            px(cx, tip - i, col); px(cx + 1, tip - i, col)
        for w in range(1, 4):                        # arrowhead
            px(cx - w, tip + w - 1, col); px(cx + 1 + w, tip + w - 1, col)
    else:
        for i in range(5):                           # shaft down
            px(cx, tip + i, col); px(cx + 1, tip + i, col)
        for w in range(1, 4):                        # arrowhead
            px(cx - w, tip + 4 - w, col); px(cx + 1 + w, tip + 4 - w, col)

    return img


# ─── SIMULATOR GUI ───────────────────────────────────────────────────────────

class SimulatorApp(tk.Tk):
    def __init__(self, channel="PCAN_USBBUS1", bitrate=500_000, interval=0.01):
        super().__init__()
        self.title("CAN Simulator")
        self.configure(bg=BG)
        self.geometry("920x720")
        self.minsize(750, 580)

        self._channel = channel
        self._bitrate = bitrate
        self._interval = interval
        self._dbc_path = None

        self._bus = None
        self._running = False
        self._state = VehicleSimState()
        self._lock = threading.Lock()
        self._latest: dict = {}
        self._cycle = 0
        self._tx_rate = 0.0
        self._rate_count = 0
        self._last_rate_time = time.time()

        self._db = cantools.database.Database()
        self._db.add_dbc_string(DBC_STRING)

        self._icon = _draw_can_icon(32, tx=True)
        self.iconphoto(False, self._icon)

        self._build_ui()
        self._schedule_ui_update()

    # ── UI BUILD ────────────────────────────────────────────────────────────

    def _build_ui(self):
        self._build_config_bar()
        self._build_header()
        self._build_controls()
        self._build_stats()
        self._build_log()
        self._build_footer()

    def _build_config_bar(self):
        cfg = tk.Frame(self, bg=PANEL, pady=6)
        cfg.pack(fill=tk.X)

        # ── Row 1: COM port, bitrate, TX interval
        row1 = tk.Frame(cfg, bg=PANEL)
        row1.pack(fill=tk.X, padx=12, pady=2)

        tk.Label(row1, text="COM PORT", bg=PANEL, fg=TEXT_SEC,
                 font=("Courier New", 9, "bold"), width=10,
                 anchor="w").pack(side=tk.LEFT)
        channels = [f"PCAN_USBBUS{i}" for i in range(1, 9)]
        self._channel_var = tk.StringVar(value=self._channel)
        ttk.Combobox(row1, textvariable=self._channel_var,
                     values=channels, width=18).pack(side=tk.LEFT, padx=6)

        tk.Label(row1, text="BITRATE", bg=PANEL, fg=TEXT_SEC,
                 font=("Courier New", 9, "bold"), width=8,
                 anchor="w").pack(side=tk.LEFT, padx=(14, 0))
        self._bitrate_var = tk.StringVar(value="500k")
        ttk.Combobox(row1, textvariable=self._bitrate_var,
                     values=["125k", "250k", "500k", "1M"],
                     width=7, state="readonly").pack(side=tk.LEFT, padx=6)

        tk.Label(row1, text="TX INTERVAL", bg=PANEL, fg=TEXT_SEC,
                 font=("Courier New", 9, "bold"), width=12,
                 anchor="w").pack(side=tk.LEFT, padx=(14, 0))
        self._interval_var = tk.StringVar(value="10 ms")
        ttk.Combobox(row1, textvariable=self._interval_var,
                     values=["1 ms", "5 ms", "10 ms", "20 ms", "50 ms", "100 ms"],
                     width=8, state="readonly").pack(side=tk.LEFT, padx=6)

        # ── Row 2: DBC file
        row2 = tk.Frame(cfg, bg=PANEL)
        row2.pack(fill=tk.X, padx=12, pady=2)

        tk.Label(row2, text="DBC FILE", bg=PANEL, fg=TEXT_SEC,
                 font=("Courier New", 9, "bold"), width=10,
                 anchor="w").pack(side=tk.LEFT)
        self._dbc_path_var = tk.StringVar(value="(built-in default)")
        tk.Entry(row2, textvariable=self._dbc_path_var, bg=CARD, fg=TEXT_PRI,
                 insertbackground=TEXT_PRI, font=("Courier New", 9),
                 width=44, relief="flat").pack(side=tk.LEFT, padx=6)
        tk.Button(row2, text="BROWSE", bg=CARD, fg=TEXT_PRI,
                  font=("Courier New", 9), relief="flat",
                  padx=8, command=self._on_dbc_browse).pack(side=tk.LEFT, padx=4)
        tk.Button(row2, text="LOAD DBC", bg=ACCENT2, fg=BG,
                  font=("Courier New", 9, "bold"), relief="flat",
                  padx=8, command=self._on_dbc_load).pack(side=tk.LEFT, padx=4)

        tk.Frame(cfg, bg=BORDER, height=1).pack(fill=tk.X, pady=(6, 0))

    def _build_header(self):
        hdr = tk.Frame(self, bg=PANEL, pady=10)
        hdr.pack(fill=tk.X)

        tk.Label(hdr, image=self._icon, bg=PANEL).pack(side=tk.LEFT, padx=(12, 4))
        tk.Label(hdr, text="CAN SIMULATOR", bg=PANEL, fg=TEXT_PRI,
                 font=("Courier New", 14, "bold")).pack(side=tk.LEFT)

        self._status_dot = tk.Label(hdr, text="⬤", bg=PANEL, fg=YELLOW,
                                    font=("Courier New", 14))
        self._status_dot.pack(side=tk.RIGHT, padx=6)
        self._status_label = tk.Label(hdr, text="IDLE", bg=PANEL,
                                      fg=YELLOW, font=("Courier New", 10))
        self._status_label.pack(side=tk.RIGHT, padx=4)

        self._tx_rate_label = tk.Label(hdr, text="TX: 0 msg/s", bg=PANEL,
                                       fg=TEXT_SEC, font=("Courier New", 10))
        self._tx_rate_label.pack(side=tk.RIGHT, padx=16)

        self._chan_label = tk.Label(
            hdr, text=f"Channel: {self._channel}  @  {self._bitrate//1000}k",
            bg=PANEL, fg=TEXT_SEC, font=("Courier New", 9))
        self._chan_label.pack(side=tk.RIGHT, padx=12)

    def _build_controls(self):
        ctrl = tk.Frame(self, bg=BG, pady=10)
        ctrl.pack(fill=tk.X)
        self._start_btn = tk.Button(
            ctrl, text="▶   START SIMULATION",
            bg=GREEN, fg=BG, font=("Courier New", 11, "bold"),
            relief="flat", padx=24, pady=6,
            command=self._on_toggle)
        self._start_btn.pack()

    def _build_stats(self):
        tk.Frame(self, bg=BORDER, height=1).pack(fill=tk.X, padx=4)

        stats_outer = tk.Frame(self, bg=PANEL, pady=6)
        stats_outer.pack(fill=tk.BOTH, expand=True, padx=4, pady=(4, 0))

        tk.Label(stats_outer, text="  LIVE SIGNAL VALUES", bg=PANEL, fg=ACCENT,
                 font=("Courier New", 9, "bold")).pack(anchor="w", pady=(0, 2))

        # Scrollable canvas — same pattern as the dashboard
        outer = tk.Frame(stats_outer, bg=PANEL)
        outer.pack(fill=tk.BOTH, expand=True, padx=4)

        self._stats_canvas = tk.Canvas(outer, bg=PANEL, bd=0, highlightthickness=0)
        vsb = ttk.Scrollbar(outer, orient="vertical",
                            command=self._stats_canvas.yview)
        self._stats_canvas.configure(yscrollcommand=vsb.set)
        vsb.pack(side=tk.RIGHT, fill=tk.Y)
        self._stats_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self._stats_grid_frame = tk.Frame(self._stats_canvas, bg=PANEL)
        _fid = self._stats_canvas.create_window(
            (0, 0), window=self._stats_grid_frame, anchor="nw")

        self._stats_grid_frame.bind(
            "<Configure>",
            lambda _: self._stats_canvas.configure(
                scrollregion=self._stats_canvas.bbox("all")))
        self._stats_canvas.bind(
            "<Configure>",
            lambda e: self._stats_canvas.itemconfig(_fid, width=e.width))
        self._stats_canvas.bind_all(
            "<MouseWheel>",
            lambda e: self._stats_canvas.yview_scroll(
                -int(e.delta / 120), "units"))

        self._stat_vars: dict[str, tk.StringVar] = {}
        self._rebuild_stats()

    def _rebuild_stats(self):
        for widget in self._stats_grid_frame.winfo_children():
            widget.destroy()
        self._stat_vars.clear()

        # Collect ALL signals from the current DBC, ordered by message then name
        all_sigs: list[tuple[str, str]] = [
            (sig.name, sig.unit or "")
            for msg in sorted(self._db.messages, key=lambda m: m.frame_id)
            for sig in sorted(msg.signals, key=lambda s: s.name)
        ]

        col_frames = [tk.Frame(self._stats_grid_frame, bg=PANEL),
                      tk.Frame(self._stats_grid_frame, bg=PANEL)]
        col_frames[0].pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        col_frames[1].pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        for i, (sig_name, unit) in enumerate(all_sigs):
            col = i % 2
            row_f = tk.Frame(col_frames[col], bg=CARD, pady=3)
            row_f.pack(fill=tk.X, pady=2, padx=4)

            tk.Label(row_f, text=sig_name[:14], bg=CARD, fg=TEXT_SEC,
                     font=("Courier New", 9), width=14,
                     anchor="w").pack(side=tk.LEFT, padx=8)

            var = tk.StringVar(value="——")
            tk.Label(row_f, textvariable=var, bg=CARD, fg=TEXT_VAL,
                     font=("Courier New", 11, "bold"),
                     width=10, anchor="e").pack(side=tk.LEFT)

            tk.Label(row_f, text=unit, bg=CARD, fg=TEXT_SEC,
                     font=("Courier New", 9), width=6,
                     anchor="w").pack(side=tk.LEFT, padx=4)

            self._stat_vars[sig_name] = var

    def _build_log(self):
        tk.Frame(self, bg=BORDER, height=1).pack(fill=tk.X, padx=4, pady=(4, 0))

        log_outer = tk.Frame(self, bg=BG)
        log_outer.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)

        tk.Label(log_outer, text="  TRANSMIT LOG", bg=BG, fg=ACCENT,
                 font=("Courier New", 9, "bold")).pack(anchor="w")

        log_frame = tk.Frame(log_outer, bg=BG)
        log_frame.pack(fill=tk.BOTH, expand=True)

        vsb = ttk.Scrollbar(log_frame, orient="vertical")
        self._log_text = tk.Text(
            log_frame, bg=CARD, fg=TEXT_SEC,
            font=("Courier New", 9), relief="flat",
            yscrollcommand=vsb.set, state="disabled", wrap="none")
        vsb.config(command=self._log_text.yview)
        vsb.pack(side=tk.RIGHT, fill=tk.Y)
        self._log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

    def _build_footer(self):
        foot = tk.Frame(self, bg=PANEL, pady=4)
        foot.pack(fill=tk.X, side=tk.BOTTOM)
        tk.Label(foot, text="CAN Simulator  •  Physics-based vehicle bus generator",
                 bg=PANEL, fg=TEXT_SEC, font=("Courier New", 8)).pack()

    # ── DBC ACTIONS ─────────────────────────────────────────────────────────

    def _on_dbc_browse(self):
        path = filedialog.askopenfilename(
            title="Select DBC File",
            filetypes=[("DBC files", "*.dbc"), ("All files", "*.*")])
        if path:
            self._dbc_path_var.set(path)

    def _on_dbc_load(self):
        if self._running:
            messagebox.showwarning("Simulator Running",
                                   "Stop the simulation before loading a new DBC.")
            return
        path = self._dbc_path_var.get().strip()
        if path in ("(built-in default)", ""):
            self._db = cantools.database.Database()
            self._db.add_dbc_string(DBC_STRING)
            self._dbc_path = None
            self._rebuild_stats()
            self._log_msg("Loaded built-in DBC.")
        elif not os.path.isfile(path):
            messagebox.showerror("DBC Load Error", f"File not found:\n{path}")
        else:
            try:
                db = cantools.database.Database()
                db.add_dbc_file(path)
                self._db = db
                self._dbc_path = path
                self._rebuild_stats()
                self._log_msg(f"Loaded DBC: {os.path.basename(path)}")
            except Exception as e:
                messagebox.showerror("DBC Load Error", str(e))

    # ── START / STOP ─────────────────────────────────────────────────────────

    def _on_toggle(self):
        if self._running:
            self._stop()
        else:
            self._start()

    def _start(self):
        channel = self._channel_var.get().strip()
        bitrate = {"125k": 125_000, "250k": 250_000,
                   "500k": 500_000, "1M": 1_000_000}.get(
                   self._bitrate_var.get(), 500_000)
        interval = {"1 ms": 0.001, "5 ms": 0.005, "10 ms": 0.01,
                    "20 ms": 0.02, "50 ms": 0.05, "100 ms": 0.1}.get(
                   self._interval_var.get(), 0.01)

        try:
            self._bus = can.interface.Bus(bustype="pcan",
                                          channel=channel, bitrate=bitrate)
        except Exception as e:
            messagebox.showerror("Connection Error",
                                 f"Could not connect to {channel}:\n{e}")
            return

        self._channel = channel
        self._bitrate = bitrate
        self._interval = interval
        self._running = True
        self._state = VehicleSimState()
        self._cycle = 0
        self._tx_rate = 0.0
        self._rate_count = 0
        self._last_rate_time = time.time()

        self._chan_label.config(
            text=f"Channel: {channel}  @  {bitrate//1000}k")
        self._set_status("running")
        self._start_btn.config(text="■   STOP SIMULATION", bg=RED)
        self._log_msg(
            f"Started on {channel} @ {bitrate//1000}k, "
            f"interval={int(interval*1000)} ms, "
            f"{len(self._db.messages)} messages in DBC")

        threading.Thread(target=self._tx_loop, daemon=True).start()

    def _stop(self):
        self._running = False
        if self._bus:
            try:
                self._bus.shutdown()
            except Exception:
                pass
            self._bus = None
        self._set_status("idle")
        self._start_btn.config(text="▶   START SIMULATION", bg=GREEN)
        self._log_msg("Simulation stopped.")

    # ── TX LOOP (background thread) ──────────────────────────────────────────

    def _tx_loop(self):
        while self._running:
            self._state.step(dt=self._interval)
            values = self._state.signal_values()

            sent = 0
            tx_snapshot: dict = {}
            for msg in self._db.messages:
                sig_vals = {sig.name: clamp_to_signal(sig, values.get(sig.name, 0.0))
                            for sig in msg.signals}
                try:
                    data = msg.encode(sig_vals)
                    self._bus.send(can.Message(arbitration_id=msg.frame_id,
                                               data=data, is_extended_id=False))
                    sent += 1
                    tx_snapshot.update(sig_vals)
                except Exception:
                    pass

            now = time.time()
            self._rate_count += sent
            elapsed = now - self._last_rate_time
            if elapsed >= 1.0:
                self._tx_rate = self._rate_count / elapsed
                self._last_rate_time = now
                self._rate_count = 0

            self._cycle += 1
            with self._lock:
                self._latest = tx_snapshot

            if self._cycle % 100 == 0:
                snap = dict(values)
                cyc = self._cycle
                self.after(0, lambda s=snap, c=cyc: self._log_msg(
                    f"Cycle {c:6d} | "
                    f"Speed={s.get('VS', 0):6.1f} km/h  "
                    f"RPM={s.get('ES', 0):6.0f}  "
                    f"SOC={s.get('BT_SOC', 0):5.1f}%"))

            time.sleep(self._interval)

    # ── UI UPDATE LOOP ───────────────────────────────────────────────────────

    def _schedule_ui_update(self):
        with self._lock:
            vals = dict(self._latest)

        for key, var in self._stat_vars.items():
            v = vals.get(key)
            if v is None:
                var.set("——")
            elif isinstance(v, float):
                var.set(f"{v:>10.1f}")
            else:
                var.set(f"{int(v):>10}")

        if self._running:
            self._tx_rate_label.config(text=f"TX: {self._tx_rate:.0f} msg/s")

        self.after(100, self._schedule_ui_update)   # 10 Hz UI refresh

    # ── HELPERS ─────────────────────────────────────────────────────────────

    def _set_status(self, state: str):
        colors = {"running": GREEN, "idle": YELLOW, "error": RED}
        labels = {"running": "TRANSMITTING", "idle": "IDLE", "error": "ERROR"}
        c = colors.get(state, YELLOW)
        self._status_dot.config(fg=c)
        self._status_label.config(fg=c, text=labels.get(state, state.upper()))

    def _log_msg(self, msg: str):
        ts = datetime.datetime.now().strftime("%H:%M:%S")
        self._log_text.config(state="normal")
        self._log_text.insert("end", f"[{ts}]  {msg}\n")
        self._log_text.see("end")
        self._log_text.config(state="disabled")

    def on_close(self):
        self._running = False
        if self._bus:
            try:
                self._bus.shutdown()
            except Exception:
                pass
        self.destroy()


# ─── HEADLESS CLI ENTRY POINT ────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="CAN Simulator Sender (headless)")
    parser.add_argument("--channel", default="PCAN_USBBUS1",
                        help="PCAN channel (default: PCAN_USBBUS1)")
    parser.add_argument("--bitrate", type=int, default=500000,
                        help="CAN bitrate (default: 500000)")
    parser.add_argument("--interval", type=float, default=0.01,
                        help="TX interval in seconds (default: 0.01 = 10 ms)")
    args = parser.parse_args()

    db = cantools.database.Database()
    db.add_dbc_string(DBC_STRING)

    print(f"Connecting to {args.channel} @ {args.bitrate} bps ...")
    bus = can.interface.Bus(bustype="pcan", channel=args.channel, bitrate=args.bitrate)
    print("Connected. Starting simulation — press Ctrl+C to stop.\n")

    state = VehicleSimState()
    cycle = 0

    try:
        while True:
            state.step(dt=args.interval)
            values = state.signal_values()

            for msg in db.messages:
                sig_vals = {sig.name: clamp_to_signal(sig, values.get(sig.name, 0.0))
                            for sig in msg.signals}
                try:
                    data = msg.encode(sig_vals)
                    frame = can.Message(arbitration_id=msg.frame_id,
                                       data=data, is_extended_id=False)
                    bus.send(frame)
                except Exception as e:
                    print(f"  Encode/send error [{msg.name}]: {e}")

            cycle += 1
            if cycle % 100 == 0:
                print(f"  Cycle {cycle:6d} | "
                      f"Speed={values['VS']:6.1f} km/h  "
                      f"RPM={values['ES']:6.0f}  "
                      f"SOC={values['BT_SOC']:5.1f}%")

            time.sleep(args.interval)

    except KeyboardInterrupt:
        print("\nSimulation stopped.")
    finally:
        bus.shutdown()


# ─── ENTRY POINT ─────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import sys
    if "--no-gui" in sys.argv:
        sys.argv.remove("--no-gui")
        main()
    else:
        app = SimulatorApp()
        app.protocol("WM_DELETE_WINDOW", app.on_close)
        app.mainloop()
