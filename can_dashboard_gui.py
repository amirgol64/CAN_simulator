"""
CAN Dashboard GUI
=================
Receives CAN frames from a PCAN-USB interface, decodes them using the DBC,
and displays all signals with live values and units.

Requirements:
    pip install python-can cantools

Usage:
    python can_dashboard_gui.py [--channel PCAN_USBBUS2] [--bitrate 500000]

Library usage (no GUI):
    from can_monitor import CANMonitor
    mon = CANMonitor(channel="PCAN_USBBUS2", bitrate=500000)
    mon.subscribe("VS", lambda name, val: print(f"Speed: {val}"))
    mon.start()
"""

import argparse
import os
import threading
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from can_monitor import CANMonitor, SIGNAL_DESCRIPTIONS

# ─── COLORS / THEME ─────────────────────────────────────────────────────────

BG         = "#0d1117"   # near black
PANEL      = "#161b22"   # dark panel
CARD       = "#1c2128"   # card bg
BORDER     = "#30363d"
ACCENT     = "#00d4aa"   # teal accent
ACCENT2    = "#ff6b35"   # orange for highlights
TEXT_PRI   = "#e6edf3"
TEXT_SEC   = "#8b949e"
TEXT_VAL   = "#ffffff"   # white for values
RED        = "#f85149"
GREEN      = "#3fb950"
YELLOW     = "#d29922"

# ─── SPARKLINE CANVAS ───────────────────────────────────────────────────────

class Sparkline(tk.Canvas):
    def __init__(self, parent, width=80, height=24, **kw):
        super().__init__(parent, width=width, height=height,
                         bg=CARD, bd=0, highlightthickness=0, **kw)
        self._spark_w = width
        self._spark_h = height

    def update_data(self, data):
        self.delete("all")
        pts = list(data)
        if len(pts) < 2:
            return
        mn, mx = min(pts), max(pts)
        rng = mx - mn or 1e-9
        w, h = self._spark_w, self._spark_h
        pad = 2
        coords = []
        for i, v in enumerate(pts):
            x = pad + (i / (len(pts) - 1)) * (w - 2 * pad)
            y = h - pad - ((v - mn) / rng) * (h - 2 * pad)
            coords.extend([x, y])
        if len(coords) >= 4:
            self.create_line(*coords, fill=ACCENT, width=1.5, smooth=True)
        # last value dot
        self.create_oval(coords[-2]-2, coords[-1]-2,
                         coords[-2]+2, coords[-1]+2,
                         fill=ACCENT2, outline="")


# ─── SIGNAL ROW WIDGET ──────────────────────────────────────────────────────

class SignalRow(tk.Frame):
    def __init__(self, parent, signal_name, unit, description, **kw):
        super().__init__(parent, bg=CARD, pady=2, **kw)
        self.signal_name = signal_name
        # ── Signal name
        sig_frame = tk.Frame(self, bg=CARD, width=120)
        sig_frame.pack(side=tk.LEFT, padx=6, fill=tk.Y)
        sig_frame.pack_propagate(False)
        tk.Label(sig_frame, text=signal_name, bg=CARD, fg=TEXT_PRI,
                 font=("Courier New", 9, "bold"), anchor="w",
                 wraplength=114, justify="left").pack(fill=tk.BOTH, expand=True)

        # ── Description
        desc_frame = tk.Frame(self, bg=CARD, width=150)
        desc_frame.pack(side=tk.LEFT, fill=tk.Y)
        desc_frame.pack_propagate(False)
        tk.Label(desc_frame, text=description, bg=CARD, fg=TEXT_SEC,
                 font=("Courier New", 9), anchor="w",
                 wraplength=144, justify="left").pack(fill=tk.BOTH, expand=True)

        # ── Value
        self.val_label = tk.Label(self, text="—", bg=CARD, fg=TEXT_VAL,
                                  font=("Courier New", 11, "bold"),
                                  width=12, anchor="e")
        self.val_label.pack(side=tk.LEFT, padx=4)

        # ── Unit
        tk.Label(self, text=unit, bg=CARD, fg=TEXT_SEC,
                 font=("Courier New", 9), width=7, anchor="w").pack(side=tk.LEFT)

        # Separator line
        sep = tk.Frame(self, bg=BORDER, height=1)
        sep.pack(side=tk.BOTTOM, fill=tk.X)

    def update_value(self, value):
        if isinstance(value, (int, float)):
            self.val_label.config(text=f"{value:>10.3f}")
        else:
            self.val_label.config(text=f"{str(value):>10}")



# ─── MAIN DASHBOARD APP ─────────────────────────────────────────────────────

class CANDashboard(tk.Tk):
    def __init__(self, channel, bitrate):
        super().__init__()
        self.channel = channel
        self.bitrate = bitrate
        self._dbc_path = None

        self.title("CAN Dashboard")
        self.configure(bg=BG)
        self.geometry("1100x860")
        self.minsize(900, 650)

        self._monitor = CANMonitor(channel=channel, bitrate=bitrate)
        self._monitor.on_connect(lambda: self.after(0, lambda: self._set_status(True)))
        self._monitor.on_disconnect(lambda e: self.after(0, lambda: self._set_status(False, e)))
        self._monitor.subscribe("*", self._on_signal)

        self._signal_rows: dict = {}
        self._pending: dict = {}
        self._lock = threading.Lock()

        self._build_ui()
        self._monitor.start()
        self._schedule_ui_update()

    # ── UI BUILD ────────────────────────────────────────────────────────────

    def _build_ui(self):
        self._build_config_bar()

        # ── Header
        hdr = tk.Frame(self, bg=PANEL, pady=10)
        hdr.pack(fill=tk.X)

        tk.Label(hdr, text="●  CAN", bg=PANEL, fg=ACCENT,
                 font=("Courier New", 18, "bold")).pack(side=tk.LEFT, padx=16)
        tk.Label(hdr, text="CAN DASHBOARD", bg=PANEL, fg=TEXT_PRI,
                 font=("Courier New", 14)).pack(side=tk.LEFT)

        self.status_dot = tk.Label(hdr, text="⬤", bg=PANEL, fg=RED,
                                   font=("Courier New", 14))
        self.status_dot.pack(side=tk.RIGHT, padx=6)
        self.status_label = tk.Label(hdr, text="DISCONNECTED", bg=PANEL,
                                     fg=RED, font=("Courier New", 10))
        self.status_label.pack(side=tk.RIGHT, padx=4)

        self.rx_label = tk.Label(hdr, text="RX: 0 msg/s", bg=PANEL,
                                 fg=TEXT_SEC, font=("Courier New", 10))
        self.rx_label.pack(side=tk.RIGHT, padx=16)

        self._chan_info_label = tk.Label(
            hdr, text=f"Channel: {self.channel}  @  {self.bitrate//1000}k",
            bg=PANEL, fg=TEXT_SEC, font=("Courier New", 9))
        self._chan_info_label.pack(side=tk.RIGHT, padx=12)

        # ── Column headers
        col_hdr = tk.Frame(self, bg=BG)
        col_hdr.pack(fill=tk.X, padx=8, pady=4)
        for txt, w in [("SIGNAL", 14), ("DESCRIPTION", 20), ("VALUE", 12),
                        ("UNIT", 7)]:
            tk.Label(col_hdr, text=txt, bg=BG, fg=TEXT_SEC,
                     font=("Courier New", 8), width=w, anchor="w").pack(side=tk.LEFT)

        tk.Frame(self, bg=BORDER, height=1).pack(fill=tk.X, padx=4)

        # ── Scrollable signal area
        outer = tk.Frame(self, bg=BG)
        outer.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)

        self._scroll_canvas = tk.Canvas(outer, bg=BG, bd=0, highlightthickness=0)
        vsb = ttk.Scrollbar(outer, orient="vertical", command=self._scroll_canvas.yview)
        self._scroll_canvas.configure(yscrollcommand=vsb.set)

        vsb.pack(side=tk.RIGHT, fill=tk.Y)
        self._scroll_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.signal_frame = tk.Frame(self._scroll_canvas, bg=BG)
        self._frame_id = self._scroll_canvas.create_window(
            (0, 0), window=self.signal_frame, anchor="nw")

        def on_configure(e):
            self._scroll_canvas.configure(
                scrollregion=self._scroll_canvas.bbox("all"))
        self.signal_frame.bind("<Configure>", on_configure)

        def on_canvas_resize(e):
            self._scroll_canvas.itemconfig(self._frame_id, width=e.width)
        self._scroll_canvas.bind("<Configure>", on_canvas_resize)

        self._scroll_canvas.bind_all(
            "<MouseWheel>",
            lambda e: self._scroll_canvas.yview_scroll(-int(e.delta/120), "units"))

        self._rebuild_signals()

        # ── Footer
        foot = tk.Frame(self, bg=PANEL, pady=4)
        foot.pack(fill=tk.X, side=tk.BOTTOM)
        tk.Label(foot, text="CAN Data Logger  •  CAN Simulator Dashboard",
                 bg=PANEL, fg=TEXT_SEC, font=("Courier New", 8)).pack()

    def _build_config_bar(self):
        cfg = tk.Frame(self, bg=PANEL, pady=6)
        cfg.pack(fill=tk.X)

        # ── COM Port row
        row1 = tk.Frame(cfg, bg=PANEL)
        row1.pack(fill=tk.X, padx=12, pady=2)

        tk.Label(row1, text="COM PORT", bg=PANEL, fg=TEXT_SEC,
                 font=("Courier New", 9, "bold"), width=10,
                 anchor="w").pack(side=tk.LEFT)

        channels = [f"PCAN_USBBUS{i}" for i in range(1, 9)]
        self._channel_var = tk.StringVar(value=self.channel)
        ttk.Combobox(row1, textvariable=self._channel_var,
                     values=channels, width=20).pack(side=tk.LEFT, padx=6)

        tk.Button(row1, text="CONNECT", bg=ACCENT, fg=BG,
                  font=("Courier New", 9, "bold"), relief="flat",
                  padx=10, command=self._on_connect_clicked).pack(side=tk.LEFT, padx=6)

        # ── DBC File row
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

    def _rebuild_signals(self):
        for widget in self.signal_frame.winfo_children():
            widget.destroy()
        self._signal_rows.clear()

        for msg in sorted(self._monitor.db.messages, key=lambda m: m.frame_id):
            grp = tk.Frame(self.signal_frame, bg=PANEL, pady=3)
            grp.pack(fill=tk.X, pady=4)
            tk.Label(grp, text=f"  0x{msg.frame_id:03X}  {msg.name}",
                     bg=PANEL, fg=ACCENT, font=("Courier New", 10, "bold"),
                     anchor="w").pack(side=tk.LEFT, padx=8)
            if msg.comment:
                tk.Label(grp, text=msg.comment, bg=PANEL, fg=TEXT_SEC,
                         font=("Courier New", 8), anchor="w").pack(side=tk.LEFT, padx=4)

            for sig in sorted(msg.signals, key=lambda s: s.name):
                desc = SIGNAL_DESCRIPTIONS.get(sig.name) or sig.comment or sig.name
                unit = sig.unit or ""
                row = SignalRow(self.signal_frame, sig.name, unit, desc)
                row.pack(fill=tk.X, padx=2, pady=1)
                self._signal_rows[sig.name] = row

    # ── CONFIG ACTIONS ───────────────────────────────────────────────────────

    def _on_connect_clicked(self):
        new_channel = self._channel_var.get().strip()
        if not new_channel:
            return
        self._monitor.stop()
        self.channel = new_channel
        self._monitor = CANMonitor(channel=new_channel, bitrate=self.bitrate,
                                   dbc_path=self._dbc_path)
        self._monitor.on_connect(lambda: self.after(0, lambda: self._set_status(True)))
        self._monitor.on_disconnect(lambda e: self.after(0, lambda: self._set_status(False, e)))
        self._monitor.subscribe("*", self._on_signal)
        self._set_status(False)
        self._chan_info_label.config(
            text=f"Channel: {self.channel}  @  {self.bitrate//1000}k")
        self._monitor.start()

    def _on_dbc_browse(self):
        path = filedialog.askopenfilename(
            title="Select DBC File",
            filetypes=[("DBC files", "*.dbc"), ("All files", "*.*")])
        if path:
            self._dbc_path_var.set(path)

    def _on_dbc_load(self):
        path = self._dbc_path_var.get().strip()
        if path in ("(built-in default)", ""):
            dbc_path = None
        elif not os.path.isfile(path):
            messagebox.showerror("DBC Load Error", f"File not found:\n{path}")
            return
        else:
            dbc_path = path

        self._dbc_path = dbc_path
        self._monitor.stop()
        self._monitor = CANMonitor(channel=self.channel, bitrate=self.bitrate,
                                   dbc_path=dbc_path)
        self._monitor.on_connect(lambda: self.after(0, lambda: self._set_status(True)))
        self._monitor.on_disconnect(lambda e: self.after(0, lambda: self._set_status(False, e)))
        self._monitor.subscribe("*", self._on_signal)
        self._set_status(False)
        self._rebuild_signals()
        self._monitor.start()

    # ── CALLBACKS ────────────────────────────────────────────────────────────

    def _on_signal(self, name, value):
        """Called by CANMonitor for every decoded or injected signal."""
        with self._lock:
            self._pending[name] = value

    def _set_status(self, connected, msg=""):
        color = GREEN if connected else RED
        txt   = f"CONNECTED  {self.channel}" if connected else f"DISCONNECTED  {msg[:40]}"
        self.status_dot.config(fg=color)
        self.status_label.config(fg=color, text=txt)

    # ── UI UPDATE LOOP ───────────────────────────────────────────────────────

    def _schedule_ui_update(self):
        with self._lock:
            batch = dict(self._pending)
            self._pending.clear()

        for sig_name, value in batch.items():
            if sig_name in self._signal_rows:
                row = self._signal_rows[sig_name]
                row.update_value(value)

        rate = self._monitor.rx_rate
        self.rx_label.config(text=f"RX: {rate:.0f} msg/s")

        self.after(50, self._schedule_ui_update)   # 20 Hz UI refresh

    # ── CLOSE ────────────────────────────────────────────────────────────────

    def on_close(self):
        self._monitor.stop()
        self.destroy()


# ─── ENTRY POINT ────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="CAN Dashboard GUI")
    parser.add_argument("--channel", default="PCAN_USBBUS2",
                        help="PCAN channel (default: PCAN_USBBUS2)")
    parser.add_argument("--bitrate", type=int, default=500000,
                        help="CAN bitrate (default: 500000)")
    args = parser.parse_args()

    app = CANDashboard(channel=args.channel, bitrate=args.bitrate)
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()


if __name__ == "__main__":
    main()
