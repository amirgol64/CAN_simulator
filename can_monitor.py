"""
can_monitor.py
==============
Headless CAN signal monitor — no GUI dependencies.

Import in any Python project to receive decoded CAN signals:

    from can_monitor import CANMonitor

    mon = CANMonitor(channel="PCAN_USBBUS2", bitrate=500000)
    mon.subscribe("VS",  lambda name, val: print(f"Speed: {val:.1f} km/h"))
    mon.subscribe("soc", lambda name, val: print(f"SOC: {val}%"))
    mon.subscribe("*",   lambda name, val: ...)   # every signal
    mon.start()

    import time; time.sleep(10)
    speed = mon.get("VS")          # latest value (polling)
    all_signals = mon.get_all()    # dict snapshot

    mon.inject("soc", 75)          # push a simulated / virtual value
    mon.stop()

    # Context-manager form:
    with CANMonitor() as mon:
        time.sleep(5)
        print(mon.get_all())

Requirements
------------
    pip install python-can cantools
"""

import threading
import time
import can
import cantools

# ─── DBC ──────────────────────────────────────────────────────────────────────
#
#  Real CAN messages: 0x210-0x221 (RC40 vehicle bus)
#                     0x123  TRAILER_STATUS
#                     0x234  CHARGER_STATUS
#  Virtual messages:  0x7F8  VIRTUAL_STATUS   (never on bus; use inject())
#                     0x7F9  VIRTUAL_NUMERIC  (never on bus; use inject())
#
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

BO_ 291 TRAILER_STATUS: 8 Vector__XXX
 SG_ trailer_state : 0|2@1+ (1,0) [0|3] "" Vector__XXX
 SG_ trailer_assistance : 2|2@1+ (1,0) [0|3] "" Vector__XXX

BO_ 564 CHARGER_STATUS: 8 Vector__XXX
 SG_ charger_state : 0|2@1+ (1,0) [0|3] "" Vector__XXX

BO_ 2040 VIRTUAL_STATUS: 8 Vector__XXX
 SG_ soc : 0|8@1+ (1,0) [0|100] "%" Vector__XXX
 SG_ drive_mode : 8|3@1+ (1,0) [0|7] "" Vector__XXX
 SG_ truck_eng_stat : 11|1@1+ (1,0) [0|1] "" Vector__XXX
 SG_ vs : 12|1@1+ (1,0) [0|1] "" Vector__XXX
 SG_ ss : 13|1@1+ (1,0) [0|1] "" Vector__XXX
 SG_ energy_usage_stat : 14|3@1+ (1,0) [0|7] "" Vector__XXX

BO_ 2041 VIRTUAL_NUMERIC: 8 Vector__XXX
 SG_ energy_usage_kw : 0|16@1- (0.1,0) [-3276.8|3276.7] "kW" Vector__XXX

VAL_ 291 trailer_state 0 "disconnected" 1 "connected" 2 "unknown" ;
VAL_ 291 trailer_assistance 0 "inactive" 1 "active" 2 "max" ;
VAL_ 564 charger_state 0 "disconnected" 1 "connected" 2 "unknown" ;
VAL_ 2040 drive_mode 0 "off" 1 "hybrid" 2 "elec" ;
VAL_ 2040 truck_eng_stat 0 "off" 1 "on" ;
VAL_ 2040 vs 0 "n" 1 "y" ;
VAL_ 2040 ss 0 "off" 1 "on" ;
VAL_ 2040 energy_usage_stat 0 "free" 1 "discharge" 2 "generate" ;

"""

# ─── SIGNAL DESCRIPTIONS ──────────────────────────────────────────────────────

SIGNAL_DESCRIPTIONS = {
    # RC40 vehicle signals
    "ACC_PP":    "Accelerator Pedal",
    "BR_PP":     "Brake Pedal",
    "AETQ":      "Engine Torque",
    "VS":        "Vehicle Speed",
    "CG":        "Current Gear",
    "SG":        "Selected Gear",
    "HRDT":      "Distance Travelled",
    "HRFC":      "Fuel Consumption",
    "AP":        "Ambient Pressure",
    "AT":        "Ambient Temperature",
    "ELCS":      "Engine Load",
    "ES":        "Engine Speed",
    "BRS":       "Brake Switch",
    "CLS":       "Clutch Switch",
    "CCS":       "Cruise Control",
    "IMU_P":     "Pitch",
    "IMU_PR":    "Pitch Rate",
    "IMU_R":     "Roll",
    "IMU_RR":    "Roll Rate",
    "IMU_Y":     "Yaw",
    "IMU_YR":    "Yaw Rate",
    "AA":        "Articulation Angle",
    "BTC":       "Battery Control",
    "CLC":       "Cooling Control",
    "OPM":       "Op. Mode",
    "OPR":       "Op. Request",
    "BTS":       "Battery Status",
    "TQR":       "Torque Request",
    "CNTL_OP":   "Controller Output",
    "S_CNTL_OP": "Slip Ctrl Output",
    "WSS_VS":    "WSS Vehicle Speed",
    "MN_TQ":     "Manual Torque",
    "BT_SOC":    "Battery SOC",
    "BT_V":      "Battery Voltage",
    "BT_C":      "Battery Current",
    "MTQ_FB":    "Motor Torque FB",
    "PTQ_AV":    "Prop. Torque Avail",
    "RTQ_AV":    "Regen Torque Avail",
    "TP_M":      "Motor Temp",
    "TP_AB":     "Ambient Temp",
    "TP_EMI":    "EMI Temp",
    "TP_MCU":    "MCU Temp",
    "TP_AUX1":   "Aux1 Temp",
    "TP_AUX2":   "Aux2 Temp",
    "TP_PCB":    "PCB Temp",
    "TP_Cell":   "Cell Temp",
    "WSS1_FR":   "WSS1 Frequency",
    "WSS2_FR":   "WSS2 Frequency",
    "Omega_1":   "Omega 1",
    "Omega_2":   "Omega 2",
    # Virtual / derived signals
    "trailer_state":      "Trailer State",
    "trailer_assistance": "Trailer Assistance",
    "charger_state":      "Charger State",
    "soc":                "State of Charge",
    "energy_usage_stat":  "Energy Usage",
    "energy_usage_kw":    "Energy (kW)",
    "vs":                 "Vehicle Stationary",
    "drive_mode":         "Drive Mode",
    "truck_eng_stat":     "Engine Status",
    "ss":                 "Standstill",
}


# ─── CAN MONITOR ──────────────────────────────────────────────────────────────

class CANMonitor:
    """
    Thread-safe CAN signal monitor.

    Connects to a PCAN bus, decodes all messages via DBC, and fires callbacks
    for each signal update.  Also computes derived virtual signals automatically
    (soc, energy_usage_kw, energy_usage_stat, vs).

    Virtual signals (drive_mode, truck_eng_stat, ss, trailer_state, etc.) that
    are not present on the bus can be pushed from external code via inject().
    """

    def __init__(self, channel: str = "PCAN_USBBUS2", bitrate: int = 500_000,
                 retry_interval: float = 3.0, dbc_path: str = None):
        self.channel = channel
        self.bitrate = bitrate
        self._retry_interval = retry_interval

        self._db = cantools.database.Database()
        if dbc_path:
            self._db.add_dbc_file(dbc_path)
        else:
            self._db.add_dbc_string(DBC_STRING)

        self._values: dict = {}
        self._lock = threading.Lock()
        self._callbacks: dict[str, list] = {}   # signal_name -> [cb, ...]  "*" = all

        self._running = False
        self._bus = None
        self._rx_count = 0
        self._rx_rate: float = 0.0

        self._on_connect_cb = None
        self._on_disconnect_cb = None

    # ── Public API ─────────────────────────────────────────────────────────────

    @property
    def db(self):
        """Access the cantools database (e.g. to iterate messages for UI)."""
        return self._db

    @property
    def rx_rate(self) -> float:
        """Most recent RX rate in messages/second."""
        return self._rx_rate

    @property
    def rx_count(self) -> int:
        return self._rx_count

    def subscribe(self, signal_name: str, callback):
        """
        Register a callback for a specific signal (or "*" for all signals).
        callback signature:  callback(signal_name: str, value)
        """
        self._callbacks.setdefault(signal_name, []).append(callback)

    def get(self, signal_name: str, default=None):
        """Return the latest decoded value, or default if not yet received."""
        with self._lock:
            return self._values.get(signal_name, default)

    def get_all(self) -> dict:
        """Return a snapshot dict of all current signal values."""
        with self._lock:
            return dict(self._values)

    def inject(self, signal_name: str, value):
        """
        Push a value for any signal — real or virtual.
        Fires registered callbacks just like a real CAN decode would.
        Useful for simulation, testing, or computed/derived signals.
        """
        self._dispatch(signal_name, value)

    def on_connect(self, callback):
        """callback() called when the CAN bus connects successfully."""
        self._on_connect_cb = callback

    def on_disconnect(self, callback):
        """callback(error_msg: str) called on disconnect or error."""
        self._on_disconnect_cb = callback

    def start(self):
        """Connect to CAN bus and start the receive loop in a daemon thread."""
        threading.Thread(target=self._connect_and_run, daemon=True).start()

    def stop(self):
        """Stop receiving and shut down the CAN bus."""
        self._running = False
        if self._bus:
            try:
                self._bus.shutdown()
            except Exception:
                pass

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *args):
        self.stop()

    # ── Internal ───────────────────────────────────────────────────────────────

    def _dispatch(self, name: str, value):
        """Store value and fire callbacks (specific + wildcard)."""
        with self._lock:
            self._values[name] = value
        for cb in self._callbacks.get(name, []):
            try:
                cb(name, value)
            except Exception:
                pass
        for cb in self._callbacks.get("*", []):
            try:
                cb(name, value)
            except Exception:
                pass

    def _connect_and_run(self):
        while True:
            try:
                self._bus = can.interface.Bus(interface="pcan",
                                              channel=self.channel,
                                              bitrate=self.bitrate)
                self._running = True
                if self._on_connect_cb:
                    self._on_connect_cb()
                self._rx_loop()
            except Exception as e:
                self._running = False
                if self._on_disconnect_cb:
                    self._on_disconnect_cb(str(e))
            if not self._retry_interval or not self._running is False:
                break
            time.sleep(self._retry_interval)
            if not self._running and self._retry_interval > 0:
                # restart was requested by retry logic
                pass
            else:
                break

    def _rx_loop(self):
        last_rate_time = time.time()
        interval_count = 0

        while self._running:
            try:
                msg = self._bus.recv(timeout=0.5)
                if msg is None:
                    continue

                self._rx_count += 1
                interval_count += 1
                now = time.time()
                elapsed = now - last_rate_time
                if elapsed >= 1.0:
                    self._rx_rate = interval_count / elapsed
                    last_rate_time = now
                    interval_count = 0

                try:
                    decoded = self._db.decode_message(msg.arbitration_id, msg.data)
                    for name, value in decoded.items():
                        self._dispatch(name, value)
                    self._compute_virtual()
                except Exception:
                    pass

            except can.CanError as e:
                self._running = False
                if self._on_disconnect_cb:
                    self._on_disconnect_cb(str(e))
                break
            except Exception:
                pass

    def _compute_virtual(self):
        """
        Automatically derive virtual signals from decoded CAN values.
        Called after every decoded message.
        """
        vals = self._values

        # soc — alias for BT_SOC (already a CAN signal, exposed under friendly name)
        bt_soc = vals.get("BT_SOC")
        if bt_soc is not None:
            self._dispatch("soc", round(bt_soc, 1))

        # energy_usage_kw and energy_usage_stat — derived from battery power
        # >0 A = discharging, <0 A = regenerating, =0 = free wheeling
        bt_c = vals.get("BT_C")
        bt_v = vals.get("BT_V")
        if bt_c is not None and bt_v is not None:
            kw = round(bt_c * bt_v / 1000.0, 2)
            self._dispatch("energy_usage_kw", kw)
            if bt_c > 0:
                self._dispatch("energy_usage_stat", "discharge")
            elif bt_c < 0:
                self._dispatch("energy_usage_stat", "generate")
            else:
                self._dispatch("energy_usage_stat", "free")

        # vs (vehicle stationary) — n=0 not moving, y=1 moving
        # ss (standstill)        — on=1 when stationary, off=0 when moving
        speed = vals.get("VS")
        if speed is not None:
            stationary = speed < 0.5
            self._dispatch("vs", "0" if stationary else "1")
            self._dispatch("ss", "on" if stationary else "off")

        # truck_eng_stat — derived from AETQ torque: >0 = on, <=0 = off
        aetq = vals.get("AETQ")
        if aetq is not None:
            self._dispatch("truck_eng_stat", "on" if aetq > 0 else "off")
