"""
CAN Simulator Sender
====================
Reads the DBC definition, generates realistic random signal
values, encodes them into CAN frames and transmits via a PCAN-USB interface.

Requirements:
    pip install python-can cantools

Usage:
    python can_simulator_sender.py [--channel PCAN_USBBUS1] [--bitrate 500000]
"""

import argparse
import time
import random
import math
import struct
import threading
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
        # Gentle oscillations to simulate driving
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
            # VP1
            "ACC_PP": self.acc_pedal,
            "BR_PP": self.brake_pedal,
            "AETQ": self.acc_pedal * 1.2 - 10,
            "VS": self.speed,
            "CG": float(self.gear),
            "SG": float(self.gear),
            # VP2
            "HRDT": self.distance,
            "HRFC": self.fuel,
            # VP3
            "AP": 101.3 + random.gauss(0, 0.1),
            "AT": 25.0 + random.gauss(0, 0.2),
            "ELCS": self.acc_pedal * 0.8,
            "ES": self.rpm,
            "BRS": 1.0 if self.brake_pedal > 5 else 0.0,
            "CLS": 0.0,
            "CCS": 0.0,
            # IMU1
            "IMU_P": 0.2 * math.sin(t * 0.5) + random.gauss(0, 0.02),
            "IMU_PR": 0.5 * math.sin(t * 1.1) + random.gauss(0, 0.05),
            "IMU_R": 0.15 * math.sin(t * 0.7) + random.gauss(0, 0.02),
            "IMU_RR": 0.3 * math.cos(t * 0.9) + random.gauss(0, 0.05),
            # IMU2
            "IMU_Y": 0.1 * math.sin(t * 0.3) + random.gauss(0, 0.01),
            "IMU_YR": 0.8 * math.sin(t * 0.6) + random.gauss(0, 0.05),
            # LLC1
            "AA": random.gauss(0, 2),
            "BTC": 1.0,
            "CLC": 1.0,
            "OPM": 2.0,
            "OPR": 2.0,
            "BTS": 1.0,
            # LLC2
            "TQR": self.acc_pedal * 100,
            "CNTL_OP": self.acc_pedal - 20,
            "S_CNTL_OP": random.gauss(0, 5),
            "WSS_VS": self.speed,
            "MN_TQ": 50.0,
            # LLC3
            "BT_SOC": self.soc,
            "BT_V": self.battery_voltage,
            "BT_C": (self.acc_pedal - 30) * 2.0 + random.gauss(0, 1),
            # LLC4
            "MTQ_FB": self.acc_pedal * 30 - 500,
            "PTQ_AV": 3000.0,
            "RTQ_AV": 1500.0,
            # LLC5
            "TP_M": self.motor_temp,
            "TP_AB": 25.0 + random.gauss(0, 0.5),
            "TP_EMI": self.motor_temp - 5,
            "TP_MCU": self.motor_temp + 10,
            "TP_AUX1": 40.0,
            "TP_AUX2": 42.0,
            "TP_PCB": 38.0,
            "TP_Cell": self.motor_temp - 10,
            # WSS1
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


def main():
    parser = argparse.ArgumentParser(description="CAN Simulator Sender")
    parser.add_argument("--channel", default="PCAN_USBBUS1", help="PCAN channel (default: PCAN_USBBUS1)")
    parser.add_argument("--bitrate", type=int, default=500000, help="CAN bitrate (default: 500000)")
    parser.add_argument("--interval", type=float, default=0.01, help="TX interval in seconds (default: 0.01 = 10 ms)")
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
                sig_vals = {}
                for sig in msg.signals:
                    raw = values.get(sig.name, 0.0)
                    sig_vals[sig.name] = clamp_to_signal(sig, raw)
                try:
                    data = msg.encode(sig_vals)
                    frame = can.Message(arbitration_id=msg.frame_id,
                                       data=data,
                                       is_extended_id=False)
                    bus.send(frame)
                except Exception as e:
                    print(f"  Encode/send error [{msg.name}]: {e}")

            cycle += 1
            if cycle % 100 == 0:
                spd = values["VS"]
                rpm = values["ES"]
                soc = values["BT_SOC"]
                print(f"  Cycle {cycle:6d} | Speed={spd:6.1f} km/h  RPM={rpm:6.0f}  SOC={soc:5.1f}%")

            time.sleep(args.interval)

    except KeyboardInterrupt:
        print("\nSimulation stopped.")
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
