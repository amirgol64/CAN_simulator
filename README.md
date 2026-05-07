# CAN Dashboard

A real-time CAN bus monitor and dashboard built with Python and Tkinter.  
Connects to a PCAN-USB adapter, decodes frames via a DBC file, and displays all signals live with values and units.

![Dashboard Screenshot](docs/screenshots/dashboard.png)

---

## Features

- **Live signal display** — all CAN signals grouped by message ID, updating at 20 Hz
- **COM port selector** — choose any PCAN USB channel (PCAN\_USBBUS1–8) from a dropdown and reconnect on the fly
- **DBC file loader** — browse and load any `.dbc` file; signal rows rebuild automatically from the new database
- **Built-in DBC** — ships with a full RC40 vehicle bus definition (speed, torque, battery, IMU, wheel speed, etc.)
- **Virtual / derived signals** — SOC alias, energy usage (kW), vehicle stationary, engine status, computed from raw CAN values
- **CAN simulator** — `can_simulator_sender.py` generates a physics-aware fake vehicle bus for offline testing
- **Headless library** — `can_monitor.py` can be imported in any Python project without a GUI

---

## Hardware Requirements

- [PEAK PCAN-USB](https://www.peak-system.com/PCAN-USB.199.0.html) or PCAN-USB Pro adapter  
- CAN bus with 120 Ω termination resistors at each end of the bus

### Connection Diagram

```
┌─────────────┐   USB   ┌──────────────────┐   CAN   ┌─────────────────────────┐
│             │─────────│                  │─────────│                         │
│  PC / Laptop│         │  PCAN-USB Adapter│  CAN_H  │  Vehicle CAN Bus        │
│             │         │  (PEAK Systems)  │─────────│  (ECUs, controllers...) │
└─────────────┘         │                  │  CAN_L  │                         │
                        └──────────────────┘─────────└─────────────────────────┘
```

```mermaid
graph LR
    PC["💻 PC / Laptop\n(USB)"] -->|USB cable| PCAN["🔌 PCAN-USB Adapter\nPeak Systems"]
    PCAN -->|"CAN_H / CAN_L\n(DB9 or bare wire)"| BUS["🚌 CAN Bus\n500 kbps"]
    BUS --> ECU1["ECU 1\n(Motor Controller)"]
    BUS --> ECU2["ECU 2\n(Battery BMS)"]
    BUS --> ECU3["ECU N\n(...)"]
```

### Wiring Reference

| PCAN-USB DB9 Pin | Signal | Wire colour (typical) |
|:---:|---|---|
| 2 | CAN_L | Blue |
| 7 | CAN_H | White |
| 3 / 6 | GND (optional shield) | Black |

> **Tip:** Always ensure 120 Ω termination resistors are fitted at both physical ends of the CAN bus.  
> The PCAN-USB adapter has a built-in software-selectable terminator.

### Dashboard UI

| Config bar | Live signals |
|---|---|
| ![Config bar](docs/screenshots/config_bar.png) | ![Signal table](docs/screenshots/signals.png) |

---

## Installation

### 1. Clone the repository

```bash
git clone https://github.com/<your-username>/CAN_simulator.git
cd CAN_simulator
```

### 2. Create a virtual environment (recommended)

```bash
python -m venv .venv
# Windows
.venv\Scripts\activate
# macOS / Linux
source .venv/bin/activate
```

### 3. Install dependencies

```bash
pip install python-can cantools
```

### 4. Install PCAN drivers (Windows)

Download and install the [PEAK PCAN Basic driver](https://www.peak-system.com/Software-APIs.305.0.html?&L=1) from the PEAK website.

---

## Usage

### Dashboard GUI

```bash
python can_dashboard_gui.py
```

With explicit channel and bitrate:

```bash
python can_dashboard_gui.py --channel PCAN_USBBUS1 --bitrate 500000
```

#### Runtime controls

| Control | How to use |
|---|---|
| **COM PORT** dropdown | Select the PCAN channel (`PCAN_USBBUS1`–`PCAN_USBBUS8`) or type a custom name |
| **CONNECT** button | Reconnect the monitor on the selected channel |
| **DBC FILE** field | Type a path or use the BROWSE button to pick a `.dbc` file |
| **LOAD DBC** button | Reload the database; signal rows rebuild automatically |

### CAN Simulator (no hardware needed)

Generates a physics-based fake vehicle on the bus for testing.  
Launching without arguments opens the GUI:

```bash
python can_simulator_sender.py
```

![Simulator Screenshot](docs/screenshots/simulator.png)

#### Simulator GUI controls

| Control | Description |
|---|---|
| **COM PORT** dropdown | PCAN channel to transmit on |
| **BITRATE** dropdown | 125k / 250k / 500k / 1M |
| **TX INTERVAL** dropdown | Frame period: 1 ms – 100 ms |
| **DBC FILE** + BROWSE | Load a custom DBC to transmit its message set |
| **LOAD DBC** button | Apply the selected DBC (only when stopped) |
| **▶ START** / **■ STOP** | Toggle simulation on/off |
| Live values panel | Shows Speed, RPM, SOC, Battery V/A, Gear, Motor Temp, Distance in real time |
| Transmit log | Scrollable log of cycle snapshots and connection events |

Run the simulator in one window and the dashboard in another to see live data without real hardware.

#### Headless / CLI mode

```bash
python can_simulator_sender.py --no-gui --channel PCAN_USBBUS1 --bitrate 500000 --interval 0.01
```

### Library usage (headless)

```python
from can_monitor import CANMonitor

mon = CANMonitor(channel="PCAN_USBBUS1", bitrate=500000)
mon.subscribe("VS", lambda name, val: print(f"Speed: {val:.1f} km/h"))
mon.subscribe("BT_SOC", lambda name, val: print(f"SOC: {val:.1f} %"))
mon.subscribe("*", lambda name, val: ...)   # every signal
mon.start()

import time
time.sleep(10)
print(mon.get_all())   # snapshot dict of all current values
mon.stop()
```

Load a custom DBC file:

```python
mon = CANMonitor(channel="PCAN_USBBUS1", bitrate=500000, dbc_path="my_vehicle.dbc")
```

---

## DBC File Support

The built-in DBC covers the **RC40 vehicle bus**:

| Message | ID | Signals |
|---|---|---|
| RC40VP1 | 0x210 | Speed, throttle, brake, torque, gear |
| RC40VP2 | 0x211 | Distance, fuel consumption |
| RC40VP3 | 0x212 | Engine speed/load, ambient temp/pressure |
| RC40IMU1/2 | 0x213/214 | Pitch, roll, yaw, angular rates |
| RC40LLC1–5 | 0x215–219 | Motor torque, battery SOC/V/A, temperatures |
| RC40WSS1 | 0x220 | Wheel speed sensors |
| TRAILER\_STATUS | 0x123 | Trailer connection state |
| CHARGER\_STATUS | 0x234 | Charger connection state |

To use your own DBC: click **BROWSE** in the dashboard, select the file, then click **LOAD DBC**.  
Signal descriptions fall back to the DBC `SG_` comment field, then the raw signal name.

---

## Project Structure

```
CAN_simulator/
├── can_dashboard_gui.py     # Tkinter GUI — dashboard + config bar
├── can_monitor.py           # Headless CAN monitor library (no GUI deps)
├── can_simulator_sender.py  # Physics-based CAN frame generator for testing
├── simpleTest.py            # Minimal usage example
├── docs/
│   └── screenshots/         # Place your UI screenshots here
└── README.md
```

### Python Files

#### `can_monitor.py` — CAN Monitor Library

The core engine. No GUI dependencies — can be imported standalone in any Python project.

| Component | Description |
|---|---|
| `CANMonitor` | Main class. Connects to PCAN bus, decodes frames, fires callbacks |
| `CANMonitor.start()` | Spawns a daemon thread that reads and decodes CAN frames |
| `CANMonitor.stop()` | Gracefully shuts down the bus connection |
| `CANMonitor.subscribe(signal, cb)` | Register a callback for a named signal or `"*"` for all |
| `CANMonitor.get(signal)` | Poll the latest value of any signal |
| `CANMonitor.get_all()` | Snapshot dict of all current signal values |
| `CANMonitor.inject(signal, value)` | Push a simulated or virtual value into the pipeline |
| `CANMonitor.db` | Access to the underlying `cantools` database |
| `CANMonitor.rx_rate` | Current message receive rate in messages/second |
| `SIGNAL_DESCRIPTIONS` | Dict mapping raw signal names → human-readable labels |

**Constructor parameters:**

```python
CANMonitor(
    channel="PCAN_USBBUS2",   # PCAN channel name
    bitrate=500_000,           # CAN bus bitrate in bps
    retry_interval=3.0,        # seconds between reconnect attempts
    dbc_path=None,             # path to a .dbc file (None = use built-in)
)
```

**Virtual / derived signals** computed automatically on every decoded frame:

| Signal | Derived from | Description |
|---|---|---|
| `soc` | `BT_SOC` | Battery state of charge alias |
| `energy_usage_kw` | `BT_C × BT_V / 1000` | Instantaneous power in kW |
| `energy_usage_stat` | `BT_C` sign | `"discharge"` / `"generate"` / `"free"` |
| `vs` | `VS < 0.5` | Vehicle stationary flag (`"0"` / `"1"`) |
| `ss` | `VS < 0.5` | Standstill state (`"on"` / `"off"`) |
| `truck_eng_stat` | `AETQ > 0` | Engine status (`"on"` / `"off"`) |

---

#### `can_dashboard_gui.py` — Dashboard GUI

Built on `tkinter`. Imports `CANMonitor` and renders signal values in real time.

| Component | Description |
|---|---|
| `CANDashboard` | Main `tk.Tk` window — wires monitor, builds UI, runs update loop |
| `_build_config_bar()` | Top bar: COM port dropdown + CONNECT, DBC path + BROWSE/LOAD |
| `_build_ui()` | Header, column labels, scrollable signal area, footer |
| `_rebuild_signals()` | Clears and recreates all signal rows from the current DBC |
| `_on_connect_clicked()` | Stops monitor, creates new one on selected channel, restarts |
| `_on_dbc_browse()` | Opens file-picker dialog filtered to `.dbc` files |
| `_on_dbc_load()` | Loads selected DBC, restarts monitor, rebuilds signal rows |
| `_schedule_ui_update()` | 20 Hz loop: flushes pending signal updates to the UI |
| `SignalRow` | One row widget — signal name, description, live value, unit |
| `Sparkline` | Mini trend chart canvas (available, disabled by default) |

**Data flow:**

```
CAN bus frames
    ↓  (python-can)
CANMonitor._rx_loop()          ← daemon thread
    ↓  cantools decode
CANMonitor._dispatch()         ← fires callbacks + stores values
    ↓  thread-safe queue
CANDashboard._on_signal()      ← writes to _pending dict
    ↓  50 ms timer (20 Hz)
CANDashboard._schedule_ui_update()
    ↓  batch flush
SignalRow.update_value()       ← updates label text
```

---

#### `can_simulator_sender.py` — CAN Frame Simulator

Generates a realistic fake vehicle on the CAN bus. Use this when no real hardware is available.

| Component | Description |
|---|---|
| `VehicleSimState` | Physics model: speed, RPM, SOC, temperatures, pedals, gear |
| `VehicleSimState.step(dt)` | Advance simulation by `dt` seconds with Gaussian noise |
| `VehicleSimState.signal_values()` | Returns dict of all signal values for the current state |
| `clamp_to_signal(sig, val)` | Clamps a value to the signal's defined min/max range |
| `SimulatorApp` | `tk.Tk` GUI window — config bar, live stats, transmit log, start/stop |
| `main()` | Headless CLI entry point (`--no-gui` flag) |

Signals simulated per cycle:

- Vehicle speed (oscillating 0–120 km/h), RPM, gear
- Accelerator / brake pedal position
- Motor torque request and feedback
- Battery SOC, voltage, current
- Motor and battery temperatures
- IMU pitch, roll, yaw (sinusoidal)
- Wheel speed sensor frequencies
- Articulation angle

---

#### `simpleTest.py` — Minimal Example

```python
from can_monitor import CANMonitor
mon = CANMonitor(channel="PCAN_USBBUS2")
mon.start()
# prints SOC value + unit every second
```

Use this as a starting point for custom integrations.

---

## Requirements

| Package | Purpose |
|---|---|
| `python-can` | CAN bus interface (PCAN driver backend) |
| `cantools` | DBC parsing and message decoding |
| `tkinter` | GUI (included with Python on Windows) |

Python **3.9+** required.

---

## License

MIT License — see [LICENSE](LICENSE) for details.
