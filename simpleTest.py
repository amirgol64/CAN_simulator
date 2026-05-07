from can_monitor import CANMonitor
import time

# Connect to CAN bus
monitor = CANMonitor(channel="PCAN_USBBUS2", bitrate=500000)
monitor.start()

# Subscribe to a signal
# Note: The unit is defined in the DBC file, so we can retrieve it from the signal metadata.
# You can skip this step if you just want to print the raw value without the unit.
signal = monitor.db.get_message_by_name("VIRTUAL_STATUS").get_signal_by_name("soc")
unit = signal.unit   # → "%"

while True:
    soc = monitor.get("soc")

    print(f"SOC: {soc} {unit}")
    # print(f"SOC: {soc} %")

    time.sleep(1)