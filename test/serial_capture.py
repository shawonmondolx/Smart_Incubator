"""Read-only serial capture helper for diagnosing the ESP32 over COM5.

Usage:
  python serial_capture.py [seconds] [port]

Reads lines from the serial monitor without ever opening the port in a way
that would hold DTR/RTS asserted (which resets the ESP32). Default filter
keeps only sensor / fault / reset lines so long captures stay readable.
"""
import re
import sys
import time

import serial

SECONDS = int(sys.argv[1]) if len(sys.argv) > 1 else 60
PORT = sys.argv[2] if len(sys.argv) > 2 else "COM5"

KEEP = re.compile(
    r"(Sensors raw|DHT_|DS18B20|sensor fault|brownout|Boot health|"
    r"rst:0x|RTC lost|DHT_RECOVERY|DHT_FAILURE|DHT_RECOVERED|"
    r"TEMPERATURE|HUMIDITY|checksum|read_failed|stuck|"
    r"EGG_|HEATER_|PRIMARY|ROOM|RECOVERY|INVALID|fault)",
    re.IGNORECASE,
)

ser = serial.Serial()
ser.port = PORT
ser.baudrate = 115200
ser.dtr = False
ser.rts = False
ser.timeout = 1
ser.open()
try:
    ser.setDTR(False)
    ser.setRTS(False)
except Exception:
    pass

start = time.time()
lines = 0
while time.time() - start < SECONDS:
    try:
        raw = ser.readline()
    except serial.SerialException as exc:
        print(f"[capture] serial error: {exc}")
        break
    if not raw:
        continue
    try:
        line = raw.decode("utf-8", "replace").rstrip()
    except Exception:
        continue
    if not line:
        continue
    if KEEP.search(line):
        print(line)
        lines += 1

ser.close()
print(f"[capture] done: {lines} matching lines in {SECONDS}s")
