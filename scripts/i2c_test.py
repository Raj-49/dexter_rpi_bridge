#!/usr/bin/env python3
"""
i2c_test.py
===========
Run this on the Raspberry Pi BEFORE starting the full stack.
Scans the I2C bus and prints a report.

Usage:
  python3 scripts/i2c_test.py

Expected output with both PCA9685 boards wired correctly:
  Found 1 device(s): [0x40]
  ✓ PCA9685 at 0x40 — OK
"""

import sys

try:
    import board
    import busio
except ImportError:
    print("ERROR: adafruit-blinka not installed.")
    print("Run: pip3 install adafruit-blinka")
    sys.exit(1)

def scan():
    print("=" * 50)
    print("  Dexter RPi — I2C Bus Scan")
    print("=" * 50)

    try:
        i2c = busio.I2C(board.SCL, board.SDA)
    except Exception as e:
        print(f"FAILED to open I2C bus: {e}")
        print("→ Check: sudo raspi-config → Interface Options → I2C → Enable")
        sys.exit(1)

    # Lock bus for scan
    while not i2c.try_lock():
        pass

    try:
        addresses = i2c.scan()
    finally:
        i2c.unlock()

    if not addresses:
        print("WARNING: No I2C devices found!")
        print("→ Check SDA (Pin 3 / GPIO2) and SCL (Pin 5 / GPIO3) wiring")
        print("→ Check PCA9685 VCC is connected to RPi 3.3V (Pin 1)")
        print("→ Check GND is common between RPi and PCA9685")
        sys.exit(1)

    print(f"Found {len(addresses)} device(s): {[hex(a) for a in addresses]}")
    print()

    if 0x40 in addresses:
        print("✓ PCA9685 at 0x40 — FOUND")
    else:
        print("✗ PCA9685 NOT found at 0x40!")
        print("  Devices on bus:", [hex(a) for a in addresses])
        print("  → Check address jumpers on PCA9685 boards")

    print()
    print("Done. If you see 0x40 above, wiring is correct.")
    print("Next step: start the dexter-rpi-bridge service.")

if __name__ == '__main__':
    scan()
