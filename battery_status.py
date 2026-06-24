#!/usr/bin/env python3
"""Reads and prints the current battery status of the PiCar-X Robot HAT."""

from robot_hat import ADC

def main():
    try:
        # Initialize the battery monitor pin (A4)
        battery_pin = ADC("A4")

        # Read raw ADC pin value (0-4095)
        raw_value = battery_pin.read()

        # Calculate voltage (A4 uses a 3x voltage divider)
        voltage = (raw_value * 3.3 / 4095) * 3

        # Map voltage from ~6.6V (empty) to ~8.4V (full) to a 0-100% scale
        percentage = (voltage - 6.6) / (8.4 - 6.6) * 100
        percentage = max(0.0, min(100.0, percentage))

        print("PiCar-X Battery Status:")
        print(f"  Raw ADC Value: {raw_value}")
        print(f"  Voltage:       {voltage:.2f} V")
        print(f"  Charge:        {percentage:.1f} %")
    except Exception as e:
        print(f"Error reading battery status: {e}")

if __name__ == "__main__":
    main()
