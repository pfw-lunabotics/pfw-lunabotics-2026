"""
test_hx711.py — HX711 Load Cell Quick Test
==========================================
Tests both pin orientations to diagnose wiring.

Wiring (expected):
  HX711 SCK  → GP26 (pin 31)
  HX711 DT   → GP27 (pin 32)
  HX711 VCC  → 3V3  (pin 36)
  HX711 GND  → GND  (pin 38)
"""

from machine import Pin
import time

SAMPLES = 5


def test_hx711(dt_pin, sck_pin, label=""):
    print(f"\n── Testing {label}: DT=GP{dt_pin}, SCK=GP{sck_pin} ──")

    data = Pin(dt_pin, Pin.IN, Pin.PULL_UP)
    clk  = Pin(sck_pin, Pin.OUT)
    clk.value(0)

    # Check resting state of DT pin
    time.sleep_ms(100)
    dt_state = data.value()
    print(f"  DT pin resting state: {'HIGH (not ready)' if dt_state else 'LOW (ready)'}")

    if dt_state == 1:
        # Try toggling SCK to wake it up
        print("  Sending 25 SCK pulses to reset HX711...")
        for _ in range(25):
            clk.value(1)
            time.sleep_us(1)
            clk.value(0)
            time.sleep_us(1)
        time.sleep_ms(100)
        dt_state = data.value()
        print(f"  DT pin after reset: {'HIGH (still not ready)' if dt_state else 'LOW (ready!)'}")

    # Try to read
    print("  Attempting read (2s timeout)...")
    start = time.ticks_ms()
    while data.value() == 1:
        if time.ticks_diff(time.ticks_ms(), start) > 2000:
            print("  TIMEOUT — HX711 not responding on these pins")
            # Power-down the HX711 by holding SCK high for >60us
            clk.value(1)
            time.sleep_ms(1)
            clk.value(0)
            return False

    # Read 24 bits
    value = 0
    for _ in range(24):
        clk.value(1)
        time.sleep_us(1)
        value = (value << 1) | data.value()
        clk.value(0)
        time.sleep_us(1)

    # One extra pulse for gain=128
    clk.value(1)
    time.sleep_us(1)
    clk.value(0)
    time.sleep_us(1)

    # 24-bit two's complement
    if value & 0x800000:
        value -= 0x1000000

    print(f"  SUCCESS! Raw value: {value}")

    # Read a few more
    for i in range(4):
        time.sleep_ms(100)
        start = time.ticks_ms()
        while data.value() == 1:
            if time.ticks_diff(time.ticks_ms(), start) > 2000:
                print(f"  Read #{i+2}: timeout")
                break
        else:
            v = 0
            for _ in range(24):
                clk.value(1)
                time.sleep_us(1)
                v = (v << 1) | data.value()
                clk.value(0)
                time.sleep_us(1)
            clk.value(1)
            time.sleep_us(1)
            clk.value(0)
            time.sleep_us(1)
            if v & 0x800000:
                v -= 0x1000000
            print(f"  Read #{i+2}: {v}")

    return True


def main():
    print("══════════════════════════════════════")
    print("  HX711 Wiring Diagnostic")
    print("══════════════════════════════════════")

    # First check raw pin states
    for gp in (26, 27):
        p = Pin(gp, Pin.IN, Pin.PULL_UP)
        print(f"  GP{gp} raw state (pull-up): {p.value()}")

    for gp in (26, 27):
        p = Pin(gp, Pin.IN, Pin.PULL_DOWN)
        print(f"  GP{gp} raw state (pull-down): {p.value()}")

    # Test expected orientation: SCK=GP26, DT=GP27
    ok = test_hx711(dt_pin=27, sck_pin=26, label="SCK=GP26, DT=GP27")

    if not ok:
        # Test swapped: SCK=GP27, DT=GP26
        time.sleep_ms(200)
        ok = test_hx711(dt_pin=26, sck_pin=27, label="SWAPPED: SCK=GP27, DT=GP26")

    if not ok:
        print("\n════════════════════════════════════")
        print("  NEITHER PIN COMBO WORKS")
        print("  Check:")
        print("  1. HX711 VCC → 3.3V (pin 36)")
        print("  2. HX711 GND → GND  (pin 38)")
        print("  3. Jumper wires seated properly")
        print("  4. Load cells wired to E+/E-/A+/A-")
        print("  5. HX711 board LED on?")
        print("════════════════════════════════════")
    else:
        print("\n  HX711 is responding!")


main()
