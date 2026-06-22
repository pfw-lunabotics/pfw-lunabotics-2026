"""
test_encoder_ppr.py — Measure Pulses Per Revolution from BLD-510B PG output
=============================================================================
Board: Raspberry Pi Pico 2 W (RP2350)
Purpose: Count PG pulses while you hand-spin a wheel one full revolution.

WIRING (connect ONE motor's PG first):
  BLD-510B (FL) PG  -->  Pico GP17
  BLD-510B (FR) PG  -->  Pico GP18  (optional, wire later)
  BLD-510B (RL) PG  -->  Pico GP19  (optional, wire later)
  BLD-510B (RR) PG  -->  Pico GP20  (optional, wire later)
  BLD-510B GND      -->  Pico GND   (shared ground, already connected)

HOW TO USE:
  1. Flash this file to Pico as main.py:
       mpremote cp test_encoder_ppr.py :main.py
  2. Open serial terminal:
       screen /dev/ttyACM0 115200
     or use Thonny
  3. Follow the on-screen instructions
  4. Record the PPR number — software team needs it

IMPORTANT: This script does NOT drive motors. It only reads PG pins.
           The robot will NOT move. Safe to run on the bench.

After measuring, flash the real main.py back:
  mpremote cp main.py :main.py
"""

from machine import Pin
import time
import sys
import select

# =================================================================
# PG PIN ASSIGNMENTS — match your wiring
# =================================================================
PG_PINS = {
    "FL": 17,   # GP17
    "FR": 18,   # GP18
    "RL": 19,   # GP19
    "RR": 20,   # GP20
}

# =================================================================
# PULSE COUNTERS (interrupt-driven)
# =================================================================
counts = {}
pins = {}
last_edge_us = {}

def make_callback(name):
    """Create an interrupt callback for a specific motor."""
    def callback(pin):
        counts[name] += 1
        last_edge_us[name] = time.ticks_us()
    return callback

# Set up each PG pin as input with pull-up, interrupt on rising edge
for name, gpio in PG_PINS.items():
    counts[name] = 0
    last_edge_us[name] = 0
    try:
        p = Pin(gpio, Pin.IN, Pin.PULL_UP)
        p.irq(trigger=Pin.IRQ_RISING, handler=make_callback(name))
        pins[name] = p
    except Exception as e:
        print(f"  {name} (GP{gpio}): SKIP - {e}")

# Non-blocking serial read
poll_obj = select.poll()
poll_obj.register(sys.stdin, select.POLLIN)

def read_key():
    """Read a key if available, non-blocking."""
    if poll_obj.poll(0):
        return sys.stdin.read(1)
    return None


# =================================================================
# MAIN PROGRAM
# =================================================================

print()
print("=" * 55)
print("  BLD-510B PG ENCODER — PULSES PER REVOLUTION TEST")
print("=" * 55)
print()
print("  Wired PG pins:")
for name, gpio in PG_PINS.items():
    print(f"    {name} -> GP{gpio}")
print()
print("  Connected motors will show pulse counts below.")
print("  Unconnected pins will stay at 0.")
print()

# ------------------------------------------------------------------
# Step 1: Verify PG signal is working
# ------------------------------------------------------------------
print("-" * 55)
print("  STEP 1: SIGNAL CHECK")
print("-" * 55)
print("  Spin ANY wheel slowly by hand.")
print("  You should see the count increase.")
print("  Press 's' to skip to measurement.")
print()

check_start = time.ticks_ms()
last_print = 0

while True:
    now = time.ticks_ms()

    # Print counts every 500ms
    if time.ticks_diff(now, last_print) > 500:
        last_print = now
        parts = []
        for name in ["FL", "FR", "RL", "RR"]:
            c = counts[name]
            if c > 0:
                parts.append(f"{name}:{c} *")
            else:
                parts.append(f"{name}:{c}")
        elapsed = time.ticks_diff(now, check_start) // 1000
        print(f"  [{elapsed:3d}s]  {' | '.join(parts)}")

    # Check for key press
    key = read_key()
    if key == 's' or key == 'S':
        break

    # Auto-advance if any count > 5
    any_active = any(counts[n] > 5 for n in counts)
    if any_active and time.ticks_diff(now, check_start) > 3000:
        active = [n for n in counts if counts[n] > 5]
        print(f"\n  Signal detected on: {', '.join(active)}")
        break

    time.sleep_ms(50)

# ------------------------------------------------------------------
# Step 2: Measure PPR
# ------------------------------------------------------------------
print()
print("-" * 55)
print("  STEP 2: MEASURE PULSES PER REVOLUTION")
print("-" * 55)
print()
print("  1. Put a mark on ONE wheel (tape or marker)")
print("  2. Align the mark to a reference point")
print("  3. Press ENTER to zero the counters")
print("  4. Spin that wheel EXACTLY one full revolution")
print("     (slow and steady, stop at the mark)")
print("  5. Press ENTER to read the count")
print()
print("  You can repeat this multiple times for accuracy.")
print("  Press 'q' to quit.")
print()

measurement_num = 0
measurements = {}  # name -> [list of PPR values]

while True:
    # Wait for ENTER to start measurement
    print("  >>> Press ENTER to zero counters (or 'q' to quit)")
    while True:
        key = read_key()
        if key == '\r' or key == '\n':
            break
        if key == 'q' or key == 'Q':
            break
        time.sleep_ms(50)

    if key == 'q' or key == 'Q':
        break

    # Zero all counters
    for name in counts:
        counts[name] = 0

    measurement_num += 1
    print(f"\n  --- Measurement #{measurement_num} ---")
    print("  Counters zeroed. Spin wheel ONE full revolution now.")
    print("  Press ENTER when done.")
    print()

    # Show live counts while spinning
    spin_start = time.ticks_ms()
    last_print = 0

    while True:
        now = time.ticks_ms()

        if time.ticks_diff(now, last_print) > 300:
            last_print = now
            parts = []
            for name in ["FL", "FR", "RL", "RR"]:
                c = counts[name]
                if c > 0:
                    parts.append(f"{name}:{c:4d}")
                else:
                    parts.append(f"{name}:   -")
            print(f"\r  Live: {' | '.join(parts)}  ", end="")

        key = read_key()
        if key == '\r' or key == '\n':
            break
        time.sleep_ms(50)

    # Show results
    print()
    print()
    print(f"  Measurement #{measurement_num} results:")
    for name in ["FL", "FR", "RL", "RR"]:
        c = counts[name]
        if c > 0:
            print(f"    {name}: {c} pulses per revolution")
            if name not in measurements:
                measurements[name] = []
            measurements[name].append(c)
    print()

# ------------------------------------------------------------------
# Step 3: Summary
# ------------------------------------------------------------------
print()
print("=" * 55)
print("  RESULTS SUMMARY")
print("=" * 55)

if not measurements:
    print("  No measurements taken.")
else:
    for name in ["FL", "FR", "RL", "RR"]:
        if name in measurements:
            vals = measurements[name]
            avg = sum(vals) / len(vals)
            print(f"  {name}: measurements={vals}  average={avg:.1f}")
    print()

    # Calculate overall PPR
    all_vals = []
    for vals in measurements.values():
        all_vals.extend(vals)

    if all_vals:
        overall_avg = sum(all_vals) / len(all_vals)
        print(f"  OVERALL PPR (average): {overall_avg:.1f}")
        print()
        print("  ┌────────────────────────────────────────────┐")
        print(f"  │  Tell the software team: PPR = {overall_avg:.0f}          │")
        print("  │                                            │")
        print("  │  This number goes into pico_bridge.py      │")
        print("  │  to calculate wheel odometry.              │")
        print("  └────────────────────────────────────────────┘")

print()
print("  Done. Flash the real firmware back:")
print("    mpremote cp main.py :main.py")
print()
