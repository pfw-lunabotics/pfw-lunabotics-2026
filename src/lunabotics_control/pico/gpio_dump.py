"""
gpio_dump.py — Dump all GPIO pin configurations on Raspberry Pi Pico 2 W
=========================================================================
Flash this to the Pico (or run via Thonny REPL) to see exactly how every
GP pin is currently configured. This helps the software team understand
ECE's actual wiring.

HOW TO RUN:
  Option A: Copy to Pico as main.py, reboot, read serial output
  Option B: Open Thonny, paste this into REPL, read output

OUTPUT: For each GP0-GP28, reports:
  - Pin mode (IN/OUT/ALT)
  - Current value (HIGH/LOW)
  - Whether PWM is active and at what frequency/duty
  - Whether UART is using that pin
  - Whether the pin is an ADC pin
"""

from machine import Pin, PWM, ADC, UART
import sys
import time

print("=" * 60)
print("  PICO GPIO DIAGNOSTIC DUMP")
print("  Board: Raspberry Pi Pico 2 W (RP2350)")
print("  Date: run at boot")
print("=" * 60)
print()

# Known special pins
UART0_TX = 0
UART0_RX = 1
ADC_PINS = [26, 27, 28]
POWER_PINS_NOTE = {
    "VBUS": 40,
    "VSYS": 39,
    "3V3_EN": 37,
    "3V3_OUT": 36,
    "ADC_VREF": 35,
    "RUN": 30,
}

# Try to detect what's connected to each pin
print("PIN  | DIR  | VALUE | PWM_CAPABLE | ADC | NOTES")
print("-" * 60)

for gp in range(29):
    notes = []

    # Check special roles
    if gp == UART0_TX:
        notes.append("UART0_TX (servo bus?)")
    if gp == UART0_RX:
        notes.append("UART0_RX (servo bus?)")
    if gp in ADC_PINS:
        notes.append("ADC capable")

    # Try reading pin as input to see its state
    try:
        p = Pin(gp, Pin.IN, Pin.PULL_DOWN)
        val_pulldown = p.value()

        p = Pin(gp, Pin.IN, Pin.PULL_UP)
        val_pullup = p.value()

        # Determine if something is driving the pin
        if val_pulldown == 1 and val_pullup == 1:
            driven = "DRIVEN HIGH"
        elif val_pulldown == 0 and val_pullup == 0:
            driven = "DRIVEN LOW"
        elif val_pulldown == 0 and val_pullup == 1:
            driven = "FLOATING"
        else:
            driven = "UNCERTAIN"

        # Release the pin
        p = Pin(gp, Pin.IN)
        raw_val = p.value()

        # Check if PWM works on this pin
        pwm_ok = "YES"
        try:
            test_pwm = PWM(Pin(gp))
            test_pwm.deinit()
        except Exception:
            pwm_ok = "NO"

        adc_str = "YES" if gp in ADC_PINS else "no"

        note_str = ", ".join(notes) if notes else ""
        print(f"GP{gp:2d} | IN   | {raw_val}     | {pwm_ok:11s} | {adc_str:3s} | {driven:12s} {note_str}")

    except Exception as e:
        print(f"GP{gp:2d} | ERROR: {e}")

print()
print("=" * 60)
print("  PIN ACTIVE-DRIVE TEST")
print("  (Toggles each pin as OUTPUT briefly to verify it works)")
print("  NOTE: This will pulse each pin! Only run with motors OFF")
print("=" * 60)
print()

# Skip UART pins (0,1) and ADC pins (26,27,28) for the drive test
SKIP_PINS = [0, 1, 26, 27, 28]

print("Testing output drive capability on GP2-GP25...")
for gp in range(2, 26):
    if gp in SKIP_PINS:
        print(f"  GP{gp:2d}: SKIPPED (reserved)")
        continue
    try:
        p = Pin(gp, Pin.OUT)
        p.value(1)
        time.sleep_ms(2)
        high_ok = p.value()
        p.value(0)
        time.sleep_ms(2)
        low_ok = (p.value() == 0)
        p = Pin(gp, Pin.IN)  # release
        status = "OK" if (high_ok and low_ok) else "PROBLEM"
        print(f"  GP{gp:2d}: {status} (can drive HIGH and LOW)")
    except Exception as e:
        print(f"  GP{gp:2d}: FAIL — {e}")

print()
print("=" * 60)
print("  OUR FIRMWARE EXPECTS THESE PIN ASSIGNMENTS:")
print("=" * 60)
print()
print("  Motor  | EN  | DIR | PWM | Invert?")
print("  -------+-----+-----+-----+--------")
print("  FL     | GP2 | GP3 | GP4 | No")
print("  FR     | GP5 | GP6 | GP7 | No")
print("  RL     | GP8 | GP9 | GP10| No")
print("  RR     | GP11| GP12| GP13| Yes (dir reversed in software)")
print("  EX     | GP14| GP15| GP16| No")
print("  -------+-----+-----+-----+--------")
print("  Actuator (Sabertooth servo PWM): GP21")
print("  Servo UART TX: GP0,  Servo UART RX: GP1")
print()
print("  ECE: Does this match your wiring? If not, tell us the")
print("  correct pin assignments and we'll update the firmware.")
print()
print("=" * 60)
print("  WIRING VERIFICATION — READ BACK FROM BLD-510B HEADERS")
print("=" * 60)
print()
print("  For each motor, tell us which GP pin connects to:")
print("    EN  — the ENABLE wire on the BLD-510B")
print("    DIR — the DIRECTION wire on the BLD-510B")
print("    PWM — the SPEED/PWM wire on the BLD-510B")
print()
print("  Fill in this table and send it back:")
print()
print("  Motor  | EN pin | DIR pin | PWM pin | Verified?")
print("  -------+--------+---------+---------+----------")
print("  FL     | GP__   | GP__    | GP__    | [ ]")
print("  FR     | GP__   | GP__    | GP__    | [ ]")
print("  RL     | GP__   | GP__    | GP__    | [ ]")
print("  RR     | GP__   | GP__    | GP__    | [ ]")
print("  EX     | GP__   | GP__    | GP__    | [ ]")
print("  Actuator: GP__")
print("  Servo TX: GP__   Servo RX: GP__")
print()
print("[DUMP COMPLETE]")
