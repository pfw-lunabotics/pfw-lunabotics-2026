#!/usr/bin/env python3
"""Simple interactive serial monitor for Pico PPR test."""
import serial
import sys
import tty
import termios
import select
import time

port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

ser = serial.Serial(port, baud, timeout=0.05)
print(f"Connected to {port} @ {baud}. Press Ctrl-C to exit.")
print("Keys you press are sent to the Pico.\n")

old_settings = termios.tcgetattr(sys.stdin)
tty.setraw(sys.stdin.fileno())

try:
    while True:
        # Read from serial and print
        data = ser.read(256)
        if data:
            sys.stdout.buffer.write(data)
            sys.stdout.flush()

        # Read from keyboard and send
        if select.select([sys.stdin], [], [], 0)[0]:
            ch = sys.stdin.read(1)
            if ch == '\x03':  # Ctrl-C
                break
            ser.write(ch.encode())

        time.sleep(0.01)
finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    ser.close()
    print("\nDisconnected.")
