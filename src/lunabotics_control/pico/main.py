"""
main.py — Lunabotics 2026 Pico Motor Controller
=================================================
Board: Raspberry Pi Pico 2 W (RP2350)
Receives serial commands from Jetson via USB, drives:
  - 4 drivetrain BLDC motors (BLD-510B)
  - 1 excavation belt motor (BLD-510B)
  - 1 linear actuator (Sabertooth)
  - 1 deposition tilt servo (Waveshare ST3215 via bus servo adapter)

Serial protocol (from Jetson):
  Drivetrain: <FL:±val,FR:±val,RL:±val,RR:±val>\n    val = -65535..+65535
  Excavation: <EX:±val>\n                             val = -65535..+65535
  Actuator:   <AC:val>\n                               val = -100..+100
  Servo tilt: <SV:angle,duration>\n                    angle=0..107, duration=ms

See PICO_MOTOR_CONTROLLER_SPEC.md for full documentation.
"""

import machine
from machine import Pin, PWM, UART
import sys
import select
import time
import struct

# =================================================================
# PIN ASSIGNMENTS
# GP0/GP1: UART0 → Waveshare Bus Servo Adapter (ST3215)
# GP2-GP16: Motor drivers (BLD-510B)
# GP17-GP20: Encoder PG inputs (BLD-510B PG output)
# GP21: Actuator servo (Sabertooth)
# GP26-GP28: Reserved (ADC)
# =================================================================

ACTUATOR_PIN = 21

#              (EN,   DIR,  PWM)
MOTOR_PINS = {
    "FL":      (2,    3,    4),
    "FR":      (5,    6,    7),
    "RL":      (8,    9,    10),
    "RR":      (11,   12,   13),
    "EX":      (14,   15,   16),
}

# Encoder PG pins (BLD-510B PG output → Pico GPIO)
ENCODER_PINS = {
    "FL": 17,
    "FR": 18,
    "RL": 19,
    "RR": 20,
}

# =================================================================
# CONFIGURATION
# =================================================================
PWM_FREQ = 1000         # 1 kHz — BLD-510B V2.0 SV input expects 1-2 kHz
WATCHDOG_MS = 2000      # Stop all motors if no command for 2s (testing — was 500ms)
PWM_MAX = 65535         # 16-bit max
BUF_MAX = 128           # Max command buffer length
ENC_REPORT_MS = 50      # Report encoder counts every 50ms (20 Hz, matches pico_bridge)

# BLD-510B V2.0 polarity (ACTIVE-LOW on EN and DIR)
EN_ENABLE   = 0         # LOW = motor enabled
EN_DISABLE  = 1         # HIGH = motor disabled
DIR_FORWARD = 0         # LOW = forward
DIR_REVERSE = 1         # HIGH = reverse

# Actuator servo timing (Sabertooth)
ACTUATOR_FREQ = 50
ACTUATOR_PULSE_STOP = 1500
ACTUATOR_PULSE_RANGE = 500
ACTUATOR_PERIOD_US = 20000

# ST3215 servo config
SERVO_ID = 1
SERVO_BAUDRATE = 1000000
SERVO_STOWED_POS = 2048           # 0° — calibrate on robot
SERVO_DEGS_TO_STEPS = 4096 / 360  # ~11.378 steps per degree


# =================================================================
# ST3215 BUS SERVO DRIVER (half-duplex UART)
# =================================================================
class ST3215:
    """Minimal driver for Waveshare ST3215 bus servo over half-duplex UART."""

    # Instructions
    INST_WRITE = 0x03
    INST_READ = 0x02

    # Key registers
    REG_TORQUE_ENABLE = 0x28    # 1 byte
    REG_GOAL_POSITION = 0x2A   # 2 bytes
    REG_GOAL_TIME = 0x2C       # 2 bytes
    REG_GOAL_SPEED = 0x2E      # 2 bytes
    REG_PRESENT_POSITION = 0x38  # 2 bytes

    def __init__(self, uart_id=0, tx_pin=0, rx_pin=1, baudrate=1000000, servo_id=1):
        self.servo_id = servo_id
        self.uart = UART(uart_id, baudrate=baudrate, tx=Pin(tx_pin), rx=Pin(rx_pin))
        self.available = False

    def _checksum(self, servo_id, length, instruction, params):
        s = servo_id + length + instruction
        for p in params:
            s += p
        return (~s) & 0xFF

    def _write_packet(self, servo_id, instruction, params):
        """Send a packet to the servo."""
        length = len(params) + 2  # params + instruction + checksum
        cs = self._checksum(servo_id, length, instruction, params)
        pkt = bytes([0xFF, 0xFF, servo_id, length, instruction]) + bytes(params) + bytes([cs])
        # Flush any stale data
        if self.uart.any():
            self.uart.read()
        self.uart.write(pkt)

    def _read_response(self, timeout_ms=10):
        """Read a response packet. Returns params bytes or None."""
        start = time.ticks_ms()
        buf = bytearray()
        while time.ticks_diff(time.ticks_ms(), start) < timeout_ms:
            if self.uart.any():
                buf.extend(self.uart.read())
                # Look for a complete packet
                while len(buf) >= 2:
                    # Find header
                    idx = -1
                    for i in range(len(buf) - 1):
                        if buf[i] == 0xFF and buf[i+1] == 0xFF:
                            idx = i
                            break
                    if idx < 0:
                        buf = buf[-1:]  # keep last byte in case it's 0xFF
                        break
                    if idx > 0:
                        buf = buf[idx:]  # trim before header
                    if len(buf) < 4:
                        break  # need more bytes
                    pkt_len = buf[3]
                    total = 4 + pkt_len
                    if len(buf) < total:
                        break  # need more bytes
                    # Got full packet
                    params = buf[5:4+pkt_len-1] if pkt_len > 2 else b''
                    return bytes(params)
            time.sleep_ms(1)
        return None

    def ping(self):
        """Check if servo is reachable."""
        self._write_packet(self.servo_id, 0x01, [])  # PING instruction
        resp = self._read_response(timeout_ms=50)
        self.available = resp is not None
        return self.available

    def torque_enable(self, on):
        self._write_packet(self.servo_id, self.INST_WRITE,
                           [self.REG_TORQUE_ENABLE, 1 if on else 0])
        time.sleep_ms(2)

    def write_pos_time(self, position, time_ms):
        """Move to position over time_ms milliseconds.
        position: 0-4095 (servo raw units)
        time_ms: 0-65535 (0 = max speed)
        """
        pos_lo = position & 0xFF
        pos_hi = (position >> 8) & 0xFF
        time_lo = time_ms & 0xFF
        time_hi = (time_ms >> 8) & 0xFF
        # Write goal position (0x2A) + goal time (0x2C) = 4 bytes starting at 0x2A
        self._write_packet(self.servo_id, self.INST_WRITE,
                           [self.REG_GOAL_POSITION, pos_lo, pos_hi, time_lo, time_hi])
        time.sleep_ms(2)

    def read_position(self):
        """Read current position. Returns raw value 0-4095 or None."""
        self._write_packet(self.servo_id, self.INST_READ,
                           [self.REG_PRESENT_POSITION, 2])
        resp = self._read_response(timeout_ms=20)
        if resp and len(resp) >= 2:
            return resp[0] | (resp[1] << 8)
        return None

    def deg_to_raw(self, degrees):
        """Convert degrees to raw servo position."""
        return int(SERVO_STOWED_POS + degrees * SERVO_DEGS_TO_STEPS)

    def move_to_deg(self, degrees, duration_ms):
        """Move to angle in degrees over duration_ms."""
        raw = self.deg_to_raw(degrees)
        raw = max(0, min(4095, raw))
        duration_ms = max(0, min(65535, duration_ms))
        self.torque_enable(True)
        self.write_pos_time(raw, duration_ms)

    def stow(self, duration_ms=4500):
        """Return to stowed position (0°)."""
        self.move_to_deg(0, duration_ms)

    def stop(self):
        """Stop and hold current position."""
        pos = self.read_position()
        if pos is not None:
            self.write_pos_time(pos, 0)


# =================================================================
# MOTOR CLASS (BLD-510B)
# =================================================================
class Motor:
    def __init__(self, en_pin, dir_pin, pwm_pin, invert=False):
        self.en = Pin(en_pin, Pin.OUT, value=EN_DISABLE)
        self.dir = Pin(dir_pin, Pin.OUT, value=DIR_FORWARD)
        self.pwm = PWM(Pin(pwm_pin))
        self.pwm.freq(PWM_FREQ)
        self.pwm.duty_u16(0)
        self.invert = invert

    def drive(self, value):
        if value > PWM_MAX:
            value = PWM_MAX
        elif value < -PWM_MAX:
            value = -PWM_MAX
        if value == 0:
            self.en.value(EN_DISABLE)
            self.pwm.duty_u16(0)
            return
        effective = -value if self.invert else value
        mag = abs(effective)
        if effective > 0:
            self.dir.value(DIR_FORWARD)
        else:
            self.dir.value(DIR_REVERSE)
        self.pwm.duty_u16(mag)
        self.en.value(EN_ENABLE)

    def stop(self):
        self.en.value(EN_DISABLE)
        self.pwm.duty_u16(0)


# =================================================================
# ACTUATOR CLASS (Sabertooth via servo PWM)
# =================================================================
class Actuator:
    def __init__(self, pin_num):
        self.pwm = PWM(Pin(pin_num))
        self.pwm.freq(ACTUATOR_FREQ)
        self._set_pulse(ACTUATOR_PULSE_STOP)

    def _set_pulse(self, pulse_us):
        duty = int((pulse_us / ACTUATOR_PERIOD_US) * 65535)
        self.pwm.duty_u16(duty)

    def drive(self, percent):
        percent = max(-100, min(100, percent))
        pulse_us = ACTUATOR_PULSE_STOP + int(percent * (ACTUATOR_PULSE_RANGE / 100))
        self._set_pulse(pulse_us)

    def stop(self):
        self._set_pulse(ACTUATOR_PULSE_STOP)


# =================================================================
# INITIALIZE
# =================================================================
# Right-side motors (FR, RR) are mounted reversed — invert in software
MOTOR_INVERT = {"FL": False, "FR": True, "RL": False, "RR": True, "EX": True}

motors = {}
for name, (en, dir_p, pwm_p) in MOTOR_PINS.items():
    motors[name] = Motor(en, dir_p, pwm_p, invert=MOTOR_INVERT.get(name, False))

actuator = Actuator(ACTUATOR_PIN)

servo = ST3215(uart_id=0, tx_pin=0, rx_pin=1,
               baudrate=SERVO_BAUDRATE, servo_id=SERVO_ID)

# Check if servo is connected (non-fatal if not)
if servo.ping():
    print("[SERVO:OK]")
else:
    print("[SERVO:NOT_FOUND]")


# =================================================================
# HX711 LOAD CELL (weight sensor on deposition box)
# =================================================================
HX711_SCK_PIN = 26
HX711_DT_PIN = 27
WT_REPORT_MS = 200       # Report weight every 200ms (5 Hz)

hx711_data = Pin(HX711_DT_PIN, Pin.IN, Pin.PULL_UP)
hx711_clk = Pin(HX711_SCK_PIN, Pin.OUT)
hx711_clk.value(0)


# =================================================================
# STATUS LED (GP25 — onboard green)
# =================================================================
# Behavior:
#   - Steady ON at power-up (firmware running, USB serial ready)
#   - Toggles on every command received from Jetson (blinks while
#     Jetson is sending traffic at 20Hz → visible ~10Hz blink)
#   - Returns to steady ON on WATCHDOG (no recent commands)
status_led = Pin(25, Pin.OUT, value=1)


def read_hx711():
    """Read 24-bit value from HX711. Returns None on timeout."""
    if hx711_data.value() == 1:
        return None  # not ready
    value = 0
    for _ in range(24):
        hx711_clk.value(1)
        time.sleep_us(1)
        value = (value << 1) | hx711_data.value()
        hx711_clk.value(0)
        time.sleep_us(1)
    # One extra pulse for gain=128
    hx711_clk.value(1)
    time.sleep_us(1)
    hx711_clk.value(0)
    time.sleep_us(1)
    # 24-bit two's complement
    if value & 0x800000:
        value -= 0x1000000
    return value


# =================================================================
# ENCODER SETUP (interrupt-driven pulse counting)
# =================================================================
enc_counts = {"FL": 0, "FR": 0, "RL": 0, "RR": 0}
enc_dirs = {"FL": 0, "FR": 0, "RL": 0, "RR": 0}  # Track direction from motor DIR pin


def _make_enc_cb(name):
    """Create encoder interrupt callback that tracks direction."""
    def cb(pin):
        # Read direction from the motor's DIR pin to get signed count
        d = motors[name].dir.value()
        inv = motors[name].invert
        # DIR_FORWARD=0 means positive; invert flips it
        if (d == DIR_FORWARD) != inv:
            enc_counts[name] += 1
        else:
            enc_counts[name] -= 1
    return cb


enc_pins = {}
for name, gpio in ENCODER_PINS.items():
    try:
        p = Pin(gpio, Pin.IN, Pin.PULL_UP)
        p.irq(trigger=Pin.IRQ_RISING, handler=_make_enc_cb(name))
        enc_pins[name] = p
    except Exception as e:
        print(f"[ENC_ERR:{name}:GP{gpio}:{e}]")


# =================================================================
# SERIAL SETUP (non-blocking USB read)
# =================================================================
poll_obj = select.poll()
poll_obj.register(sys.stdin, select.POLLIN)


# =================================================================
# COMMAND PARSING
# =================================================================
def parse_and_drive(cmd):
    """Parse command string and drive the appropriate device."""
    if "FL:" in cmd:
        # Drivetrain command
        for part in cmd.split(","):
            part = part.strip()
            if ":" in part:
                label, val_str = part.split(":", 1)
                label = label.strip()
                if label in ("FL", "FR", "RL", "RR"):
                    try:
                        motors[label].drive(int(val_str))
                    except (ValueError, KeyError):
                        pass
        print("[OK]")

    elif "EX:" in cmd:
        # Excavation motor command
        try:
            val = int(cmd.split("EX:")[1].split(",")[0])
            motors["EX"].drive(val)
            print("[OK]")
        except (ValueError, IndexError, KeyError):
            motors["EX"].stop()
            print("[ERROR:bad EX value]")

    elif "AC:" in cmd:
        # Actuator command (-100 to +100)
        try:
            val = int(cmd.split("AC:")[1].split(",")[0])
            actuator.drive(val)
            print("[OK]")
        except (ValueError, IndexError):
            actuator.stop()
            print("[ERROR:bad AC value]")

    elif "SV:" in cmd:
        # Servo tilt command: SV:angle,duration_ms
        try:
            parts = cmd.split("SV:")[1].split(",")
            angle = float(parts[0])
            duration_ms = int(parts[1]) if len(parts) > 1 else 4500
            angle = max(0, min(110, angle))
            servo.move_to_deg(angle, duration_ms)
            print("[OK:SV:{}deg,{}ms]".format(angle, duration_ms))
        except (ValueError, IndexError):
            print("[ERROR:bad SV value]")

    else:
        print("[ERROR:unknown command]")


def stop_all():
    for m in motors.values():
        m.stop()
    actuator.stop()
    # Don't stop servo on watchdog — it should hold position


# =================================================================
# MAIN LOOP
# =================================================================
buf = ""
in_command = False
last_cmd_time = time.ticks_ms()
last_enc_time = time.ticks_ms()
last_wt_time = time.ticks_ms()
watchdog_printed = False

print("[READY]")

while True:
    # Non-blocking serial read
    if poll_obj.poll(0):
        char = sys.stdin.read(1)
        if char == '<':
            buf = ""
            in_command = True
        elif char == '>' and in_command:
            in_command = False
            if buf:
                parse_and_drive(buf)
                last_cmd_time = time.ticks_ms()
                watchdog_printed = False
                # Blink the status LED on each command received
                status_led.toggle()
            buf = ""
        elif in_command and len(buf) < BUF_MAX:
            buf += char

    # Report encoder counts at fixed interval (delta counts, then reset)
    now = time.ticks_ms()
    if time.ticks_diff(now, last_enc_time) >= ENC_REPORT_MS:
        last_enc_time = now
        # Disable interrupts briefly to get consistent snapshot
        state = machine.disable_irq()
        fl = enc_counts["FL"]; enc_counts["FL"] = 0
        fr = enc_counts["FR"]; enc_counts["FR"] = 0
        rl = enc_counts["RL"]; enc_counts["RL"] = 0
        rr = enc_counts["RR"]; enc_counts["RR"] = 0
        machine.enable_irq(state)
        print("[ENC:{},{},{},{}]".format(fl, fr, rl, rr))

    # Report HX711 weight at fixed interval
    if time.ticks_diff(now, last_wt_time) >= WT_REPORT_MS:
        last_wt_time = now
        wt = read_hx711()
        if wt is not None:
            print("[WT:{}]".format(wt))

    # Watchdog — stop motors + actuator if no command received recently
    # Servo is excluded — it holds position independently
    if time.ticks_diff(time.ticks_ms(), last_cmd_time) > WATCHDOG_MS:
        stop_all()
        if not watchdog_printed:
            print("[WATCHDOG]")
            watchdog_printed = True
            # Idle (no Jetson traffic) → return LED to steady ON
            status_led.value(1)
