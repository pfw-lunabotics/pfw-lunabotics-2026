#!/usr/bin/env python3
"""
servo_driver.py — Direct ST3215 bus servo control from Jetson
==============================================================
Talks to the Waveshare ST3215 servo via the Waveshare Bus Servo Adapter
plugged directly into a Jetson USB port (shows up as /dev/ttyUSB0).

Uses the Feetech SCS/STS binary protocol (same as Dynamixel v1):
  Header: 0xFF 0xFF
  Packet: [ID] [Length] [Instruction] [Params...] [Checksum]

Subscribes to:
  /deposition/tilt  (std_msgs/String)  — "angle,duration_ms" e.g. "107,4500"

Publishes:
  /servo/status     (std_msgs/String)  — servo state feedback
  /servo/position   (std_msgs/Int32)   — raw servo position (0-4095)

Parameters:
  serial_port    (str,   '/dev/ttyUSB0')  — Waveshare adapter USB device
  baud_rate      (int,   1000000)         — ST3215 default baud
  servo_id       (int,   1)               — servo ID on the bus
  stowed_pos     (int,   3283)            — raw position for 0 degrees (door closed)
  poll_rate      (float, 5.0)             — Hz, position read rate
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
import serial
import time
import struct


class ST3215Protocol:
    """Feetech ST3215 binary serial protocol (half-duplex)."""

    INST_PING = 0x01
    INST_READ = 0x02
    INST_WRITE = 0x03

    REG_TORQUE_ENABLE = 0x28      # 1 byte
    REG_GOAL_POSITION = 0x2A      # 2 bytes
    REG_GOAL_TIME = 0x2C          # 2 bytes
    REG_GOAL_SPEED = 0x2E         # 2 bytes
    REG_PRESENT_POSITION = 0x38   # 2 bytes

    def __init__(self, ser: serial.Serial, servo_id: int = 1):
        self.ser = ser
        self.servo_id = servo_id

    def _checksum(self, servo_id, length, instruction, params):
        s = servo_id + length + instruction
        for p in params:
            s += p
        return (~s) & 0xFF

    def _send_packet(self, instruction, params):
        """Build and send a protocol packet."""
        length = len(params) + 2  # params + instruction + checksum
        cs = self._checksum(self.servo_id, length, instruction, params)
        pkt = bytes([0xFF, 0xFF, self.servo_id, length, instruction]) + bytes(params) + bytes([cs])
        # Flush stale input
        if self.ser.in_waiting:
            self.ser.read(self.ser.in_waiting)
        self.ser.write(pkt)
        self.ser.flush()

    def _read_response(self, timeout_s=0.05):
        """Read a response packet. Returns param bytes or None."""
        deadline = time.monotonic() + timeout_s
        buf = bytearray()
        while time.monotonic() < deadline:
            if self.ser.in_waiting:
                buf.extend(self.ser.read(self.ser.in_waiting))
                # Look for complete packet
                while len(buf) >= 2:
                    # Find header 0xFF 0xFF
                    idx = -1
                    for i in range(len(buf) - 1):
                        if buf[i] == 0xFF and buf[i + 1] == 0xFF:
                            idx = i
                            break
                    if idx < 0:
                        buf = buf[-1:]
                        break
                    if idx > 0:
                        buf = buf[idx:]
                    if len(buf) < 4:
                        break
                    pkt_len = buf[3]
                    total = 4 + pkt_len
                    if len(buf) < total:
                        break
                    # Full packet received
                    params = bytes(buf[5:4 + pkt_len - 1]) if pkt_len > 2 else b''
                    return params
            time.sleep(0.001)
        return None

    def ping(self) -> bool:
        self._send_packet(self.INST_PING, [])
        return self._read_response(timeout_s=0.1) is not None

    def torque_enable(self, on: bool):
        self._send_packet(self.INST_WRITE,
                          [self.REG_TORQUE_ENABLE, 1 if on else 0])
        time.sleep(0.002)

    def write_pos_time(self, position: int, time_ms: int):
        """Move to position (0-4095) over time_ms milliseconds."""
        pos_lo = position & 0xFF
        pos_hi = (position >> 8) & 0xFF
        time_lo = time_ms & 0xFF
        time_hi = (time_ms >> 8) & 0xFF
        self._send_packet(self.INST_WRITE,
                          [self.REG_GOAL_POSITION, pos_lo, pos_hi, time_lo, time_hi])
        time.sleep(0.002)

    def read_position(self) -> int | None:
        """Read current position. Returns 0-4095 or None."""
        self._send_packet(self.INST_READ,
                          [self.REG_PRESENT_POSITION, 2])
        resp = self._read_response(timeout_s=0.02)
        if resp and len(resp) >= 2:
            return resp[0] | (resp[1] << 8)
        return None


class ServoDriver(Node):

    # Steps per degree (4096 positions / 360 degrees)
    DEGS_TO_STEPS = 4096.0 / 360.0

    def __init__(self):
        super().__init__('servo_driver')

        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 1000000)
        self.declare_parameter('servo_id', 1)
        self.declare_parameter('stowed_pos', 3283)
        self.declare_parameter('poll_rate', 5.0)

        self._port = self.get_parameter('serial_port').value
        self._baud = self.get_parameter('baud_rate').value
        self._servo_id = self.get_parameter('servo_id').value
        self._stowed_pos = self.get_parameter('stowed_pos').value
        poll_rate = self.get_parameter('poll_rate').value

        # Serial + protocol
        self._serial = None
        self._proto = None
        self._connected = False

        # Publishers
        self._pub_status = self.create_publisher(String, '/servo/status', 10)
        self._pub_position = self.create_publisher(Int32, '/servo/position', 10)

        # Subscriber
        self.create_subscription(String, '/deposition/tilt', self._tilt_cb, 10)

        # Connect
        self._connect()

        # Position polling timer
        if poll_rate > 0:
            self.create_timer(1.0 / poll_rate, self._poll_position)

        self.get_logger().info(
            f'Servo driver ready — port={self._port}, baud={self._baud}, '
            f'id={self._servo_id}, stowed={self._stowed_pos}'
        )

    def _connect(self):
        try:
            self._serial = serial.Serial(
                self._port, self._baud, timeout=0.01
            )
            self._proto = ST3215Protocol(self._serial, self._servo_id)
            time.sleep(0.1)  # let adapter settle

            if self._proto.ping():
                self._connected = True
                self.get_logger().info(f'ST3215 servo found on {self._port} (ID {self._servo_id})')
                self._publish_status('CONNECTED')
            else:
                self._connected = False
                self.get_logger().warn(
                    f'ST3215 servo not responding on {self._port} (ID {self._servo_id}). '
                    'Check wiring and servo ID.'
                )
                self._publish_status('NO_RESPONSE')
        except serial.SerialException as e:
            self._serial = None
            self._proto = None
            self._connected = False
            self.get_logger().error(f'Cannot open {self._port}: {e}')
            self._publish_status('DISCONNECTED')

    def _tilt_cb(self, msg: String):
        """Handle "angle,duration_ms" command."""
        if not self._connected or self._proto is None:
            self.get_logger().warn('Servo not connected — ignoring tilt command')
            return

        try:
            parts = msg.data.split(',')
            angle = float(parts[0])
            duration_ms = int(parts[1]) if len(parts) > 1 else 4500

            angle = max(-10.0, min(60.0, angle))
            duration_ms = max(0, min(65535, duration_ms))

            raw_pos = int(self._stowed_pos + angle * self.DEGS_TO_STEPS)
            raw_pos = max(0, min(4095, raw_pos))

            self._proto.torque_enable(True)
            self._proto.write_pos_time(raw_pos, duration_ms)

            self.get_logger().info(
                f'Servo tilt: {angle:.1f}° ({raw_pos} raw) over {duration_ms}ms'
            )
            self._publish_status(f'MOVING:{angle:.1f}deg,{duration_ms}ms')

        except (ValueError, IndexError) as e:
            self.get_logger().error(f'Bad tilt command "{msg.data}": {e}')

    def _poll_position(self):
        """Periodically read and publish servo position."""
        if not self._connected or self._proto is None:
            return

        try:
            pos = self._proto.read_position()
            if pos is not None:
                pos_msg = Int32()
                pos_msg.data = pos
                self._pub_position.publish(pos_msg)
        except serial.SerialException:
            self.get_logger().error('Serial read failed — servo disconnected')
            self._connected = False
            self._publish_status('DISCONNECTED')

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self._pub_status.publish(msg)

    def destroy_node(self):
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
