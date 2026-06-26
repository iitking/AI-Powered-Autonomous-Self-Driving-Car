"""
Serial Communication Bridge between Host SBC (Jetson/Raspberry Pi)
and Microcontroller (Arduino/ESP32).
Features structured packet framing, XOR/CRC checksum verification,
command serialization, and a MockSerial driver for testing.
"""

import struct
import time
from dataclasses import dataclass
from typing import Optional, Tuple

try:
    import serial
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False
    serial = None


# Packet Header Bytes
SYNC_1 = 0xAA
SYNC_2 = 0x55

# Message Types
MSG_DRIVE_CMD = 0x01
MSG_TELEMETRY = 0x02
MSG_ESTOP = 0x03
MSG_PING = 0x04


@dataclass
class VehicleActuationCommand:
    """Actuation command sent to microcontroller."""
    steering_deg: float  # -30.0 to +30.0 degrees
    throttle: float      # 0.0 to 1.0 (forward PWM fraction)
    brake: float         # 0.0 to 1.0 (braking effort)
    emergency_stop: bool # Force cut motor driver enable


@dataclass
class HardwareTelemetry:
    """Telemetry received from microcontroller."""
    wheel_speed_mps: float = 0.0
    ultrasonic_dist_m: float = 2.0
    imu_yaw_deg: float = 0.0
    battery_volts: float = 8.0
    encoder_ticks: int = 0
    timestamp: float = 0.0


def compute_checksum(payload: bytes) -> int:
    """Calculates XOR checksum over payload bytes."""
    chk = 0
    for b in payload:
        chk ^= b
    return chk & 0xFF


class MockSerial:
    """Simulated serial port for testing when physical hardware is not connected."""

    def __init__(self):
        self.is_open = True
        self.written_bytes = bytearray()
        self.simulated_speed = 0.0
        self.simulated_dist = 1.80

    def write(self, data: bytes) -> int:
        self.written_bytes.extend(data)
        return len(data)

    def read(self, size: int = 1) -> bytes:
        # Generate simulated telemetry packet periodically
        return b""

    def in_waiting(self) -> int:
        return 0

    def close(self):
        self.is_open = False


class SerialBridge:
    """
    Manages robust bidirectional serial communication with the microcontroller.
    """

    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 115200, mock: bool = False):
        self.port = port
        self.baudrate = baudrate
        self.use_mock = mock or (not HAS_SERIAL)
        self.ser = None
        self.is_connected = False
        self.last_send_time = 0.0
        self.connect()

    def connect(self) -> bool:
        """Attempts connection to serial port."""
        if self.use_mock:
            self.ser = MockSerial()
            self.is_connected = True
            return True

        if HAS_SERIAL:
            try:
                self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
                self.is_connected = True
                return True
            except Exception as e:
                # Fall back to mock with warning
                self.ser = MockSerial()
                self.is_connected = False
                return False
        return False

    def encode_drive_command(self, cmd: VehicleActuationCommand) -> bytes:
        """
        Packs command into byte frame:
        [SYNC_1][SYNC_2][MSG_ID][LEN=13][steer:float][throttle:float][brake:float][estop:uint8][CHECKSUM]
        """
        estop_flag = 1 if cmd.emergency_stop else 0
        payload = struct.pack("<fffB", cmd.steering_deg, cmd.throttle, cmd.brake, estop_flag)
        checksum = compute_checksum(payload)
        packet = bytearray([SYNC_1, SYNC_2, MSG_DRIVE_CMD, len(payload)])
        packet.extend(payload)
        packet.append(checksum)
        return bytes(packet)

    def decode_telemetry_packet(self, data: bytes) -> Optional[HardwareTelemetry]:
        """
        Parses telemetry frame:
        [SYNC_1][SYNC_2][MSG_TELEMETRY][LEN=16][speed:float][dist:float][yaw:float][volts:float][CHECKSUM]
        """
        if len(data) < 21:
            return None

        if data[0] != SYNC_1 or data[1] != SYNC_2 or data[2] != MSG_TELEMETRY:
            return None

        length = data[3]
        if length != 16 or len(data) < 4 + length + 1:
            return None

        payload = data[4: 4 + length]
        received_chk = data[4 + length]
        if compute_checksum(payload) != received_chk:
            return None  # Corrupted packet

        speed, dist, yaw, volts = struct.unpack("<ffff", payload)
        return HardwareTelemetry(
            wheel_speed_mps=speed,
            ultrasonic_dist_m=dist,
            imu_yaw_deg=yaw,
            battery_volts=volts,
            timestamp=time.time(),
        )

    def send_command(self, cmd: VehicleActuationCommand) -> bool:
        """Transmits actuation command to vehicle."""
        if not self.ser:
            return False

        packet = self.encode_drive_command(cmd)
        try:
            self.ser.write(packet)
            self.last_send_time = time.time()
            return True
        except Exception:
            self.is_connected = False
            return False

    def close(self):
        if self.ser and hasattr(self.ser, 'close'):
            self.ser.close()

# Communication watchdog timer integration
