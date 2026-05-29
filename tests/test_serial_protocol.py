"""Unit tests for Serial Packet Framing and Checksums."""

import unittest
from src.communication.serial_bridge import (
    SerialBridge,
    VehicleActuationCommand,
    HardwareTelemetry,
    compute_checksum,
    SYNC_1,
    SYNC_2,
    MSG_TELEMETRY,
)


class TestSerialProtocol(unittest.TestCase):

    def setUp(self):
        self.bridge = SerialBridge(mock=True)

    def test_encode_drive_command(self):
        cmd = VehicleActuationCommand(
            steering_deg=15.5,
            throttle=0.75,
            brake=0.0,
            emergency_stop=False,
        )
        packet = self.bridge.encode_drive_command(cmd)
        self.assertEqual(packet[0], SYNC_1)
        self.assertEqual(packet[1], SYNC_2)
        self.assertEqual(packet[2], 0x01) # MSG_DRIVE_CMD
        self.assertEqual(packet[3], 13)   # Payload length
        self.assertEqual(len(packet), 4 + 13 + 1) # 18 bytes total

    def test_decode_telemetry(self):
        # Construct synthetic valid packet
        import struct
        payload = struct.pack("<ffff", 1.25, 0.45, 90.0, 7.8)
        chk = compute_checksum(payload)
        packet = bytes([SYNC_1, SYNC_2, MSG_TELEMETRY, len(payload)]) + payload + bytes([chk])

        telemetry = self.bridge.decode_telemetry_packet(packet)
        self.assertIsNotNone(telemetry)
        self.assertAlmostEqual(telemetry.wheel_speed_mps, 1.25, places=2)
        self.assertAlmostEqual(telemetry.ultrasonic_dist_m, 0.45, places=2)
        self.assertAlmostEqual(telemetry.imu_yaw_deg, 90.0, places=2)
        self.assertAlmostEqual(telemetry.battery_volts, 7.8, places=2)

    def test_corrupted_checksum_rejection(self):
        import struct
        payload = struct.pack("<ffff", 1.0, 1.0, 0.0, 8.0)
        corrupted_chk = 0xFF
        packet = bytes([SYNC_1, SYNC_2, MSG_TELEMETRY, len(payload)]) + payload + bytes([corrupted_chk])

        telemetry = self.bridge.decode_telemetry_packet(packet)
        self.assertIsNone(telemetry)


if __name__ == "__main__":
    unittest.main()
