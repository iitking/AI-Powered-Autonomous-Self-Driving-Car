"""Communication and hardware interface modules."""

from src.communication.serial_bridge import (
    SerialBridge,
    VehicleActuationCommand,
    HardwareTelemetry,
    compute_checksum,
)

__all__ = [
    'SerialBridge',
    'VehicleActuationCommand',
    'HardwareTelemetry',
    'compute_checksum',
]
