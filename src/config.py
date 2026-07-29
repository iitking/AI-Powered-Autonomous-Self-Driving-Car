"""
Central Configuration for Autonomous Self-Driving Vehicle.
Contains all hardware parameters, control tuning gains, perception constants,
safety thresholds, and simulator configuration.
"""

from dataclasses import dataclass, field
import math


@dataclass
class VehicleGeometry:
    """Physical dimensions and kinematic limits of the prototype vehicle."""
    wheelbase: float = 0.26          # Distance between front and rear axle (meters)
    track_width: float = 0.18        # Width between left and right wheels (meters)
    wheel_radius: float = 0.033      # Wheel radius (meters)
    max_steering_angle: float = 30.0 # Maximum front wheel turn angle (degrees)
    max_steering_rad: float = field(init=False)
    max_speed: float = 1.8           # Maximum safe linear speed (m/s) (~6.5 km/h for small RC car)
    min_speed: float = 0.2           # Minimum crawl speed (m/s)
    max_accel: float = 1.5           # Max acceleration (m/s^2)
    max_decel: float = 2.5           # Max deceleration / braking (m/s^2)

    def __post_init__(self):
        self.max_steering_rad = math.radians(self.max_steering_angle)


@dataclass
class PIDGains:
    """PID tuning gains for steering and longitudinal speed control."""
    # Steering PID (Cross-track error in meters to front wheel angle in degrees)
    kp_steer: float = 24.0
    ki_steer: float = 0.40
    kd_steer: float = 3.00
    steer_anti_windup: float = 8.0
    steer_filter_alpha: float = 0.35
    lookahead_dist_m: float = 0.85

    # Speed PID (Target velocity to motor PWM throttle 0.0 - 1.0)
    kp_speed: float = 0.50
    ki_speed: float = 0.05
    kd_speed: float = 0.02
    speed_anti_windup: float = 0.50


@dataclass
class CameraConfig:
    """Camera resolution and perspective transformation parameters."""
    image_width: int = 640
    image_height: int = 480
    fps: int = 30
    camera_fov_h: float = 65.0      # Horizontal Field of View (degrees)
    camera_height: float = 0.15     # Height from ground (meters)
    camera_pitch: float = 15.0      # Downward pitch angle (degrees)

    # Lane detection Region of Interest (ROI) normalized polygon [ymin, ymax]
    roi_top: float = 0.55
    roi_bottom: float = 0.95
    lane_width_meters: float = 0.40  # Track lane width in meters
    pixels_per_meter_x: float = 800.0
    pixels_per_meter_y: float = 600.0


@dataclass
class SafetyConfig:
    """Collision avoidance and failsafe thresholds."""
    critical_stop_distance: float = 0.30  # Hard emergency stop distance (meters)
    slowdown_distance: float = 0.80       # Begin slowing down (meters)
    safe_following_distance: float = 0.60 # Safe headway (meters)
    watchdog_timeout_sec: float = 0.35    # Communication timeout to trigger fail-safe stop
    traffic_light_stop_line_dist: float = 0.50 # Distance to stop line (meters)


@dataclass
class SerialConfig:
    """Serial communication parameters between SBC and microcontroller."""
    port: str = "/dev/ttyUSB0"      # Default for Linux/Jetson, /dev/cu.usbserial on Mac
    baudrate: int = 115200
    timeout: float = 0.1            # Read timeout in seconds
    sync_byte_1: int = 0xAA
    sync_byte_2: int = 0x55


@dataclass
class SimulationConfig:
    """Simulation world and time-step configuration."""
    dt: float = 0.05                # 20 Hz simulation update rate
    track_width: float = 8.0        # meters in virtual world
    track_height: float = 6.0       # meters in virtual world
    lidar_num_beams: int = 16       # Number of raycast beams
    lidar_max_range: float = 3.5    # meters
    ultrasonic_max_range: float = 2.0 # meters
    dashboard_port: int = 8080      # Web dashboard port


@dataclass
class Config:
    """Master application configuration container."""
    vehicle: VehicleGeometry = field(default_factory=VehicleGeometry)
    pid: PIDGains = field(default_factory=PIDGains)
    camera: CameraConfig = field(default_factory=CameraConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    serial: SerialConfig = field(default_factory=SerialConfig)
    simulation: SimulationConfig = field(default_factory=SimulationConfig)


# Global default instance
DEFAULT_CONFIG = Config()

# Physical vehicle dimensions: Wheelbase 0.26m, Track width 0.18m

# Camera FOV horizontal: 65.0 deg, pitch: 15.0 deg
