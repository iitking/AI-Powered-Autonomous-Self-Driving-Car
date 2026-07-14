"""
Object Detection and Distance Estimation Pipeline.
Supports YOLOv8/v11 inference when installed, with geometry-based
distance estimation (pinhole camera model) and simulated mock fallback.
"""

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from src.config import CameraConfig
from src.utils.math_utils import clamp

try:
    from ultralytics import YOLO
    HAS_YOLO = True
except ImportError:
    HAS_YOLO = False
    YOLO = None


@dataclass
class BoundingBox:
    """Bounding box coordinates in image space."""
    xmin: float
    ymin: float
    xmax: float
    ymax: float

    @property
    def width(self) -> float:
        return max(0.0, self.xmax - self.xmin)

    @property
    def height(self) -> float:
        return max(0.0, self.ymax - self.ymin)

    @property
    def center_x(self) -> float:
        return (self.xmin + self.xmax) / 2.0

    @property
    def center_y(self) -> float:
        return (self.ymin + self.ymax) / 2.0

    @property
    def area(self) -> float:
        return self.width * self.height


@dataclass
class DetectedObject:
    """Classified object with 3D spatial estimation relative to the vehicle."""
    class_id: int
    class_name: str
    confidence: float
    bbox: BoundingBox
    distance_m: float           # Longitudinal distance in front of car
    lateral_offset_m: float     # Lateral offset (+ right, - left)
    is_in_vehicle_path: bool    # True if collision risk exists in current lane


# Standard real-world heights (meters) for distance estimation
OBJECT_REAL_HEIGHTS: Dict[str, float] = {
    'person': 1.70,
    'pedestrian': 1.70,
    'car': 1.50,
    'truck': 2.80,
    'bus': 3.10,
    'bicycle': 1.00,
    'motorcycle': 1.10,
    'traffic light': 0.70,
    'stop sign': 0.75,
    'obstacle': 0.25,
    'box': 0.30,
    'cone': 0.45,
}


class ObjectDetector:
    """
    Real-time object detector with distance & lateral offset estimation.
    """

    def __init__(self, model_path: Optional[str] = None, config: Optional[CameraConfig] = None):
        self.config = config or CameraConfig()
        self.model = None

        # Compute focal length in pixels: f = (w / 2) / tan(FOV_h / 2)
        fov_rad = math.radians(self.config.camera_fov_h)
        self.focal_length_px = (self.config.image_width / 2.0) / math.tan(fov_rad / 2.0)

        # Attempt to load YOLO if requested and library is available
        if HAS_YOLO and model_path:
            try:
                self.model = YOLO(model_path)
            except Exception as e:
                print(f"[ObjectDetector] Warning: Failed to load YOLO from {model_path}: {e}")

    def estimate_distance(self, class_name: str, bbox: BoundingBox) -> Tuple[float, float]:
        """
        Estimates (longitudinal_distance, lateral_offset) using pinhole camera model:
        d = (f * H_real) / h_pixel
        lateral = (center_x - image_center_x) * d / f
        """
        real_h = OBJECT_REAL_HEIGHTS.get(class_name.lower(), 0.50)
        h_px = max(bbox.height, 5.0)

        distance = (self.focal_length_px * real_h) / h_px

        # Lateral offset from camera centerline
        dx_px = bbox.center_x - (self.config.image_width / 2.0)
        lateral_offset = (dx_px * distance) / self.focal_length_px

        return float(distance), float(lateral_offset)

    def is_in_path(self, distance: float, lateral_offset: float, lane_width: float = 0.40) -> bool:
        """Determines if object is in vehicle corridor."""
        half_lane = lane_width / 2.0 + 0.10  # include margin
        return abs(lateral_offset) <= half_lane and 0.10 < distance < 3.0

    def detect(self, frame) -> List[DetectedObject]:
        """
        Infers objects on image frame.
        Uses YOLO model if loaded, or falls back to synthetic parsing.
        """
        if self.model is not None and frame is not None:
            return self._detect_yolo(frame)
        elif isinstance(frame, list):
            # Synthetic / simulated objects passed directly
            return frame
        return []

    def _detect_yolo(self, frame) -> List[DetectedObject]:
        """Runs Ultralytics YOLO inference."""
        results = self.model(frame, verbose=False)
        detections: List[DetectedObject] = []

        for r in results:
            boxes = r.boxes
            for b in boxes:
                cls_id = int(b.cls[0])
                cls_name = self.model.names.get(cls_id, f"class_{cls_id}")
                conf = float(b.conf[0])
                if conf < 0.40:
                    continue

                xyxy = b.xyxy[0].tolist()
                bbox = BoundingBox(xmin=xyxy[0], ymin=xyxy[1], xmax=xyxy[2], ymax=xyxy[3])
                dist, lat = self.estimate_distance(cls_name, bbox)
                in_path = self.is_in_path(dist, lat, self.config.lane_width_meters)

                detections.append(DetectedObject(
                    class_id=cls_id,
                    class_name=cls_name,
                    confidence=conf,
                    bbox=bbox,
                    distance_m=dist,
                    lateral_offset_m=lat,
                    is_in_vehicle_path=in_path,
                ))

        return detections
