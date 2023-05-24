from dataclasses import dataclass

@dataclass
class Quaternion:
    x: float
    y: float
    z: float
    w: float

@dataclass
class IMU:
    sensor_id: int
    orientation: Quaternion


