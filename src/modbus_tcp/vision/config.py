"""RealSense 카메라 및 AprilTag 검출 알고리즘 전용 설정"""

from dataclasses import dataclass


@dataclass(frozen=True)
class CameraConfig:
    width: int = 640
    height: int = 480
    fps: int = 30
    show_preview: bool = True  # 모니터 없는 운영 환경에서는 False로 변경


@dataclass(frozen=True)
class AprilTagConfig:
    family: str = "tag36h11"
    tag_size_m: float = 0.048  # 실제 태그 한 변의 길이 [m] (예: 48mm = 0.048)
    min_decision_margin: float = 35.0  # 오검출 필터링 임계값


CAMERA_CONFIG = CameraConfig()
APRILTAG_CONFIG = AprilTagConfig()
