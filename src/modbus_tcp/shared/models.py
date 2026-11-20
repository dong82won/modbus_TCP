"""로봇 및 비전 도메인에서 공통으로 사용하는 순수 데이터 모델"""

from dataclasses import dataclass

Pose6DoF = tuple[float, float, float, float, float, float]


@dataclass(frozen=True)
class TagDetection:
    """검출된 마커 ID 및 로봇 손목(TCP) 기준 6-DoF 포즈"""

    tag_id: int
    pose: Pose6DoF  # (x, y, z [m], roll, pitch, yaw [rad]) in Robot TCP frame
