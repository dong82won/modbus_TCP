"""TagDetection 도메인 모델과 Modbus uint16 레지스터 간 변환 코덱"""

import math
import struct
from collections.abc import Sequence

from modbus_tcp.shared.modbus_protocol import DATA_REG_COUNT, MODBUS_CONFIG
from modbus_tcp.shared.models import TagDetection


def float_to_uint16(val: float, scale: float = MODBUS_CONFIG.scale) -> int:
    """부호 있는 실수 -> 고정소수점 int16 -> Modbus용 uint16(0~65535) 변환"""
    if math.isnan(val) or math.isinf(val):
        raise ValueError(f"유효하지 않은 수치(NaN/Inf)가 입력되었습니다: {val}")

    scaled = int(round(val * scale))
    if not (-32768 <= scaled <= 32767):
        max_val = 32767 / scale
        raise ValueError(f"값({val})이 표현 가능 범위(±{max_val:.4f})를 초과했습니다. 단위(m, rad)를 확인하세요.")
    return struct.unpack("H", struct.pack("h", scaled))[0]


def uint16_to_float(raw_val: int, scale: float = MODBUS_CONFIG.scale) -> float:
    """Modbus용 uint16(0~65535) -> 부호 있는 실수로 복원"""
    signed = struct.unpack("h", struct.pack("H", raw_val & 0xFFFF))[0]
    return signed / scale


def encode_detection(detection: TagDetection, scale: float = MODBUS_CONFIG.scale) -> list[int]:
    """TagDetection(tag_id, 6-DoF) -> 7개의 uint16 레지스터 리스트([x, y, z, r, p, y, tag_id])"""
    if not (0 <= detection.tag_id <= 65535):
        raise ValueError(f"유효하지 않은 Tag ID 범위: {detection.tag_id}")

    pose_regs = [float_to_uint16(v, scale) for v in detection.pose]
    return [*pose_regs, int(detection.tag_id)]


def decode_detection(regs: Sequence[int], scale: float = MODBUS_CONFIG.scale) -> TagDetection:
    """7개의 uint16 레지스터 시퀀스 -> TagDetection 복원"""
    if len(regs) != DATA_REG_COUNT:
        raise ValueError(f"{DATA_REG_COUNT}개의 레지스터가 필요하지만 {len(regs)}개를 수신했습니다.")

    pose_vals = tuple(uint16_to_float(r, scale) for r in regs[:6])
    tag_id = int(regs[6])
    return TagDetection(tag_id=tag_id, pose=pose_vals)  # type: ignore[arg-type]
