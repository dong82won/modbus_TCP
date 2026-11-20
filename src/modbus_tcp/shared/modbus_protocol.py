from dataclasses import dataclass
from enum import IntEnum

"""Modbus TCP 통신 및 고정소수점 프로토콜 설정"""


@dataclass(frozen=True)
class ModbusConfig:
    server_host: str = "0.0.0.0"  # 서버 바인딩 IP (모든 인터페이스 허용)
    target_ip: str = "127.0.0.1"  # 테스트 클라이언트가 접속할 서버 IP
    port: int = 5020  # 비특권 TCP 포트 (502 대신 사용)
    device_id: int = 1  # 서버/클라이언트 공통 Device ID

    register_count: int = 100  # Holding Register 전체 할당 크기
    poll_interval: float = 0.01  # 핸드셰이크 감시 주기 (10ms)
    scale: float = 10000.0  # 고정소수점 스케일 (소수점 4자리 보존)


MODBUS_CONFIG = ModbusConfig()


class RegAddr(IntEnum):
    """Modbus Holding Register 주소 맵"""

    REQ_CMD = 0  # UR(Client) -> PC(Server): 1=요청, 0=대기
    READY_ACK = 1  # PC(Server) -> UR(Client): 0=대기, 1=기록완료, 2=에러
    HEARTBEAT = 2  # PC(Server) -> UR(Client): 생존 확인 카운터 (0~65535)

    # [Eye-in-Hand 주석 수정] Robot Base 절대 좌표가 아닌 TCP 기준 '상대 좌표'임을 명시
    POSE_X = 10  # Robot TCP 기준 X 상대거리 [m]
    POSE_Y = 11  # Robot TCP 기준 Y 상대거리 [m]
    POSE_Z = 12  # Robot TCP 기준 Z 상대거리 [m]
    POSE_R = 13  # Robot TCP 기준 Roll 상대각도 [rad]
    POSE_P = 14  # Robot TCP 기준 Pitch 상대각도 [rad]
    POSE_YAW = 15  # Robot TCP 기준 Yaw 상대각도 [rad]
    TAG_ID = 16  # 검출된 tag36h11 ID (uint16)


class AckStatus(IntEnum):
    """READY_ACK 레지스터 상태 코드"""

    IDLE = 0
    READY = 1
    ERROR = 2


DATA_START_ADDR = RegAddr.POSE_X
DATA_REG_COUNT = 7  # X, Y, Z, R, P, YAW (6개) + TAG_ID (1개)
