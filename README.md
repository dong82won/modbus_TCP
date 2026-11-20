# modbus-tcp

**Intel RealSense 카메라 기반 AprilTag(`tag36h11`) 6-DoF 포즈 추정 및 Eye-in-Hand 좌표 변환 Modbus TCP 서버**

로봇 손목(TCP)에 장착된 Intel RealSense 카메라로 AprilTag 마커를 실시간 검출하고, Eye-in-Hand 캘리브레이션 행렬을 적용해 로봇 TCP 기준 6-DoF 상대 좌표 $(x, y, z, \text{roll}, \text{pitch}, \text{yaw})$로 변환한 뒤 **Modbus TCP Holding Register**를 통해 로봇 제어기(Universal Robots 등)에 전달하는 비전 통신 미들웨어입니다.

---

## 주요 기능

- **Intel RealSense 백그라운드 수집 및 자동 복구 (`vision/camera.py`)**
  - `pyrealsense2`를 통해 컬러 영상 스트림과 카메라 내부 파라미터($f_x, f_y, c_x, c_y$)를 자동으로 추출합니다.
  - USB 케이블 분리 등 예기치 않은 연결 끊김 발생 시 백그라운드 스레드에서 설정된 주기(기본 `2.0초`)마다 자동 재연결을 수행합니다.
- **스레드 안전 AprilTag 검출 및 시각화 (`vision/detector.py`)**
  - `pupil-apriltags` 라이브러리를 래핑하여 `tag36h11` 마커의 3D 회전·이동 포즈($R, t$)를 추정하며, C 라이브러리의 불필요한 콘솔 출력을 스레드 안전하게 억제합니다.
  - `decision_margin` 임계값(기본 `35.0`) 및 허용 태그 ID(`allowed_tag_ids`) 필터링으로 오검출을 걸러내고, 다수 검출 시 신뢰도가 가장 높은 마커를 선택합니다.
- **Eye-in-Hand 6-DoF 좌표계 변환 (`vision/hand_in_eye.py`)**
  - $4 \times 4$ 동차변환행렬($T_{\text{tcp\_tag}} = T_{\text{tcp\_cam}} \cdot T_{\text{cam\_tag}}$)을 통해 카메라 기준 좌표를 로봇 TCP 기준 좌표로 변환합니다.
  - Universal Robots의 `rpy2rotvec([r, p, y])` 규격과 일치하는 Extrinsic `'XYZ'` 오일러 각(rad)을 출력합니다.
- **비동기 Modbus TCP 핸드셰이크 서버 (`server.py`, `shared/`)**
  - `pymodbus` 기반 비동기 TCP 서버(`0.0.0.0:5020`, Device ID `1`)를 구동하며, `0.5초` 주기의 `HEARTBEAT` 갱신과 `10ms` 주기의 `REQ_CMD`/`READY_ACK` 핸드셰이크를 처리합니다.
  - 부호 있는 실수형 6-DoF 좌표(`m`, `rad`)를 고정소수점 스케일(`10000.0`) 기반 `uint16` 레지스터 값으로 직렬화/역직렬화합니다.
- **실시간 OpenCV 프리뷰 및 테스트 클라이언트 (`provider.py`, `client.py`)**
  - 검출된 마커의 바운딩 박스, 3D 좌표축, Camera/TCP 기준 XYZ 좌표를 실시간 오버레이로 표시합니다(모니터 없는 환경에서는 비활성화 가능).
  - 로봇 제어기의 통신 시퀀스를 검증할 수 있는 비동기 테스트 클라이언트(`client.py`)를 내장하고 있습니다.

---

## 프로젝트 구조

```text
modbus-tcp
├── pyproject.toml                 # 프로젝트 메타데이터 및 의존성 정의
├── uv.lock                        # uv 패키지 매니저 락 파일
├── README.md
└── src
   └── modbus_tcp
      ├── __init__.py
      ├── server.py                  # Modbus TCP 서버 구동, 핸드셰이크 워커 및 OpenCV GUI 루프
      ├── client.py                  # UR 제어기 동작을 모사하는 비동기 Modbus TCP 테스트 클라이언트
      ├── shared                     # 서버·클라이언트·비전 공통 프로토콜 및 데이터 모델
      │  ├── __init__.py
      │  ├── modbus_protocol.py    # Modbus TCP 네트워크 설정, Holding Register 주소 맵, 상태 코드
      │  ├── codec.py              # float <-> 고정소수점 uint16 레지스터 변환 코덱
      │  └── models.py             # Pose6DoF 타입 및 TagDetection 도메인 모델
      └── vision                     # RealSense 카메라, AprilTag 검출 및 좌표 변환 모듈
         ├── __init__.py
         ├── config.py             # 카메라 해상도/FPS/프리뷰 및 AprilTag 검출 파라미터
         ├── camera.py             # RealSense 하드웨어 연결, 내부 파라미터 추출, 프레임 수집 스레드
         ├── detector.py           # AprilTag 검출 래퍼, 유효성 필터링 및 OpenCV 시각화 오버레이
         ├── hand_in_eye.py        # 카메라 -> 로봇 TCP 6-DoF 동차변환 행렬 연산
         └── provider.py           # 카메라·검출기·변환기를 통합하는 상위 비전 서비스 인터페이스
```

---

## Modbus Holding Register 맵

모든 통신은 **Function Code 3 (Read Holding Registers)** 및 **Write Single Register** 기준으로 동작하며, 데이터 레지스터는 고정소수점(`scale = 10000.0`, 소수점 4자리 정밀도) 2의 보수 `int16`을 `uint16`(`0 ~ 65535`)으로 매핑하여 전송합니다.

| 주소 (Address) | 레지스터명 | 방향 | 타입 / 인코딩 | 단위 / 값 설명 |
| :---: | :--- | :---: | :---: | :--- |
| `0` | `REQ_CMD` | Client → Server | `uint16` | `0`: 대기(IDLE), `1`: AprilTag 검출 요청 |
| `1` | `READY_ACK` | Server → Client | `uint16` | `0`: 대기(`IDLE`), `1`: 기록 완료(`READY`), `2`: 검출 실패/오류(`ERROR`) |
| `2` | `HEARTBEAT` | Server → Client | `uint16` | 서버 생존 확인용 카운터 (`0 ~ 65535`, `0.5초`마다 +1 증가) |
| `10` | `POSE_X` | Server → Client | `int16 × 10000` (`uint16`) | Robot TCP 기준 X 상대 거리 (`m`, 표현 범위: `±3.2767 m`) |
| `11` | `POSE_Y` | Server → Client | `int16 × 10000` (`uint16`) | Robot TCP 기준 Y 상대 거리 (`m`, 표현 범위: `±3.2767 m`) |
| `12` | `POSE_Z` | Server → Client | `int16 × 10000` (`uint16`) | Robot TCP 기준 Z 상대 거리 (`m`, 표현 범위: `±3.2767 m`) |
| `13` | `POSE_R` | Server → Client | `int16 × 10000` (`uint16`) | Robot TCP 기준 Roll 상대 각도 (`rad`, 표현 범위: `±3.2767 rad`) |
| `14` | `POSE_P` | Server → Client | `int16 × 10000` (`uint16`) | Robot TCP 기준 Pitch 상대 각도 (`rad`, 표현 범위: `±3.2767 rad`) |
| `15` | `POSE_YAW` | Server → Client | `int16 × 10000` (`uint16`) | Robot TCP 기준 Yaw 상대 각도 (`rad`, 표현 범위: `±3.2767 rad`) |
| `16` | `TAG_ID` | Server → Client | `uint16` | 검출된 `tag36h11` 마커 ID (`0 ~ 65535`) |

---

## 통신 핸드셰이크 시퀀스

```text
Client (UR Robot)                                 Server (Vision PC)
       │                                                  │
       │  1. REQ_CMD (Addr 0) = 1 쓰기 ──────────────────►│
       │                                                  ├─ RealSense 최신 프레임 획득
       │                                                  ├─ AprilTag 검출 (최대 6회 재시도)
       │                                                  ├─ Eye-in-Hand TCP 6-DoF 변환
       │                                                  ├─ Addr 10~16에 포즈 및 Tag ID 기록
       │  2. READY_ACK (Addr 1) 폴링 ◄────────────────────┤─ READY_ACK = 1 (성공) 또는 2 (에러)
       │     (READY=1 확인)                               │
       │                                                  │
       │  3. Addr 10~16 (7개 레지스터) 일괄 읽기 ─────────►│
       │                                                  │
       │  4. REQ_CMD (Addr 0) = 0 초기화 ────────────────►│
       │                                                  ├─ READY_ACK = 0 (IDLE) 복귀
       │  5. READY_ACK == 0 확인 후 사이클 완료 ◄─────────┤
```

---

## 설치 및 실행 방법

### 1. 사전 요구 사항
- **Python**: `3.10` 이상
- **패키지 매니저**: [`uv`](https://docs.astral.sh/uv/)
- **하드웨어**: Intel RealSense 카메라 (USB 3.0 연결 권장)
- **주요 의존성**: `pymodbus`, `pyrealsense2`, `opencv-python`, `numpy`, `pupil-apriltags`, `scipy`

### 2. 환경 구성

```bash
git clone <repository-url>
cd modbus-tcp
uv sync
```

### 3. Modbus TCP 비전 서버 실행

```bash
uv run python -m modbus_tcp.server
```
- 서버가 `0.0.0.0:5020` (Device ID: `1`)에서 대기하며, `show_preview = True`인 경우 OpenCV 실시간 검출 창이 열립니다.
- 프리뷰 창에서 `q` 또는 `ESC` 키를 누르거나 터미널에서 `Ctrl+C`를 입력하면 카메라 파이프라인과 서버가 안전하게 종료됩니다.

### 4. 테스트 클라이언트 실행 (별도 터미널)

```bash
uv run python -m modbus_tcp.client
```
- 로컬 서버(`127.0.0.1:5020`)에 접속하여 1초 간격으로 총 20회의 AprilTag 검출 요청 및 레지스터 디코딩 테스트를 수행합니다.

---

## 주요 설정 커스터마이징

- **네트워크 및 프로토콜 (`src/modbus_tcp/shared/modbus_protocol.py`)**
  - `MODBUS_CONFIG`: 바인딩 IP(`server_host`), 접속 대상 IP(`target_ip`), 포트(`port=5020`), `device_id=1`, 폴링 주기(`poll_interval=0.01`), 고정소수점 스케일(`scale=10000.0`)을 변경할 수 있습니다.
- **카메라 및 AprilTag 파라미터 (`src/modbus_tcp/vision/config.py`)**
  - `CAMERA_CONFIG`: 해상도(`640x480`), 프레임레이트(`30fps`), GUI 창 출력 여부(`show_preview=True`)를 설정합니다.
  - `APRILTAG_CONFIG`: 태그 패밀리(`tag36h11`), **실제 출력된 태그 한 변의 물리적 길이(`tag_size_m=0.048`)**, 최소 검출 신뢰도(`min_decision_margin=35.0`)를 설정합니다.
- **Eye-in-Hand 캘리브레이션 행렬 (`src/modbus_tcp/vision/hand_in_eye.py`)**
  - `DEFAULT_T_TCP_CAM`에 실제 로봇-카메라 핸드아이 캘리브레이션 결과($4 \times 4$ 변환 행렬, 단위: `m`)를 반영해야 정확한 로봇 TCP 목표 좌표가 산출됩니다.

---

## 운영 시 주의사항

1. **고정소수점 표현 범위 제한**: `scale = 10000.0` 설정 시 `int16`이 표현할 수 있는 값은 `-3.2768 ~ +3.2767` (`m` 또는 `rad`)입니다. 범위를 초과하는 좌표가 입력되면 `codec.py`에서 `ValueError`를 발생시키고 서버는 클라이언트에게 `READY_ACK = 2 (ERROR)`를 반환합니다.
2. **헤드리스(Monitorless) 운영 환경**: 모니터가 연결되지 않은 로봇 제어 PC(Ubuntu 등)에서 백그라운드 서비스로 구동할 때는 `src/modbus_tcp/vision/config.py`의 `CameraConfig.show_preview`를 `False`로 설정하세요.
3. **물리적 태그 크기 일치**: `APRILTAG_CONFIG.tag_size_m` 값이 실제 인쇄된 마커의 검은색 테두리 한 변 길이(미터 단위)와 다르면 Z축 거리 및 3D 포즈 추정값에 비례 오차가 발생합니다.
