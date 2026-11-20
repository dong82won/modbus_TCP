"""Modbus TCP 서버 구동, 핸드셰이크 감시 및 GUI 메인 루프 모듈"""

import os

# Ubuntu Wayland 환경에서 OpenCV(Qt) 폰트 경고 메시지 도배 방지
os.environ["QT_LOGGING_RULES"] = "*.debug=false;qt.qpa.*=false"

import asyncio
import logging
import threading
import time
from collections.abc import Callable

import cv2
from pymodbus.server import ModbusTcpServer
from pymodbus.simulator import DataType, SimData, SimDevice

from modbus_tcp.shared.codec import encode_detection
from modbus_tcp.shared.modbus_protocol import DATA_START_ADDR, MODBUS_CONFIG, AckStatus, RegAddr
from modbus_tcp.shared.models import TagDetection
from modbus_tcp.vision.config import CAMERA_CONFIG
from modbus_tcp.vision.provider import RealSenseAprilTagProvider

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s", datefmt="%Y-%m-%d %H:%M:%S")
logger = logging.getLogger(__name__)


def create_sim_device(device_id: int = MODBUS_CONFIG.device_id) -> SimDevice:
    """pymodbus 규격에 맞춘 시뮬레이션 디바이스 메모리 생성"""
    registers = SimData(address=0, values=[0] * MODBUS_CONFIG.register_count, datatype=DataType.REGISTERS)
    return SimDevice(id=device_id, simdata=[registers])


async def handshake_worker(server: ModbusTcpServer, detect_fn: Callable[[], TagDetection], device_id: int = MODBUS_CONFIG.device_id) -> None:
    """Modbus 클라이언트(UR)의 요청을 감시하고 검출 결과를 전달하는 비동기 워커"""
    func_code = 3
    was_requested = False
    heartbeat = 0
    loop_count = 0

    while True:
        try:
            # 1. 주기적 하트비트 갱신
            loop_count += 1
            if loop_count >= int(0.5 / MODBUS_CONFIG.poll_interval):
                loop_count = 0
                heartbeat = (heartbeat + 1) % 65536
                await server.async_setValues(device_id, func_code, RegAddr.HEARTBEAT, [heartbeat])

            # 2. UR 요청 확인
            values = await server.async_getValues(device_id, func_code, RegAddr.REQ_CMD, count=1)
            if not isinstance(values, list):
                await asyncio.sleep(MODBUS_CONFIG.poll_interval)
                continue

            req_cmd = int(values[0])

            if req_cmd == 1 and not was_requested:
                was_requested = True
                try:
                    result = await asyncio.to_thread(detect_fn)
                    regs = encode_detection(result)

                    await server.async_setValues(device_id, func_code, DATA_START_ADDR, regs)
                    await server.async_setValues(device_id, func_code, RegAddr.READY_ACK, [AckStatus.READY])
                    logger.info(f"[ID={device_id}] 전송 완료 | Tag ID: {result.tag_id} | TCP 6-DoF: {tuple(round(v, 4) for v in result.pose)}")

                except Exception as det_err:
                    logger.warning(f"[ID={device_id}] 검출/변환 실패 (UR에 ERROR=2 전달): {det_err}")
                    await server.async_setValues(device_id, func_code, RegAddr.READY_ACK, [AckStatus.ERROR])

            elif req_cmd == 0 and was_requested:
                was_requested = False
                await server.async_setValues(device_id, func_code, RegAddr.READY_ACK, [AckStatus.IDLE])

        except asyncio.CancelledError:
            break
        except Exception as loop_err:
            logger.exception(f"[ID={device_id}] 통신 루프 예기치 않은 오류: {loop_err}")

        await asyncio.sleep(MODBUS_CONFIG.poll_interval)


async def async_server_main(provider: RealSenseAprilTagProvider, stop_event: threading.Event, device_id: int = MODBUS_CONFIG.device_id) -> None:
    """ModbusTcpServer 인스턴스화 및 실행을 담당하는 비동기 메인 함수"""
    device = create_sim_device(device_id=device_id)
    server = ModbusTcpServer(context=device, address=(MODBUS_CONFIG.server_host, MODBUS_CONFIG.port))

    await server.serve_forever(background=True)
    logger.info(f"Ubuntu Modbus TCP 서버 대기 중 ({MODBUS_CONFIG.server_host}:{MODBUS_CONFIG.port}, Device ID={device_id})...")

    hs_task = asyncio.create_task(handshake_worker(server, provider.detect_and_transform, device_id=device_id))

    while not stop_event.is_set():
        await asyncio.sleep(0.2)

    logger.info("Modbus 서버 종료 절차 시작...")
    hs_task.cancel()
    await server.shutdown()


def run_async_server(provider: RealSenseAprilTagProvider, stop_event: threading.Event, device_id: int = MODBUS_CONFIG.device_id) -> None:
    """백그라운드 스레드 진입점"""
    asyncio.run(async_server_main(provider, stop_event, device_id=device_id))


def run(device_id: int = MODBUS_CONFIG.device_id) -> None:
    provider = RealSenseAprilTagProvider()
    stop_event = threading.Event()

    server_thread = threading.Thread(target=run_async_server, args=(provider, stop_event, device_id), daemon=True)
    server_thread.start()

    window_name = "RealSense AprilTag (tag36h11) - Press 'q' to quit"

    try:
        if CAMERA_CONFIG.show_preview:
            while True:
                frame = provider.get_preview_frame()
                cv2.imshow(window_name, frame)

                key = cv2.waitKey(33) & 0xFF
                if key in (ord("q"), 27):
                    logger.info("사용자 입력(q/ESC)으로 메인 루프를 종료합니다.")
                    break
        else:
            while True:
                time.sleep(1.0)

    except KeyboardInterrupt:
        logger.info("KeyboardInterrupt(Ctrl+C) 감지됨.")
    finally:
        stop_event.set()
        server_thread.join(timeout=3.0)
        cv2.destroyAllWindows()
        provider.stop()
        logger.info("프로그램이 정상 종료되었습니다.")


if __name__ == "__main__":
    run()
