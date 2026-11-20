"""Intel RealSense 하드웨어 연결, 내부 파라미터 추출 및 백그라운드 프레임 수집 전담 모듈"""

import logging
import threading
import time
from collections.abc import Callable
from typing import Any

import cv2
import numpy as np
import pyrealsense2 as _rs
from numpy.typing import NDArray

from modbus_tcp.vision.config import CAMERA_CONFIG

rs: Any = _rs
logger = logging.getLogger(__name__)


class RealSenseCamera:
    """Intel RealSense 장치 연결, 내부 파라미터 추출 및 백그라운드 프레임 수집"""

    def __init__(self, reconnect_interval: float = 2.0, on_frame: Callable[[NDArray[np.uint8], NDArray[np.uint8]], None] | None = None) -> None:
        self._reconnect_interval = reconnect_interval
        self._on_frame = on_frame

        self._lock = threading.Lock()
        self._latest_gray: NDArray[np.uint8] | None = None
        self._camera_params: tuple[float, float, float, float] | None = None
        self._camera_matrix: NDArray[np.float64] | None = None

        self._is_connected = False
        self._running = False
        self._capture_thread: threading.Thread | None = None
        self._pipeline = rs.pipeline()

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def camera_params(self) -> tuple[float, float, float, float] | None:
        return self._camera_params

    @property
    def camera_matrix(self) -> NDArray[np.float64] | None:
        return self._camera_matrix

    def start(self) -> None:
        """카메라 초기 연결 및 백그라운드 수집 스레드 시작"""
        self._connect()
        self._running = True
        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()

    def get_latest_gray(self) -> NDArray[np.uint8] | None:
        """가장 최근에 수집된 흑백 프레임 복사본 반환"""
        with self._lock:
            return None if self._latest_gray is None else self._latest_gray.copy()

    def _connect(self) -> bool:
        try:
            ctx = rs.context()
            devices = ctx.query_devices()
            if len(devices) == 0:
                raise RuntimeError("연결된 Intel RealSense 장치가 없습니다.")

            dev = devices[0]
            dev_name = dev.get_info(rs.camera_info.name)
            dev_serial = dev.get_info(rs.camera_info.serial_number)

            rs_config = rs.config()
            rs_config.enable_stream(rs.stream.color, CAMERA_CONFIG.width, CAMERA_CONFIG.height, rs.format.bgr8, CAMERA_CONFIG.fps)
            profile = self._pipeline.start(rs_config)

            color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
            intr = color_stream.get_intrinsics()
            fx, fy, cx, cy = (float(intr.fx), float(intr.fy), float(intr.ppx), float(intr.ppy))

            self._camera_params = (fx, fy, cx, cy)
            self._camera_matrix = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
            self._is_connected = True

            logger.info(
                f"RealSense 연결 성공 [{dev_name} (S/N: {dev_serial})] "
                f"({CAMERA_CONFIG.width}x{CAMERA_CONFIG.height}@{CAMERA_CONFIG.fps}fps) | "
                f"Intrinsics: fx={fx:.2f}, fy={fy:.2f}, cx={cx:.2f}, cy={cy:.2f}"
            )
            return True

        except Exception as err:
            self._disconnect_cleanup()
            logger.error(f"RealSense 카메라 초기화 실패 (USB 연결/권한/해상도 확인 필요): {err}")
            return False

    def _disconnect_cleanup(self) -> None:
        self._is_connected = False
        self._camera_params = None
        self._camera_matrix = None
        with self._lock:
            self._latest_gray = None
        try:
            self._pipeline.stop()
        except Exception:
            pass

    def _capture_loop(self) -> None:
        while self._running:
            if not self._is_connected:
                time.sleep(self._reconnect_interval)
                if self._running:
                    self._connect()
                continue

            try:
                frames = self._pipeline.wait_for_frames(timeout_ms=1000)
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue

                bgr_img = np.asanyarray(color_frame.get_data())
                gray_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)

                with self._lock:
                    self._latest_gray = gray_img

                if self._on_frame is not None:
                    self._on_frame(bgr_img, gray_img)

            except Exception as err:
                if self._running:
                    logger.error(f"RealSense 프레임 수신 중단 (카메라 분리/오류): {err}")
                    self._disconnect_cleanup()

    def stop(self) -> None:
        self._running = False
        if self._capture_thread and self._capture_thread.is_alive():
            self._capture_thread.join(timeout=1.5)
        self._disconnect_cleanup()
        logger.info("RealSense 파이프라인이 정상 종료되었습니다.")
