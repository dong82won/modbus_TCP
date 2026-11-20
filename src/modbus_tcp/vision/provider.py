"""카메라·검출기·변환기를 조립해 Modbus 서버(server.py)에 단일 인터페이스를 제공하는 상위 제공자"""

import threading
import time

import cv2
import numpy as np
from numpy.typing import NDArray

from modbus_tcp.shared.models import TagDetection
from modbus_tcp.vision.camera import RealSenseCamera
from modbus_tcp.vision.config import CAMERA_CONFIG
from modbus_tcp.vision.detector import AprilTagDetection, AprilTagDetector, draw_tag_detections
from modbus_tcp.vision.hand_in_eye import HandEyeTransformer


class RealSenseAprilTagProvider:
    """카메라 수집기, AprilTag 검출기, 핸드아이 좌표 변환기를 조합하는 상위 제공자"""

    def __init__(
        self,
        transformer: HandEyeTransformer | None = None,
        allowed_tag_ids: set[int] | None = None,
        reconnect_interval: float = 2.0,
        preview_detect_stride: int = 3,
    ) -> None:
        self.transformer = transformer or HandEyeTransformer()
        self._preview_detect_stride = max(1, preview_detect_stride)

        self._state_lock = threading.Lock()
        self._active_requests = 0
        self._frame_counter = 0
        self._latest_preview: NDArray[np.uint8] | None = None
        self._cached_preview_detections: list[AprilTagDetection] = []

        # 로봇 요청 처리용 검출기 & 미리보기 전용 독립 검출기
        self._main_detector = AprilTagDetector(nthreads=2, allowed_tag_ids=allowed_tag_ids)
        self._preview_detector = AprilTagDetector(nthreads=1, allowed_tag_ids=allowed_tag_ids) if CAMERA_CONFIG.show_preview else None

        # RealSense 카메라 수집기 시작
        self._camera = RealSenseCamera(reconnect_interval=reconnect_interval, on_frame=self._handle_new_frame)
        self._camera.start()

    def _handle_new_frame(self, bgr_img: NDArray[np.uint8], gray_img: NDArray[np.uint8]) -> None:
        """카메라 스레드에서 새 프레임이 들어올 때마다 호출되는 미리보기 갱신 콜백"""
        if not CAMERA_CONFIG.show_preview or self._preview_detector is None:
            return

        with self._state_lock:
            is_request_active = self._active_requests > 0
            self._frame_counter += 1
            should_detect = not is_request_active and self._frame_counter % self._preview_detect_stride == 0

        if should_detect:
            detections = self._preview_detector.detect(gray_img, self._camera.camera_params)
            with self._state_lock:
                self._cached_preview_detections = detections

        with self._state_lock:
            draw_targets = list(self._cached_preview_detections)

        preview_img = draw_tag_detections(bgr_img, draw_targets, self._camera.camera_matrix, self.transformer)

        with self._state_lock:
            self._latest_preview = preview_img

    def get_preview_frame(self) -> NDArray[np.uint8]:
        """메인 GUI 스레드에서 cv2.imshow()로 출력할 최신 프레임 반환"""
        with self._state_lock:
            if self._latest_preview is not None:
                return self._latest_preview.copy()

        blank = np.zeros((CAMERA_CONFIG.height, CAMERA_CONFIG.width, 3), dtype=np.uint8)
        cv2.putText(blank, "Waiting for Intel RealSense camera...", (40, CAMERA_CONFIG.height // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 100, 255), 2)
        return blank

    def detect_and_transform(self, max_retries: int = 6) -> TagDetection:
        """Modbus 서버 요청 시 최신 프레임에서 마커를 검출하고 로봇 TCP 6-DoF 좌표 반환"""
        if not self._camera.is_connected or self._camera.camera_params is None:
            raise RuntimeError("RealSense 카메라가 연결되어 있지 않습니다.")

        with self._state_lock:
            self._active_requests += 1
        try:
            for _ in range(max_retries):
                gray_img = self._camera.get_latest_gray()
                if gray_img is not None:
                    valid_detections = self._main_detector.detect(gray_img, self._camera.camera_params)
                    if valid_detections:
                        with self._state_lock:
                            self._cached_preview_detections = list(valid_detections)

                        best = max(valid_detections, key=lambda d: float(d.decision_margin))
                        if best.pose_R is not None and best.pose_t is not None:
                            tcp_pose = self.transformer.cam_to_tcp_pose(best.pose_R, best.pose_t)
                            return TagDetection(tag_id=int(best.tag_id), pose=tcp_pose)

                time.sleep(0.05)

            raise RuntimeError(f"유효한 tag36h11 마커가 검출되지 않았습니다. ({max_retries}회 재시도 실패)")
        finally:
            with self._state_lock:
                self._active_requests = max(0, self._active_requests - 1)

    def stop(self) -> None:
        """서버 종료 시 카메라 파이프라인 안전 해제"""
        self._camera.stop()
