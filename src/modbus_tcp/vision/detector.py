"""AprilTag 마커 검출, 유효성 필터링 및 OpenCV 시각화 오버레이 모듈"""

import os
import threading
from contextlib import contextmanager
from typing import Protocol, cast

import cv2
import numpy as np
from numpy.typing import NDArray
from pupil_apriltags import Detector

from modbus_tcp.vision.config import APRILTAG_CONFIG
from modbus_tcp.vision.hand_in_eye import HandEyeTransformer

_stdio_lock = threading.Lock()


@contextmanager
def suppress_c_output():
    """C 라이브러리(pupil_apriltags)의 stdout/stderr 강제 출력을 스레드 안전하게 무시"""
    with _stdio_lock:
        devnull = os.open(os.devnull, os.O_WRONLY)
        old_stdout = os.dup(1)
        old_stderr = os.dup(2)
        try:
            os.dup2(devnull, 1)
            os.dup2(devnull, 2)
            yield
        finally:
            os.dup2(old_stdout, 1)
            os.dup2(old_stderr, 2)
            os.close(old_stdout)
            os.close(old_stderr)
            os.close(devnull)


class AprilTagDetection(Protocol):
    """pupil_apriltags.Detection의 타입 힌트 보정용 프로토콜"""

    tag_id: int
    decision_margin: float
    center: NDArray[np.float64]
    corners: NDArray[np.float64]
    pose_R: NDArray[np.float64] | None
    pose_t: NDArray[np.float64] | None


class AprilTagDetector:
    """스레드 안전한 AprilTag 검출 및 유효성 필터링 래퍼"""

    def __init__(self, nthreads: int = 2, allowed_tag_ids: set[int] | None = None) -> None:
        self.allowed_tag_ids = allowed_tag_ids
        self._lock = threading.Lock()
        self._detector = Detector(
            families=APRILTAG_CONFIG.family, nthreads=nthreads, quad_decimate=1.0, quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25, debug=0
        )

    def detect(self, gray_img: NDArray[np.uint8], camera_params: tuple[float, float, float, float] | None) -> list[AprilTagDetection]:
        """흑백 영상에서 유효한 태그 목록을 검출하여 반환"""
        if camera_params is None:
            return []

        with self._lock:
            with suppress_c_output():
                raw_detections = cast(
                    list[AprilTagDetection],
                    self._detector.detect(gray_img, estimate_tag_pose=True, camera_params=camera_params, tag_size=APRILTAG_CONFIG.tag_size_m),
                )

        return [
            d
            for d in raw_detections
            if d.decision_margin >= APRILTAG_CONFIG.min_decision_margin
            and d.pose_R is not None
            and d.pose_t is not None
            and (self.allowed_tag_ids is None or int(d.tag_id) in self.allowed_tag_ids)
        ]


def draw_tag_detections(
    bgr_img: NDArray[np.uint8], detections: list[AprilTagDetection], camera_matrix: NDArray[np.float64] | None, transformer: HandEyeTransformer
) -> NDArray[np.uint8]:
    """검출된 마커에 바운딩 박스, 3D 좌표축, Cam/TCP 좌표 텍스트 오버레이"""
    vis = bgr_img.copy()
    img_h, img_w = vis.shape[:2]
    dist_coeffs = np.zeros((4, 1), dtype=np.float64)

    if not detections:
        cv2.putText(vis, "Searching AprilTag (tag36h11)...", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
        return vis

    best = max(detections, key=lambda item: float(item.decision_margin))

    for d in detections:
        if d.pose_R is None or d.pose_t is None:
            continue

        is_primary = d is best
        box_color = (0, 255, 0) if is_primary else (255, 180, 0)

        pts = d.corners.astype(np.int32).reshape((-1, 1, 2))
        cv2.polylines(vis, [pts], isClosed=True, color=box_color, thickness=2)

        cx_px, cy_px = int(round(d.center[0])), int(round(d.center[1]))
        cv2.circle(vis, (cx_px, cy_px), 4, (0, 0, 255), -1)

        if camera_matrix is not None:
            rvec, _ = cv2.Rodrigues(d.pose_R)
            cv2.drawFrameAxes(vis, camera_matrix, dist_coeffs, rvec, d.pose_t, APRILTAG_CONFIG.tag_size_m * 0.6)

        cam_x, cam_y, cam_z = (float(v) for v in d.pose_t.reshape(3))
        tcp_x, tcp_y, tcp_z, _, _, _ = transformer.cam_to_tcp_pose(d.pose_R, d.pose_t)

        lines = [
            f"ID: {int(d.tag_id)} (margin: {d.decision_margin:.1f})",
            f"Cam XYZ:({cam_x:+.3f}, {cam_y:+.3f}, {cam_z:+.3f}) m",
            f"TCP XYZ:({tcp_x:+.3f}, {tcp_y:+.3f}, {tcp_z:+.3f}) m",
        ]

        min_x = max(10, min(img_w - 330, int(np.min(d.corners[:, 0]))))
        min_y = int(np.min(d.corners[:, 1]))
        text_y_start = min_y - 62 if min_y > 75 else int(np.max(d.corners[:, 1])) + 22
        text_y_start = max(20, min(img_h - 65, text_y_start))

        cv2.rectangle(vis, (min_x - 4, text_y_start - 16), (min_x + 330, text_y_start + 46), (20, 20, 20), -1)
        for idx, text in enumerate(lines):
            cv2.putText(
                vis,
                text,
                (min_x, text_y_start + idx * 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                box_color if idx == 0 else (255, 255, 255),
                1 if idx > 0 else 2,
                cv2.LINE_AA,
            )

    return vis
