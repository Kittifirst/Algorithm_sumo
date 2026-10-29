"""Camera acquisition keeps only the latest observation; no queued old frames."""

from dataclasses import dataclass
import math
import threading
import time

import cv2
import numpy as np

from read_apriltag import create_detector
from .planner import Pose, Vec, wrap


@dataclass
class Observation:
    frame: object
    timestamp: float
    detections: dict
    duplicates: set
    error: str = ""


class CameraWorker:
    def __init__(self, index, backend, family):
        self.index, self.backend, self.family = index, backend, family
        self.latest = None
        self.lock = threading.Lock()
        self.halt = threading.Event()
        self.retry_seconds = 1.0
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()

    def stop(self):
        self.halt.set()
        # A blocked camera driver must never prevent Stop or application shutdown.
        self.thread.join(timeout=0.25)

    def snapshot(self):
        with self.lock:
            return self.latest

    def _publish(self, observation):
        with self.lock:
            self.latest = observation

    def _run(self):
        while not self.halt.is_set():
            camera = None
            try:
                detector = create_detector(cv2, self.family)
                camera = cv2.VideoCapture(self.index, self.backend)
                camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
                camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                if not camera.isOpened():
                    raise RuntimeError("Cannot open camera; retrying automatically")
                while not self.halt.is_set():
                    timestamp = time.monotonic()
                    ok, frame = camera.read()
                    if not ok or frame is None:
                        raise RuntimeError("Camera stopped returning frames; retrying automatically")
                    corners, ids, _ = detector.detectMarkers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
                    detections, duplicates = {}, set()
                    for tag, points in zip(ids.flatten() if ids is not None else [], corners):
                        tag = int(tag)
                        if tag in detections:
                            duplicates.add(tag)
                        detections[tag] = points.reshape(4, 2).copy()
                    self._publish(Observation(frame, timestamp, detections, duplicates))
            except Exception as error:
                self._publish(Observation(None, time.monotonic(), {}, set(), str(error)))
            finally:
                if camera is not None:
                    camera.release()
            self.halt.wait(self.retry_seconds)


class Calibration:
    """Four OUTER arena corners in top-left, top-right, bottom-right, bottom-left order."""
    def __init__(self, points, image_size, arena_width=840, arena_height=840):
        source = np.asarray(points, dtype=np.float32)
        if source.shape != (4, 2) or not np.isfinite(source).all():
            raise ValueError("Click four outer corners: TL, TR, BR, BL")
        if not cv2.isContourConvex(source) or abs(cv2.contourArea(source)) < 1000:
            raise ValueError("Corners must form a convex area in TL, TR, BR, BL order")
        # Image y is down: TL,TR,BR,BL must wind clockwise, preventing mirrored steering.
        if cv2.contourArea(source, oriented=True) <= 0:
            raise ValueError("Wrong point order: click TL, TR, BR, BL clockwise")
        width, height = image_size
        if width <= 0 or height <= 0 or (source < 0).any() or (source[:, 0] >= width).any() or (source[:, 1] >= height).any():
            raise ValueError("Calibration points must be inside the camera image")
        if not all(math.isfinite(v) and v > 0 for v in (arena_width, arena_height)):
            raise ValueError("Arena dimensions must be finite and positive")
        x, y = arena_width / 2, arena_height / 2
        target = np.float32([[-x, y], [x, y], [x, -y], [-x, -y]])
        self.matrix = cv2.getPerspectiveTransform(source, target)
        if not np.isfinite(self.matrix).all() or abs(np.linalg.det(self.matrix)) < 1e-12:
            raise ValueError("Degenerate camera calibration")
        self.points, self.image_size = source.tolist(), tuple(image_size)
        self.arena_width, self.arena_height = arena_width, arena_height

    def transform(self, points):
        return cv2.perspectiveTransform(np.float32(points).reshape(1, -1, 2), self.matrix)[0]

    def as_dict(self):
        return dict(points=self.points, image_size=list(self.image_size), scheme="rectangle-v1")


class PoseTracker:
    def __init__(self):
        self.previous = {}
        self.status = {}
        self.rejected = {}

    def reset(self):
        self.previous.clear()
        self.status.clear()
        self.rejected.clear()

    def extract(self, observation, calibration, ids=(1, 2), offsets=(0, 0)):
        result = []
        for tag, offset in zip(ids, offsets):
            if tag in observation.duplicates or tag not in observation.detections:
                self.status[tag] = "ID ซ้ำ" if tag in observation.duplicates else "ภาพนี้ไม่พบ ID"
                result.append(None)
                continue
            corners = calibration.transform(observation.detections[tag])
            center = corners.mean(axis=0)
            # OpenCV returns decoded top-left, top-right, bottom-right, bottom-left.
            front = (corners[0] + corners[1]) * 0.5 - center
            if np.linalg.norm(front) < 2 or not np.isfinite(corners).all():
                self.status[tag] = "มุมแท็กเล็กหรือพิกัดไม่ถูกต้อง"
                result.append(None)
                continue
            position = Vec(float(center[0]), float(center[1]))
            heading = wrap(math.atan2(float(front[1]), float(front[0])) + math.radians(offset))
            velocity = Vec(0, 0)
            previous = self.previous.get(tag)
            if previous:
                dt = observation.timestamp - previous.timestamp
                if 0 < dt < 0.35:
                    if (position - previous.position).norm() > 2500 * dt + 60:
                        # A one-frame corner glitch must not teleport the robot.
                        # Two mutually consistent new detections can reacquire a
                        # legitimately moved robot without a permanent rejection.
                        rejected = self.rejected.get(tag)
                        if rejected is None or (position - rejected).norm() > 35:
                            self.rejected[tag] = position
                            self.status[tag] = "เห็น ID แต่ตำแหน่งกระโดด รอยืนยันภาพถัดไป"
                            result.append(None)
                            continue
                        previous = None
                    if previous is not None:
                        measured = (position - previous.position) * (1 / dt)
                        velocity = previous.velocity * 0.6 + measured * 0.4
            pose = Pose(position, heading, observation.timestamp, velocity)
            self.previous[tag] = pose
            self.rejected.pop(tag, None)
            self.status[tag] = "ใช้คำนวณแล้ว"
            result.append(pose)
        return tuple(result)
