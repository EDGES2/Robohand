"""
Robotic Hand Controller — Hand Tracking via Webcam
Tracks hand via webcam and sends commands to Arduino Nano over USB Serial.

Protocol: <IDX,ANGLE>
  IDX   — finger index 0–4
  ANGLE — target angle 0–180°

Dependencies: pip install opencv-python mediapipe pyserial
"""

import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import serial
import serial.tools.list_ports
import numpy as np
import time
import sys

# ══════════════════════════════════════════════════════════════════════════════
#  CALIBRATION — angles per finger
# ══════════════════════════════════════════════════════════════════════════════
#
#  ANGLE_OPEN[i]  — fully extended angle (usually 180°)
#  ANGLE_CLOSE[i] — fully curled angle
#
#               [Thumb, Index, Middle, Ring, Pinky]
ANGLE_OPEN  = [180,   180,   180,   180,  180]
ANGLE_CLOSE = [100,    80,   105,   100,   90]

# ══════════════════════════════════════════════════════════════════════════════
#  STAGES
# ══════════════════════════════════════════════════════════════════════════════
#
#  STAGES[i]           — target angles (°) for finger i.
#                        First = open (= ANGLE_OPEN), last = closed (≈ ANGLE_CLOSE).
#                        Servo only moves between these values.
#
#  STAGE_THRESHOLDS[i] — bend detection thresholds (0–100) for stage selection.
#                        Each threshold is the lower bound of the next stage.
#                        First threshold must always be 0.
#
#  Example (index finger):
#    Stages:     [180, 145, 115, 80]   ← angles
#    Thresholds: [  0,  25,  50, 75]   ← bend% lower bounds
#
#    bend%=  0  →  0 >=  0 and < 25  → stage 1 → 180°
#    bend%= 30  → 30 >= 25 and < 50  → stage 2 → 145°
#    bend%= 60  → 60 >= 50 and < 75  → stage 3 → 115°
#    bend%= 80  → 80 >= 75           → stage 4 →  80°
#
#  Rules: len(STAGES[i]) == len(STAGE_THRESHOLDS[i])
#         STAGES sorted descending, THRESHOLDS sorted ascending, first threshold = 0
#
#               [Thumb,              Index,               Middle,              Ring,                Pinky               ]
STAGES = [
    [180, 150, 125, 100],   # Thumb
    [180, 145, 115,  80],   # Index
    [180, 155, 130, 105],   # Middle
    [180, 150, 125, 100],   # Ring
    [180, 160, 135,  90],   # Pinky
]

STAGE_THRESHOLDS = [
    [  0,  70,  80, 90],   # Thumb  (custom thresholds)
    [  0,  25,  40, 60],   # Index
    [  0,  25,  40, 60],   # Middle
    [  0,  25,  40, 60],   # Ring
    [  0,  25,  40, 50],   # Pinky
]

# Dead zone: 0.0 = react to any movement  |  0.15 = ignore fluctuations < 15%
BEND_THRESHOLD = 0.15

# Minimum time between sent commands (seconds)
SEND_INTERVAL = 0.08    # ~12 Hz

SERIAL_BAUD  = 9600
SERIAL_PORT  = None   # None = auto-detect, or set manually: "COM3" / "/dev/ttyUSB0"
CAMERA_INDEX = 0

# ══════════════════════════════════════════════════════════════════════════════


def detect_stage(bend_pct: float, thresholds: list[int], stages: list[int]) -> int:
    """
    Returns the target servo angle based on bend percentage and stage thresholds.

    bend_pct   — bend level 0–100
    thresholds — ascending lower bounds, first = 0
    stages     — angles per stage, descending (open → closed)
    """
    result = stages[0]
    for thresh, stage_val in zip(thresholds, stages):
        if bend_pct >= thresh:
            result = stage_val
    return result


def find_arduino_port() -> str | None:
    ports = serial.tools.list_ports.comports()
    for port in ports:
        desc = (port.description or "").lower()
        if any(kw in desc for kw in ("arduino", "ch340", "cp210", "ftdi", "usb serial")):
            return port.device
    return ports[0].device if ports else None


def _lm3(landmarks, idx: int) -> np.ndarray:
    """Landmark → 3D numpy array."""
    lm = landmarks[idx]
    return np.array([lm.x, lm.y, lm.z], dtype=np.float64)


def build_palm_basis(landmarks):
    """
    Builds an orthonormal coordinate system for the palm.

    Returns (origin, x_axis, y_axis, z_axis, palm_size):
      origin    — wrist (lm 0) in global coordinates
      y_axis    — wrist → middle MCP (lm 9), along the palm
      x_axis    — across the palm (pinky → index MCP)
      z_axis    — palm normal (back → front)
      palm_size — scale (wrist → middle MCP distance)
    """
    wrist   = _lm3(landmarks, 0)
    mid_mcp = _lm3(landmarks, 9)
    idx_mcp = _lm3(landmarks, 5)
    pin_mcp = _lm3(landmarks, 17)

    y_raw     = mid_mcp - wrist
    palm_size = float(np.linalg.norm(y_raw)) + 1e-6
    y_axis    = y_raw / palm_size

    x_raw  = pin_mcp - idx_mcp
    x_raw -= np.dot(x_raw, y_axis) * y_axis
    x_norm = np.linalg.norm(x_raw)
    x_axis = x_raw / x_norm if x_norm > 1e-6 else np.array([1., 0., 0.])

    z_axis = np.cross(x_axis, y_axis)
    z_axis /= (np.linalg.norm(z_axis) + 1e-6)

    return wrist, x_axis, y_axis, z_axis, palm_size


def _to_local(point: np.ndarray,
              origin: np.ndarray,
              x_axis: np.ndarray,
              y_axis: np.ndarray,
              z_axis: np.ndarray) -> np.ndarray:
    """Global 3D point → local palm coordinates (x, y, z)."""
    v = point - origin
    return np.array([np.dot(v, x_axis),
                     np.dot(v, y_axis),
                     np.dot(v, z_axis)])


def angle_between(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float:
    ba = a - b
    bc = c - b
    cos_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc) + 1e-6)
    return float(np.degrees(np.arccos(np.clip(cos_angle, -1.0, 1.0))))


def finger_bend_ratio(landmarks, tip_idx: int, pip_idx: int, mcp_idx: int,
                      origin, x_axis, y_axis, z_axis) -> float:
    """
    Finger bend ratio in palm-local coordinates: 0.0 = straight, 1.0 = fully curled.
    """
    tip_l = _to_local(_lm3(landmarks, tip_idx),  origin, x_axis, y_axis, z_axis)
    pip_l = _to_local(_lm3(landmarks, pip_idx),  origin, x_axis, y_axis, z_axis)
    mcp_l = _to_local(_lm3(landmarks, mcp_idx),  origin, x_axis, y_axis, z_axis)
    raw_angle = angle_between(tip_l, pip_l, mcp_l)
    return float(np.clip(180.0 - raw_angle, 0, 180) / 180.0)


def thumb_bend_ratio(landmarks, origin, x_axis, y_axis, z_axis, palm_size: float) -> float:
    """
    Thumb bend ratio in palm-local coordinates.
    """
    mcp_l = _to_local(_lm3(landmarks, 2), origin, x_axis, y_axis, z_axis)
    tip_l = _to_local(_lm3(landmarks, 4), origin, x_axis, y_axis, z_axis)

    vec_xy = np.array([tip_l[0] - mcp_l[0],
                       tip_l[1] - mcp_l[1]])
    norm = np.linalg.norm(vec_xy)
    if norm < 1e-6:
        return 0.0
    vec_xy /= norm

    angle = float(np.degrees(np.arctan2(abs(vec_xy[1]), abs(vec_xy[0]))))
    ratio = np.clip(angle / 75.0, 0.0, 1.0)
    return float(ratio)


def build_packet(idx: int, angle: int) -> bytes:
    """Builds an <IDX,ANGLE> packet for Arduino."""
    return f"<{idx},{angle}>\n".encode("ascii")


def draw_ui(frame, bend_ratios: list[float], current_angles: list[int],
            stages_cfg: list[list[int]], fps: float, connected: bool):
    h, w = frame.shape[:2]
    finger_names = ["Big", "Index", "Middle", "Ring", "Pinky"]

    cv2.rectangle(frame, (0, 0), (260, h), (20, 20, 28), -1)

    cv2.putText(frame, "ROBO HAND", (12, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)

    sc = (0, 220, 80) if connected else (60, 60, 220)
    st = "SERIAL OK" if connected else "NO SERIAL"
    cv2.putText(frame, f"FPS {fps:.1f}  {st}", (12, 52),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, sc, 1)

    for i, (name, ratio) in enumerate(zip(finger_names, bend_ratios)):
        y     = 76 + i * 42
        bar_w = int(ratio * 170)

        cv2.putText(frame, name, (12, y - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.36, (160, 160, 160), 1)

        # Bend bar
        cv2.rectangle(frame, (12, y), (182, y + 14), (40, 40, 50), -1)
        color = (int(ratio * 255), int((1 - ratio) * 180), 80)
        cv2.rectangle(frame, (12, y), (12 + bar_w, y + 14), color, -1)

        # Current angle and range
        angle     = current_angles[i]
        max_angle = stages_cfg[i][0]
        cv2.putText(frame, f"{angle}/{max_angle}", (186, y + 11),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.30, (180, 180, 180), 1)

        # Stage markers on the bar (normalized relative to max_angle)
        for s in stages_cfg[i]:
            if max_angle > 0:
                norm = (max_angle - s) / max_angle
                sx = 12 + int(norm * 170)
                cv2.line(frame, (sx, y), (sx, y + 14), (255, 255, 100), 1)

    cv2.putText(frame, "Q - quit", (12, h - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (80, 80, 80), 1)


def draw_landmarks(rgb_image, detection_result):
    if not detection_result.hand_landmarks:
        return rgb_image
    connections = [
        (0,1),(1,2),(2,3),(3,4),(0,5),(5,6),(6,7),(7,8),
        (5,9),(9,10),(10,11),(11,12),(9,13),(13,14),(14,15),(15,16),
        (13,17),(17,18),(18,19),(19,20),(0,17)
    ]
    h, w = rgb_image.shape[:2]
    for lms in detection_result.hand_landmarks:
        for lm in lms:
            cv2.circle(rgb_image, (int(lm.x * w), int(lm.y * h)), 3, (0, 255, 0), -1)
        for s, e in connections:
            cv2.line(rgb_image,
                     (int(lms[s].x * w), int(lms[s].y * h)),
                     (int(lms[e].x * w), int(lms[e].y * h)),
                     (0, 255, 255), 1)
    return rgb_image


def main():
    # Validate stage configuration
    assert len(STAGES) == 5,           "STAGES must have exactly 5 entries"
    assert len(STAGE_THRESHOLDS) == 5, "STAGE_THRESHOLDS must have exactly 5 entries"
    for i in range(5):
        st, th = STAGES[i], STAGE_THRESHOLDS[i]
        assert len(st) >= 2,                   f"STAGES[{i}] must have at least 2 stages"
        assert len(st) == len(th),             f"STAGES[{i}] and STAGE_THRESHOLDS[{i}] must have the same length"
        assert th[0] == 0,                     f"STAGE_THRESHOLDS[{i}][0] must be 0"
        assert st == sorted(st, reverse=True), f"STAGES[{i}] must be sorted descending"
        assert th == sorted(th),               f"STAGE_THRESHOLDS[{i}] must be sorted ascending"

    # Serial
    ser = None
    port = SERIAL_PORT or find_arduino_port()
    if port:
        try:
            ser = serial.Serial(port, SERIAL_BAUD, timeout=1)
            time.sleep(2)
        except serial.SerialException:
            ser = None

    # MediaPipe
    base_options = python.BaseOptions(model_asset_path='hand_landmarker.task')
    detector = vision.HandLandmarker.create_from_options(
        vision.HandLandmarkerOptions(
            base_options=base_options,
            num_hands=1,
            min_hand_detection_confidence=0.7,
            min_tracking_confidence=0.6,
        )
    )

    # Camera
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        sys.exit(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    FINGER_POINTS = [
        (4,  3,  2),   # Thumb
        (8,  6,  5),   # Index
        (12, 10, 9),   # Middle
        (16, 14, 13),  # Ring
        (20, 18, 17),  # Pinky
    ]

    # Start with all fingers open
    current_angles: list[int]   = list(ANGLE_OPEN)
    prev_bend:      list[float] = [0.0] * 5
    bend_ratios:    list[float] = [0.0] * 5

    last_send   = 0.0
    fps_counter = 0
    fps_timer   = time.time()
    fps         = 0.0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame            = cv2.flip(frame, 1)
        mp_image         = mp.Image(image_format=mp.ImageFormat.SRGB,
                                    data=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        detection_result = detector.detect(mp_image)

        if detection_result.hand_landmarks:
            lms = detection_result.hand_landmarks[0]

            origin, x_axis, y_axis, z_axis, palm_size = build_palm_basis(lms)

            bend_ratios = (
                [thumb_bend_ratio(lms, origin, x_axis, y_axis, z_axis, palm_size)] +
                [finger_bend_ratio(lms, t, p, m, origin, x_axis, y_axis, z_axis)
                 for t, p, m in FINGER_POINTS[1:]]
            )
            frame = draw_landmarks(frame, detection_result)

            now = time.time()
            if (now - last_send) >= SEND_INTERVAL:
                for i in range(5):
                    delta = bend_ratios[i] - prev_bend[i]

                    # Dead zone: ignore micro-fluctuations
                    if abs(delta) < BEND_THRESHOLD:
                        continue

                    # bend_ratio (0.0–1.0) → bend% (0–100) → stage → target angle
                    bend_pct     = bend_ratios[i] * 100.0
                    target_angle = detect_stage(bend_pct, STAGE_THRESHOLDS[i], STAGES[i])

                    if target_angle == current_angles[i]:
                        prev_bend[i] = bend_ratios[i]
                        continue

                    current_angles[i] = target_angle
                    packet = build_packet(i, target_angle)

                    if ser and ser.is_open:
                        try:
                            ser.write(packet)
                        except serial.SerialException:
                            pass

                    prev_bend[i] = bend_ratios[i]
                last_send = now

        fps_counter += 1
        if time.time() - fps_timer >= 1.0:
            fps         = fps_counter / (time.time() - fps_timer)
            fps_counter = 0
            fps_timer   = time.time()

        draw_ui(frame, bend_ratios, current_angles, STAGES, fps,
                ser is not None and ser.is_open)
        cv2.imshow("Robo Hand — Hand Tracking", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    # Return all fingers to open position before exit
    if ser and ser.is_open:
        for i in range(5):
            ser.write(build_packet(i, ANGLE_OPEN[i]))
        time.sleep(0.2)
        ser.close()

    detector.close()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
