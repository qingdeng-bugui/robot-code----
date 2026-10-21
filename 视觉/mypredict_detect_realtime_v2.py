from collections import deque
import platform
import threading
import time

import cv2
from ultralytics import YOLO


# -------------------- Basic configuration --------------------
MODEL_PATH = r"E:\qiqiu3\best_mixed_ncnn_model"
CAMERA_DEVICE = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
CONFIDENCE = 0.50
IMAGE_SIZE = 320
# None means all model classes are accepted (0 through 7 for qiqiu2.pt).
TARGET_CLASS_ID = None
RED_CLASS_ID = 3

# The camera optical axis and needle axis are intended to be collinear.
# Use the image center as dx=0 so the lower controller and vision share
# the same horizontal reference.
ATTACK_X_RATIO = 0.50

# Use target-box height ratio for rough distance estimation.
# These values must be calibrated with the real balloon and camera.
MIN_HEIGHT_RATIO = 0.50
MAX_HEIGHT_RATIO = 1.00

CENTER_TOLERANCE_X_RATIO = 90 / FRAME_WIDTH

SMOOTH_WINDOW = 3
EMA_ALPHA = 0.50
READY_FRAMES_REQUIRED = 2
LOST_FRAMES_TO_RESET = 8

# Set to True only after the visual algorithm is stable.
SERIAL_ENABLED = False
SERIAL_DEVICE = "/dev/ttyUSB0"
SERIAL_BAUDRATE = 115200
WAIT_FOR_START = False

# Print each message after it is written to the lower controller.
SERIAL_LOG_ENABLED = False

SHOW_WINDOW = True


# The capture thread continuously replaces this with the newest frame.
# Inference never queues old frames, which keeps control latency bounded.
latest_frame = None
latest_frame_lock = threading.Lock()
capture_stop = threading.Event()
serial_stop = threading.Event()
detect_enabled = threading.Event()


class RobustValueFilter:
    """Median filter followed by exponential smoothing."""

    def __init__(self, window_size=5, alpha=0.25):
        self.values = deque(maxlen=window_size)
        self.alpha = alpha
        self.value = None

    def update(self, raw_value):
        self.values.append(float(raw_value))
        ordered = sorted(self.values)
        median_value = ordered[len(ordered) // 2]

        if self.value is None:
            self.value = median_value
        else:
            self.value = (
                self.alpha * median_value
                + (1.0 - self.alpha) * self.value
            )

        return self.value

    def reset(self):
        self.values.clear()
        self.value = None


class BalloonTarget:
    def __init__(self):
        self.center_x_filter = RobustValueFilter(
            SMOOTH_WINDOW, EMA_ALPHA
        )
        self.height_filter = RobustValueFilter(
            SMOOTH_WINDOW, EMA_ALPHA
        )
        self.area_filter = RobustValueFilter(
            SMOOTH_WINDOW, EMA_ALPHA
        )
        self.track_id = None
        self.lost_frames = 0

    def reset(self):
        self.center_x_filter.reset()
        self.height_filter.reset()
        self.area_filter.reset()
        self.track_id = None
        self.lost_frames = 0

    def update(self, target):
        self.lost_frames = 0
        self.track_id = target["track_id"]

        center_x = self.center_x_filter.update(target["center_x"])
        height = self.height_filter.update(target["height"])
        area = self.area_filter.update(target["area"])

        return {
            "class_id": target["class_id"],
            "track_id": self.track_id,
            "center_x": center_x,
            "center_y": target["center_y"],
            "height": height,
            "area": area,
            "confidence": target["confidence"],
            "box": target["box"],
        }

    def mark_lost(self):
        self.lost_frames += 1
        if self.lost_frames >= LOST_FRAMES_TO_RESET:
            self.reset()


def open_serial():
    if not SERIAL_ENABLED:
        return None

    try:
        import serial

        return serial.Serial(
            SERIAL_DEVICE,
            SERIAL_BAUDRATE,
            timeout=0.05,
        )
    except Exception as error:
        print("Serial disabled:", error)
        return None


def serial_command_loop(serial_port):
    """Wait for START/STOP commands from the lower controller."""
    while not serial_stop.is_set():
        try:
            raw_command = serial_port.readline()
        except Exception as error:
            print("Serial receive error:", error)
            detect_enabled.clear()
            return

        if not raw_command:
            continue

        command = raw_command.decode("ascii", errors="ignore").strip().upper()
        if command == "START":
            detect_enabled.set()
            print("Received START: detection enabled", flush=True)
        elif command in ("STOP", "PAUSE"):
            detect_enabled.clear()
            print("Received STOP: detection paused", flush=True)


def open_camera(camera_index=0):
    """Open a camera with the backend appropriate for the current OS."""
    if platform.system() == "Windows":
        backends = [
            cv2.CAP_DSHOW,
            cv2.CAP_MSMF,
            cv2.CAP_ANY,
        ]
    else:
        backends = [
            cv2.CAP_V4L2,
            cv2.CAP_ANY,
        ]

    for backend in backends:
        camera = cv2.VideoCapture(camera_index, backend)
        if camera.isOpened():
            print(
                "Camera opened:",
                f"index={camera_index}",
                f"backend={backend}",
            )
            return camera
        camera.release()

    return None


def capture_loop(camera):
    """Read frames continuously and retain only the newest one."""
    global latest_frame

    while not capture_stop.is_set():
        ok, frame = camera.read()
        if not ok:
            continue

        with latest_frame_lock:
            latest_frame = frame


def send_status(serial_port, target, error_x, can_hit):
    if serial_port is None:
        return

    if target is None:
        message = "none\n"
    else:
        message = f"{int(round(error_x))},{int(can_hit)}\n"

    serial_port.write(message.encode("ascii"))
    if SERIAL_LOG_ENABLED:
        print(f"[TX] {message.rstrip()}", flush=True)


def send_target_locked(serial_port):
    """Notify the lower controller once when a target returns after none."""
    if serial_port is not None:
        message = "ok\n"
        serial_port.write(message.encode("ascii"))
        if SERIAL_LOG_ENABLED:
            print(f"[TX] {message.rstrip()}", flush=True)


def extract_targets(result):
    targets = []
    boxes = result.boxes

    if boxes is None or len(boxes) == 0:
        return targets

    coordinates = boxes.xyxy.cpu().numpy()
    confidences = boxes.conf.cpu().numpy()
    classes = boxes.cls.int().cpu().tolist()

    if boxes.id is not None:
        track_ids = boxes.id.int().cpu().tolist()
    else:
        track_ids = [-1] * len(coordinates)

    for box, confidence, class_id, track_id in zip(
        coordinates,
        confidences,
        classes,
        track_ids,
    ):
        if TARGET_CLASS_ID is not None and class_id != TARGET_CLASS_ID:
            continue

        x1, y1, x2, y2 = box
        width = max(0.0, x2 - x1)
        height = max(0.0, y2 - y1)

        targets.append(
            {
                "class_id": int(class_id),
                "box": (int(x1), int(y1), int(x2), int(y2)),
                "center_x": (x1 + x2) / 2.0,
                "center_y": (y1 + y2) / 2.0,
                "width": width,
                "height": height,
                "area": width * height,
                "confidence": float(confidence),
                "track_id": int(track_id),
            }
        )

    return targets


def choose_target(targets, previous_track_id):
    attack_targets = [
        target
        for target in targets
        if target["class_id"] != RED_CLASS_ID
    ]

    if not attack_targets:
        return None

    if previous_track_id is not None:
        same_id = [
            target
            for target in attack_targets
            if target["track_id"] == previous_track_id
        ]
        if same_id:
            return same_id[0]

    # Before a target is locked, prefer the largest balloon.
    return max(attack_targets, key=lambda target: target["area"])


def main():
    global latest_frame

    model = YOLO(MODEL_PATH, task="detect")

    camera = open_camera(CAMERA_DEVICE)
    if camera is None:
        raise RuntimeError(
            "Cannot open camera. "
            "Check the camera connection and CAMERA_DEVICE."
        )

    camera.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    camera.set(cv2.CAP_PROP_FPS, 30)
    camera.set(
        cv2.CAP_PROP_FOURCC,
        cv2.VideoWriter_fourcc(*"MJPG"),
    )
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not camera.isOpened():
        raise RuntimeError("Cannot open camera")

    serial_port = open_serial()
    if WAIT_FOR_START and serial_port is None:
        camera.release()
        raise RuntimeError(
            "WAIT_FOR_START requires a working serial port. "
            "Set SERIAL_DEVICE to the STM32 port."
        )

    serial_stop.clear()
    detect_enabled.clear()
    serial_thread = None
    if serial_port is not None:
        serial_thread = threading.Thread(
            target=serial_command_loop,
            args=(serial_port,),
            name="serial-command",
            daemon=True,
        )
        serial_thread.start()

    if not WAIT_FOR_START:
        detect_enabled.set()

    capture_stop.clear()
    with latest_frame_lock:
        latest_frame = None
    capture_thread = threading.Thread(
        target=capture_loop,
        args=(camera,),
        name="camera-capture",
        daemon=True,
    )
    capture_thread.start()

    target_filter = BalloonTarget()
    ready_count = 0
    can_hit = False
    waiting_for_target = False

    frame_count = 0
    fps_start = time.time()
    fps = 0.0

    try:
        print("Warming up model...", flush=True)
        warmup_frame = None
        while warmup_frame is None:
            with latest_frame_lock:
                if latest_frame is not None:
                    warmup_frame = latest_frame.copy()
            if warmup_frame is None:
                time.sleep(0.001)

        model.predict(
            warmup_frame,
            imgsz=IMAGE_SIZE,
            conf=CONFIDENCE,
            verbose=False,
        )
        print("System ready. Waiting for START.", flush=True)

        was_running = False

        while True:
            with latest_frame_lock:
                frame = (
                    None
                    if latest_frame is None
                    else latest_frame.copy()
                )

            if frame is None:
                time.sleep(0.001)
                continue

            if WAIT_FOR_START and not detect_enabled.is_set():
                if was_running:
                    send_status(serial_port, None, 0, False)
                    target_filter.reset()
                    ready_count = 0
                    can_hit = False
                    waiting_for_target = False
                    was_running = False

                if SHOW_WINDOW:
                    cv2.putText(
                        frame,
                        "WAITING FOR START",
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 255, 255),
                        2,
                    )
                    cv2.imshow("Balloon Tracking", frame)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break

                time.sleep(0.005)
                continue

            was_running = True

            frame_height, frame_width = frame.shape[:2]
            attack_x = frame_width * ATTACK_X_RATIO

            result = model.track(
                frame,
                persist=True,
                tracker="bytetrack.yaml",
                imgsz=IMAGE_SIZE,
                conf=CONFIDENCE,
                verbose=False,
            )[0]

            targets = extract_targets(result)
            previous_track_id = target_filter.track_id
            selected = choose_target(
                targets,
                previous_track_id,
            )

            if selected is None:
                target_filter.mark_lost()
                ready_count = 0
                can_hit = False
                waiting_for_target = True
                send_status(
                    serial_port,
                    None,
                    0,
                    False,
                )
            else:
                target_reacquired = waiting_for_target
                if target_reacquired:
                    send_target_locked(serial_port)
                    # A reacquired target may have moved substantially while
                    # the car was backing away; discard the old EMA position.
                    target_filter.reset()
                    ready_count = 0
                    can_hit = False
                    waiting_for_target = False

                target_changed = (
                    previous_track_id is not None
                    and selected["track_id"] != previous_track_id
                )
                if target_changed:
                    # The previous balloon disappeared; start the new
                    # target with clean filters and a fresh hit count.
                    target_filter.reset()
                    ready_count = 0
                    can_hit = False

                target = target_filter.update(selected)

                error_x = target["center_x"] - attack_x
                height_ratio = target["height"] / frame_height
                raw_height_ratio = selected["height"] / frame_height

                position_ok = abs(error_x) <= (
                    frame_width * CENTER_TOLERANCE_X_RATIO
                )

                distance_ok = (
                    MIN_HEIGHT_RATIO
                    <= raw_height_ratio
                    <= MAX_HEIGHT_RATIO
                )

                # Hysteresis prevents can_hit from rapidly toggling
                # when the measurement is close to a threshold.
                if not can_hit:
                    if position_ok and distance_ok:
                        ready_count += 1
                    else:
                        ready_count = 0

                    if ready_count >= READY_FRAMES_REQUIRED:
                        can_hit = True
                else:
                    exit_distance_ok = (
                        MIN_HEIGHT_RATIO - 0.02
                        <= raw_height_ratio
                        <= MAX_HEIGHT_RATIO + 0.02
                    )
                    if not position_ok or not exit_distance_ok:
                        can_hit = False
                        ready_count = 0

                send_status(
                    serial_port,
                    target,
                    error_x,
                    can_hit,
                )

            if SHOW_WINDOW:
                for detected_target in targets:
                    x1, y1, x2, y2 = detected_target["box"]

                    if detected_target is selected:
                        color = (0, 255, 0)
                        thickness = 3
                    elif detected_target["class_id"] == RED_CLASS_ID:
                        color = (0, 0, 255)
                        thickness = 2
                    else:
                        color = (0, 255, 255)
                        thickness = 2

                    cv2.rectangle(
                        frame,
                        (x1, y1),
                        (x2, y2),
                        color,
                        thickness,
                    )

                    label = (
                        f"class={detected_target['class_id']} "
                        f"id={detected_target['track_id']} "
                        f"conf={detected_target['confidence']:.2f}"
                    )
                    cv2.putText(
                        frame,
                        label,
                        (x1, max(20, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        color,
                        2,
                    )

                if selected is not None:
                    cv2.circle(
                        frame,
                        (
                            int(target["center_x"]),
                            int(target["center_y"]),
                        ),
                        5,
                        (0, 0, 255),
                        -1,
                    )

                    text = (
                        f"target_class={target['class_id']} "
                        f"id={target['track_id']} "
                        f"h={target['height']:.0f} "
                        f"hr={height_ratio:.2f} "
                        f"dx={error_x:.0f} "
                        f"hit={can_hit}"
                    )
                    cv2.putText(
                        frame,
                        text,
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (0, 255, 0),
                        2,
                    )

            frame_count += 1
            elapsed = time.time() - fps_start
            if elapsed >= 1.0:
                fps = frame_count / elapsed
                frame_count = 0
                fps_start = time.time()
                if not SHOW_WINDOW:
                    print(f"FPS={fps:.1f}", flush=True)

            if SHOW_WINDOW:
                cv2.line(
                    frame,
                    (int(attack_x), 0),
                    (int(attack_x), frame_height - 1),
                    (255, 0, 0),
                    2,
                )

                cv2.putText(
                    frame,
                    f"FPS={fps:.1f}",
                    (10, frame_height - 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 255),
                    2,
                )

                cv2.imshow("Balloon Tracking", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

    finally:
        capture_stop.set()
        serial_stop.set()
        detect_enabled.clear()
        capture_thread.join(timeout=1.0)
        if serial_thread is not None:
            serial_thread.join(timeout=1.0)
        camera.release()
        if serial_port is not None:
            serial_port.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
