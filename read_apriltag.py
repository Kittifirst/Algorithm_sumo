"""Read AprilTag IDs from a webcam (36h11 by default).

Install: python -m pip install -r requirements.txt
Run:     python read_apriltag.py
Optional: python read_apriltag.py --camera 0 (skip the camera picker)
Exit:    press Q or Esc in the camera window, or Ctrl+C in the terminal.
"""

import argparse
import sys


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--camera", type=int, help="Camera index; omit to open the camera picker."
    )
    parser.add_argument(
        "--family",
        choices=("36h11", "36h10"),
        default="36h11",
        help="AprilTag family; must match the printed tag (default: 36h11).",
    )
    args = parser.parse_args()
    if args.camera is not None and args.camera < 0:
        parser.error("--camera must be 0 or greater")
    return args


def camera_backend(cv2):
    if sys.platform == "win32":
        return cv2.CAP_DSHOW
    if sys.platform == "darwin":
        return cv2.CAP_AVFOUNDATION
    return cv2.CAP_V4L2


def discover_cameras(cv2, enumerate_cameras):
    """Keep indices tied to their backend, including virtual-camera sources."""
    backends = [camera_backend(cv2)]
    if sys.platform == "win32":
        backends.append(cv2.CAP_MSMF)
    cameras, errors = [], []
    seen = set()
    for backend in backends:
        backend_name = cv2.videoio_registry.getBackendName(backend)
        try:
            for camera in enumerate_cameras(backend):
                key = camera.index, camera.backend
                if key not in seen:
                    cameras.append(camera)
                    seen.add(key)
        except Exception as error:
            errors.append(f"{backend_name}: {error}")
    return cameras, errors


def select_camera(cv2, family):
    """Show named cameras without opening a video stream during enumeration."""
    from cv2_enumerate_cameras import enumerate_cameras
    from PySide6 import QtWidgets

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([sys.argv[0]])
    dialog = QtWidgets.QDialog()
    dialog.setWindowTitle("AprilTag - เลือกกล้อง")
    dialog.setMinimumWidth(480)
    layout = QtWidgets.QVBoxLayout(dialog)
    layout.addWidget(QtWidgets.QLabel("เลือกกล้องและตระกูล AprilTag แล้วกดเริ่มอ่าน ID"))

    form = QtWidgets.QFormLayout()
    cameras = QtWidgets.QComboBox()
    families = QtWidgets.QComboBox()
    families.addItems(["36h11", "36h10"])
    families.setCurrentText(family)
    form.addRow("กล้อง", cameras)
    form.addRow("ตระกูลแท็ก", families)
    layout.addLayout(form)
    status = QtWidgets.QLabel()
    status.setWordWrap(True)
    layout.addWidget(status)

    buttons = QtWidgets.QHBoxLayout()
    refresh = QtWidgets.QPushButton("ค้นหากล้องใหม่")
    cancel = QtWidgets.QPushButton("ยกเลิก")
    start = QtWidgets.QPushButton("เริ่มอ่าน ID")
    start.setDefault(True)
    # Only the start button should accept the dialog when Enter is pressed.
    refresh.setAutoDefault(False)
    cancel.setAutoDefault(False)
    buttons.addWidget(refresh)
    buttons.addStretch()
    buttons.addWidget(cancel)
    buttons.addWidget(start)
    layout.addLayout(buttons)

    def refresh_cameras():
        previous = cameras.currentData()
        cameras.clear()
        start.setEnabled(False)
        found, errors = discover_cameras(cv2, enumerate_cameras)
        for camera in found:
            backend_name = cv2.videoio_registry.getBackendName(camera.backend)
            cameras.addItem(
                f"{camera.name} ({backend_name}, {camera.index})",
                (camera.index, camera.backend),
            )
        previous_index = next(
            (index for index in range(cameras.count()) if cameras.itemData(index) == previous),
            -1,
        )
        if previous_index >= 0:
            cameras.setCurrentIndex(previous_index)
        start.setEnabled(cameras.count() > 0)
        message = (
            f"พบแหล่งภาพ {cameras.count()} รายการ กล้องเดียวกันอาจมีหลายรายการ"
            if cameras.count()
            else "ไม่พบกล้อง กรุณาเชื่อมต่อกล้องแล้วกดค้นหากล้องใหม่"
        )
        if sys.platform == "win32" and not any("camo" in camera.name.lower() for camera in found):
            message += "\nหากไม่พบ Camo ให้ตรวจกล้องเสมือนใน Camo Studio แล้วค้นหาใหม่"
        if errors:
            message += "\nค้นหาได้บางส่วน: " + "; ".join(errors)
        status.setText(message)

    refresh.clicked.connect(refresh_cameras)
    cancel.clicked.connect(dialog.reject)
    start.clicked.connect(dialog.accept)
    refresh_cameras()
    if dialog.exec() != QtWidgets.QDialog.DialogCode.Accepted:
        return None
    index, backend = cameras.currentData()
    selection = index, backend, families.currentText()
    dialog.close()
    app.processEvents()
    return selection


def create_detector(cv2, family):
    dictionary_id = getattr(cv2.aruco, "DICT_APRILTAG_" + family)
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    parameters = cv2.aruco.DetectorParameters()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(dictionary, parameters)


def main():
    args = parse_args()
    try:
        import cv2
    except ImportError:
        print(
            "OpenCV is missing. Run: python -m pip install -r requirements.txt",
            file=sys.stderr,
        )
        return 1

    if not hasattr(cv2, "aruco") or not hasattr(cv2.aruco, "ArucoDetector"):
        print(
            "OpenCV with ArucoDetector is required. Run: "
            "python -m pip install --upgrade -r requirements.txt",
            file=sys.stderr,
        )
        return 1

    camera_index, backend, family = args.camera, camera_backend(cv2), args.family
    if camera_index is None:
        try:
            selection = select_camera(cv2, family)
        except ImportError:
            print(
                "Camera picker dependencies are missing. Run: "
                "python -m pip install -r requirements.txt",
                file=sys.stderr,
            )
            return 1
        if selection is None:
            return 0
        camera_index, backend, family = selection

    detector = create_detector(cv2, family)
    camera = cv2.VideoCapture(camera_index, backend)
    window_name = "AprilTag " + family
    previous_ids = None

    try:
        if not camera.isOpened():
            print(
                f"Cannot open camera {camera_index}. Try another camera and close "
                "other apps using the camera.",
                file=sys.stderr,
            )
            return 1

        print(f"Reading AprilTag {family}. Press Q or Esc to exit.", flush=True)
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

        while True:
            ok, frame = camera.read()
            if not ok or frame is None:
                print("Cannot read a frame from the camera.", file=sys.stderr)
                return 1

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)
            tag_ids = sorted(set(ids.flatten().tolist())) if ids is not None else []

            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # Print again when a tag appears, disappears, or the ID set changes.
            if tag_ids != previous_ids:
                if tag_ids:
                    print("ID: " + ", ".join(map(str, tag_ids)), flush=True)
                else:
                    print("No AprilTag detected.", flush=True)
                previous_ids = tag_ids

            label = "ID: " + ", ".join(map(str, tag_ids)) if tag_ids else "No AprilTag"
            cv2.putText(
                frame, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2
            )
            cv2.imshow(window_name, frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break
            if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
                break

    except KeyboardInterrupt:
        pass
    except cv2.error as error:
        print(f"OpenCV error: {error}", file=sys.stderr)
        return 1
    finally:
        camera.release()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
