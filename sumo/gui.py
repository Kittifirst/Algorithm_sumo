"""PySide6 desktop interface for calibrated top-view sumo control."""

from collections import deque
from dataclasses import asdict
import json
import math
from pathlib import Path
import os
import time

import cv2
import numpy as np
from PySide6 import QtCore, QtGui, QtWidgets

from read_apriltag import discover_cameras
from .link import RobotLink
from .planner import Config, Plan, Planner, Vec, direction, rectangle_corners
from .simulation import Simulation
from .vision import Calibration, CameraWorker, PoseTracker
from .wifi import WifiJoiner, ROBOT_SSID

ROOT = Path(__file__).resolve().parent.parent
SETTINGS = ROOT / "sumo_settings.json"
CYAN, RED, YELLOW = "#48ddc9", "#ff727e", "#ffcf66"
TRAIL_GREEN = "#67f0a0"
CORNER_NAMES = ("TL", "TR", "BR", "BL")
CORNER_THAI = ("ซ้ายบน", "ขวาบน", "ขวาล่าง", "ซ้ายล่าง")


def ensure_fonts(app):
    # Qt's offscreen platform may not enumerate Windows system fonts. Loading a
    # local system font also gives the preview the same Thai glyphs as the app.
    fonts = Path(os.environ.get("WINDIR", "C:/Windows")) / "Fonts"
    for name in ("tahoma.ttf", "tahomabd.ttf", "segoeui.ttf", "seguisb.ttf", "consola.ttf"):
        path = fonts / name
        if path.exists():
            QtGui.QFontDatabase.addApplicationFont(str(path))
    app.setFont(QtGui.QFont("Tahoma", 10))


def line(painter, points, color, width=2, dashed=False):
    if len(points) < 2:
        return
    pen = QtGui.QPen(QtGui.QColor(color), width)
    if dashed:
        pen.setStyle(QtCore.Qt.PenStyle.DashLine)
    painter.setPen(pen)
    painter.setBrush(QtCore.Qt.BrushStyle.NoBrush)
    painter.drawPolyline(QtGui.QPolygonF(points))


def draw_trail(painter, trail, project):
    """Draw observed positions, breaking the line across missing/jumping tags."""
    segment = []
    for point in [*trail, None]:
        if point is None:
            line(painter, segment, "#081a12", 5)
            line(painter, segment, TRAIL_GREEN, 3)
            segment = []
        else:
            segment.append(project(point))


class CameraCanvas(QtWidgets.QWidget):
    clicked = QtCore.Signal(float, float)

    def __init__(self):
        super().__init__()
        self.setMinimumSize(300, 300)
        self.observation = None
        self.calibration = None
        self.points = []
        self.calibrating = False
        self.plan = None
        self.poses = (None, None)
        self.own_id, self.enemy_id = None, None
        self.trail = ()
        self.show_trail = True
        self.offset_rect = QtCore.QRectF()

    def mousePressEvent(self, event):
        observation = self.observation
        if not self.calibrating or observation is None or observation.frame is None:
            return
        pos = event.position()
        if self.offset_rect.contains(pos):
            h, w = observation.frame.shape[:2]
            self.clicked.emit((pos.x() - self.offset_rect.x()) / self.offset_rect.width() * w,
                              (pos.y() - self.offset_rect.y()) / self.offset_rect.height() * h)

    def paintEvent(self, event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QtGui.QColor("#0b121c"))
        observation = self.observation
        if observation is None or observation.frame is None:
            p.setPen(QtGui.QColor("#9eafc4"))
            message = "CAMERA / APRILTAG\nเปิดกล้องเพื่อดูภาพสนาม\nหรือเลือกโหมดจำลองเพื่อทดลองแผน"
            if observation and observation.error:
                message = observation.error
            p.drawText(self.rect().adjusted(20, 20, -20, -20), QtCore.Qt.AlignmentFlag.AlignCenter, message)
            return
        frame = observation.frame
        h, w = frame.shape[:2]
        scale = min(self.width() / w, self.height() / h)
        self.offset_rect = QtCore.QRectF((self.width() - w * scale) / 2, (self.height() - h * scale) / 2, w * scale, h * scale)
        image = QtGui.QImage(frame.data, w, h, frame.strides[0], QtGui.QImage.Format.Format_BGR888)
        p.drawImage(self.offset_rect, image)

        def pixel(point):
            return QtCore.QPointF(self.offset_rect.x() + float(point[0]) * scale,
                                 self.offset_rect.y() + float(point[1]) * scale)

        for tag, corners in observation.detections.items():
            color = CYAN if tag == self.own_id else RED if tag == self.enemy_id else "#ffffff"
            line(p, [pixel(v) for v in [*corners, corners[0]]], color, 2)
            p.setPen(QtGui.QColor(color))
            p.drawText(pixel(corners[0]) + QtCore.QPointF(0, -8), f"ID {tag}")
        for i, point in enumerate(self.points):
            q = pixel(point)
            p.setPen(QtGui.QPen(QtGui.QColor(YELLOW), 2))
            p.drawEllipse(q, 5, 5)
            p.drawText(q + QtCore.QPointF(8, -8), CORNER_NAMES[i])
        if self.calibration and not self.calibrating:
            inverse = np.linalg.inv(self.calibration.matrix)

            def world(point):
                a = cv2.perspectiveTransform(np.float32([[[point.x, point.y]]]), inverse)[0, 0]
                return pixel(a)

            if self.show_trail:
                draw_trail(p, self.trail, world)
            if self.plan:
                line(p, [world(v) for v in self.plan.approach], YELLOW, 3)
                line(p, [world(v) for v in self.plan.path], "#7ab5ff", 3)
            for pose, color in zip(self.poses, (CYAN, RED)):
                if pose:
                    a, b = world(pose.position), world(pose.position + direction(pose.heading) * 100)
                    line(p, [a, b], color, 4)
                    p.setPen(QtGui.QColor(color))
                    p.drawText(b, "FRONT")


class ArenaCanvas(QtWidgets.QWidget):
    def __init__(self, cfg):
        super().__init__()
        self.setMinimumSize(320, 340)
        self.cfg, self.poses, self.plan = cfg, (None, None), None
        self.ids = (None, None)
        self.trail = deque(maxlen=600)
        self.show_trail = True

    def paintEvent(self, event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QtGui.QColor("#0b121c"))
        center = QtCore.QPointF(self.width() / 2, self.height() / 2 + 8)
        scale = min((self.width() - 65) / self.cfg.arena_width,
                    (self.height() - 85) / self.cfg.arena_height)

        def screen(v):
            return center + QtCore.QPointF(v.x * scale, -v.y * scale)

        def box(x, y):
            return QtCore.QRectF(screen(Vec(-x, y)), screen(Vec(x, -y)))

        p.setPen(QtGui.QPen(QtGui.QColor("#e2e8f0"), 1))
        p.setBrush(QtGui.QColor("#e2e8f0"))
        p.drawRect(box(self.cfg.arena_width / 2, self.cfg.arena_height / 2))
        p.setPen(QtCore.Qt.PenStyle.NoPen)
        p.setBrush(QtGui.QColor("#121e2b"))
        p.drawRect(box(self.cfg.black_half_width, self.cfg.black_half_height))
        heading = self.poses[0].heading if self.poses[0] else 0
        low, high = self.cfg.center_bounds(heading)
        p.setPen(QtGui.QPen(QtGui.QColor("#496079"), 1, QtCore.Qt.PenStyle.DashLine))
        p.setBrush(QtCore.Qt.BrushStyle.NoBrush)
        p.drawRect(QtCore.QRectF(screen(Vec(low.x, high.y)), screen(Vec(high.x, low.y))))
        if self.poses[0]:
            pose = self.poses[0]
            front = self.cfg.front_clearance(pose.position, heading)
            outer = self.cfg.outer_clearance(pose.position, heading)
            p.setPen(QtGui.QColor(CYAN if self.cfg.inside(pose.position, heading) else YELLOW))
            p.drawText(15, 44, f"FRONT → WHITE {front:.0f} mm / BODY → OUTER {outer:.0f} mm")
        p.setPen(QtGui.QPen(QtGui.QColor("#203245"), 1))
        p.drawLine(screen(Vec(-self.cfg.black_half_width, 0)), screen(Vec(self.cfg.black_half_width, 0)))
        p.drawLine(screen(Vec(0, -self.cfg.black_half_height)), screen(Vec(0, self.cfg.black_half_height)))
        p.setPen(QtGui.QColor("#8398b0"))
        p.drawText(15, 25, f"ARENA {self.cfg.arena_width:.0f} × {self.cfg.arena_height:.0f} mm  /  WHITE {self.cfg.white_border:.0f} mm")
        p.drawText(15, self.height() - 14, "เหลือง = โค้งถึงศัตรู  •  ฟ้า = คำสั่งรอบนี้  •  เขียว = ทางจริง")
        if self.show_trail:
            draw_trail(p, self.trail, screen)
        if self.plan:
            line(p, [screen(v) for v in self.plan.approach], YELLOW, 3)
            line(p, [screen(v) for v in self.plan.path], "#7ab5ff", 3)
            target = self.plan.approach[-1] if self.plan.approach else self.plan.target
            if target:
                p.setPen(QtGui.QPen(QtGui.QColor(YELLOW), 2))
                p.drawEllipse(screen(target), 6, 6)
        sizes = ((self.cfg.robot_width, self.cfg.robot_length), (self.cfg.enemy_width, self.cfg.enemy_length))
        for tag, pose, color, (width, length) in zip(self.ids, self.poses, (CYAN, RED), sizes):
            if pose is None:
                continue
            q = screen(pose.position)
            p.setPen(QtGui.QPen(QtGui.QColor(color), 2))
            fill = QtGui.QColor(color)
            fill.setAlpha(40)
            p.setBrush(fill)
            p.drawPolygon(QtGui.QPolygonF([screen(v) for v in rectangle_corners(pose.position, pose.heading, width, length)]))
            tip = screen(pose.position + direction(pose.heading) * (length / 2 + 28))
            line(p, [q, tip], color, 4)
            wing1 = screen(pose.position + direction(pose.heading) * (length / 2 + 10) + direction(pose.heading + math.pi / 2) * 10)
            wing2 = screen(pose.position + direction(pose.heading) * (length / 2 + 10) - direction(pose.heading + math.pi / 2) * 10)
            line(p, [wing1, tip, wing2], color, 3)
            p.setPen(QtGui.QColor(color))
            p.drawText(q + QtCore.QPointF(-16, 5), f"ID {tag}")


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, demo=False):
        super().__init__()
        numeric_locale = QtCore.QLocale("en_US")
        numeric_locale.setNumberOptions(QtCore.QLocale.NumberOption.OmitGroupSeparator)
        QtCore.QLocale.setDefault(numeric_locale)
        self.setLocale(numeric_locale)
        ensure_fonts(QtWidgets.QApplication.instance())
        self.setWindowTitle("Sumo Top View — Wi-Fi Controller")
        self.resize(1420, 860)
        self.cfg = Config()
        self.link = RobotLink()
        self.wifi_joiner = None
        self.worker, self.calibration = None, None
        self.active_camera, self.active_family = None, None
        self.tracker = PoseTracker()
        self.planner = Planner(self.cfg)
        self.sim = Simulation(self.cfg)
        self.running = self.arming = False
        self.poses = (None, None)
        self.plan = Plan("IDLE", "เลือกกล้องหรือเปิดโหมดจำลอง")
        self.last_observation = None
        self.previous_tick = time.monotonic()
        self.previous_state = ""
        self.duplicate_selected = False
        self.match_result = None
        self.last_wifi_retry = 0.0
        self.close_requested = self.close_wait_stop = False
        self.elapsed_started_at = None
        self.elapsed_frozen = 0.0
        self.last_trail_pose = None
        self.calibration_points = []
        self.own_tag_id = self.enemy_tag_id = None
        self.visible_ids = set()
        self.saved = self._read_settings()
        self._build_ui(demo)
        self._restore_settings()
        self.timer = QtCore.QTimer(self)
        self.timer.setTimerType(QtCore.Qt.TimerType.PreciseTimer)
        self.timer.timeout.connect(self.tick)
        self.timer.start(50)
        if demo:
            self.own_tag_id, self.enemy_tag_id = 1, 2
            self._sync_tag_choices({1, 2})
        else:
            # Opening the launcher prepares the link and camera list, never ARM.
            QtCore.QTimer.singleShot(100, self.refresh_devices)
            QtCore.QTimer.singleShot(250, self.prepare_wifi)

    def _read_settings(self):
        try:
            data = json.loads(SETTINGS.read_text(encoding="utf-8"))
            return data if isinstance(data, dict) else {}
        except (OSError, ValueError):
            return {}

    def _build_ui(self, demo):
        root = QtWidgets.QWidget()
        self.setCentralWidget(root)
        outer = QtWidgets.QVBoxLayout(root)
        header = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel("SUMO / TOP VIEW")
        title.setStyleSheet("font-size:24px; font-weight:700; color:#48ddc9")
        header.addWidget(title)
        header.addStretch()
        self.mode_label = QtWidgets.QLabel("เลือกหุ่นเราจาก AprilTag ที่กล้องเห็น")
        header.addWidget(self.mode_label)
        outer.addLayout(header)
        body = QtWidgets.QHBoxLayout()
        outer.addLayout(body, 1)
        sidebar = QtWidgets.QWidget()
        sidebar.setFixedWidth(330)
        side = QtWidgets.QVBoxLayout(sidebar)
        side.setContentsMargins(0, 0, 8, 0)
        self.demo = QtWidgets.QCheckBox("โหมดจำลอง (ไม่ส่งคำสั่งหุ่นจริง)")
        self.demo.setChecked(demo)
        self.demo.toggled.connect(self.change_mode)
        side.addWidget(self.demo)
        connection_card = QtWidgets.QFrame()
        connection_card.setStyleSheet("QFrame { background:#162436; border-radius:6px; }")
        connection_card_layout = QtWidgets.QVBoxLayout(connection_card)
        self.connection_label = QtWidgets.QLabel("Wi-Fi • ยังไม่เชื่อมต่อ")
        self.connection_label.setStyleSheet("font-size:15px; font-weight:bold; color:#9aaec6; background:transparent;")
        self.connection_detail = QtWidgets.QLabel("ค้นหา IP ของ ESP32 ในฮอตสปอตเดียวกับคอม")
        self.connection_detail.setWordWrap(True)
        self.connection_detail.setStyleSheet("font-size:11px; color:#9aaec6; background:transparent;")
        connection_card_layout.addWidget(self.connection_label)
        connection_card_layout.addWidget(self.connection_detail)
        side.addWidget(connection_card)
        buttons = QtWidgets.QHBoxLayout()
        self.start_button = QtWidgets.QPushButton("START")
        self.stop_button = QtWidgets.QPushButton("■  STOP  [Esc]")
        self.start_button.setObjectName("start")
        self.stop_button.setObjectName("stop")
        for button in (self.start_button, self.stop_button):
            button.setMinimumHeight(52)
            buttons.addWidget(button)
        self.start_button.clicked.connect(self.start_control)
        self.stop_button.clicked.connect(lambda: self.stop_control("หยุดโดยผู้ใช้"))
        side.addLayout(buttons)
        QtGui.QShortcut(QtGui.QKeySequence("Escape"), self, activated=lambda: self.stop_control("หยุดด้วย Esc"))
        self.status = QtWidgets.QLabel("พร้อมตั้งค่า")
        self.status.setWordWrap(True)
        self.status.setMinimumHeight(65)
        side.addWidget(self.status)
        self.tabs = QtWidgets.QTabWidget()
        side.addWidget(self.tabs, 1)

        connection = QtWidgets.QWidget()
        form = QtWidgets.QFormLayout(connection)
        self.camera_box = QtWidgets.QComboBox()
        self.camera_box.setMinimumWidth(190)
        self.family = QtWidgets.QComboBox()
        self.family.addItems(["36h11", "36h10"])
        form.addRow("กล้อง", self.camera_box)
        form.addRow("AprilTag", self.family)
        refresh = QtWidgets.QPushButton("ค้นหากล้องใหม่")
        refresh.clicked.connect(self.refresh_devices)
        form.addRow(refresh)
        self.open_camera_button = QtWidgets.QPushButton("เปิดภาพกล้อง")
        self.open_camera_button.clicked.connect(self.open_camera)
        form.addRow(self.open_camera_button)
        self.calibrate_button = QtWidgets.QPushButton("กำหนดมุมนอก: TL → TR → BR → BL")
        self.calibrate_button.clicked.connect(self.begin_calibration)
        form.addRow(self.calibrate_button)
        self.calibration_label = QtWidgets.QLabel("ยังไม่ได้กำหนดสนาม")
        self.calibration_label.setWordWrap(True)
        form.addRow(self.calibration_label)
        self.own_id_box = QtWidgets.QComboBox()
        self.enemy_id_box = QtWidgets.QComboBox()
        self.own_id_box.addItem("เลือก ID ของหุ่นเรา", None)
        self.enemy_id_box.addItem("เลือก ID ของศัตรู", None)
        self.own_id_box.currentIndexChanged.connect(self.tag_selection_changed)
        self.enemy_id_box.currentIndexChanged.connect(self.tag_selection_changed)
        form.addRow("หุ่นเรา", self.own_id_box)
        form.addRow("ศัตรู", self.enemy_id_box)
        self.host = QtWidgets.QLineEdit("auto")
        self.host.setPlaceholderText("auto = ค้นหา IP หรือใส่ IP จาก ESP32")
        self.port = QtWidgets.QSpinBox()
        self.port.setRange(1, 65535)
        self.port.setValue(4210)
        form.addRow("ส่งผ่าน", QtWidgets.QLabel("Wi-Fi UDP"))
        form.addRow("IP หุ่น / auto", self.host)
        form.addRow("UDP port", self.port)
        self.connect_button = QtWidgets.QPushButton("เชื่อมต่อ Wi-Fi หุ่น")
        self.connect_button.clicked.connect(self.connect_robot)
        form.addRow(self.connect_button)
        wifi_settings = QtWidgets.QPushButton("เปิดการตั้งค่า Wi-Fi ของคอม")
        wifi_settings.clicked.connect(lambda: QtGui.QDesktopServices.openUrl(QtCore.QUrl("ms-settings:network-wifi")))
        form.addRow(wifi_settings)
        note = QtWidgets.QLabel(f"คอมและ ESP32 ต่อฮอตสปอต {ROBOT_SSID}\nIP = auto เพื่อค้นหาหุ่นในเครือข่าย\nเลือก ID หุ่นเรา แล้วตรวจลูกศร FRONT\nสถานะสีเขียว = ESP32 ตอบกลับแล้ว")
        note.setWordWrap(True)
        note.setStyleSheet("color:#9aaec6; padding-top:8px")
        form.addRow(note)
        connection_scroll = QtWidgets.QScrollArea()
        connection_scroll.setWidgetResizable(True)
        connection_scroll.setWidget(connection)
        self.tabs.addTab(connection_scroll, "กล้อง / หุ่น")

        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        settings_widget = QtWidgets.QWidget()
        settings_form = QtWidgets.QFormLayout(settings_widget)
        self.fields = {}
        specs = [
            ("arena_width", "สนามกว้างรวมขอบ mm", 400, 6000, 840),
            ("arena_height", "สนามยาวรวมขอบ mm", 400, 6000, 840),
            ("white_border", "ขอบขาวแต่ละด้าน mm", 1, 200, 50),
            ("robot_width", "หุ่นเรากว้าง mm", 20, 1000, 180),
            ("robot_length", "หุ่นเรายาว mm", 20, 1000, 210),
            ("enemy_width", "ศัตรูกว้าง mm", 20, 1000, 180),
            ("enemy_length", "ศัตรูยาว mm", 20, 1000, 210),
            ("edge_margin", "เผื่อขอบ mm", 10, 200, 25),
            ("wheel_base", "ระยะระหว่างล้อ mm", 30, 500, 120),
            ("max_wheel_speed", "ความเร็วที่ PWM255 mm/s", 50, 3000, 550),
            ("max_pwm", "PWM สูงสุด", 60, 255, 255),
            ("min_pwm", "PWM เริ่มหมุน", 1, 200, 60),
            ("left_gain", "ชดเชยล้อซ้าย", 0.3, 2, 1),
            ("right_gain", "ชดเชยล้อขวา", 0.3, 2, 1),
            ("stall_seconds", "ชนค้างกี่วินาทีจึงถอย", 0.5, 5, 1.3),
            ("stall_distance", "ขยับต่ำกว่า mm = ค้าง", 5, 100, 22),
            ("frame_timeout", "อายุแท็กก่อนค้นหา วินาที", 0.10, 0.75, 0.35),
        ]
        for name, label, low, high, value in specs:
            box = QtWidgets.QDoubleSpinBox()
            box.setRange(low, high)
            fractional = high <= 5
            box.setDecimals(2 if fractional else 0)
            box.setSingleStep(0.05 if fractional else 5)
            box.setValue(value)
            settings_form.addRow(label, box)
            self.fields[name] = box
        self.own_offset, self.enemy_offset = QtWidgets.QSpinBox(), QtWidgets.QSpinBox()
        for box, label in ((self.own_offset, "มุมหัวหุ่นเรา +CCW องศา"), (self.enemy_offset, "มุมหัวศัตรู +CCW องศา")):
            box.setRange(-180, 180)
            box.setSingleStep(90)
            settings_form.addRow(label, box)
        self.apply_button = QtWidgets.QPushButton("ใช้ค่า / บันทึกค่า")
        self.apply_button.clicked.connect(self.apply_settings)
        settings_form.addRow(self.apply_button)
        hint = QtWidgets.QLabel("ขนาดรวมใบดัน/ส่วนยื่นทั้งหมด\nท้ายทับขอบขาวได้หากยังอยู่บนสนาม\nเส้นประ = ขอบเขตศูนย์กลางตามมุมหุ่น\nFRONT → WHITE = หน้าหุ่นถึงขอบขาว\nBODY → OUTER = ตัวหุ่นถึงขอบตก\nหากย้าย/ซูมกล้อง ให้กำหนดสนามใหม่")
        hint.setWordWrap(True)
        settings_form.addRow(hint)
        scroll.setWidget(settings_widget)
        self.tabs.addTab(scroll, "ขนาด / มอเตอร์")

        demo_panel = QtWidgets.QWidget()
        demo_form = QtWidgets.QFormLayout(demo_panel)
        self.behavior = QtWidgets.QComboBox()
        self.behavior.addItems(["ศัตรูอยู่นิ่ง", "ศัตรูหันหน้ามาสู้", "ศัตรูหันหน้าและวิ่งชน"])
        self.behavior.setCurrentIndex(1)
        demo_form.addRow("พฤติกรรม", self.behavior)
        reset = QtWidgets.QPushButton("จัดตำแหน่งจำลองใหม่")
        reset.clicked.connect(lambda: self.reset_simulation(False))
        jam = QtWidgets.QPushButton("จำลองชนค้างหน้าชนหน้า")
        jam.clicked.connect(lambda: self.reset_simulation(True))
        demo_form.addRow(reset)
        demo_form.addRow(jam)
        desc = QtWidgets.QLabel("ทดลองเส้นทางและสถานะก่อนใช้หุ่นจริง\nเข้าโค้ง → IR / INTERCEPT → PUSH\nไม่มีช่องอ้อม → ATTACK_FRONT\nชนค้าง → ถอยสั้น → เปลี่ยนมุม\nแท็กหลุด → SEARCH → กลับเข้าแผน\nแบบจำลองไม่ได้จำลองแรงยึดเกาะจริง")
        desc.setWordWrap(True)
        demo_form.addRow(desc)
        self.tabs.addTab(demo_panel, "จำลอง")
        view = QtWidgets.QVBoxLayout()
        body.addWidget(sidebar)
        body.addLayout(view, 1)
        display_controls = QtWidgets.QHBoxLayout()
        self.elapsed_label = QtWidgets.QLabel("เวลา 00:00.0")
        self.elapsed_label.setStyleSheet("font-size:17px; font-weight:bold; color:#e3eaf4")
        display_controls.addWidget(self.elapsed_label)
        display_controls.addStretch()
        self.trail_checkbox = QtWidgets.QCheckBox("แสดงเส้นวิ่งจริง")
        self.trail_checkbox.setChecked(True)
        self.trail_checkbox.toggled.connect(self.set_trail_visibility)
        display_controls.addWidget(self.trail_checkbox)
        self.reset_display_button = QtWidgets.QPushButton("รีเซ็ตเส้น / เวลา")
        self.reset_display_button.setToolTip("ล้างเส้นที่วิ่งแล้วและเริ่มจับเวลาใหม่ กดระหว่าง AUTO ได้ หุ่นทำงานต่อ")
        self.reset_display_button.clicked.connect(self.reset_visuals)
        display_controls.addWidget(self.reset_display_button)
        view.addLayout(display_controls)
        images = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        self.camera_canvas = CameraCanvas()
        self.camera_canvas.clicked.connect(self.add_calibration_point)
        self.arena = ArenaCanvas(self.cfg)
        self.camera_canvas.trail = self.arena.trail
        images.addWidget(self.camera_canvas)
        images.addWidget(self.arena)
        images.setSizes([480, 500])
        view.addWidget(images, 1)
        self.state_label = QtWidgets.QLabel("IDLE")
        self.state_label.setStyleSheet("font-size:22px; font-weight:700; color:#ffcf66; padding:8px")
        self.reason_label = QtWidgets.QLabel()
        self.reason_label.setWordWrap(True)
        view.addWidget(self.state_label)
        view.addWidget(self.reason_label)
        self.motor_label = QtWidgets.QLabel("PWM L: 0     R: 0")
        self.motor_label.setStyleSheet("font-size:18px; color:#48ddc9; padding:8px")
        self.telemetry_label = QtWidgets.QLabel("ยังไม่เชื่อมต่อหุ่น")
        self.telemetry_label.setWordWrap(True)
        view.addWidget(self.motor_label)
        view.addWidget(self.telemetry_label)
        self.sensor_label = QtWidgets.QLabel("IR: ซ้าย — | หน้าซ้าย — | กลาง — | หน้าขวา — | ขวา —")
        self.sensor_label.setWordWrap(True)
        view.addWidget(self.sensor_label)
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(150)
        self.log.setMaximumHeight(110)
        view.addWidget(self.log)
        self.setStyleSheet("""
            QMainWindow, QWidget { background:#111b28; color:#e3eaf4; font-family:'Tahoma'; font-size:12px; }
            QPushButton { background:#24364b; border:1px solid #39516c; border-radius:6px; padding:8px; }
            QPushButton:hover { background:#304861; }
            QPushButton:disabled { color:#65758b; background:#1a2636; }
            QPushButton#start { background:#187267; color:white; font-size:17px; font-weight:bold; }
            QPushButton#stop { background:#aa3c4d; color:white; font-size:17px; font-weight:bold; }
            QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox { background:#0b1420; border:1px solid #334b66; border-radius:3px; padding:4px; }
            QTabWidget::pane { border:1px solid #334356; }
            QTabBar::tab { padding:8px 5px; background:#1a2a3d; }
            QTabBar::tab:selected { background:#304861; }
            QPlainTextEdit { background:#0b1420; border:1px solid #334356; font-family:Consolas; }
            QCheckBox { padding:6px 0; }
        """)

    def _restore_settings(self):
        try:
            for name, value in self.saved.get("config", {}).items():
                if name in self.fields:
                    self.fields[name].setValue(float(value))
            self.own_offset.setValue(int(self.saved.get("own_offset", 0)))
            self.enemy_offset.setValue(int(self.saved.get("enemy_offset", 0)))
            self.family.setCurrentText(self.saved.get("family", "36h11"))
            saved_host = str(self.saved.get("host", "auto"))
            if saved_host == "192.168.4.1" and self.saved.get("network_mode") != "station":
                saved_host = "auto"
            self.host.setText(saved_host)
            self.port.setValue(int(self.saved.get("port", 4210)))
            self._apply_values()
        except (TypeError, ValueError, OverflowError):
            self.status.setText("ค่าที่บันทึกไม่ถูกต้อง กรุณาตั้งค่าใหม่")
        # Enumeration is explicit: opening the app or demo never opens hardware.
        self.camera_box.addItem("กดค้นหากล้องก่อน", None)

    def _apply_values(self):
        values = asdict(self.cfg)
        values.update({name: box.value() for name, box in self.fields.items()})
        values["max_pwm"], values["min_pwm"] = int(values["max_pwm"]), int(values["min_pwm"])
        cfg = Config(**values)
        cfg.validate()
        geometry_changed = (cfg.arena_width, cfg.arena_height) != (self.cfg.arena_width, self.cfg.arena_height)
        self.cfg = cfg
        self.arena.cfg = cfg
        self.planner = Planner(cfg)
        self.sim.cfg = cfg
        self.tracker.reset()
        if geometry_changed and self.calibration:
            self.calibration = Calibration(self.calibration.points, self.calibration.image_size, cfg.arena_width, cfg.arena_height)
        self.camera_canvas.calibration = self.calibration

    def _sync_tag_choices(self, ids):
        self.visible_ids = set(ids)
        if not self.demo.isChecked():
            if self.own_tag_id is None and self.saved.get("own_id") in self.visible_ids:
                self.own_tag_id = self.saved["own_id"]
            if self.enemy_tag_id is None and self.saved.get("enemy_id") in self.visible_ids:
                self.enemy_tag_id = self.saved["enemy_id"]
        if self.enemy_tag_id == self.own_tag_id:
            self.enemy_tag_id = None
        remaining = self.visible_ids - {self.own_tag_id}
        if self.own_tag_id is not None and self.enemy_tag_id is None and len(remaining) == 1:
            self.enemy_tag_id = next(iter(remaining))
        for box, chosen, prompt, excluded in (
            (self.own_id_box, self.own_tag_id, "เลือก ID ของหุ่นเรา", None),
            (self.enemy_id_box, self.enemy_tag_id, "เลือก ID ของศัตรู", self.own_tag_id),
        ):
            available = self.visible_ids - {excluded}
            if chosen is not None:
                available.add(chosen)
            entries = [(None, prompt)] + [(tag, f"ID {tag}" + (" (ไม่เห็น)" if tag not in self.visible_ids else ""))
                                           for tag in sorted(available)]
            existing = [(box.itemData(i), box.itemText(i)) for i in range(box.count())]
            box.blockSignals(True)
            if existing != entries:
                box.clear()
                for tag, label in entries:
                    box.addItem(label, tag)
            box.setCurrentIndex(max(0, box.findData(chosen)))
            box.blockSignals(False)
        self.camera_canvas.own_id, self.camera_canvas.enemy_id = self.own_tag_id, self.enemy_tag_id
        self.arena.ids = self.own_tag_id, self.enemy_tag_id
        own = f"ID {self.own_tag_id}" if self.own_tag_id is not None else "เลือก ID"
        enemy = f"ID {self.enemy_tag_id}" if self.enemy_tag_id is not None else "เลือก ID"
        self.mode_label.setText(f"หุ่นเรา: {own}    /    ศัตรู: {enemy}")

    def tag_selection_changed(self):
        self.stop_control("เลือกหุ่นแล้ว ตรวจทิศหัวและแผนก่อน Start")
        self.own_tag_id = self.own_id_box.currentData()
        self.enemy_tag_id = self.enemy_id_box.currentData()
        self._sync_tag_choices(self.visible_ids)
        self.tracker.reset()
        self.poses = (None, None)
        self.last_observation = None
        self.reset_visuals()
        self.update_connection_status()

    def update_connection_status(self):
        if self.demo.isChecked():
            text, color = "DEMO • โหมดจำลอง", "#b39cff"
            detail = "ไม่เชื่อมต่อหุ่นจริง"
        else:
            state = self.link.connection_state
            if self.wifi_joiner and not self.link.online:
                state = "CONNECTING"
            text, color = {
                "DISCONNECTED": ("Wi-Fi • ยังไม่เชื่อมต่อ", "#9aaec6"),
                "CONNECTING": ("Wi-Fi • กำลังเชื่อมต่อ…", YELLOW),
                "ONLINE": ("Wi-Fi • เชื่อมต่อแล้ว", CYAN),
                "LOST": ("Wi-Fi • การเชื่อมต่อขาด", RED),
                "OFFLINE": ("Wi-Fi • ไม่พบการตอบกลับ", RED),
            }[state]
            endpoint = self.link.address
            detail = (f"ESP32 {endpoint[0]}:{endpoint[1]}" if endpoint
                      else f"ค้นหา IP ของ ESP32 ในฮอตสปอต {ROBOT_SSID}")
            if self.link.online:
                detail += f"  |  RTT {self.link.rtt_ms:.0f} ms\nตอบกลับล่าสุด {max(0, time.monotonic() - self.link.last_pong_at):.1f} วินาที"
                if self.link.protocol_version < 4:
                    detail += "\nอัปโหลด STA_V6_AUTO เพื่อให้หุ่นเดินหาเองเมื่อกล้อง/Wi-Fi หลุด"
            elif state in {"LOST", "OFFLINE"}:
                detail += f"\nให้ ESP32 และคอมต่อ {ROBOT_SSID} เหมือนกัน หรือใส่ IP จาก Serial Monitor"
        self.connection_label.setText(text)
        if not self.demo.isChecked() and (self.running or self.arming) and not self.link.online:
            detail += "\nESP32 ใช้ state เดินหา/IR ต่อเอง รอเชื่อมกลับอัตโนมัติ"
        if self.link.stop_pending:
            detail += "\nกำลังส่ง STOP / รอยืนยันว่าหุ่นหยุดแล้ว"
        self.connection_label.setStyleSheet(f"font-size:15px; font-weight:bold; color:{color}; background:transparent;")
        self.connection_detail.setText(detail)
        self.connect_button.setText("ตัดการเชื่อมต่อ Wi-Fi" if self.link.online else "เชื่อมต่อ Wi-Fi หุ่น")
        roles_ready = self.own_tag_id is not None and self.enemy_tag_id is not None and self.own_tag_id != self.enemy_tag_id
        self.start_button.setEnabled(not (self.running or self.arming) and
                                     not self.link.stop_pending and
                                     (self.demo.isChecked() or (self.link.online and roles_ready)))
        self.stop_button.setText("STOP • รอยืนยัน" if self.link.stop_pending else "■ STOP  [Esc]")

    def apply_settings(self):
        self.stop_control("เปลี่ยนค่าควบคุม")
        try:
            self._apply_values()
            self.save_settings()
            self.status.setText("ใช้ค่าและบันทึกแล้ว ตรวจลูกศร FRONT ก่อน Start")
        except (ValueError, OSError) as error:
            self.status.setText(str(error))

    def save_settings(self):
        data = dict(config=asdict(self.cfg), own_offset=self.own_offset.value(), enemy_offset=self.enemy_offset.value(),
                    family=self.active_family or self.family.currentText(), host=self.host.text(), port=self.port.value(),
                    camera=self.active_camera, calibration=self.calibration.as_dict() if self.calibration else None,
                    own_id=self.own_tag_id, enemy_id=self.enemy_tag_id, transport="wifi_udp", network_mode="station")
        temporary = SETTINGS.with_suffix(".tmp")
        temporary.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")
        temporary.replace(SETTINGS)
        self.saved = data

    def refresh_devices(self):
        self.stop_control("ค้นหาอุปกรณ์")
        try:
            from cv2_enumerate_cameras import enumerate_cameras
            cameras, errors = discover_cameras(cv2, enumerate_cameras)
            previous_camera = self.camera_box.currentData() or self.saved.get("camera")
            self.camera_box.clear()
            for camera in cameras:
                backend = cv2.videoio_registry.getBackendName(camera.backend)
                self.camera_box.addItem(f"{camera.name} ({backend})", [camera.index, camera.backend])
            for index in range(self.camera_box.count()):
                if self.camera_box.itemData(index) == previous_camera:
                    self.camera_box.setCurrentIndex(index)
                    break
            if errors:
                self.status.setText("; ".join(errors))
            else:
                self.status.setText(f"พบกล้อง {len(cameras)} รายการ เลือกแล้วกดเปิดภาพ")
        except Exception as error:
            self.status.setText(str(error))

    def open_camera(self):
        self.stop_control("เปิดกล้อง")
        selection = self.camera_box.currentData()
        if not selection:
            self.status.setText("กดค้นหากล้อง แล้วเลือกอุปกรณ์ก่อน")
            return
        self.demo.setChecked(False)
        if self.worker:
            self.worker.stop()
        self.last_observation = None
        self.poses = (None, None)
        self.tracker.reset()
        self.calibration = None
        self.camera_canvas.calibration = None
        saved_calibration = self.saved.get("calibration")
        if (self.saved.get("camera") == selection and self.saved.get("family") == self.family.currentText()
                and saved_calibration and saved_calibration.get("scheme") == "rectangle-v1"):
            try:
                self.calibration = Calibration(saved_calibration["points"], saved_calibration["image_size"], self.cfg.arena_width, self.cfg.arena_height)
                self.camera_canvas.calibration = self.calibration
                self.calibration_label.setText("โหลดสนามเดิมแล้ว หากย้ายกล้องให้กำหนดใหม่")
            except (ValueError, KeyError):
                pass
        else:
            self.calibration_label.setText("ยังไม่ได้กำหนดสนาม")
        self.active_camera, self.active_family = list(selection), self.family.currentText()
        self.worker = CameraWorker(selection[0], selection[1], self.active_family)
        self.worker.start()
        self.status.setText("กำลังเปิดกล้อง…")

    def begin_calibration(self):
        self.stop_control("กำหนดสนามใหม่")
        if self.demo.isChecked() or not self.last_observation or self.last_observation.frame is None:
            self.status.setText("เปิดภาพกล้องจริงก่อนกำหนดสนาม")
            return
        self.calibration = None
        self.camera_canvas.calibration = None
        self.poses = (None, None)
        self.reset_visuals()
        self.camera_canvas.calibrating = True
        self.calibration_points = []
        self.camera_canvas.points = []
        self.calibration_label.setText("คลิกมุมนอกซ้ายบน TL รวมขอบขาว (1/4)")

    def add_calibration_point(self, x, y):
        self.calibration_points.append([x, y])
        self.camera_canvas.points = list(self.calibration_points)
        if len(self.calibration_points) < 4:
            index = len(self.calibration_points)
            self.calibration_label.setText(f"คลิกมุมนอก{CORNER_THAI[index]} {CORNER_NAMES[index]} ({index + 1}/4)")
            return
        self.camera_canvas.calibrating = False
        try:
            h, w = self.last_observation.frame.shape[:2]
            self.calibration = Calibration(self.calibration_points, (w, h), self.cfg.arena_width, self.cfg.arena_height)
            self.camera_canvas.calibration = self.calibration
            self.tracker.reset()
            self.last_observation = None
            self.calibration_label.setText("กำหนดสนามแล้ว ตรวจขนาดและลูกศร FRONT ทั้งสองหุ่น")
            self.save_settings()
        except (ValueError, OSError) as error:
            self.calibration_label.setText(str(error))

    def connect_robot(self):
        self.stop_control("เปลี่ยนการเชื่อมต่อ")
        if self.wifi_joiner:
            self.wifi_joiner.stop()
            self.wifi_joiner = None
        if self.link.online:
            self.link.close()
            self.status.setText("ตัดการเชื่อมต่อแล้ว")
            return
        if self.demo.isChecked():
            self.status.setText("ปิดโหมดจำลองก่อนเชื่อมต่อหุ่นจริง")
            return
        try:
            self.link.connect_udp(self.host.text().strip(), self.port.value())
            self.status.setText("เปิดช่อง Wi-Fi แล้ว กำลังตรวจเครือข่ายและการตอบกลับจาก ESP32…")
            if self.host.text().strip().lower() in {"", "auto"}:
                self.wifi_joiner = WifiJoiner(ROOT / "robot_wifi.xml")
                self.wifi_joiner.start()
        except Exception as error:
            self.link.close()
            self.status.setText(str(error))
        self.update_connection_status()

    def prepare_wifi(self):
        if not self.demo.isChecked() and not self.link.connected:
            self.connect_robot()

    def change_mode(self):
        self.stop_control("เปลี่ยนโหมด")
        if self.wifi_joiner:
            self.wifi_joiner.stop()
            self.wifi_joiner = None
        self.link.close()
        if self.demo.isChecked():
            if self.worker:
                self.worker.stop()
                self.worker = None
            self.camera_canvas.observation = None
            self.last_observation = None
            self.own_tag_id, self.enemy_tag_id = 1, 2
            self._sync_tag_choices({1, 2})
            self.reset_simulation(False)
        else:
            self.own_tag_id = self.enemy_tag_id = None
            self._sync_tag_choices(set())
            self.poses = (None, None)
        self.update_connection_status()

    def reset_simulation(self, head_on):
        if not self.demo.isChecked():
            self.status.setText("เลือกโหมดจำลองก่อน")
            return
        self.stop_control("จัดตำแหน่งจำลองแล้ว กด Start เพื่อเริ่ม")
        self.sim.reset(head_on)
        self.poses = (self.sim.own, self.sim.enemy)
        self.reset_visuals()
        if head_on:
            self.behavior.setCurrentIndex(1)
            self.planner.state = "PUSH"
        self.demo_head_on = head_on

    def start_control(self):
        if self.running or self.arming:
            return
        try:
            self.match_result = None
            self._apply_values()
            if self.demo.isChecked():
                self.planner.reset()
                if getattr(self, "demo_head_on", False):
                    self.planner.state = "PUSH"
                self.running = True
                self.status.setText("กำลังจำลอง • ไม่ส่งข้อมูลไปหุ่นจริง")
            else:
                if not self.link.online:
                    raise ValueError("รอสถานะ Wi-Fi เชื่อมต่อแล้ว (สีเขียว) ก่อน Start")
                if self.link.protocol_version < 4:
                    raise ValueError("อัปโหลด SUMO_TOPVIEW_WIFI_STA_V6_AUTO ก่อน เพื่อให้หุ่นเล่นต่อเองเมื่อกล้อง/Wi-Fi หลุด")
                if self.own_tag_id is None or self.enemy_tag_id is None or self.own_tag_id == self.enemy_tag_id:
                    raise ValueError("เลือก ID หุ่นเราและศัตรูให้ต่างกันก่อน Start")
                self.save_settings()
                self.link.begin_arm()
                self.arming = True
                self.status.setText("กำลังรอ ESP32 ยืนยัน ARM…")
            self.reset_visuals()
            self.tabs.setEnabled(False)
            self.start_button.setEnabled(False)
        except Exception as error:
            self.stop_control(str(error))

    def stop_control(self, reason="STOP"):
        was_active = self.running or self.arming
        if self.elapsed_started_at is not None:
            self.elapsed_frozen = max(0.0, time.monotonic() - self.elapsed_started_at)
            self.elapsed_started_at = None
        self.running = self.arming = False
        self.link.stop()
        self.planner.reset()
        self.plan = Plan("STOPPED", reason)
        if hasattr(self, "status"):
            self.status.setText(reason)
            self.tabs.setEnabled(True)
            self.start_button.setEnabled(True)
            if was_active:
                self.log.appendPlainText(f"{time.strftime('%H:%M:%S')}  STOP  {reason}")
        self.update_elapsed()

    def update_elapsed(self, now=None):
        now = time.monotonic() if now is None else now
        elapsed = self.elapsed_frozen if self.elapsed_started_at is None else max(0, now - self.elapsed_started_at)
        tenths = int(elapsed * 10)
        self.elapsed_label.setText(f"เวลา {tenths // 600:02d}:{tenths // 10 % 60:02d}.{tenths % 10}")

    def reset_visuals(self):
        self.arena.trail.clear()
        self.last_trail_pose = None
        self.elapsed_frozen = 0.0
        self.elapsed_started_at = time.monotonic() if self.running or self.arming else None
        self.update_elapsed()
        self.arena.update()
        self.camera_canvas.update()

    def set_trail_visibility(self, visible):
        self.arena.show_trail = self.camera_canvas.show_trail = visible
        self.arena.update()
        self.camera_canvas.update()

    def record_trail(self, pose, now):
        trail = self.arena.trail
        if pose is None or now - pose.timestamp > self.cfg.frame_timeout:
            if trail and trail[-1] is not None:
                trail.append(None)
            self.last_trail_pose = None
            return
        previous = self.last_trail_pose
        if previous and pose.timestamp <= previous[0]:
            return
        if previous:
            dt = pose.timestamp - previous[0]
            if dt > self.cfg.frame_timeout or (pose.position - previous[1]).norm() > max(80, self.cfg.max_wheel_speed * dt * 1.8 + 20):
                if trail and trail[-1] is not None:
                    trail.append(None)
        if not trail or trail[-1] is None or (pose.position - trail[-1]).norm() >= 3:
            trail.append(pose.position)
        self.last_trail_pose = (pose.timestamp, pose.position)

    def tick(self):
        try:
            self._tick()
        except Exception as error:
            if self.running or self.arming:
                self.plan = Plan("LOCAL_ERROR", "ESP32 เดินหา/ใช้ IR ต่อ: " + str(error), assist=2)
                self.status.setText(self.plan.reason)
                if self.link.armed and self.link.address:
                    self.link.drive(0, 0, 2, self.cfg.max_pwm)
            else:
                self.status.setText("Controller error: " + str(error))

    def _tick(self):
        now = time.monotonic()
        dt, self.previous_tick = now - self.previous_tick, now
        self.link.poll()
        if self.wifi_joiner and self.wifi_joiner.done.is_set():
            if self.wifi_joiner.result and not self.link.online:
                self.status.setText(self.wifi_joiner.result)
            self.wifi_joiner = None
        if ((self.running or self.arming) and not self.demo.isChecked() and not self.link.online
                and not self.wifi_joiner and now - self.last_wifi_retry > 8):
            self.last_wifi_retry = now
            self.wifi_joiner = WifiJoiner(ROOT / "robot_wifi.xml")
            self.wifi_joiner.start()
        if self.link.remote_stop and (self.running or self.arming):
            self.stop_control("หยุดจาก ESP32: " + self.link.remote_stop)
        new_observation = False
        if self.demo.isChecked():
            left, right = (self.plan.left, self.plan.right) if self.running else (0, 0)
            self.poses = self.sim.step(left, right, dt, now, self.behavior.currentIndex() if self.running else 0)
            new_observation = True
        elif self.worker:
            observation = self.worker.snapshot()
            if observation is not None and observation is not self.last_observation:
                self.last_observation = observation
                self.duplicate_selected = bool(observation.duplicates & {self.own_tag_id, self.enemy_tag_id})
                self.camera_canvas.observation = observation
                self._sync_tag_choices(set(observation.detections))
                new_observation = True
                self.poses = (None, None)
                if observation.error:
                    self.status.setText("กล้องไม่พร้อม ใช้ IR ต่อ: " + observation.error)
                elif self.calibration:
                    h, w = observation.frame.shape[:2]
                    if (w, h) != self.calibration.image_size:
                        self.calibration = None
                        self.camera_canvas.calibration = None
                        self.status.setText("ขนาดภาพเปลี่ยน ใช้ IR ต่อจนกว่าจะกำหนดสนามใหม่")
                    elif self.own_tag_id is not None and self.enemy_tag_id is not None:
                        self.poses = self.tracker.extract(observation, self.calibration,
                                                         ids=(self.own_tag_id, self.enemy_tag_id),
                                                         offsets=(self.own_offset.value(), self.enemy_offset.value()))
        valid = all(p is not None and now - p.timestamp <= self.cfg.frame_timeout for p in self.poses)
        # Record actual camera measurements even when the opponent is missing or
        # AUTO is using onboard IR. Never invent a path from motor commands.
        if new_observation or (self.last_trail_pose is not None and not valid):
            self.record_trail(self.poses[0], now)
        if self.arming and self.link.armed:
            self.arming, self.running = False, True
            self.planner.reset()
            self.status.setText("กำลังควบคุมหุ่นจริง • กด Stop หรือ Esc เพื่อหยุด")
        if self.running:
            sensors = self.link.telemetry if not self.demo.isChecked() and self.link.healthy(now) else {}
            observation = self.last_observation
            camera_ready = self.demo.isChecked() or (self.calibration is not None and observation is not None
                            and not observation.error and now - observation.timestamp <= max(.7, self.cfg.frame_timeout + .2))
            if self.duplicate_selected and not self.demo.isChecked():
                self.plan = Plan("LOCAL_SEARCH", "ID ซ้ำ ใช้ state เดินหา/IR จนแยกแท็กได้", assist=2)
            elif not camera_ready:
                self.plan = self.planner.update(None, None, now, sensors=sensors)
            else:
                self.plan = self.planner.update(*self.poses, now, sensors=sensors)
            if self.plan.state in {"WIN", "LOSS"}:
                result = self.plan
                self.stop_control("ชนะ: ยืนยันศัตรูออกสนาม" if result.state == "WIN" else "แพ้: ยืนยันหุ่นเราออกสนาม")
                self.plan = result
                self.match_result = result
            elif not self.demo.isChecked():
                if self.plan.state == "INVALID":
                    self.plan = Plan("LOCAL_SEARCH", "พิกัดกล้องไม่ถูกต้อง ใช้ IR ต่อ", assist=2)
                send_time = time.monotonic()
                if send_time - now > 0.15:
                    self.plan = Plan("LOCAL_COMPUTE", "กล้องคำนวณช้า ใช้ state บนหุ่นต่อ", assist=2)
                if self.link.armed and self.link.address:
                    self.link.drive(self.plan.left, self.plan.right, self.plan.assist, self.cfg.max_pwm)
                elif not self.link.online:
                    self.plan = Plan("LOCAL_LINK", "Wi-Fi หลุด: ESP32 เดินหา/IR ต่อเอง รอเชื่อมกลับ", assist=2)
        elif not self.arming and valid:
            preview = Planner(self.cfg).update(*self.poses, now)
            preview.state = "PREVIEW / " + preview.state
            preview.reason = "แผนก่อน Start: " + preview.reason
            preview.left = preview.right = 0
            self.plan = preview
        self.arena.poses, self.arena.plan = self.poses, self.plan
        self.camera_canvas.poses, self.camera_canvas.plan = self.poses, self.plan
        self.arena.update()
        self.camera_canvas.update()
        self.update_elapsed(now)
        self.state_label.setText(("RUNNING / " if self.running else "") + ("ARMING" if self.arming else self.plan.state))
        self.reason_label.setText(self.plan.reason)
        if self.plan.state.startswith("PREVIEW"):
            self.motor_label.setText("PREVIEW • ดูเส้นทางแล้วกด START เพื่อเริ่ม")
        elif self.running and self.plan.assist == 2:
            self.motor_label.setText("ESP32 AUTO • เดินหา / IR เจอ = เข้าชนเต็มกำลัง • ดู PWM จริงด้านล่าง")
        else:
            self.motor_label.setText(f"สั่ง PWM   L: {self.plan.left:+4d}     R: {self.plan.right:+4d}    |    ฝั่งเข้าข้าง: {self.plan.side:+d}")
        telemetry = self.link.telemetry
        age = max(0, now - self.poses[0].timestamp) * 1000 if self.poses[0] else None
        observation_status = f"อายุภาพ {age:.0f} ms" if age is not None else "ยังไม่มีตำแหน่งที่ใช้ได้"
        if telemetry:
            ir = telemetry['ir']
            names = ("ซ้าย", "หน้าซ้าย", "กลาง", "หน้าขวา", "ขวา")
            assist_text = ("หยุดมอเตอร์ / IR อ่านค่า" if not self.running else
                           {0: "กล้องกำหนดมุม / หลบขอบ", 1: "เสริมเข้าชน", 2: "ค้นหา"}[self.plan.assist])
            self.sensor_label.setText("IR: " + " | ".join(f"{name} {'เจอ' if ir & (1 << i) else '—'}" for i, name in enumerate(names))
                                      + f"  |  ขอบ L={telemetry['floor_left']} R={telemetry['floor_right']}  |  {assist_text}")
            self.sensor_label.setToolTip("IR ตามขาจริง: ซ้าย GPIO12 / หน้าซ้าย GPIO14 / กลาง GPIO27 / หน้าขวา GPIO21 / ขวา GPIO19\nเจอวัตถุ = LOW (0); เซนเซอร์พื้นมีสิทธิ์ก่อน IR เสมอ")
            self.telemetry_label.setText(f"{observation_status}  |  ESP32: {telemetry['action']}  |  PWM จริง L={telemetry['left']} R={telemetry['right']}  |  ACK={telemetry['sequence']}/{self.link.sequence}")
        else:
            self.telemetry_label.setText(observation_status + ("  |  DEMO: แบบจำลองการเคลื่อนที่" if self.demo.isChecked() else "  |  ยังไม่มี telemetry จาก ESP32"))
            self.sensor_label.setText("DEMO: ไม่มีเซนเซอร์จริง" if self.demo.isChecked() else "IR: ยังไม่มีข้อมูลจากหุ่น")
        if not self.demo.isChecked() and self.last_observation:
            detected = ', '.join(str(tag) for tag in sorted(self.last_observation.detections)) or 'ไม่มี'
            details = []
            for tag, pose in zip((self.own_tag_id, self.enemy_tag_id), self.poses):
                if tag is None:
                    continue
                note = self.tracker.status.get(tag, 'ยังไม่ประมวลผล — ตรวจการกำหนดสนาม')
                if pose and now - pose.timestamp > self.cfg.frame_timeout:
                    note = f"เห็น ID แต่ภาพเก่า {(now - pose.timestamp) * 1000:.0f} ms"
                details.append(f"ID {tag}: {note}")
            diagnostic = ' | '.join(details)
            self.telemetry_label.setText(self.telemetry_label.text() + f"\nภาพพบ [{detected}]  |  {diagnostic}")
        if self.running and self.plan.state != self.previous_state:
            self.log.appendPlainText(f"{time.strftime('%H:%M:%S')}  {self.plan.state}  {self.plan.reason}")
        self.previous_state = self.plan.state
        if self.running:
            self.status.setText("AUTO: " + self.plan.reason)
        self.update_connection_status()
        if self.close_requested and not self.link.stop_pending:
            QtCore.QTimer.singleShot(0, self.close)

    def closeEvent(self, event):
        if not self.close_requested:
            self.close_requested = True
            self.close_wait_stop = (self.running or self.arming or self.link.remote_armed
                                    or (self.link.stop_pending and getattr(self.link, "ever_armed", False)))
            self.stop_control("ปิดโปรแกรม")
        if self.link.stop_pending and self.close_wait_stop:
            self.status.setText("กำลังรอเครือข่ายเพื่อส่ง STOP ให้หุ่น กด EN ที่ ESP32 เพื่อหยุดทันทีได้")
            event.ignore()
            return
        self.timer.stop()
        self.link.close()
        if self.wifi_joiner:
            self.wifi_joiner.stop()
        if self.worker:
            self.worker.stop()
        event.accept()
