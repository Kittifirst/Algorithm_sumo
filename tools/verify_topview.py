"""Hardware-free regression checks. Does not read or modify the test directory.

Run: .venv\Scripts\python.exe tools/verify_topview.py
"""

from collections import Counter
import math
import os
from pathlib import Path
import random
import socket
import sys
import time

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import cv2
import numpy as np

from read_apriltag import create_detector
from sumo.link import RobotLink
from sumo.planner import Config, Plan, Planner, Pose, Vec, direction, rectangles_overlap, rectangle_corners, wrap
from sumo.simulation import Simulation
from sumo.vision import Calibration, CameraWorker, Observation, PoseTracker


def check_planning():
    cfg = Config()
    assert (cfg.arena_width, cfg.arena_height, cfg.white_border) == (840, 840, 50)
    assert (cfg.robot_width, cfg.robot_length, cfg.enemy_width, cfg.enemy_length) == (180, 210, 180, 210)
    own, enemy = Pose(Vec(-140, -200), 0, 10), Pose(Vec(80, 100), math.pi, 10)
    planner = Planner(cfg)
    plan = planner.update(own, enemy, 10)
    assert plan.state == "FLANK" and 0 < plan.left < plan.right, plan
    assert len(plan.path) > 5
    assert plan.approach[0] == own.position and plan.approach[-1] == enemy.position
    assert len(plan.approach) > len(plan.path)
    route_tangents = [(b-a).angle() for a,b in zip(plan.approach, plan.approach[1:]) if (b-a).norm() > .1]
    assert abs(route_tangents[-1] - route_tangents[0]) > .3
    # A curved plan has a changing tangent, rather than a straight waypoint line.
    tangents = [(b-a).angle() for a,b in zip(plan.path, plan.path[1:]) if (b-a).norm() > .1]
    assert abs(tangents[-1]-tangents[0]) > .1
    assert planner.wheels(own, own.position + Vec(-100, 0), reverse=True)[0] < 0
    assert Planner(cfg).update(None, enemy, 10).assist == 2
    outside = Pose(Vec(430, 0), 0, 10)
    assert Planner(cfg).update(outside, enemy, 10).state == "RESULT_CHECK"
    edge = Pose(Vec(cfg.safe_limits(0).x + 10, 0), 0, 10)
    recovery = Planner(cfg).update(edge, enemy, 10)
    assert recovery.state == "EDGE_RETURN" and recovery.left < 0 and recovery.right < 0
    assert abs(cfg.boundary_clearance(Vec(0,0),0)-265) < .01

    # Rear of an opponent on the right-hand boundary has no reachable stance.
    # Taking the already available frontal contact must still issue forward drive.
    a, b = Pose(Vec(10,0), 0,10), Pose(Vec(220,0),math.pi,10)
    attack = Planner(cfg).update(a,b,10)
    assert attack.state == "ATTACK_FRONT" and attack.left > 0 and attack.right > 0, attack
    # Mid-game side contact must advance immediately, not stop at a flank point.
    a, b = Pose(Vec(0,-195),math.pi/2,10), Pose(Vec(0,0),0,10)
    capture = Planner(cfg).update(a,b,10, sensors={"ir":4})
    assert capture.state == "PUSH" and capture.left > 0 and capture.right > 0, capture
    assert capture.assist == 1
    assert cfg.max_pwm == 255
    # Rear white is allowed at every orientation, including diagonal corners.
    # The front is still black and every corner remains on the physical field.
    for heading, radius in ((0,300),(math.pi/2,300),(math.pi,300),(-math.pi/2,300),(math.pi/4,360)):
        a=Pose(direction(heading)*-radius,heading,10)
        b=Pose(Vec(0,0),wrap(heading+math.pi),10)
        assert cfg.boundary_clearance(a.position,a.heading) < 0
        assert cfg.front_clearance(a.position,a.heading) > cfg.edge_margin
        assert cfg.outer_clearance(a.position,a.heading) > 4
        assert cfg.inside(a.position,a.heading)
        attack=Planner(cfg).update(a,b,10,sensors={"ir":4})
        assert attack.state not in {"EDGE_RETURN","CLEARANCE_HOLD","OUTSIDE"}, attack
        assert attack.left==attack.right==255, attack
    # Front white and an overhanging rear still require inward recovery.
    assert not cfg.inside(Vec(300,0),0)
    assert not cfg.inside(Vec(-320,0),0)

    rng = random.Random(41)
    checked = 0
    for _ in range(100):
        a = Pose(Vec(rng.uniform(-200,200),rng.uniform(-200,200)),rng.uniform(-math.pi,math.pi),10)
        b = Pose(Vec(rng.uniform(-180,180),rng.uniform(-180,180)),rng.uniform(-math.pi,math.pi),10)
        if rectangles_overlap(a.position,a.heading,180,210,b.position,b.heading,180,210,12):
            continue
        planner = Planner(cfg)
        q = planner.update(a,b,10)
        assert max(abs(q.left),abs(q.right)) <= cfg.max_pwm
        if q.state not in {"FLANK","INTERCEPT","PUSH","ATTACK_FRONT"}:
            continue
        # Independently integrate at 5 ms and inspect every physical body corner.
        pose = a
        horizon = (len(q.path)-1)*.04
        for step in range(round(horizon/.005)):
            vl, vr = q.left/255*cfg.max_wheel_speed, q.right/255*cfg.max_wheel_speed
            yaw = (vr-vl)/cfg.wheel_base
            h = pose.heading + yaw*.005
            pos = pose.position + direction(pose.heading+yaw*.0025)*((vl+vr)/2*.005)
            pose = Pose(pos,h,10)
            for corner in rectangle_corners(pos,h,180,210):
                assert max(abs(corner.x),abs(corner.y)) < 420-3.5, (q.state,corner)
        checked += 1
    assert checked >= 5, checked
    print(f"PASS planning: curved opening, rear-at-white frontal fallback, immediate flank capture, {checked} swept-body checks")


def check_recovery():
    cfg = Config()
    planner = Planner(cfg)
    transitions = []
    for f in range(65):
        now = 10+f*.05
        a,b = Pose(Vec(-105,0),0,now), Pose(Vec(105,0),math.pi,now)
        q = planner.update(a,b,now)
        if not transitions or transitions[-1] != q.state:
            transitions.append(q.state)
        if q.state == "RETREAT":
            assert q.left == q.right == -255, q
    assert transitions[0] == "ATTACK_FRONT" and "RETREAT" in transitions, transitions
    assert transitions.count("ATTACK_FRONT") >= 2, transitions
    assert transitions.count("RETREAT") == 1, transitions
    planner = Planner(cfg)
    for f in range(45):
        now = 20+f*.05
        shift = f*2
        a,b = Pose(Vec(-105+shift,0),0,now),Pose(Vec(105+shift,0),math.pi,now)
        assert planner.update(a,b,now).state != "RETREAT"
    # Slow successful pushing (under the old 22mm stall threshold) is progress.
    planner=Planner(cfg)
    for f in range(130):
        now=25+f*.05
        shift=f*.3
        a,b=Pose(Vec(-105+shift,0),0,now),Pose(Vec(105+shift,0),math.pi,now)
        assert planner.update(a,b,now,sensors={"ir":4,"left":255,"right":255}).state != "RETREAT"
    # Near a drop, hold pressure longer and forbid repeated back/ram cycles.
    planner=Planner(cfg)
    retreat_starts=[]
    previous=""
    for f in range(210):
        now=35+f*.05
        a,b=Pose(Vec(90,0),0,now),Pose(Vec(300,0),math.pi,now)
        q=planner.update(a,b,now,sensors={"ir":4,"left":255,"right":255})
        if q.state=="RETREAT" and previous!="RETREAT": retreat_starts.append(now)
        previous=q.state
    assert retreat_starts and retreat_starts[0]>=38.2, retreat_starts
    assert all(b-a>=6 for a,b in zip(retreat_starts,retreat_starts[1:])), retreat_starts
    sim,planner = Simulation(cfg),Planner(cfg)
    left=right=0
    states=Counter()
    for f in range(180):
        now=30+f*.05
        a,b=sim.step(left,right,.05,now,behavior=0)
        q=planner.update(a,b,now); left,right=q.left,q.right
        states[q.state]+=1
        assert cfg.outer_clearance(a.position,a.heading) >= 4
    assert states["PUSH"] > 0, states
    # Hold is bounded; no dead waypoint or permanently frozen tactical state.
    assert states["CLEARANCE_HOLD"] == 0, states
    print("PASS recovery: frontal pressure -> bounded retreat -> retry; moving contact avoids false stall; closed-loop demo pushes")


def check_search():
    cfg=Config()
    planner=Planner(cfg)
    a,b=Pose(Vec(0,0),0,10),Pose(Vec(220,0),math.pi,10)
    planner.update(a,b,10)
    for now in (10.05,10.5,11,13,99):
        q=planner.update(None,None,now)
        assert q.state=="LOCAL_SEARCH" and q.assist==2 and q.left>0 and q.right>0, q
    q=planner.update(Pose(a.position,0,100),Pose(b.position,b.heading,100),100)
    assert q.assist != 2 and q.state != "LOCAL_SEARCH", q
    # Reproduce both supplied screenshots: no strict start/geometry freeze at 3mm
    # and an escape maneuver when facing a nearby edge beside the opponent.
    corner=Pose(Vec(0,312),-math.pi/2,10)
    q=Planner(cfg).update(corner,Pose(Vec(-150,0),0,10),10)
    assert q.state=="EDGE_RETURN" and q.left+q.right>0, q
    tight=Config(white_border=10,wheel_base=170)
    a=Pose(Vec(175,-270),-math.pi/2,10)
    b=Pose(Vec(-40,-245),1.85,10)
    q=Planner(tight).update(a,b,10)
    assert q.state not in {"CLEARANCE_HOLD","SEARCH_HOLD"}
    assert q.assist==2 or q.left+q.right!=0, q
    # A tag must be outside for several distinct fresh frames before a result.
    for which,result in (("enemy","WIN"),("own","LOSS")):
        planner=Planner(cfg)
        for frame in range(10):
            now=20+frame*.05
            a=Pose(Vec(440,0) if which=="own" else Vec(0,0),0,now)
            b=Pose(Vec(440,0) if which=="enemy" else Vec(-100,0),0,now)
            q=planner.update(a,b,now)
        assert q.state==result, q
    print("PASS AUTO: sustained walking search, automatic vision takeover, 3mm-edge recovery, screenshot escape, confirmed win/loss")


def check_vision():
    calibration = Calibration([[100, 100], [700, 100], [700, 700], [100, 700]], (800, 800), 840, 840)
    np.testing.assert_allclose(calibration.transform([[100, 100], [700, 100], [400, 400]]), [[-420, 420], [420, 420], [0, 0]], atol=1e-4)
    assert calibration.as_dict()["scheme"] == "rectangle-v1"
    detector = create_detector(cv2, "36h11")
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    marker = cv2.aruco.generateImageMarker(dictionary, 1, 160)
    for quarter_turn in range(4):
        gray = np.full((800, 800), 255, dtype=np.uint8)
        gray[320:480, 320:480] = np.rot90(marker, quarter_turn)
        corners, ids, _ = detector.detectMarkers(gray)
        assert ids is not None and ids.flatten().tolist() == [1]
        obs = Observation(cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR), 10, {1: corners[0].reshape(4, 2)}, set())
        own, _ = PoseTracker().extract(obs, calibration)
        expected = direction(math.pi / 2 + quarter_turn * math.pi / 2)
        assert direction(own.heading).dot(expected) > 0.999, (quarter_turn, own.heading)
        shifted, _ = PoseTracker().extract(obs, calibration, offsets=(90, 0))
        assert abs(direction(own.heading).dot(direction(shifted.heading))) < 0.01
        obs.duplicates.add(1)
        assert PoseTracker().extract(obs, calibration)[0] is None
    for invalid in ([[0, 0]] * 4, [[400, 100], [100, 400], [400, 700], [700, 400]]):
        try:
            Calibration(invalid, (800, 800), 300)
        except ValueError:
            pass
        else:
            raise AssertionError("Invalid calibration accepted")
    # A missed frame does not discard motion history or permanently reject a tag.
    points=np.float32([[280,380],[320,380],[320,420],[280,420]])
    tracker=PoseTracker()
    first=Observation(np.zeros((800,800,3),np.uint8),10,{17:points},set())
    old=tracker.extract(first,calibration,ids=(17,42))[0]
    tracker.extract(Observation(first.frame,10.05,{},set()),calibration,ids=(17,42))
    assert tracker.previous[17] is old
    # A single teleport is rejected, two consistent new readings reacquire.
    shifted=points+np.float32([300,0])
    assert tracker.extract(Observation(first.frame,10.10,{17:shifted},set()),calibration,ids=(17,42))[0] is None
    assert tracker.extract(Observation(first.frame,10.15,{17:shifted},set()),calibration,ids=(17,42))[0] is not None
    print("PASS vision: generated AprilTag decoded at all four rotations, offsets, duplicate rejection, calibration")


def check_camera_recovery():
    from unittest.mock import patch
    worker=CameraWorker(0,cv2.CAP_DSHOW,"36h11")
    worker.retry_seconds=.001
    frames=[]
    publish=worker._publish
    def record(observation):
        frames.append(observation)
        publish(observation)
    worker._publish=record
    class FakeCapture:
        def __init__(self,working): self.working=working
        def set(self,*args): pass
        def isOpened(self): return True
        def release(self): pass
        def read(self):
            if not self.working: return False,None
            worker.halt.set()
            return True,np.zeros((240,320,3),np.uint8)
    with patch("sumo.vision.cv2.VideoCapture",side_effect=[FakeCapture(False),FakeCapture(True)]):
        worker.start()
        worker.thread.join(timeout=2)
        assert not worker.thread.is_alive()
    assert frames[0].error and frames[-1].frame is not None and not frames[-1].error
    print("PASS camera: failed capture reopens automatically and publishes fresh observations")


def check_link():
    # A local peer exercises the real UDP socket, framing, handshake and telemetry
    # parser. This is not a substitute for testing the physical ESP32 watchdog.
    peer = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    peer.bind(("127.0.0.1", 0))
    peer.settimeout(0.5)
    client = RobotLink()
    try:
        client.connect_udp("127.0.0.1", peer.getsockname()[1])
        assert client.connection_state == "CONNECTING" and not client.online
        while True:
            packet, sender = peer.recvfrom(2048)
            if packet.startswith(b"PING"):
                break
        probe = int(packet.split()[1])
        peer.sendto(f"PONG {probe + 1} 1 1000 0\n".encode(), sender)
        client.poll()
        assert not client.online
        peer.sendto(f"PONG {probe} 4 1000 0 4 0 0 BOOT_STOP 0 0\n".encode(), sender)
        client.poll()
        assert client.online and client.connection_state == "ONLINE" and client.rtt_ms is not None
        assert client.protocol_version == 4 and client.telemetry["ir"] == 4 and not client.armed
        client.last_pong_at -= 2
        assert client.connection_state == "LOST" and not client.online
        client._ping()
        packet, sender = peer.recvfrom(2048)
        peer.sendto(f"PONG {int(packet.split()[1])} 1 1000 0\n".encode(), sender)
        client.poll()
        assert client.online
        client.begin_arm()
        while True:
            packet, sender = peer.recvfrom(2048)
            if packet.startswith(b"HELLO"):
                break
        token = int(packet.split()[1])
        peer.sendto(f"READY {token + 1} 6789 1\n".encode(), sender)
        client.poll()
        assert client.pending == "HELLO" and not client.armed
        peer.sendto(f"READY {token} 6789 4\n".encode(), sender)
        client.poll()
        packet, sender = peer.recvfrom(2048)
        assert packet.decode().strip() == f"ARM {token} 6789"
        peer.sendto(f"ARMED {token}\n".encode(), sender)
        client.poll()
        assert client.armed
        client.drive(120, -90, assist=1, cap=130)
        packet, sender = peer.recvfrom(2048)
        assert packet.decode().strip() == f"DRIVE {token} 1 120 -90 350 1 130"
        peer.sendto(f"TEL {token} 1 1 DRIVE 120 -90 0 0 5 5000\n".encode(), sender)
        client.poll()
        assert client.telemetry["left"] == 120 and client.healthy(time.monotonic())
        assert not client.healthy(time.monotonic() + 1)
        client.stop()
        assert not client.armed and not client.pending
        # A delayed ARM acknowledgement after Stop cannot re-enable the link.
        peer.sendto(f"ARMED {token}\n".encode(), sender)
        client.poll()
        assert not client.armed
        try:
            client.drive(255, 255)
        except RuntimeError:
            pass
        else:
            raise AssertionError("DRIVE accepted while stopped")
    finally:
        client.close()
        peer.close()
    print("PASS Wi-Fi: verified PING/PONG, loss/recovery, RTT, ARM handshake, stale ACK rejection, motors, Stop")


def check_auto_reconnect():
    peer=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    peer.bind(("127.0.0.1",0))
    peer.settimeout(.5)
    client=RobotLink()
    def receive(prefix):
        for _ in range(30):
            packet,sender=peer.recvfrom(2048)
            if packet.startswith(prefix):
                return packet.decode().split(),sender
        raise AssertionError(prefix)
    try:
        client.connect_udp("127.0.0.1",peer.getsockname()[1])
        ping,sender=receive(b"PING ")
        peer.sendto(f"PONG {ping[1]} 4 1000 0 0 0 0 BOOT_STOP 0 0\n".encode(),sender)
        client.poll()
        client.begin_arm()
        hello,sender=receive(b"HELLO ")
        session=client.session
        peer.sendto(f"READY {session} 1234 4\n".encode(),sender)
        client.poll()
        receive(b"ARM ")
        peer.sendto(f"ARMED {session}\n".encode(),sender)
        client.poll()
        assert client.auto_requested and client.armed
        client.drive(255,255,1,255)
        receive(b"DRIVE ")
        # Simulate a stale network, then the same robot running onboard AUTO.
        client.last_pong_at-=2
        client.poll()
        assert client.auto_requested and not client.armed and client.session==session
        client._ping()
        ping,sender=receive(b"PING ")
        client.last_hello_at=0
        peer.sendto(f"PONG {ping[1]} 4 2000 1 4 0 0 AUTO_IR_ATTACK 255 255\n".encode(),sender)
        client.poll()
        hello,sender=receive(b"HELLO ")
        assert int(hello[1])==session
        peer.sendto(f"RESUMED {session} 80 4\n".encode(),sender)
        client.poll()
        assert client.armed and client.auto_requested and client.sequence==80
        client.drive(120,255,1,255)
        drive,sender=receive(b"DRIVE ")
        assert drive[2]=="81" and drive[5]=="350"
        # A physical/serial user stop must cancel AUTO, including across reconnect.
        client.armed=False
        peer.sendto(f"TEL {session} 81 0 USER_STOP 0 0 0 0 0 5000\n".encode(),sender)
        client.poll()
        assert not client.auto_requested and client.remote_stop=="USER_STOP"
        # Stop stays sticky: delayed ARM/RESUMED cannot restart the session.
        client.stop()
        peer.sendto(f"RESUMED {session} 100 4\nARMED {session}\n".encode(),sender)
        client.poll()
        assert not client.auto_requested and not client.armed
        assert client.stop_pending
        peer.sendto(b"STOPPED\n",sender)
        client.poll()
        assert not client.stop_pending
        # Reset/EN is a real physical stop, not a network handoff to re-arm.
        client.auto_requested=True
        client.armed=True
        client._remote_state(0,"BOOT_STOP")
        assert not client.auto_requested and not client.armed and client.remote_stop=="BOOT_STOP"
    finally:
        client.close(); peer.close()
    print("PASS autonomous reconnect: same-session resume, sequence recovery, sticky user Stop and reset-to-Stop")


def check_discovery():
    # Use an ephemeral service port and a loopback peer. Discovery must learn
    # its endpoint only from a fresh matching PONG, never from any UDP packet.
    peer = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    peer.bind(("127.0.0.1", 0))
    peer.settimeout(0.5)
    stranger = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    stranger.bind(("127.0.0.1", 0))
    client = RobotLink()
    try:
        client.connect_udp("auto", peer.getsockname()[1])
        assert client.address is None and not client.online and not client.armed
        recipient = ("127.0.0.1", client.device.getsockname()[1])
        assert recipient[1] > 0
        probe = next(reversed(client.probes))
        peer.sendto(f"PONG {probe + 1} 1 1000 0\n".encode(), recipient)
        client.poll()
        assert client.address is None and not client.online
        client.probes[probe] -= 2
        peer.sendto(f"PONG {probe} 1 1000 0\n".encode(), recipient)
        client.poll()
        assert client.address is None and not client.online
        client._ping()
        probe = next(reversed(client.probes))
        peer.sendto(f"READY {probe} 1234 1\n".encode(), recipient)
        client.poll()
        assert client.address is None and not client.online and not client.pending
        peer.sendto(f"PONG {probe} 1 1000 0\n".encode(), recipient)
        client.poll()
        assert client.address == peer.getsockname() and client.online
        assert not client.armed and client.pending == ""  # Discovery alone cannot drive.

        # After learning the peer, even a correctly numbered response from
        # another sender must not refresh health or replace the robot endpoint.
        client._ping()
        probe = next(reversed(client.probes))
        observed_at = client.last_pong_at
        stranger.sendto(f"PONG {probe} 1 1100 0\n".encode(), recipient)
        client.poll()
        assert client.address == peer.getsockname() and client.last_pong_at == observed_at
        peer.sendto(f"PONG {probe} 1 1100 0\n".encode(), recipient)
        client.poll()
        assert client.last_pong_at >= observed_at and client.online
    finally:
        client.close()
        peer.close()
        stranger.close()
    print("PASS hotspot discovery: fresh nonce required, actual IP learned, endpoint locked, no ARM from discovery")


def check_wifi_joiner():
    from types import SimpleNamespace
    from sumo.wifi import WifiJoiner, visible_ssids, connected_ssid, ROBOT_SSID
    assert visible_ssids("SSID 1 : Other\nSSID 2 : SumoRobot\nBSSID 1 : aa:bb") == {"Other", "SumoRobot"}
    assert connected_ssid("    BSSID : aa:bb\n    SSID : SumoRobot\n") == "SumoRobot"
    if sys.platform == "win32":
        for visible in (False, True):
            calls = []
            worker = WifiJoiner(ROOT / "robot_wifi.xml")

            def fake_netsh(*args):
                calls.append(args)
                output = "SSID : Other" if args == ("show", "interfaces") else (
                    f"SSID 1 : {ROBOT_SSID}" if visible else f"SSID 1 : {ROBOT_SSID}Other")
                return SimpleNamespace(returncode=0, stdout=output)

            worker._netsh = fake_netsh
            worker._run()
            assert worker.done.is_set()
            assert any(args[0] == "connect" for args in calls) == visible
            assert any(args[0] == "add" for args in calls) == visible
    print("PASS Wi-Fi helper: exact robot SSID matching; no network changes when AP is absent (mocked commands)")


def check_gui():
    from PySide6 import QtCore, QtWidgets
    from sumo.gui import MainWindow
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    previous_locale = QtCore.QLocale()
    QtCore.QLocale.setDefault(QtCore.QLocale("th_TH"))
    window = MainWindow(demo=True)
    for field in [*window.fields.values(), window.port, window.own_offset, window.enemy_offset]:
        assert field.locale().zeroDigit() == "0"
        assert field.locale().decimalPoint() == "."
        assert not any("\u0e50" <= character <= "\u0e59" for character in field.text())
    window.timer.stop()
    window.show()
    app.processEvents()
    window.tick()
    window.start_control()
    assert window.running and not window.link.connected
    window.tick()
    for _ in range(5):
        window.previous_tick=time.monotonic()-.05
        window.tick()
    assert len(window.arena.trail)>=3  # Mutable simulation poses must not suppress new samples.
    app.processEvents()
    artifacts = ROOT / "artifacts"
    artifacts.mkdir(exist_ok=True)
    assert window.grab().save(str(artifacts / "topview_preview.png"))
    window.stop_button.click()
    assert not window.running and window.plan.left == window.plan.right == 0
    window.reset_simulation(True)
    window.start_button.click()
    assert window.running and window.planner.state == "PUSH"
    window.stop_control("verification complete")
    # Exercise the LIVE camera branch with recorded observations and a recording
    # link. Missing tags, duplicate IDs and stale camera delegate to onboard AUTO.
    class RecordedCamera:
        observation = None

        def snapshot(self):
            return self.observation

        def stop(self):
            pass

    class RecordingLink:
        error = ""
        armed = True
        protocol_version = 4
        stop_pending = False
        remote_stop = ""
        auto_requested = True
        remote_armed = False
        connected = True
        telemetry = {}
        sequence = 0
        online = True
        connection_state = "ONLINE"
        address = ("127.0.0.1", 4210)
        rtt_ms = 1.0
        last_pong_at = time.monotonic()

        def __init__(self):
            self.commands = []

        def poll(self):
            pass

        def healthy(self, now):
            return True

        def drive(self, left, right, assist=0, cap=255):
            self.commands.append((left, right, assist))

        def begin_arm(self):
            self.auto_requested=True
            self.armed=True

        def stop(self):
            self.commands.append("STOP")

        def close(self):
            self.stop()

    window.demo.setChecked(False)
    window.worker = RecordedCamera()
    window.link = RecordingLink()
    window.calibration = Calibration([[100, 100], [700, 100], [700, 700], [100, 700]], (800, 800), window.cfg.arena_width, window.cfg.arena_height)
    window.camera_canvas.calibration = window.calibration
    frame = np.zeros((800, 800, 3), np.uint8)
    detections = {
        17: np.float32([[280, 480], [320, 480], [320, 520], [280, 520]]),
        42: np.float32([[480, 380], [520, 380], [520, 420], [480, 420]]),
    }
    window.worker.observation = Observation(frame, time.monotonic(), detections, set())
    window.tick()
    assert window.own_tag_id is None and window.enemy_tag_id is None
    window.own_id_box.setCurrentIndex(window.own_id_box.findData(17))
    assert (window.own_tag_id, window.enemy_tag_id) == (17, 42)
    # Fresh stationary observations show the opening route before Start,
    # including after the previous match, without sending motor commands.
    window.match_result=Plan("WIN","previous match")
    preview_commands=list(window.link.commands)
    for _ in range(3):
        window.worker.observation=Observation(frame,time.monotonic(),dict(detections),set())
        window.tick()
        assert not window.running and window.plan.state.startswith("PREVIEW / ")
        assert window.plan.left==window.plan.right==0 and len(window.plan.approach)>2
        assert window.plan.approach[-1]==window.poses[1].position
    assert window.link.commands==preview_commands
    window.match_result=None
    window.worker.observation.timestamp = time.monotonic()
    window.running = True
    window.tick()
    assert window.running and isinstance(window.link.commands[-1], tuple)
    assert window.arena.ids == (17, 42) and window.camera_canvas.own_id == 17
    assert "RTT" in window.connection_detail.text()
    assert window.connection_label.text().endswith("เชื่อมต่อแล้ว")
    # Reset display while live AUTO is running: no STOP/ARM/DRIVE is sent, and
    # the current planner remains intact. Only elapsed time and history reset.
    window.arena.trail.extend([Vec(-50,-50),Vec(-40,-40)])
    window.elapsed_started_at=time.monotonic()-73.4
    window.update_elapsed()
    assert "01:13." in window.elapsed_label.text()
    before_commands=list(window.link.commands)
    before_planner=window.planner
    before_plan=window.plan
    window.reset_display_button.click()
    assert window.running and window.link.armed and window.planner is before_planner and window.plan is before_plan
    assert window.link.commands==before_commands
    assert not window.arena.trail and window.camera_canvas.trail is window.arena.trail
    assert "00:00.0" in window.elapsed_label.text()
    # Trail must really render on both canvases, with a gap across a lost tag.
    observed=time.monotonic()
    window.record_trail(Pose(Vec(-60,-60),0,observed-.10),observed)
    window.record_trail(Pose(Vec(-20,0),0,observed-.05),observed)
    window.record_trail(None,observed)
    window.record_trail(Pose(Vec(60,70),0,observed),observed)
    assert None in window.arena.trail
    from PySide6 import QtGui
    def green_pixels(canvas):
        image=canvas.grab().toImage().convertToFormat(QtGui.QImage.Format.Format_RGBA8888)
        rgba=np.frombuffer(image.bits(),np.uint8).reshape(image.height(),image.width(),4)
        return np.count_nonzero(np.max(np.abs(rgba[:,:,:3].astype(int)-[103,240,160]),axis=2)<3)
    assert green_pixels(window.camera_canvas)>8
    assert green_pixels(window.arena)>8
    window.trail_checkbox.setChecked(False)
    assert green_pixels(window.camera_canvas)==0 and green_pixels(window.arena)==0
    window.trail_checkbox.setChecked(True)
    app.processEvents()
    window.log.appendPlainText("Verification with recorded camera and local recording link; not a physical robot")
    assert window.grab().save(str(artifacts / "wifi_status_preview.png"))
    assert window.grab().save(str(artifacts / "trail_reset_preview.png"))
    observation = Observation(frame,time.monotonic(),dict(detections),set())
    del observation.detections[42]
    window.worker.observation=observation
    window.tick()
    assert window.running and window.plan.state == "LOCAL_SEARCH"
    observation=Observation(frame,time.monotonic(),dict(detections),{17})
    window.worker.observation=observation
    window.tick()
    assert window.running and window.plan.state=="LOCAL_SEARCH"
    assert window.link.commands[-1][2]==2
    window.worker.observation=Observation(frame,time.monotonic(),dict(detections),set())
    window.tick()
    assert window.running and window.plan.state != "LOCAL_SEARCH"
    window.worker.observation=Observation(frame,time.monotonic()-1,dict(detections),set())
    window.tick()
    assert window.running and window.link.commands[-1][2]==2
    # Start at a body-to-edge estimate of 3 mm is accepted, not an error dialog.
    window.stop_control("start-edge regression")
    window.save_settings=lambda: None
    window.poses=(Pose(Vec(0,312),-math.pi/2,time.monotonic()),Pose(Vec(-100,0),0,time.monotonic()))
    window.start_control()
    assert window.arming
    window.worker.observation=Observation(frame,time.monotonic(),dict(detections),set())
    window.tick()
    assert window.running
    # Simulate transport loss while the camera is still producing observations.
    window.link.online=False
    window.last_wifi_retry=time.monotonic()  # Keep this recording test away from real Windows Wi-Fi.
    window.link.armed=False
    window.tick()
    assert window.running and window.plan.state=="LOCAL_LINK"
    window.link.online=True
    window.link.armed=True
    window.worker.observation=Observation(frame,time.monotonic(),dict(detections),set())
    window.tick()
    assert window.running
    window.stop_button.click()
    assert not window.running and window.link.commands[-1]=="STOP"
    frozen=window.elapsed_label.text()
    window.update_elapsed(time.monotonic()+10)
    assert window.elapsed_label.text()==frozen
    window.reset_display_button.click()
    assert not window.running and "00:00.0" in window.elapsed_label.text()
    window.close()
    app.processEvents()
    QtCore.QLocale.setDefault(previous_locale)
    print("PASS GUI: trail rendering on camera/map, noninterrupting reset, timer start/reset/freeze, AUTO fallbacks, vision recovery and explicit Stop")


if __name__ == "__main__":
    check_planning()
    check_recovery()
    check_search()
    check_vision()
    check_camera_recovery()
    check_link()
    check_auto_reconnect()
    check_discovery()
    check_wifi_joiner()
    check_gui()
    print("All top-view verification checks passed.")
