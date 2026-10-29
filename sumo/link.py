"""Nonblocking Wi-Fi UDP client for protocol v4 autonomous sessions.

Arming requires a fresh challenge and a matching session acknowledgement.
STOP invalidates that challenge; old ARM/DRIVE packets cannot restart motors.
"""

import secrets
import ipaddress
import socket
import time


class RobotLink:
    def __init__(self):
        self.device = None
        self.kind = ""
        self.address = None
        self.pending = ""
        self.buffer = b""
        self.session = 0
        self.sequence = 0
        self.armed = False
        self.last_rx = 0.0
        self.arm_started = 0.0
        self.telemetry = {}
        self.error = ""
        self.opened_at = 0.0
        self.last_pong_at = 0.0
        self.last_ping_at = 0.0
        self.rtt_ms = None
        self.remote_armed = False
        self.probes = {}
        self.probe_error = ""
        self.discovery = False
        self.target_port = 4210
        self.last_discovery_refresh = 0.0
        self.discovery_targets = []
        self.protocol_version = 0
        self.auto_requested = False
        self.stop_pending = False
        self.remote_stop = ""
        self.last_hello_at = self.last_stop_at = 0.0
        self.ever_armed = False

    @property
    def connected(self):
        return self.device is not None

    @property
    def online(self):
        return self.connected and self.last_pong_at > 0 and time.monotonic() - self.last_pong_at < 1.5

    @property
    def connection_state(self):
        if not self.connected:
            return "DISCONNECTED"
        if self.online:
            return "ONLINE"
        if self.last_pong_at:
            return "LOST"
        return "CONNECTING" if time.monotonic() - self.opened_at < 2 else "OFFLINE"

    def connect_udp(self, host, port=4210):
        # Discover a DHCP address on a shared hotspot, or use an explicit IPv4.
        discover = str(host).strip().lower() in {"", "auto"}
        address = None if discover else str(ipaddress.IPv4Address(host))
        self.close()
        self.device = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.device.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.device.setblocking(False)
        self.device.bind(("", 0))
        self.discovery = discover
        self.target_port = port
        self.last_discovery_refresh = 0
        self.discovery_targets = []
        self.address = None if discover else (address, port)
        self.kind = "UDP"
        self.error = ""
        self.opened_at = time.monotonic()
        self.last_pong_at = self.last_ping_at = 0
        self.rtt_ms = None
        self.probes = {}
        self.probe_error = ""
        self.protocol_version = 0
        self._ping()

    def _send(self, line):
        if not self.device:
            raise RuntimeError("Robot is not connected")
        if not self.address:
            raise RuntimeError("Robot IP has not been discovered yet")
        data = (line + "\n").encode("ascii")
        self.device.sendto(data, self.address)

    def _broadcast_addresses(self):
        targets = {"255.255.255.255"}
        try:
            from PySide6.QtNetwork import QNetworkInterface
            wifi, other = set(), set()
            for interface in QNetworkInterface.allInterfaces():
                if not interface.flags() & QNetworkInterface.InterfaceFlag.IsUp:
                    continue
                for entry in interface.addressEntries():
                    if not entry.ip().toIPv4Address():
                        continue
                    ip = ipaddress.IPv4Address(entry.ip().toString())
                    broadcast = entry.broadcast().toString()
                    if ip.is_loopback or ip.is_link_local or not broadcast:
                        continue
                    selected = wifi if interface.type() == QNetworkInterface.InterfaceType.Wifi else other
                    selected.add(broadcast)
            targets.update(wifi or other)
        except (ImportError, OSError, ValueError):
            pass
        return [(address, self.target_port) for address in sorted(targets)]

    def _ping(self):
        now = time.monotonic()
        self.last_ping_at = now
        # A DHCP address may change after reconnection. Rediscover only when no
        # control session is active, keeping all motor commands unicast.
        if self.discovery and self.address and not self.online and self.last_pong_at:
            self.address = None
            self.armed = False
            self.pending = ""
        self.probes = {token: sent for token, sent in self.probes.items() if now - sent < 1.5}
        token = secrets.randbelow(2**31 - 1) + 1
        self.probes[token] = now
        if self.address:
            targets = [self.address]
        else:
            if not self.discovery_targets or now - self.last_discovery_refresh > 2:
                self.discovery_targets = self._broadcast_addresses()
                self.last_discovery_refresh = now
            targets = self.discovery_targets
        succeeded = False
        for target in targets:
            try:
                self.device.sendto(f"PING {token}\n".encode("ascii"), target)
                succeeded = True
            except OSError as error:
                self.probe_error = str(error)
        if succeeded:
            self.probe_error = ""

    def begin_arm(self):
        if not self.online:
            raise RuntimeError("Wait for a verified Wi-Fi connection before Start")
        self.stop()
        self.auto_requested = True
        self.stop_pending = False
        self.remote_stop = ""
        self.ever_armed = False
        self.session = secrets.randbelow(2**31 - 1) + 1
        self.sequence = 0
        self.pending = "HELLO"
        self.arm_started = time.monotonic()
        self.last_hello_at = self.arm_started
        self.last_rx = 0
        self.telemetry = {}
        self.error = ""
        self._send(f"HELLO {self.session}")

    def drive(self, left, right, assist=0, cap=255):
        if not self.armed:
            raise RuntimeError("Robot has not acknowledged ARM")
        self.sequence += 1
        left, right = max(-255, min(255, int(left))), max(-255, min(255, int(right)))
        extra = f" {int(assist)} {max(1, min(255, int(cap)))}" if self.protocol_version >= 2 else ""
        try:
            self._send(f"DRIVE {self.session} {self.sequence} {left} {right} 350{extra}")
            return True
        except (OSError, RuntimeError) as error:
            self.error = str(error)
            return False

    def stop(self):
        self.auto_requested = False
        self.stop_pending = bool(self.device)
        self.armed = False
        self.pending = ""
        if self.device and self.address:
            try:
                # Repeated UDP STOP improves delivery; watchdog remains the backstop.
                for _ in range(3 if self.kind == "UDP" else 1):
                    self._send("STOP")
                self.last_stop_at = time.monotonic()
            except (OSError, RuntimeError) as error:
                self.error = str(error)

    def close(self):
        self.stop()
        if self.device:
            self.device.close()
        self.device = None
        self.address = None
        self.buffer = b""
        self.telemetry = {}
        self.last_pong_at = 0
        self.rtt_ms = None
        self.remote_armed = False
        self.probes = {}

    def healthy(self, now):
        return self.armed and self.online and now - self.last_rx < 0.8 and not self.error

    def poll(self):
        if not self.device:
            return
        try:
            for _ in range(32):
                try:
                    data, sender = self.device.recvfrom(2048)
                except (BlockingIOError, ConnectionResetError):
                    break
                if self.address is not None and sender != self.address:
                    continue
                for raw in data.splitlines():
                    line = raw.decode("ascii", errors="replace").strip()
                    if self.address is None and not line.startswith("PONG "):
                        continue
                    self._parse(line, sender)
            if time.monotonic() - self.last_ping_at >= 0.4:
                self._ping()
            now = time.monotonic()
            if self.stop_pending and self.address and now - self.last_stop_at >= 0.25:
                self._send("STOP")
                self.last_stop_at = now
            if self.auto_requested:
                if not self.online:
                    self.armed = False
                if self.pending and now - self.arm_started > 1.0:
                    self.pending = ""  # Retry automatically, never cancel AUTO.
                if self.online and not self.pending and (not self.armed or now - self.last_rx > 1.0):
                    if now - self.last_hello_at > 0.3:
                        self.pending = "HELLO"
                        self.arm_started = self.last_hello_at = now
                        self._send(f"HELLO {self.session}")
        except (OSError, RuntimeError, ValueError) as error:
            self.error = str(error)
            # The robot falls back to onboard states. Keep the Start intent and
            # session so a later PONG can resume without another user action.

    def _remote_state(self, remote_armed, action):
        if not remote_armed:
            if action in {"USER_STOP", "BOOT_STOP", "PWM_FAIL"} and self.auto_requested and self.ever_armed:
                self.remote_stop = action
                self.auto_requested = False
                self.pending = ""
            self.armed = False

    def _parse(self, line, sender=None):
        fields = line.split()
        if not fields:
            return
        try:
            if fields[0] == "PONG" and len(fields) in {5, 11}:
                token = int(fields[1])
                sent = self.probes.get(token)
                now = time.monotonic()
                if (sent is not None and now - sent < 1.5 and fields[2] in {"1", "2", "3", "4"}
                        and fields[4] in {"0", "1"} and int(fields[3]) >= 0
                        and sender is not None and sender[1] == self.target_port):
                    self.probes.pop(token, None)
                    if self.address is None:
                        if not self.discovery:
                            return
                        self.address = sender
                    self.remote_armed = bool(int(fields[4]))
                    self.protocol_version = int(fields[2])
                    self.rtt_ms = (now - sent) * 1000
                    self.last_pong_at = now
                    self.probe_error = ""
                    self.error = ""
                    if len(fields) == 11:
                        self._remote_state(int(fields[4]), fields[8])
                        if self.stop_pending and fields[4] == "0":
                            self.stop_pending = False
                    # Read hardware before Start too, without arming or granting
                    # command authority. Armed health still requires real TEL.
                    if len(fields) == 11 and not self.armed and not self.pending:
                        self.telemetry = dict(sequence=self.sequence, armed=int(fields[4]),
                                              ir=int(fields[5]), floor_left=int(fields[6]),
                                              floor_right=int(fields[7]), action=fields[8],
                                              left=int(fields[9]), right=int(fields[10]), uptime=int(fields[3]))
            elif fields[0] == "READY" and len(fields) == 4 and self.pending == "HELLO":
                if self.auto_requested and int(fields[1]) == self.session and fields[3] in {"1", "2", "3", "4"}:
                    self.protocol_version = int(fields[3])
                    self._send(f"ARM {self.session} {int(fields[2])}")
                    self.pending = "ARM"
            elif fields[0] == "ARMED" and len(fields) == 2 and self.pending == "ARM":
                if self.auto_requested and int(fields[1]) == self.session:
                    self.armed, self.pending = True, ""
                    self.ever_armed = True
                    self.last_rx = time.monotonic()
                    self.error = ""
            elif fields[0] == "RESUMED" and len(fields) == 4 and self.pending == "HELLO":
                if self.auto_requested and int(fields[1]) == self.session and fields[3] == "4":
                    self.sequence = max(self.sequence, int(fields[2]))
                    self.protocol_version = 4
                    self.armed, self.pending = True, ""
                    self.ever_armed = True
                    self.last_rx = time.monotonic()
                    self.error = ""
            elif fields[0] == "STOPPED" and len(fields) == 1:
                self.stop_pending = False
            elif fields[0] == "TEL" and len(fields) == 11 and int(fields[1]) == self.session:
                self.last_rx = time.monotonic()
                self.telemetry = dict(sequence=int(fields[2]), armed=int(fields[3]), action=fields[4],
                                      left=int(fields[5]), right=int(fields[6]), floor_left=int(fields[7]),
                                      floor_right=int(fields[8]), ir=int(fields[9]), uptime=int(fields[10]))
                self.error = ""
                self._remote_state(self.telemetry["armed"], fields[4])
            elif fields[0] == "ERROR" and len(fields) >= 3 and int(fields[1]) == self.session:
                self.error = " ".join(fields[2:])
        except (ValueError, IndexError):
            # Boot logs / corrupted telemetry cannot authorize movement.
            return
