"""Join the existing shared hotspot in the background without opening a console.

Profile format: Microsoft Native Wi-Fi WPA2-Personal WLANProfile v1.
No scan match means no network change. Joining never arms or drives the robot.
"""

import os
from pathlib import Path
import re
import subprocess
import sys
import threading

ROBOT_SSID = "Kittifirst"
ROBOT_PROFILE = f"SumoTopView-{ROBOT_SSID}"


def visible_ssids(output):
    return set(re.findall(r"^\s*SSID\s+\d+\s*:\s*(.*?)\s*$", output, re.MULTILINE))


def connected_ssid(output):
    matches = re.findall(r"^\s*SSID\s*:\s*(.*?)\s*$", output, re.MULTILINE)
    return matches[0] if matches else None


class WifiJoiner:
    def __init__(self, profile):
        self.profile = Path(profile)
        self.halt = threading.Event()
        self.result = None
        self.done = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()

    def stop(self):
        self.halt.set()

    def _netsh(self, *args):
        netsh = Path(os.environ.get("WINDIR", "C:/Windows")) / "System32" / "netsh.exe"
        return subprocess.run([str(netsh), "wlan", *args], capture_output=True, text=True,
                              errors="replace", timeout=8,
                              creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))

    def _run(self):
        try:
            if sys.platform != "win32":
                self.result = f"เลือกเครือข่าย Wi-Fi {ROBOT_SSID} ในการตั้งค่าระบบ"
                return
            current = self._netsh("show", "interfaces")
            if current.returncode:
                self.result = f"Windows อ่าน Wi-Fi ไม่ได้ กรุณาต่อ {ROBOT_SSID} ในการตั้งค่า Wi-Fi"
                return
            if connected_ssid(current.stdout) == ROBOT_SSID:
                self.result = f"คอมต่อฮอตสปอต {ROBOT_SSID} แล้ว กำลังค้นหา ESP32 ในเครือข่ายเดียวกัน"
                return
            scan = self._netsh("show", "networks", "mode=ssid")
            if scan.returncode or ROBOT_SSID not in visible_ssids(scan.stdout):
                self.result = f"ยังไม่พบฮอตสปอต {ROBOT_SSID} เปิดฮอตสปอตมือถือ แล้วกดเชื่อมต่ออีกครั้ง"
                return
            if self.halt.is_set():
                return
            added = self._netsh("add", "profile", f"filename={self.profile}", "user=current")
            if added.returncode:
                self.result = f"เพิ่มโปรไฟล์ไม่ได้ กรุณาต่อ {ROBOT_SSID} ในการตั้งค่า Wi-Fi ด้วยรหัสที่ตั้งใน ESP32"
                return
            if self.halt.is_set():
                return
            joined = self._netsh("connect", f"name={ROBOT_PROFILE}", f"ssid={ROBOT_SSID}")
            self.result = (f"กำลังต่อฮอตสปอต {ROBOT_SSID} รอสถานะสีเขียวจาก ESP32" if joined.returncode == 0
                           else f"ต่อ {ROBOT_SSID} ไม่สำเร็จ กรุณาเลือกเครือข่ายในตั้งค่า Wi-Fi")
        except (OSError, subprocess.TimeoutExpired) as error:
            self.result = f"ตรวจ Wi-Fi ไม่สำเร็จ กรุณาเลือก {ROBOT_SSID} ในตั้งค่า Wi-Fi: " + str(error)
        finally:
            self.done.set()
