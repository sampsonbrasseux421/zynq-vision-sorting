# -*- coding: utf-8 -*-
"""
pad_arm3.py —— 手柄控制（丝滑版）

改进：
  1. 步长随摇杆力度变化（轻推 1.5mm，推到底 12mm）
  2. 摇杆数值低通滤波，方向不跳变
  3. 命令帧间隔降到 25ms（单次约 0.13s），动作更连贯
  4. 上一步没回来时，若摇杆已回中则丢弃累积，避免"甩过头"
"""
import ctypes
import ctypes.wintypes as wt
import math
import socket
import sys
import threading
import time

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

BOARD_IP = "192.168.1.10"
BOARD_PORT = 5000

X_MIN, X_MAX = 120, 380
Y_MIN, Y_MAX = -300, 300
Z_MIN, Z_MAX = 60, 220

DEADZONE = 0.18
STEP_MIN = 1.5      # 轻推时的步长（mm）
STEP_MAX = 12.0     # 推到底时的步长（mm）
FILTER = 0.35       # 低通滤波系数（越小越平滑，0.3~0.5 合适）
GAP_MS = 25         # 帧间隔（毫秒）


class GP(ctypes.Structure):
    _fields_ = [("wButtons", wt.WORD), ("bLeftTrigger", ctypes.c_ubyte),
                ("bRightTrigger", ctypes.c_ubyte), ("sThumbLX", ctypes.c_short),
                ("sThumbLY", ctypes.c_short), ("sThumbRX", ctypes.c_short),
                ("sThumbRY", ctypes.c_short)]


class ST(ctypes.Structure):
    _fields_ = [("dwPacketNumber", wt.DWORD), ("Gamepad", GP)]


BTN = {"A": 0x1000, "B": 0x2000, "X": 0x4000, "Y": 0x8000,
       "LB": 0x0100, "RB": 0x0200, "BACK": 0x0020,
       "UP": 0x0001, "DOWN": 0x0002, "LEFT": 0x0004, "RIGHT": 0x0008}


def load_xinput():
    for name in ("XInput1_4.dll", "XInput1_3.dll", "XInput9_1_0.dll"):
        try:
            return ctypes.WinDLL(name)
        except OSError:
            continue
    return None


def send_cmd(cmd, timeout=6.0):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(timeout)
        s.connect((BOARD_IP, BOARD_PORT))
        s.sendall((cmd + "\n").encode("ascii"))
        s.shutdown(socket.SHUT_WR)
        data = b""
        try:
            while True:
                b = s.recv(4096)
                if not b:
                    break
                data += b
        except socket.timeout:
            pass
        s.close()
        return data.decode("ascii", "replace").strip().split("\n")[-1]
    except Exception as e:
        return "ERR %s" % e


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class Arm:
    def __init__(self):
        self.pos = [250.0, 0.0, 150.0]
        self.busy = False
        self.lock = threading.Lock()
        self.send_count = 0
        self.lat = 0.0

    def step(self, dx, dy, dz):
        with self.lock:
            if self.busy:
                return False
            tgt = [clamp(self.pos[0] + dx, X_MIN, X_MAX),
                   clamp(self.pos[1] + dy, Y_MIN, Y_MAX),
                   clamp(self.pos[2] + dz, Z_MIN, Z_MAX)]
            if (abs(tgt[0]-self.pos[0]) < 0.3 and abs(tgt[1]-self.pos[1]) < 0.3
                    and abs(tgt[2]-self.pos[2]) < 0.3):
                return False
            self.busy = True
        cmd = "%d %d %d" % (round(tgt[0]), round(tgt[1]), round(tgt[2]))

        def run():
            t0 = time.time()
            resp = send_cmd(cmd)
            with self.lock:
                self.pos = tgt
                self.busy = False
                self.send_count += 1
                self.lat = (self.lat * 0.8 + (time.time() - t0) * 0.2)
            if not resp.startswith("OK"):
                print("  [WARN] %s -> %s" % (cmd, resp), flush=True)

        threading.Thread(target=run, daemon=True).start()
        return True


def main():
    lib = load_xinput()
    if lib is None:
        print("!! 找不到 XInput")
        return 1

    st = ST()
    idx = None
    for i in range(4):
        if lib.XInputGetState(i, ctypes.byref(st)) == 0:
            idx = i
            break
    if idx is None:
        print("!! 没有检测到已连接的手柄")
        return 1

    print("手柄在端口 %d" % idx)
    print("=== 板卡自检 ===")
    print("  PING -> %s" % send_cmd("PING"))
    print("  HOME -> %s" % send_cmd("HOME"))
    print("  张爪 -> %s" % send_cmd("CLAW 1150"))
    print("  GAP  -> %s" % send_cmd("GAP %d" % GAP_MS))
    print("  JOG  -> %s" % send_cmd("JOG ON"))
    time.sleep(0.8)

    arm = Arm()
    filt = [0.0, 0.0, 0.0]
    prev_btn = 0
    last_print = 0.0
    pkt_last = -1

    print()
    print("=== 手柄控制（丝滑模式）===")
    print("  左摇杆 X/Y = 平面    右摇杆 上下 = Z")
    print("  A 合爪  B 张爪  X 回零  LB 微调  RB 快速  Back 退出")
    print("  步长随力度变化 %.1f~%.0f mm，帧间隔 %d ms" % (STEP_MIN, STEP_MAX, GAP_MS))
    print()

    try:
        while True:
            if lib.XInputGetState(idx, ctypes.byref(st)) != 0:
                time.sleep(0.05)
                continue
            g = st.Gamepad

            # 低通滤波
            raw = [g.sThumbLX / 32767.0, g.sThumbLY / 32767.0, g.sThumbRY / 32767.0]
            for i in range(3):
                filt[i] = filt[i] * (1 - FILTER) + raw[i] * FILTER
            lx, ly, rz = filt

            def dz(v):
                return 0.0 if abs(v) < DEADZONE else v
            lx, ly, rz = dz(lx), dz(ly), dz(rz)

            mag = math.sqrt(lx * lx + ly * ly + rz * rz)
            if mag > 0:
                # 步长随力度线性放大
                amp = min(mag / 0.9, 1.0)
                step = STEP_MIN + (STEP_MAX - STEP_MIN) * (amp ** 1.6)
                if g.wButtons & BTN["LB"]:
                    step *= 0.35
                if g.wButtons & BTN["RB"]:
                    step *= 1.8
                n = mag or 1.0
                arm.step(lx / n * step, ly / n * step, rz / n * step)

            pressed = g.wButtons & ~prev_btn
            if pressed & BTN["A"]:
                print("  合爪 -> %s" % send_cmd("CLAW 530"), flush=True)
            if pressed & BTN["B"]:
                print("  张爪 -> %s" % send_cmd("CLAW 1150"), flush=True)
            if pressed & BTN["X"]:
                print("  HOME -> %s" % send_cmd("HOME"), flush=True)
                with arm.lock:
                    arm.pos = [250.0, 0.0, 150.0]
                    filt = [0.0, 0.0, 0.0]
            if pressed & BTN["BACK"]:
                print("  退出")
                break
            prev_btn = g.wButtons

            now = time.time()
            if now - last_print > 1.0:
                last_print = now
                with arm.lock:
                    p, lat, cnt = list(arm.pos), arm.lat, arm.send_count
                print("  位置(%4.0f,%4.0f,%4.0f)  延迟%.2fs  已发%d条"
                      % (p[0], p[1], p[2], lat, cnt), flush=True)

            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        send_cmd("JOG OFF")
        send_cmd("GAP 60")
        print("已退出（JOG 关闭，帧间隔恢复 60ms）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
