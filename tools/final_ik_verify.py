# -*- coding: utf-8 -*-
"""
最终验证：PL 硬件逆解 vs 浮点参考（参数来自 /etc/ik.conf）
参考模型与板卡上的 ik_zdt_pl 完全等价（Codex 已核对同一点输出一致）
"""
import math
import sys
import time

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

import paramiko

# /etc/ik.conf
TOOL_X, BASE_H, TOOL_Z = 35.0, 112.0, 35.0
L1, L2 = 200.0, 175.0
J1OFF, J2Z, J3Z, J4Z = 0, -23040, 23040, 1280
ELBOW = -1
HZ = BASE_H - TOOL_Z


def ik(x10, y10, z10):
    x, y, z = x10 / 10.0, y10 / 10.0, z10 / 10.0
    rr = math.hypot(x, y)
    dx = rr - TOOL_X
    dz = z - HZ
    d2 = dx * dx + dz * dz
    if d2 > (L1 + L2) ** 2 or d2 < (L1 - L2) ** 2:
        return None
    c2 = max(-1.0, min(1.0, (d2 - L1 * L1 - L2 * L2) / (2 * L1 * L2)))
    th2 = ELBOW * math.acos(c2)
    k1 = L1 + L2 * math.cos(th2)
    k2 = L2 * math.sin(th2)
    th1 = math.atan2(dz, dx) - math.atan2(k2, k1)

    def d256(rad):
        return int(round(rad * (180.0 / math.pi) * 256.0))

    a1, a2 = d256(th1), d256(th2)
    return (d256(math.atan2(y, x)) + J1OFF, J2Z + a1, J3Z + a2, J4Z + a1 + a2)


PTS = [
    (2500, 0, 1500), (2500, 1000, 1500), (2500, -1000, 1500),
    (2000, 2000, 1200), (2000, -2000, 1200),
    (3000, -1500, 2000), (1500, 0, 2500), (2800, 800, 1000),
    (2200, -2200, 1800), (1800, 1500, 2200), (2600, 0, 900),
    (2400, 1200, 1800), (2400, -1200, 1800), (2100, 0, 2000),
    (2700, 0, 1300), (2300, -800, 2100),
]

cli = paramiko.SSHClient()
cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
cli.connect("192.168.1.10", username="petalinux", password="1", timeout=15,
            allow_agent=False, look_for_keys=False)


def sudo(c, t=60):
    _, o, e = cli.exec_command("echo 1 | sudo -S sh -c \"%s\"" % c, timeout=t)
    return (o.read().decode() + e.read().decode()).replace("Password:", "").strip()


def rd(addr):
    v = int(sudo("devmem 0x%08X 32" % addr), 16)
    return v - (1 << 32) if v & 0x80000000 else v


def pl_ik(x, y, z):
    sudo("devmem 0x40000024 32 %d" % x)
    sudo("devmem 0x40000028 32 0x%08X" % (y & 0xFFFFFFFF))
    sudo("devmem 0x4000002C 32 %d" % z)
    sudo("devmem 0x40000030 32 1")
    time.sleep(0.02)
    out = []
    for i in (1, 2, 3, 4):
        sudo("devmem 0x40000034 32 %d" % i)
        out.append(rd(0x40000034))
    return out


print("=== PL 硬件逆解 vs 浮点参考（参数 = /etc/ik.conf）===")
print("  %-20s %-32s %-32s %s" % ("目标(0.1mm)", "PL 硬件", "参考", "误差"))
print("-" * 104)

ok = 0
mx = 0
for p in PTS:
    ref = ik(*p)
    if ref is None:
        print("  %-20s 超出可达范围" % (str(p),))
        continue
    got = pl_ik(*p)
    err = max(abs(a - b) for a, b in zip(got, ref))
    mx = max(mx, err)
    good = err < 64
    ok += good
    print("  %-20s %-32s %-32s %-4d %s"
          % (str(p), str(got), str(list(ref)), err, "OK" if good else "BAD"))

print("-" * 104)
print("  通过 %d / %d，最大误差 %d deg256 = %.3f 度" % (ok, len(PTS), mx, mx / 256.0))

cli.close()
