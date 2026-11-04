# -*- coding: utf-8 -*-
"""
从 ARM 的 (XYZ -> 关节角) 样本反推真实机构参数
未知量: TOOL_X, HZ(=BASE_H-TOOL_Z), L1, L2, A(j2偏置), B(j3偏置), C(j4偏置)
模型:
    rr = hypot(x,y);  dx = rr - TOOL_X;  dz = z - HZ
    d2 = dx^2 + dz^2
    c2 = (d2 - L1^2 - L2^2)/(2 L1 L2)   clamp
    th2 = acos(c2)
    k1 = L1 + L2 cos(th2); k2 = L2 sin(th2)
    th1 = atan2(dz,dx) - atan2(k2,k1)
    j1 = deg(atan2(y,x))
    j2 = A + deg(th1)
    j3 = B + deg(th2)
    j4 = C + deg(th1) + deg(th2)
"""
import json
import math
import sys

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

import numpy as np
from scipy.optimize import least_squares

S = json.load(open(r"D:\pl_detector\docs\ik_calib_samples.json", encoding="utf-8"))
print("样本数:", len(S))

D2R = 180.0 / math.pi


def model(p, x, y, z):
    TOOL_X, HZ, L1, L2, A, B, C = p
    rr = math.hypot(x, y)
    dx = rr - TOOL_X
    dz = z - HZ
    d2 = dx * dx + dz * dz
    denom = 2 * L1 * L2
    if denom <= 0:
        return None
    c2 = (d2 - L1 * L1 - L2 * L2) / denom
    if c2 > 1:
        c2 = 1.0
    if c2 < -1:
        c2 = -1.0
    th2 = math.acos(c2)
    k1 = L1 + L2 * math.cos(th2)
    k2 = L2 * math.sin(th2)
    th1 = math.atan2(dz, dx) - math.atan2(k2, k1)
    return (math.degrees(math.atan2(y, x)) * 256.0,
            A + math.degrees(th1) * 256.0,
            B + math.degrees(th2) * 256.0,
            C + math.degrees(th1) * 256.0 + math.degrees(th2) * 256.0)


def resid(p):
    r = []
    for s in S:
        m = model(p, s["x"], s["y"], s["z"])
        if m is None:
            r += [1e6] * 4
            continue
        for k in range(4):
            r.append(m[k] - s["j"][k])
    return np.array(r)


p0 = np.array([35.0, 87.0, 195.0, 115.0, -23040.0, 23040.0, 10240.0])
print("初值 residual RMS: %.1f" % np.sqrt(np.mean(resid(p0) ** 2)))

res = least_squares(resid, p0, method="lm", max_nfev=20000)
p = res.x
print()
print("拟合结果:")
names = ["TOOL_X", "HZ(=BASE_H-TOOL_Z)", "L1", "L2", "A(j2off)", "B(j3off)", "C(j4off)"]
for n, v in zip(names, p):
    print("  %-22s = %.3f" % (n, v))
print()
r = resid(p)
print("残差 RMS = %.2f deg256 (%.3f 度)" % (np.sqrt(np.mean(r ** 2)), np.sqrt(np.mean(r ** 2)) / 256))

print()
print("逐点核对:")
hdr = "  %-18s %-30s %-30s %s" % ("目标", "ARM", "模型", "误差")
print(hdr)
for s in S:
    m = model(p, s["x"], s["y"], s["z"])
    mi = [int(round(v)) for v in m]
    e = max(abs(a - b) for a, b in zip(mi, s["j"]))
    print("  %-18s %-30s %-30s %d" % (str((s["x"], s["y"], s["z"])), s["j"], mi, e))

# 输出成 RTL 参数（0.1mm 单位）
print()
print("=== 灌入 ik_solver.v 的参数（0.1mm 单位）===")
print("    P_TOOL_X = %d" % round(p[0] * 10))
print("    P_BASE_H - P_TOOL_Z = %d   (HZ)" % round(p[1] * 10))
print("    P_L1     = %d" % round(p[2] * 10))
print("    P_L2     = %d" % round(p[3] * 10))
print("    P_J2_OFF = %d" % round(p[4]))
print("    P_J3_OFF = %d" % round(p[5]))
print("    P_J4_OFF = %d" % round(p[6]))
