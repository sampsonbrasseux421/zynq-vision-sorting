#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""pc_yolo_send.py - PC: camera + YOLO -> arm coords -> TCP to board

Keys: S = grab/next, H = home, B = belt prediction on/off, Q/E = close/open, ESC = quit
"""
import ast, cv2, json, socket, threading, time, sys, os, math
from bisect import bisect_right
import numpy as np
from ultralytics import YOLO
from fb_stream import FBStream
from claw_servo_math import (is_converged, jacobian_from_axis_probes,
                             pixel_error, pixel_error_to_xy,
                             update_jacobian_secant, world_delta_to_pixel)

# ======================= config =======================
MODEL_PATH = r"D:\deeplearning\color_blocks_20261026\best.pt"
CAM_ID     = 0
FRAME_W, FRAME_H = 1280, 720
BOARD_IP   = "192.168.1.10"

# ---- 硬件 OSD 仪表盘 ----
OSD_STATS = {"ok": 0, "fault": 0}

# ---- VLM 任务级决策（异步，CPU 推理约 10s，只影响下一次抓取）----
VLM_PREFER = [None]      # VLM 建议优先抓的颜色
VLM_REASON = [""]        # 建议理由（显示用）
try:
    from vlm_advisor import VlmAdvisor
    VLM = VlmAdvisor(enabled=True)
    VLM.start()
except Exception as _e:
    print("[VLM] 未启用:", _e, flush=True)
    VLM = None
# ---- HDMI UI 数据槽 ----
TARGET_UI = [None, None, None, None]   # target, center, lead, cycle
BOX_COUNT = [0, 0]                     # 左盒 / 右盒 计数
OSD_STATE = ["IDLE"]
OSD_LAST  = [0.0]
BOARD_PORT = 5000
CLAW_SERVO_CAL_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   "claw_servo_calibration.json")

# ---- 抓取/放置动作参数（板端协议：goto "X Y Z" / "CLAW <us>"）----
GRIP_OPEN_US  = 2500      # 张（实际从 grip_open.txt 读）
GRIP_CLOSE_US = 1560      # 合（默认；实际从 grip_close.txt 读）
GRAB_Z        = 90        # 抓取高度：上抬 5mm

LIFT_Z        = 200       # 接近高度：抬高，让大臂最后下放
PLACE_ENABLE  = False     # 只抓不搬：下降 -> 合爪 -> 抬起
PLACE_X       = 257       # 放置点 X(mm)
PLACE_Y       = -204      # 放置点 Y(mm)
# 运行时微调(mm)：预览界面按方向键调整，作用于所有抓取点
TRIM_X = 0
TRIM_Y = 0
FORE_AFT_BIAS_MM = -20.0  # measured miss: target is 20 mm too far from the operator
TRIM_R = 0        # mm，沿手臂方向：正=伸出，负=收回（不会带偏底座转角）
TRIM_ANG = 0      # mm，横向：正=左，负=右
RIGHT_X_BIAS_MM = 10.0        # 右侧(Y<0)机械臂额外前伸(mm)，越靠右越多
MIDDLE_TANGENTIAL_MM = -10.0   # 中间区域的底座转角补偿（负=向右 1cm）
LEFT_SIDE_TANGENTIAL_CORRECTION_MM = 20.0   # 正=底座向左转，最边缘 2cm 弧长
LEFT_SIDE_FULL_CORRECTION_Y_MM = 210.0


def left_side_tangential_correction_mm(y):
    """底座旋转补偿（弧长 mm，正值 = 逆时针/向左转）
    随 |Y| 增大而增大，左右两侧同向。
    X/Y 会按圆弧一起变，这才是"底座转角"，不是"伸长"。"""
    _ay = abs(float(y))
    t = min(max(_ay / LEFT_SIDE_FULL_CORRECTION_Y_MM, 0.0), 1.0)
    weight = t * t * (3.0 - 2.0 * t)
    return MIDDLE_TANGENTIAL_MM + LEFT_SIDE_TANGENTIAL_CORRECTION_MM * weight


SEG_DOWN_WAIT = 3.5       # 下降/抬起等待(秒)
SEG_MOVE_WAIT = 4.0       # 横移到放置点等待(秒)
SEG_CLAW_WAIT = 1.5       # 合/开爪等待(秒)

# 分盒角度(deg256)：45° = 45*256
BOX_LEFT_45 = 19200       # 黄/蓝盒：底座左转 75°
BOX_RIGHT_45 = -19200     # 红盒：底座右转 75°
BOX_BLACK = 28160         # 黑盒：底座左转 110°
BOX_GREEN = -28160        # 绿盒：底座右转 110°（与黑对立）
PLACE_ANGLE = [0.0]       # 本次放盒的底座角度(度)
READY_POS  = (250.0, 0.0)   # 待命位 XY（与板端 ik.conf 的 READY_X/Y 一致）
BOX_XY     = [0.0, 0.0]       # 最近一次盒子位置 XY
PLACE_R    = 200.0           # 与板端 ik.conf 的 PLACE_R 一致
LAST_BASE = [0.0]         # 底座当前角度(度)，放盒/回零时更新
SAFE_SWING_DEG = 0.0    # 0 = 每次都先转底座再下臂
SAFE_MID_X = 160
SAFE_MID_Y = 0
SAFE_MID_Z = 220
SAFE_MID_WAIT = 0.9
PIVOT_R = 0.55         # 收拢位半径比例
PIVOT_WAIT = 0.9       # 转底盘等待(秒)
AUTO_RECORD = True        # 启动就自动录像
RECORD_ON = [False]       # v 键：录像开关
REC_WRITER = [None]       # 录像写入器
REC_PATH = [None]
REC_START = [0.0]
AUTO_RUN = [False]
T_MARK = [0.0]
GRAB_OK = [False]
CLAW_STATE = [None]        # 上次发出的爪脉宽，避免重复下发         # 上一次 S 是否真的发出了下降命令
AUTO_SEQ = ['grab', 'close', 'place', 'open', 'ready']   # 抓 -> 合爪 -> 送盒 -> 松爪 -> 等下一抓
AUTO_CLOSE_WAIT = 1.9      # 下降命令后等多久合爪(秒)；下降命令本身已阻塞约1.05s
AUTO_CLOSE_TIMEOUT = 3.5   # 等爪子出现的最长时间(秒)
AUTO_PLACE_WAIT = 1.4      # 送盒后等到位(秒)
AUTO_NEXT = [0.0]
AUTO_IDX = [0]
FOLD_WAIT = 1.6         # 折回零位等待(秒)，FOLD 只动手臂不动舵机
TARGET_CLASSES = ["red", "yellow", "black", "green"]   # 红、黄、黑、绿
CONF_TH = 0.6
USE_PL_DET = False         # True = 用板端 PL 硬件检测；False = 用 PC 的 YOLO
MODEL_REF = [None]          # 指向检测模型，供推流回调使用
PL_MIN_CNT = 40            # PL 结果的最小采样点数（过滤噪声）
PL_FULL_CNT = 700.0        # 满面积采样点数，用于折算置信度
PL_MIN_FILL = 0.35         # 最低填充率：真物块 ~1.0，散射噪声 <0.1
PL_PUSH_WAIT = 0.10        # 推帧后等待板子处理的时间(秒)
ROI_Y_MIN, ROI_Y_MAX = 120, 690

# ---- 4 point homography (pixel -> arm mm) ----
H = np.array([
    [0.0778211146, -0.6933763345, 684.836817],
    [-1.0734239107, -0.2341164233, 761.855225],
    [0.000304102188, 0.001687322637, 1.0],
])

# ---- 10x5 measured offset grid (mm), sampled at X_SPLIT/Y_SPLIT ----
GRID = [
    [(0, 0), (0, 0), (0, 0), (0, 0), (0, 0)],
    [(0, 0), (0, 0), (0, 0), (0, 0), (0, 0)],
    [(0, 0), (0, 0), (0, 0), (0, 0), (0, 0)],
    [(0, 0), (0, 0), (0, 0), (0, 0), (0, 0)],
    [(0, 0), (0, 0), (0, 0), (0, 0), (0, 0)],
    [(0, 0), (0, 0), (0, 0), (0, 0), (0, 0)],
    [(0, 0), (0, 0), (0, 0), (0, 0), (0, 0)],
    [(0, 0), (0, 0), (0, 0), (0, 0), (0, 0)],
    [(0, 0), (0, 0), (0, 0), (0, 0), (0, 0)],
    [(0, 0), (0, 0), (0, 0), (0, 0), (0, 0)],
]

X_SPLIT = [185.0, 225.0, 262.0, 295.0, 330.0]          # 5 列：近 -> 远
Y_SPLIT = [-300.0, -233.33333333333331, -166.66666666666666, -100.0, -33.333333333333314, 33.33333333333337, 100.0, 166.66666666666669, 233.33333333333337, 300.0]          # 10 行：右 -> 左
NROW, NCOL = len(Y_SPLIT), len(X_SPLIT)
X_LO, X_HI = X_SPLIT[0], X_SPLIT[-1]
Y_LO, Y_HI = Y_SPLIT[0], Y_SPLIT[-1]

GRID3_X = [175.0, 260.0, 345.0]
GRID3_Y = [-210.0, 0.0, 210.0]
GRID3_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "grid_result.txt")


def load_grid3(path=GRID3_FILE):
    try:
        with open(path, "r", encoding="utf-8") as f:
            rows = [ast.literal_eval(line) for line in f if line.strip()]
        if len(rows) != 3 or any(len(row) != 3 for row in rows):
            raise ValueError("expected 3 rows x 3 columns")
        result = []
        for row in rows:
            parsed = []
            for pair in row:
                if len(pair) != 2:
                    raise ValueError("each offset must contain dX,dY")
                dx, dy = float(pair[0]), float(pair[1])
                if not math.isfinite(dx) or not math.isfinite(dy):
                    raise ValueError("offsets must be finite")
                parsed.append((dx, dy))
            result.append(parsed)
        return result
    except FileNotFoundError:
        return [[(0.0, 0.0) for _ in GRID3_X] for _ in GRID3_Y]
    except (OSError, ValueError, SyntaxError, TypeError) as exc:
        print("WARNING: ignoring invalid 3x3 calibration %s: %s" % (path, exc), flush=True)
        return [[(0.0, 0.0) for _ in GRID3_X] for _ in GRID3_Y]


GRID3 = load_grid3()


def grid3_offset(X, Y):
    """Interpolate the saved 3x3 residual correction, clamping outside its nodes."""
    r0 = min(max(bisect_right(GRID3_Y, Y) - 1, 0), len(GRID3_Y) - 2)
    c0 = min(max(bisect_right(GRID3_X, X) - 1, 0), len(GRID3_X) - 2)
    fx = min(max((X - GRID3_X[c0]) / (GRID3_X[c0 + 1] - GRID3_X[c0]), 0.0), 1.0)
    fy = min(max((Y - GRID3_Y[r0]) / (GRID3_Y[r0 + 1] - GRID3_Y[r0]), 0.0), 1.0)
    weights = ((1 - fx) * (1 - fy), fx * (1 - fy), (1 - fx) * fy, fx * fy)
    offsets = (GRID3[r0][c0], GRID3[r0][c0 + 1],
               GRID3[r0 + 1][c0], GRID3[r0 + 1][c0 + 1])
    return tuple(sum(offset[i] * weight for offset, weight in zip(offsets, weights))
                 for i in range(2))


def _lerp(a, b, t):
    return a + (b - a) * t


def _axis(vals, t):
    n = len(vals)
    if n == 1:
        return vals[0]
    if t <= 0.0:
        return vals[0]
    if t >= 1.0:
        return vals[-1]
    f = t * (n - 1)
    i = int(f)
    if i >= n - 1:
        return vals[-1]
    return _lerp(vals[i], vals[i + 1], f - i)


GRID_SHARP = 2.0       # 锐化指数（越小越平滑，减少跨格跳变）
GRID_INTERP = True
USE_GRID_10X5 = True
USE_GRID3 = False


def cell_index_xy(X, Y):
    col = 0
    for i in range(NCOL - 1):
        if X > (X_SPLIT[i] + X_SPLIT[i + 1]) / 2.0:
            col = i + 1
    row = 0
    for i in range(NROW - 1):
        if Y > (Y_SPLIT[i] + Y_SPLIT[i + 1]) / 2.0:
            row = i + 1
    return row, col


def _weights(X, Y):
    """Return continuous bilinear weights for the enclosing calibration nodes."""
    r0 = min(max(bisect_right(Y_SPLIT, Y) - 1, 0), NROW - 2)
    c0 = min(max(bisect_right(X_SPLIT, X) - 1, 0), NCOL - 2)
    fx = (X - X_SPLIT[c0]) / max(1e-6, (X_SPLIT[c0 + 1] - X_SPLIT[c0]))
    fy = (Y - Y_SPLIT[r0]) / max(1e-6, (Y_SPLIT[r0 + 1] - Y_SPLIT[r0]))
    fx = min(max(fx, 0.0), 1.0)
    fy = min(max(fy, 0.0), 1.0)
    cells = [(r0, c0, (1 - fx) * (1 - fy)), (r0, c0 + 1, fx * (1 - fy)),
             (r0 + 1, c0, (1 - fx) * fy), (r0 + 1, c0 + 1, fx * fy)]
    ws = [w ** GRID_SHARP for _, _, w in cells]
    tot = sum(ws) or 1.0
    return cells, [w / tot for w in ws]


def grid_offset(X, Y):
    r, c = cell_index_xy(X, Y)
    if not GRID_INTERP:
        return float(GRID[r][c][0]), float(GRID[r][c][1])
    cells, ws = _weights(X, Y)
    dx = sum(GRID[rr][cc][0] * w for (rr, cc, _), w in zip(cells, ws))
    dy = sum(GRID[rr][cc][1] * w for (rr, cc, _), w in zip(cells, ws))
    return dx, dy


# ---- 二次多项式标定（由 fit_model.py 生成）----
USE_POLY = False     # True 时用多项式，忽略 H 和 GRID
POLY_X = [0, 0, 0, 0, 0, 0]   # X = a0 + a1 u + a2 v + a3 u^2 + a4 uv + a5 v^2
POLY_Y = [0, 0, 0, 0, 0, 0]


def poly_to_arm(u, v):
    t = (1.0, u, v, u * u, u * v, v * v)
    X = sum(c * k for c, k in zip(POLY_X, t))
    Y = sum(c * k for c, k in zip(POLY_Y, t))
    return X, Y


def pixel_to_arm_float(cx, cy):
    if USE_POLY:
        return poly_to_arm(float(cx), float(cy))
    p = H @ np.array([float(cx), float(cy), 1.0])
    X = p[0] / p[2]
    Y = p[1] / p[2]
    if USE_GRID_10X5:
        dx, dy = grid_offset(X, Y)
        X += dx
        Y += dy
    if USE_GRID3:
        dx3, dy3 = grid3_offset(X, Y)
        X += dx3
        Y += dy3
    X += TRIM_X + FORE_AFT_BIAS_MM
    Y += TRIM_Y
    if Y < 0:
        X += RIGHT_X_BIAS_MM * min(1.0, -Y / 250.0)   # 右侧额外前伸
    left_correction = left_side_tangential_correction_mm(Y)
    # Apply tangential corrections in polar coordinates so fore/aft trim stays independent.
    R = math.hypot(X, Y)
    if R > 1.0 and (TRIM_R or TRIM_ANG or left_correction):
        th = math.atan2(Y, X)
        R2 = R + TRIM_R
        th2 = th + (TRIM_ANG + left_correction) / max(R2, 1.0)
        X = R2 * math.cos(th2)
        Y = R2 * math.sin(th2)
    return float(X), float(Y)


def pixel_to_arm(cx, cy):
    X, Y = pixel_to_arm_float(cx, cy)
    return round(X), round(Y), GRAB_Z, (0, 0)

AUTO_DELAY = 10.0        # 闂備胶鍘ч〃搴㈢濠婂嫭鍙忛柍杞扮导閻旂厧鐏崇€规洖娲ㄩ、鍛攽椤曞棛鍒扮紒璇差儔閺屽苯顭ㄩ崼鐔峰壆濡炪倖姊婚弲顐﹀垂婵傚憡鈷掗柛灞剧⊕鐏忥附绻涢崼浣告处閺咁剟鏌涢鐘茬労婵炵》绻濋弻?# ======================================================

GRID_NAMES = ["right", "mid", "left"]


def cell_index(X, Y):
    i = 0 if Y < Y_SPLIT[0] else (2 if Y > Y_SPLIT[1] else 1)
    j = 0 if X < X_SPLIT[0] else (2 if X > X_SPLIT[1] else 1)
    return i, j




def osd_push(target=None, center=None, lead=None, state=None):
    """把状态推给 PL 硬件 OSD（6 行 x 32 字符）"""
    import time as _t
    now = _t.time()
    if now - OSD_LAST[0] < 0.4:      # 限速 2.5 Hz
        return
    OSD_LAST[0] = now
    if state is not None:
        OSD_STATE[0] = state
    L = []
    L.append(("STATE   " + OSD_STATE[0]).ljust(32))
    if target:
        L.append(("TARGET  X%d Y%d" % (int(target[0]), int(target[1]))).ljust(32))
    else:
        L.append("TARGET  --".ljust(32))
    if center:
        L.append(("CENTER  X%d Y%d" % (int(center[0]), int(center[1]))).ljust(32))
    else:
        L.append("CENTER  --".ljust(32))
    L.append(("BELT    %.2f MM/S" % BELT_SPEED).ljust(32))
    if lead is not None:
        L.append(("LEAD    %.2f S" % lead).ljust(32))
    else:
        L.append("LEAD    --".ljust(32))
    L.append(("OK %d   FAULT %d" % (OSD_STATS["ok"], OSD_STATS["fault"])).ljust(32))
    try:
        send_to_board("OSD " + "|".join(L) + "\n")
    except Exception:
        pass



# ==================== HDMI 仪表盘 UI ====================
UI_C_BG    = (24, 20, 16)      # 面板底 (BGR)
UI_C_LINE  = (79, 71, 55)      # 分隔线
UI_C_TITLE = (255, 229, 0)     # 青色标题
UI_C_TEXT  = (255, 255, 255)
UI_C_DIM   = (170, 170, 170)
UI_C_OK    = (118, 230, 0)     # 绿色
UI_C_ERR   = (82, 82, 255)     # 红色
UI_C_NUM   = (0, 214, 255)     # 黄色强调
UI_W, UI_H = 1024, 600
UI_BAR_TOP, UI_BAR_BOT = 42, 34
UI_SIDE_W = 232
UI_FONT  = cv2.FONT_HERSHEY_DUPLEX
UI_FONT2 = cv2.FONT_HERSHEY_SIMPLEX

_ui_ts = []


def _ui_fps():
    now = time.time()
    _ui_ts.append(now)
    while len(_ui_ts) > 30:
        _ui_ts.pop(0)
    if len(_ui_ts) < 2:
        return 0.0
    return (len(_ui_ts) - 1) / max(_ui_ts[-1] - _ui_ts[0], 1e-6)


def _panel(frame, x0, y0, x1, y1, alpha=0.80):
    ov = frame.copy()
    cv2.rectangle(ov, (x0, y0), (x1, y1), UI_C_BG, -1)
    cv2.addWeighted(ov, alpha, frame, 1 - alpha, 0, frame)
    cv2.rectangle(frame, (x0, y0), (x1, y1), UI_C_LINE, 1)


def _kv(frame, x, y, key, val, vcol=None, scale=0.55):
    cv2.putText(frame, key, (x, y), UI_FONT2, scale, UI_C_DIM, 1, cv2.LINE_AA)
    cv2.putText(frame, val, (x, y + 24), UI_FONT, scale + 0.10,
                vcol if vcol else UI_C_TEXT, 1, cv2.LINE_AA)


def draw_dashboard(frame, fps=0.0, target=None, center=None, lead=None,
                   cycle=None, box_l=0, box_r=0, board_ok=True):
    """在画面四周绘制仪表盘；frame 会被就地修改并返回。"""
    # ---------- 顶部栏 ----------
    _panel(frame, 0, 0, UI_W - 1, UI_BAR_TOP - 1, 0.85)
    cv2.putText(frame, "YOLO SORTING SYSTEM", (12, 28), UI_FONT,
                0.62, UI_C_TITLE, 1, cv2.LINE_AA)

    ok_c = UI_C_OK if board_ok else UI_C_ERR
    cv2.circle(frame, (352, 21), 6, ok_c, -1, cv2.LINE_AA)
    cv2.putText(frame, "BOARD" if board_ok else "OFFLINE",
                (364, 27), UI_FONT2, 0.5, ok_c, 1, cv2.LINE_AA)

    cv2.putText(frame, "%.1f FPS" % fps, (470, 27), UI_FONT, 0.55,
                UI_C_TEXT, 1, cv2.LINE_AA)

    auto = AUTO_RUN[0]
    mcol = UI_C_OK if auto else UI_C_DIM
    cv2.putText(frame, "AUTO" if auto else "MANUAL", (580, 27), UI_FONT,
                0.55, mcol, 1, cv2.LINE_AA)

    if REC_START[0] > 0:
        el = int(time.time() - REC_START[0])
        cv2.circle(frame, (720, 21), 6, UI_C_ERR, -1, cv2.LINE_AA)
        cv2.putText(frame, "REC %02d:%02d" % (el // 60, el % 60),
                    (732, 27), UI_FONT, 0.55, UI_C_ERR, 1, cv2.LINE_AA)

    # ---------- 右侧栏 ----------
    sx = UI_W - UI_SIDE_W
    _panel(frame, sx, UI_BAR_TOP, UI_W - 1, UI_H - UI_BAR_BOT - 1, 0.80)
    x = sx + 14
    y = UI_BAR_TOP + 34

    tgt = "X%d Y%d" % (int(target[0]), int(target[1])) if target else "--"
    ctr = "X%d Y%d" % (int(center[0]), int(center[1])) if center else "--"
    _kv(frame, x, y, "TARGET", tgt)
    y += 56
    _kv(frame, x, y, "CENTER", ctr)
    y += 56
    _kv(frame, x, y, "BELT", "%.2f mm/s" % BELT_SPEED)
    y += 56
    _kv(frame, x, y, "LEAD", ("%.2f s" % lead) if lead else "--")
    y += 62

    cv2.line(frame, (x, y - 28), (UI_W - 14, y - 28), UI_C_LINE, 1)

    _kv(frame, x, y, "SUCCESS", "%d" % OSD_STATS["ok"], UI_C_NUM, 0.62)
    y += 56
    _kv(frame, x, y, "FAULT", "%d" % OSD_STATS["fault"],
        UI_C_ERR if OSD_STATS["fault"] else UI_C_TEXT, 0.62)
    y += 56
    _kv(frame, x, y, "CYCLE", ("%.1f s" % cycle) if cycle else "--", UI_C_NUM, 0.62)
    y += 56
    _kv(frame, x, y, "BOX L/R", "%d / %d" % (box_l, box_r), UI_C_TEXT, 0.62)
    y += 48
    _vlm = VLM_PREFER[0] if VLM_PREFER is not None else None
    _kv(frame, x, y, "VLM PICK", (_vlm.upper() if _vlm else "--"),
        UI_C_OK if _vlm else UI_C_DIM, 0.58)
    if VLM_REASON and VLM_REASON[0]:
        y += 26
        _rs = VLM_REASON[0][:24]
        cv2.putText(frame, _rs, (x, y), UI_FONT2, 0.38, UI_C_DIM, 1, cv2.LINE_AA)

    # ---------- 底部栏 ----------
    _panel(frame, 0, UI_H - UI_BAR_BOT, UI_W - 1, UI_H - 1, 0.85)
    hint = "S grab   F auto   H home   Q/E claw   V rec   B belt   ESC quit"
    cv2.putText(frame, hint, (12, UI_H - 11), UI_FONT2, 0.46,
                UI_C_DIM, 1, cv2.LINE_AA)
    return frame


LAST_MOVE_DUR = [0.0]      # 上一条运动命令的实际阻塞耗时（秒）


def send_to_board(msg):
    _t0 = time.time()
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(6)
    try:
        s.connect((BOARD_IP, BOARD_PORT))
        s.sendall(msg.encode())
        s.shutdown(socket.SHUT_WR)
        response = bytearray()
        while True:
            chunk = s.recv(4096)
            if not chunk:
                break
            response.extend(chunk)
        if not response:
            print("send fail: empty board response")
            return False
        return True
    except Exception as e:
        print("send fail:", e)
        return False
    finally:
        # 只记录真正的运动命令，OSD/CLAW 等快命令不覆盖
        if not msg.startswith(("OSD", "CLAW", "PING", "DET", "GAP", "JOG")):
            LAST_MOVE_DUR[0] = time.time() - _t0
        s.close()


def _rec_release(*_a):
    """任何退出路径都先关掉录像文件，保证 MP4 索引写完"""
    try:
        if REC_WRITER[0] is not None:
            REC_WRITER[0].release()
            REC_WRITER[0] = None
            print("\n=== 录像已保存: %s ===" % REC_PATH[0], flush=True)
    except Exception:
        pass


def _install_exit_hooks():
    import atexit
    import signal
    atexit.register(_rec_release)

    def _sig(signum, _frame):
        _rec_release()
        os._exit(0)

    for _name in ("SIGINT", "SIGTERM", "SIGBREAK"):
        try:
            signal.signal(getattr(signal, _name), _sig)
        except Exception:
            pass


def wait_arrival(timeout=30):
    """等机械臂真正停稳（板端 ZDT 位置不再变化），返回 True/False"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        sock.connect((BOARD_IP, BOARD_PORT))
        sock.sendall(b"WAIT\n")
        sock.shutdown(socket.SHUT_WR)
        buf = b""
        while True:
            try:
                chunk = sock.recv(4096)
            except Exception:
                break
            if not chunk:
                break
            buf += chunk
        sock.close()
        txt = buf.decode(errors="ignore")
        ok = "ARRIVED" in txt
        print("   到位检测: %s" % ("已到位" if ok else "超时"), flush=True)
        return ok
    except Exception as e:
        print("   到位检测失败:", e, flush=True)
        return False


def send_motion_and_wait(command, timeout=30):
    """Dispatch one arm motion and wait until the board reports it has stopped."""
    if not send_to_board(command):
        return False
    return wait_arrival(timeout=timeout)




def focus_window(title="YOLO -> Board"):
    """把 OpenCV 窗口拉到最前并取得焦点（Windows）"""
    try:
        import ctypes
        h = ctypes.windll.user32.FindWindowW(None, title)
        if h:
            ctypes.windll.user32.ShowWindow(h, 5)
            ctypes.windll.user32.SetForegroundWindow(h)
    except Exception:
        pass


def handle_trim(k):
    """任意状态下都可用的微调键：l/j 改 X，i/k 改 Y"""
    g = globals()
    if k == ord("l"):
        g["TRIM_X"] += 5
    elif k == ord("j"):
        g["TRIM_X"] -= 5
    elif k == ord("i"):
        g["TRIM_Y"] += 5
    elif k == ord("k"):
        g["TRIM_Y"] -= 5
    else:
        return False
    print("TRIM", g["TRIM_X"], g["TRIM_Y"], flush=True)
    return True


GRIP_CLOSE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "grip_close.txt")
GRIP_OPEN_FILE  = os.path.join(os.path.dirname(os.path.abspath(__file__)), "grip_open.txt")
GRIP_BACK_FILE  = os.path.join(os.path.dirname(os.path.abspath(__file__)), "grip_backoff.txt")
GRIP_HOLD_BACK  = 40      # 夹住后回退多少 us（防堵转抖动）

def _pulse_from_file(path, default):
    try:
        with open(path, "r") as f:
            v = int(f.read().strip())
        return max(500, min(2500, v))
    except Exception:
        return default


def handle_claw(k):
    """q=合爪  e=松爪"""
    if k == ord("q"):
        c = _pulse_from_file(GRIP_CLOSE_FILE, GRIP_CLOSE_US)
        send_to_board("CLAW %d\n" % c)
        print("CLAW close -> %dus" % c, flush=True)
        return True
    if k == ord("e"):
        send_to_board("CLAW %d\n" % _pulse_from_file(GRIP_OPEN_FILE, GRIP_OPEN_US))
        print("CLAW open  -> %dus" % _pulse_from_file(GRIP_OPEN_FILE, GRIP_OPEN_US), flush=True)
        return True
    return False


def handle_keys(k):
    return handle_trim(k) or handle_claw(k)


def home_all():
    """回零 = 四轴回零 + 爪子张开（复位状态）"""
    send_to_board("HOME\n")
    time.sleep(0.5)
    send_to_board("CLAW %d\n" % _pulse_from_file(GRIP_OPEN_FILE, GRIP_OPEN_US))


_VK_MAP = {0x61: "s", 0x62: "q", 0x64: "e", 0x65: "h", 0x51: "q", 0x45: "e"}
_prev_vk = {}

def _normalize_window_key(k):
    # Preserve keypad aliases and make Q/E work when Shift is held.
    return {49: 115, 50: 113, 52: 101, 53: 104,
            ord("Q"): ord("q"), ord("E"): ord("e")}.get(k, k)

def gk():
    """窗口按键优先；窗口无焦点时也读取主键盘 Q/E 和小键盘 1/2/4/5。"""
    k = cv2.waitKeyEx(1)
    if k != -1:
        # 同步按键状态，防止同一次按下被 GetAsyncKeyState 再报一次
        try:
            import ctypes
            u = ctypes.windll.user32
            for vk in _VK_MAP:
                _prev_vk[vk] = bool(u.GetAsyncKeyState(vk) & 0x8000)
        except Exception:
            pass
        return _normalize_window_key(k)
    try:
        import ctypes
        u = ctypes.windll.user32
        for vk, ch in _VK_MAP.items():
            down = bool(u.GetAsyncKeyState(vk) & 0x8000)
            was = _prev_vk.get(vk, False)
            _prev_vk[vk] = down
            if down and not was:
                return ord(ch)
    except Exception:
        pass
    return -1


# ---- A 方案：运动预测参数 ----
# 实测：物块走 520mm 约 55s。B 只切换软件预测，不控制传送带电机。
BELT_SPEED = 520.0 / 55.0
BELT_RUNNING = True   # 当前用户选择不停带抓取；停带测试时按 B 关闭预测
BELT_VX = 0.0
BELT_VY = +1.0        # 实测：画面从右往左 = Y 增大

APPROACH_WAIT = 1.2   # 到高位后等待(秒)
LEAD1 = 3.0           # 实测(-r1500/-A200)
LEAD_NEXT = 6.2        # 下一抓的高位粗接近提前时间；终点由最新图像重算
APPROACH_DY = -25      # 单向逼近：总是从右侧 25mm 靠近，抵消回差
SHORT_MOVE = 50       # 横移小于此值(mm)就用一步到位
LEAD2 = 2.6           # 第一抓的高位粗接近提前时间
VEL_WIN = 4.0        # 速度估计用的时间窗(秒)
hist = {}            # {颜色: [(t, X, Y)]} 每个块单独记轨迹


def belt_velocity():
    """Configured constant belt velocity, including the selected direction."""
    if not BELT_RUNNING:
        return 0.0, 0.0
    direction = math.hypot(BELT_VX, BELT_VY)
    if not math.isfinite(direction) or direction <= 0 or not math.isfinite(BELT_SPEED) or BELT_SPEED <= 0:
        raise ValueError("invalid conveyor speed or direction")
    return BELT_SPEED * BELT_VX / direction, BELT_SPEED * BELT_VY / direction


def is_moving(color, now, win=0.9, thr=2.0):
    """用最近 win 秒内该颜色的位移判断是否在动（只看距离，不看方向）"""
    h = [p for p in hist.get(color, []) if now - p[0] <= win]
    if len(h) < 3:
        return False
    dx = h[-1][1] - h[0][1]
    dy = h[-1][2] - h[0][2]
    return (dx * dx + dy * dy) ** 0.5 > thr


def est_vel(now, color=None):
    pts = [h for h in hist.get(color, []) if now - h[0] <= VEL_WIN]
    if len(pts) < 6:
        return 0.0, 0.0
    t0 = pts[0][0]
    ts = [p[0] - t0 for p in pts]
    xs = [p[1] for p in pts]
    ys = [p[2] for p in pts]
    n = float(len(ts))
    st = sum(ts); stt = sum(t * t for t in ts)
    sx = sum(xs); sy = sum(ys)
    stx = sum(t * x for t, x in zip(ts, xs))
    sty = sum(t * y for t, y in zip(ts, ys))
    den = n * stt - st * st
    if abs(den) < 1e-9:
        return 0.0, 0.0
    vx = (n * stx - st * sx) / den
    vy = (n * sty - st * sy) / den
    return vx, vy


# ---- 可抓范围（超出就不去够，等下一个块） ----
GRAB_X_MIN, GRAB_X_MAX = 120, 380
GRAB_Y_MIN, GRAB_Y_MAX = -300, 300

def on_belt(frame, cx, cy, hw, hh, thr=110.0):
    """块周围（传送带白色背景）足够亮才算真块，避免把夹爪/线缆当块"""
    if frame is None:
        return True
    h, w = frame.shape[:2]
    x0 = int(max(0, cx - hw * 1.7)); x1 = int(min(w, cx + hw * 1.7))
    y0 = int(max(0, cy - hh * 1.7)); y1 = int(min(h, cy + hh * 1.7))
    if x1 <= x0 or y1 <= y0:
        return True
    roi = cv2.cvtColor(frame[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY)
    return float(roi.mean()) > thr

HSV_RANGES = {
    "red":    [((0, 70, 50), (10, 255, 255)), ((170, 70, 50), (180, 255, 255))],
    "yellow": [((18, 80, 80), (38, 255, 255))],
    "blue":   [((95, 80, 50), (135, 255, 255))],
    "green":  [((40, 60, 50), (90, 255, 255))],
}


def color_centroid(frame, box, color):
    """在检测框内按颜色阈值找轮廓质心，比框中心更准（亚像素级）"""
    try:
        x1, y1, x2, y2 = [int(round(v)) for v in box]
        h, w = frame.shape[:2]
        x1 = max(0, x1); y1 = max(0, y1); x2 = min(w - 1, x2); y2 = min(h - 1, y2)
        if x2 - x1 < 6 or y2 - y1 < 6:
            return None
        roi = frame[y1:y2 + 1, x1:x2 + 1]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in HSV_RANGES.get(color, []):
            mask |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None
        c = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(c) < 80:
            return None
        M = cv2.moments(c)
        if M["m00"] <= 0:
            return None
        return (x1 + M["m10"] / M["m00"], y1 + M["m01"] / M["m00"])
    except Exception:
        return None


def detect_claw_near(frame, px_pred, py_pred, rad=130):
    """只在预测位置附近找两个蓝色爪垫，避免误检画面里其它蓝色物体"""
    try:
        h, w = frame.shape[:2]
        x0 = int(max(0, px_pred - rad)); x1 = int(min(w, px_pred + rad))
        y0 = int(max(0, py_pred - rad)); y1 = int(min(h, py_pred + rad))
        if x1 - x0 < 40 or y1 - y0 < 40:
            return None
        roi = frame[y0:y1, x0:x1]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array((95, 80, 50), np.uint8),
                           np.array((135, 255, 255), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cands = []
        for c in cnts:
            a = float(cv2.contourArea(c))
            if a < 250:
                continue
            m = cv2.moments(c)
            if m["m00"] <= 0:
                continue
            cands.append((a, x0 + m["m10"] / m["m00"], y0 + m["m01"] / m["m00"]))
        if len(cands) < 2:
            return None
        cands.sort(key=lambda t: t[0], reverse=True)
        a, b = cands[0], cands[1]
        center = ((a[1] + b[1]) / 2.0, (a[2] + b[2]) / 2.0)
        if abs(a[1] - b[1]) < 8:      # 两个爪垫应左右分开
            return None
        return {"left": (min(a[1], b[1]), a[2] if a[1] < b[1] else b[2]),
                "right": (max(a[1], b[1]), a[2] if a[1] > b[1] else b[2]),
                "center": center}
    except Exception:
        return None


def detect_claw_center(frame):
    """Detect the pair of blue jaw pads and return their midpoint in image pixels."""
    try:
        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lo = np.array((95, 80, 50), dtype=np.uint8)
        hi = np.array((135, 255, 255), dtype=np.uint8)
        mask = cv2.inRange(hsv, lo, hi)
        y0, y1 = int(h * 0.55), int(h * 0.88)
        roi = mask[y0:y1, :]
        roi = cv2.morphologyEx(roi, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        min_area = max(500.0, h * w * 0.0025)
        candidates = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < min_area:
                continue
            x, y, bw, bh = cv2.boundingRect(contour)
            moments = cv2.moments(contour)
            if moments["m00"] <= 0:
                continue
            cx = moments["m10"] / moments["m00"]
            cy = y0 + moments["m01"] / moments["m00"]
            candidates.append((area, cx, cy, x, y0 + y, bw, bh))

        if len(candidates) < 2:
            return None
        candidates.sort(key=lambda item: item[0], reverse=True)
        pair = None
        for i, a in enumerate(candidates):
            for b in candidates[i + 1:]:
                gap = abs(a[1] - b[1])
                min_w = max(1.0, min(a[5], b[5]))
                if gap < min_w:
                    continue
                vertical_gap = abs(a[2] - b[2])
                if vertical_gap > max(a[6], b[6]) * 1.5:
                    continue
                pair = sorted((a, b), key=lambda item: item[1])
                break
            if pair is not None:
                break
        if pair is None:
            return None

        left, right = pair
        center = ((left[1] + right[1]) / 2.0,
                  (left[2] + right[2]) / 2.0)
        return {"left": (left[1], left[2]), "right": (right[1], right[2]),
                "center": center,
                "boxes": ((left[3], left[4], left[5], left[6]),
                          (right[3], right[4], right[5], right[6]))}
    except Exception:
        return None


def detect_servo_target(frame, model, color, claw_center, max_distance_px=360.0):
    """Choose the same-color object nearest the visible claw, not highest confidence."""
    try:
        results = model(frame, verbose=False, conf=0.45)[0]
        candidates = []
        for box in results.boxes:
            if model.names[int(box.cls)] != color:
                continue
            conf = float(box.conf)
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            box_center = ((x1 + x2) / 2.0, (y1 + y2) / 2.0)
            center = color_centroid(frame, (x1, y1, x2, y2), color)
            if center is None:
                center = box_center
            else:
                occlusion_shift = math.hypot(center[0] - box_center[0],
                                             center[1] - box_center[1])
                center_limit = max(3.0, min(x2 - x1, y2 - y1) * 0.08)
                if occlusion_shift > center_limit:
                    center = box_center
            distance = math.hypot(center[0] - claw_center[0],
                                  center[1] - claw_center[1])
            candidates.append((distance, -conf, center[0], center[1]))
        if not candidates:
            return None
        candidates.sort()
        distance, neg_conf, cx, cy = candidates[0]
        if distance > max_distance_px:
            return None
        if len(candidates) > 1 and candidates[1][0] - distance < 18.0:
            return None
        return (-neg_conf, cx, cy)
    except Exception:
        return None


def detect_visual_pair(frame, model, color):
    claw = detect_claw_center(frame)
    if claw is None:
        return None
    target = detect_servo_target(frame, model, color, claw["center"])
    if target is None:
        return None
    return {"target": (target[1], target[2]), "claw": claw["center"],
            "confidence": target[0], "claw_detail": claw}


def capture_claw_center(cap, frames=3):
    points = []
    for _ in range(frames):
        ok, frame = cap.read()

        if not ok:
            continue
        claw = detect_claw_center(frame)
        if claw is not None:
            points.append(claw["center"])
    if len(points) < max(2, (frames + 1) // 2):
        return None
    return (float(np.median([p[0] for p in points])),
            float(np.median([p[1] for p in points])))


def capture_visual_pair(cap, model, color, frames=3):
    samples = []
    for _ in range(frames):
        ok, frame = cap.read()
        if not ok:
            continue
        pair = detect_visual_pair(frame, model, color)
        if pair is not None:
            samples.append(pair)
    if len(samples) < max(2, (frames + 1) // 2):
        return None
    target = (float(np.median([p["target"][0] for p in samples])),
              float(np.median([p["target"][1] for p in samples])))
    claw = (float(np.median([p["claw"][0] for p in samples])),
            float(np.median([p["claw"][1] for p in samples])))
    spread = max(math.hypot(p["claw"][0] - claw[0], p["claw"][1] - claw[1])
                 for p in samples)
    if spread > SERVO_MAX_CLAW_SPREAD_PX:
        return None
    return {"target": target, "claw": claw,
            "confidence": min(p["confidence"] for p in samples)}


def load_claw_alignment_offset(path=CLAW_SERVO_CAL_FILE, xy=None,
                               max_reference_distance_mm=80.0):
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        if not isinstance(data, dict):
            return None
        if data.get("version") != 1:
            return None
        if (int(data.get("grab_z_mm", -1)) != GRAB_Z or
                int(data.get("lift_z_mm", -1)) != LIFT_Z):
            return None
        offset = data.get("pixel_offset")
        if not isinstance(offset, list) or len(offset) != 2:
            return None
        offset = (float(offset[0]), float(offset[1]))
        if not all(math.isfinite(v) for v in offset):
            return None
        if xy is not None:
            reference = data.get("reference_xy_mm")
            if not isinstance(reference, list) or len(reference) != 2:
                return None
            ref_x, ref_y = float(reference[0]), float(reference[1])
            x, y = float(xy[0]), float(xy[1])
            if not all(math.isfinite(v) for v in (ref_x, ref_y, x, y)):
                return None
            if math.hypot(x - ref_x, y - ref_y) > max_reference_distance_mm:
                return None
        return offset
    except (OSError, ValueError, TypeError, OverflowError, json.JSONDecodeError):
        return None


LEAD_DESC_LEFT_BONUS = 4.8    # 左侧(Y>0)额外提前量(秒)
LEAD_DESC_RIGHT_BONUS = 2.6   # 右侧(Y<0)额外提前量(秒)
FIRST_EXTRA_LEAD_S = 1.6  # 第一抓额外提前量(秒)，补晚 1.5cm
NEXT_EXTRA_LEAD_S = 1.6   # 后续物块（第2、3…个）额外提前量(秒)，补 1.5cm
GRAB_CTX = {"extra": 0.0}
LEAD_DESC = 2.79      # 等效接触预测时间：原1.2s + 实测晚15mm / 9.45mm/s
SERVO_FRAMES = 3
SERVO_PROBE_MM = 5.0
SERVO_PROBE_WAIT = 0.8
SERVO_MAX_CLAW_SPREAD_PX = 12.0
SERVO_RETURN_TOL_PX = 18.0
SERVO_TOL_PX = 8.0
SERVO_GAIN = 0.7
SERVO_MAX_STEP_MM = 5.0
SERVO_MAX_TOTAL_MM = 35.0
SERVO_MAX_ITERS = 8

# Moving targets use one fresh observation and one terminal intercept, never the
# slow 5mm stationary correction loop. Times are monotonic host-side estimates.
MOVING_MAX_IMAGE_AGE_S = 3.5   # image age limit (s), allow pivot move
MOVING_TIMEOUT_S = 8.0
MOVING_ASSOCIATION_MM = 90.0   # association window (mm): allow pivot+approach time
MOVING_AMBIGUITY_MM = 8.0
MOVING_MAX_FINAL_XY_MM = 80.0


def predict_moving_grasp(target_xy, velocity, image_age_s):
    """Future XY at contact, compensating image age and calibrated contact lag."""
    x, y = map(float, target_xy)
    vx, vy = map(float, velocity)
    age = float(image_age_s)
    if not all(math.isfinite(v) for v in (x, y, vx, vy, age, LEAD_DESC)):
        raise ValueError("non-finite motion prediction")
    if not 0.0 <= age <= MOVING_MAX_IMAGE_AGE_S or LEAD_DESC <= 0.0:
        raise ValueError("stale image or invalid descent time")
    if not (GRAB_X_MIN <= x <= GRAB_X_MAX and GRAB_Y_MIN <= y <= GRAB_Y_MAX):
        raise ValueError("observed block is outside the workspace")
    # 两侧都随 |Y| 加提前量：物块越往左(Y 越大)爪子越容易落在后面，所以左侧加得更多
    _ay = min(1.0, abs(y) / 250.0)
    _coef = LEAD_DESC_LEFT_BONUS if y > 0 else LEAD_DESC_RIGHT_BONUS
    _bonus = _coef * _ay
    horizon = age + LEAD_DESC + _bonus + float(GRAB_CTX.get("extra", 0.0))
    x, y = x + vx * horizon, y + vy * horizon
    if not (GRAB_X_MIN <= x <= GRAB_X_MAX and GRAB_Y_MIN <= y <= GRAB_Y_MAX):
        raise ValueError("predicted grasp is outside the workspace")
    return x, y


def detect_moving_target(frame, model, color, expected_xy):
    """Associate the selected color with its predicted location, not confidence."""
    candidates = []
    for b in model(frame, verbose=False, conf=0.45)[0].boxes:
        if model.names[int(b.cls)] != color:
            continue
        x1, y1, x2, y2 = b.xyxy[0].tolist()
        center = color_centroid(frame, (x1, y1, x2, y2), color)
        px, py = center if center is not None else ((x1 + x2) / 2, (y1 + y2) / 2)
        if not ROI_Y_MIN < py < ROI_Y_MAX:
            continue
        if not on_belt(frame, px, py, (x2 - x1) / 2, (y2 - y1) / 2):
            continue
        x, y = pixel_to_arm_float(px, py)
        distance = math.hypot(x - expected_xy[0], y - expected_xy[1])
        if math.isfinite(distance) and distance <= MOVING_ASSOCIATION_MM:
            candidates.append((distance, x, y))
    candidates.sort()
    if not candidates:
        return None
    if len(candidates) > 1 and candidates[1][0] - candidates[0][0] < MOVING_AMBIGUITY_MM:
        return None
    return candidates[0][1:]


def refine_target_moving(cap, model, color, X_cmd, Y_cmd, vx, vy, target_observation):
    """Approach at lift height, then plan one intercept from a fresh frame.

    observation=(world_x, world_y, monotonic_capture_time) identifies the block.
    The caller immediately sends the returned XY at GRAB_Z; no extra high move,
    wait or repeat calibration may be inserted after the terminal prediction.
    """
    started = time.monotonic()
    try:
        ref_x, ref_y, ref_at = map(float, target_observation)
        values = (ref_x, ref_y, ref_at, X_cmd, Y_cmd, vx, vy)
        if not all(math.isfinite(v) for v in values):
            raise ValueError("invalid target reference")
        if not 0 <= started - ref_at <= MOVING_MAX_IMAGE_AGE_S:
            raise ValueError("stale selected target")
        if not (GRAB_X_MIN <= X_cmd <= GRAB_X_MAX and GRAB_Y_MIN <= Y_cmd <= GRAB_Y_MAX):
            raise ValueError("high approach is outside the workspace")
    except (TypeError, ValueError) as exc:
        print("   运动预测取消：%s" % exc, flush=True)
        return None

    if not send_to_board("%d %d %d\n" % (round(X_cmd), round(Y_cmd), LIFT_Z)):
        print("   运动预测取消：高位接近下发失败", flush=True)
        return None
    time.sleep(APPROACH_WAIT)
    if time.monotonic() - started >= MOVING_TIMEOUT_S:
        print("   运动预测取消：高位接近超时", flush=True)
        return None

    # Drain older camera buffers, then timestamp before the final acquisition.
    # A camera exposure timestamp is unavailable, so read-start is conservative.
    for _ in range(2):
        if not cap.grab():
            print("   运动预测取消：摄像头取帧失败", flush=True)
            return None
    captured_at = time.monotonic()
    ok, frame = cap.read()
    if not ok:
        print("   运动预测取消：摄像头读帧失败", flush=True)
        return None
    elapsed = captured_at - ref_at
    expected = (ref_x + vx * elapsed, ref_y + vy * elapsed)
    target = detect_moving_target(frame, model, color, expected)
    if target is None:
        print("   运动预测取消：原物块丢失、跳变或同色目标不明确；保持高位", flush=True)
        return None

    now = time.monotonic()
    age = now - captured_at
    if now - started >= MOVING_TIMEOUT_S:
        print("   运动预测取消：定位超时；保持高位", flush=True)
        return None
    try:
        x, y = predict_moving_grasp(target, (vx, vy), age)
    except ValueError as exc:
        print("   运动预测取消：%s；保持高位" % exc, flush=True)
        return None
    correction = (x - X_cmd, y - Y_cmd)
    if math.hypot(*correction) > MOVING_MAX_FINAL_XY_MM:
        print("   运动预测取消：末次横移超过 %.0fmm，不再追调" % MOVING_MAX_FINAL_XY_MM,
              flush=True)
        return None
    print("   运动预测：v=(%+.2f,%+.2f)mm/s 图像耗时=%.3fs 预计接触=%.2fs"
          " -> 一次截取(%d,%d,%d)，不再反复微调"
          % (vx, vy, age, LEAD_DESC, round(x), round(y), GRAB_Z), flush=True)
    return round(x), round(y), correction


def detect_one_stable(cap, model, color, frames=None):
    """连拍多帧，取中位数，抑制单帧噪声"""
    n = frames or SERVO_FRAMES
    xs, ys = [], []
    for _ in range(n):
        ok, f = cap.retrieve()
        if not ok:
            continue
        b = detect_one(f, model, color)
        if b is not None:
            xs.append(b[1])
            ys.append(b[2])
    if not xs:
        return None
    xs.sort(); ys.sort()
    m = len(xs) // 2
    return (0.0, float(xs[m]), float(ys[m]))


def block_angle(frame, box, color):
    """返回物块朝向：图像角度(度) 和 机械臂角度(度)，无掩膜时返回 (None, None)"""
    try:
        x1, y1, x2, y2 = [int(round(v)) for v in box]
        h, w = frame.shape[:2]
        x1 = max(0, x1); y1 = max(0, y1); x2 = min(w - 1, x2); y2 = min(h - 1, y2)
        if x2 - x1 < 10 or y2 - y1 < 10:
            return None, None
        roi = frame[y1:y2 + 1, x1:x2 + 1]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in HSV_RANGES.get(color, []):
            mask |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None, None
        c = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(c) < 200:
            return None, None
        (rcx, rcy), (rw, rh), rang = cv2.minAreaRect(c)
        # 归一化到 0~90（正方块 4 重对称）
        a_img = rang % 90.0
        # 主轴方向：用长边方向
        rad = math.radians(rang)
        cx = x1 + rcx
        cy = y1 + rcy
        L = max(rw, rh) / 2.0
        p1 = (cx - L * math.cos(rad), cy - L * math.sin(rad))
        p2 = (cx + L * math.cos(rad), cy + L * math.sin(rad))
        X1, Y1, _, _ = pixel_to_arm(p1[0], p1[1])
        X2, Y2, _, _ = pixel_to_arm(p2[0], p2[1])
        a_arm = math.degrees(math.atan2(Y2 - Y1, X2 - X1)) % 180.0
        return a_img, a_arm
    except Exception:
        return None, None


def detect_one(frame, model, color, conf=0.45):
    """在同色物块里挑置信度最高的一个"""
    r = model(frame, verbose=False, conf=conf)[0]
    best = None
    for b in r.boxes:
        if model.names[int(b.cls)] != color:
            continue
        cf = float(b.conf)
        x1, y1, x2, y2 = b.xyxy[0].tolist()
        _cc = color_centroid(frame, (x1, y1, x2, y2), color)
        _cx, _cy = _cc if _cc is not None else ((x1 + x2) / 2.0, (y1 + y2) / 2.0)
        if best is None or cf > best[0]:
            best = (cf, _cx, _cy)
    return best


SERVO_WAIT = 0.8


def predict_target_pixel(px, py, vx, vy, seconds):
    """Project a world-space belt velocity into local camera pixels."""
    dx, dy = float(vx) * float(seconds), float(vy) * float(seconds)
    if abs(dx) + abs(dy) < 1e-9:
        return float(px), float(py)
    step = 2.0
    xp = pixel_to_arm_float(px + step, py)
    xm = pixel_to_arm_float(px - step, py)
    yp = pixel_to_arm_float(px, py + step)
    ym = pixel_to_arm_float(px, py - step)
    world_per_pixel = (
        ((xp[0] - xm[0]) / (2.0 * step), (yp[0] - ym[0]) / (2.0 * step)),
        ((xp[1] - xm[1]) / (2.0 * step), (yp[1] - ym[1]) / (2.0 * step)),
    )
    du, dv = world_delta_to_pixel(world_per_pixel, (dx, dy))
    return float(px) + du, float(py) + dv


def _probe_step(position, lower, upper):
    if position + SERVO_PROBE_MM <= upper:
        return SERVO_PROBE_MM
    if position - SERVO_PROBE_MM >= lower:
        return -SERVO_PROBE_MM
    return None


def probe_claw_jacobian(cap, X, Y):
    """Measure local jaw-pixel response to small X/Y commands at lift height."""
    x_step = _probe_step(X, GRAB_X_MIN, GRAB_X_MAX)
    y_step = _probe_step(Y, GRAB_Y_MIN, GRAB_Y_MAX)
    if x_step is None or y_step is None:
        print("   伺服: 当前位置没有安全的 5mm 标定空间", flush=True)
        return None

    axis_baselines = []
    probe_centers = []
    for axis, delta in ((0, x_step), (1, y_step)):
        axis_base = capture_claw_center(cap)
        if axis_base is None:
            print("   伺服: 探测前爪中心检测失败", flush=True)
            return None
        tx, ty = (X + delta, Y) if axis == 0 else (X, Y + delta)
        if not send_to_board("%d %d %d\n" % (round(tx), round(ty), LIFT_Z)):
            print("   伺服: 局部标定移动发送失败", flush=True)
            return None
        time.sleep(SERVO_PROBE_WAIT)
        moved = capture_claw_center(cap)
        returned = send_to_board("%d %d %d\n" % (round(X), round(Y), LIFT_Z))
        time.sleep(SERVO_PROBE_WAIT)
        restored = capture_claw_center(cap)
        if not returned:
            print("   伺服: 回到高位基准点失败", flush=True)
            return None
        if moved is None or restored is None:
            print("   伺服: 探测时爪中心丢失，停在高位", flush=True)
            return None
        if math.hypot(restored[0] - axis_base[0],
                      restored[1] - axis_base[1]) > SERVO_RETURN_TOL_PX:
            print("   伺服: 回程误差过大，局部雅可比无效", flush=True)
            return None
        axis_baselines.append(axis_base)
        probe_centers.append(moved)

    try:
        jacobian = jacobian_from_axis_probes(
            axis_baselines[0], probe_centers[0], x_step,
            axis_baselines[1], probe_centers[1], y_step)
        # Reject a near-singular response before using it for motion.
        pixel_error_to_xy(jacobian, (0.0, 0.0))
    except ValueError as exc:
        print("   伺服: 局部雅可比无效:", exc, flush=True)
        return None
    print("   伺服: J(px/mm)=%s" % (jacobian,), flush=True)
    return jacobian


GRAB_ALIGN_ITERS = 2      # 抓取高度上最多修正几次
GRAB_ALIGN_TOL = 6.0      # 误差小于此值(mm)不再动


def final_align_at_grab_height(cap, model, color, X0, Y0):
    """已降到 GRAB_Z 后，用摄像头量爪子与物块的偏差并横向修正
    返回最终 (X, Y)。爪子在这个高度是可见的。"""
    X, Y = float(X0), float(Y0)
    for it in range(GRAB_ALIGN_ITERS):
        for _ in range(3):
            cap.grab()
        ok, f = cap.retrieve()
        if not ok:
            return int(X), int(Y)
        claw = detect_claw_center(f)
        blk = detect_one(f, model, color)
        if claw is None or blk is None:
            print("   抓取高度对准: 爪或块不可见，跳过", flush=True)
            return int(X), int(Y)
        cxp, cyp = claw["center"]
        bxp, byp = blk[1], blk[2]
        # 把像素差换算成毫米（用标定矩阵的局部比例）
        hx1, hy1, _, _ = pixel_to_arm(cxp, cyp)
        hx2, hy2, _, _ = pixel_to_arm(bxp, byp)
        dx, dy = hx1 - hx2, hy1 - hy2
        print("   抓取高度对准#%d 爪(%d,%d) 块(%d,%d) 错(%+.0f,%+.0f)mm"
              % (it + 1, cxp, cyp, bxp, byp, dx, dy), flush=True)
        if abs(dx) <= GRAB_ALIGN_TOL and abs(dy) <= GRAB_ALIGN_TOL:
            print("   已对准", flush=True)
            return int(X), int(Y)
        dx = max(-30, min(30, dx))
        dy = max(-30, min(30, dy))
        X -= dx
        Y -= dy
        send_to_board("%d %d %d\n" % (int(X), int(Y), GRAB_Z))
        time.sleep(0.7)
    print("   抓取高度对准结束 -> (%d,%d)" % (int(X), int(Y)), flush=True)
    return int(X), int(Y)


def refine_target_block(cap, model, color, X_cmd, Y_cmd, vx, vy):
    """只靠物块做闭环（不需要看到爪子），返回 (X, Y, 末次误差)"""
    X, Y = float(X_cmd), float(Y_cmd)
    last = None
    prev = None
    # 移动目标：不做二次定位（每次迭代要多花 1.1 秒，块早走远了）
    _spd = math.hypot(vx, vy)
    if _spd > 2.0:
        print("   移动目标 %.1fmm/s -> 跳过二次定位，直接用预测点下降" % _spd, flush=True)
        return int(X), int(Y), None
    for it in range(4):
        for _ in range(4):
            cap.grab()
        b = detect_one_stable(cap, model, color)
        if b is None:
            print("   物块闭环#%d: 没检测到物块" % (it + 1), flush=True)
            break
        tx, ty, _, _ = pixel_to_arm(b[1], b[2])
        nx, ny = tx + vx * LEAD_DESC, ty + vy * LEAD_DESC
        ex, ey = nx - X, ny - Y
        # 误检防护：单轮偏差超过 40mm，重新拍一次确认
        if (ex * ex + ey * ey) ** 0.5 > 40:
            print("   可疑跳变 (%+.0f,%+.0f)mm，复核一帧" % (ex, ey), flush=True)
            for _ in range(4):
                cap.grab()
            b2 = detect_one_stable(cap, model, color)
            if b2 is not None:
                t2x, t2y, _, _ = pixel_to_arm(b2[1], b2[2])
                n2x, n2y = t2x + vx * LEAD_DESC, t2y + vy * LEAD_DESC
                if ((n2x - nx) ** 2 + (n2y - ny) ** 2) ** 0.5 > 40:
                    print("   两次结果不一致，放弃本轮修正", flush=True)
                    break
            else:
                break
            print("   复核一致，采用新位置", flush=True)
        last = (int(ex), int(ey))
        print("   物块闭环#%d 目标(%+d,%+d) 误差(%+.0f,%+.0f)mm"
              % (it + 1, nx, ny, ex, ey), flush=True)
        if abs(ex) <= 6 and abs(ey) <= 6:
            break
        # 振荡检测：误差或符号翻转且幅度没明显变小 -> 停止迭代
        if prev is not None:
            _flip = (ex * prev[0] < 0) or (ey * prev[1] < 0)
            _norm = (abs(ex) + abs(ey)) - (abs(prev[0]) + abs(prev[1]))
            if _flip and _norm > -6:
                print("   检测到振荡，停止迭代（用当前点）", flush=True)
                break
        prev = (ex, ey)
        dx = max(-40, min(40, ex * 0.6))   # 阻尼 0.6，防止过冲振荡
        dy = max(-40, min(40, ey * 0.6))
        X += dx
        Y += dy
        # 修正时直接走，不再加单向逼近偏移（避免每轮来回摆 25mm）
        send_to_board("%d %d %d\n" % (int(X), int(Y), LIFT_Z))
        time.sleep(SERVO_WAIT)
    # 二次确认：收敛后再拍一帧核对（不再移动）
    for _ in range(5):
        cap.grab()
    _b = detect_one_stable(cap, model, color)
    if _b is not None:
        _tx, _ty, _, _ = pixel_to_arm(_b[1], _b[2])
        _nx2, _ny2 = _tx + vx * LEAD_DESC, _ty + vy * LEAD_DESC
        _cex, _cey = _nx2 - X, _ny2 - Y
        print("   二次确认: 误差(%+.0f,%+.0f)mm" % (_cex, _cey), flush=True)
        if abs(_cex) > 5 or abs(_cey) > 5:
            print("   超过 5mm，补一次修正", flush=True)
            X += max(-40, min(40, _cex))
            Y += max(-40, min(40, _cey))
            send_to_board("%d %d %d\n" % (int(X), int(Y), LIFT_Z))
            time.sleep(SERVO_WAIT)
    else:
        print("   二次确认: 没拍到块", flush=True)
    print("   物块闭环结束 -> (%d,%d)" % (int(X), int(Y)), flush=True)
    return int(X), int(Y), last


def refine_target(cap, model, color, X_cmd, Y_cmd, vx, vy, *, target_observation=None):
    """Moving belt: one intercept. Stationary block: existing visual servo."""
    if math.hypot(vx, vy) > 1e-6:
        try:
            return refine_target_moving(cap, model, color, X_cmd, Y_cmd, vx, vy,
                                        target_observation)
        except Exception as exc:
            print("   运动预测异常，取消抓取并保持高位:", exc, flush=True)
            return None
    r = None
    try:
        r = refine_target_claw(cap, model, color, X_cmd, Y_cmd, vx, vy)
    except Exception as exc:
        print("   爪伺服异常:", exc, flush=True)
        r = None
    if r is not None:
        return r
    print("   回退: 看不到爪子，改用物块闭环", flush=True)
    return refine_target_block(cap, model, color, X_cmd, Y_cmd, vx, vy)


def refine_target_claw(cap, model, color, X_cmd, Y_cmd, vx, vy):
    """Close the image-space loop using the observed blue jaw center."""
    X, Y = float(X_cmd), float(Y_cmd)
    origin = (X, Y)
    if not send_to_board("%d %d %d\n" % (round(X), round(Y), LIFT_Z)):
        print("   伺服: 高位定位命令失败", flush=True)
        return None
    time.sleep(APPROACH_WAIT)

    offset = load_claw_alignment_offset(xy=(X, Y))
    if offset is None:
        print("   伺服: 当前区域没有有效对准偏移；请在该区域附近运行 claw_servo_calibrate.py",
              flush=True)
        return None

    initial = capture_visual_pair(cap, model, color)
    if initial is None:
        print("   伺服: 物块或双爪检测不稳定，停在高位", flush=True)
        return None

    jacobian = probe_claw_jacobian(cap, X, Y)
    if jacobian is None:
        return None

    previous = None
    for iteration in range(SERVO_MAX_ITERS):
        pair = capture_visual_pair(cap, model, color)
        if pair is None:
            print("   伺服: 视觉特征丢失，停在高位", flush=True)
            return None
        if previous is not None:
            old_x, old_y, old_claw = previous
            observed_move = (X - old_x, Y - old_y)
            observed_pixels = (pair["claw"][0] - old_claw[0],
                               pair["claw"][1] - old_claw[1])
            jacobian = update_jacobian_secant(jacobian, observed_move,
                                              observed_pixels)

        try:
            target_goal = predict_target_pixel(pair["target"][0], pair["target"][1],
                                               vx, vy, LEAD_DESC)
            error = pixel_error(target_goal, pair["claw"], offset)
        except (ValueError, ZeroDivisionError) as exc:
            print("   伺服: 目标投影失败，停在高位:", exc, flush=True)
            return None

        error_norm = math.hypot(error[0], error[1])
        print("   视觉伺服#%d target=(%.1f,%.1f) claw=(%.1f,%.1f) e=(%+.1f,%+.1f)px"
              % (iteration + 1, target_goal[0], target_goal[1],
                 pair["claw"][0], pair["claw"][1], error[0], error[1]), flush=True)
        if is_converged(error, SERVO_TOL_PX):
            print("   视觉伺服收敛 -> (%d,%d)" % (round(X), round(Y)), flush=True)
            return round(X), round(Y), error

        try:
            dx, dy = pixel_error_to_xy(jacobian, error, gain=SERVO_GAIN,
                                       max_step_mm=SERVO_MAX_STEP_MM)
        except ValueError as exc:
            print("   伺服: 修正矩阵无效，停在高位:", exc, flush=True)
            return None
        next_x, next_y = X + dx, Y + dy
        total = math.hypot(next_x - origin[0], next_y - origin[1])
        if total > SERVO_MAX_TOTAL_MM:
            print("   伺服: 总修正超出 %.0fmm 限幅，停在高位" % SERVO_MAX_TOTAL_MM,
                  flush=True)
            return None
        if not (GRAB_X_MIN <= next_x <= GRAB_X_MAX and
                GRAB_Y_MIN <= next_y <= GRAB_Y_MAX):
            print("   伺服: 修正会越出工作区，停在高位", flush=True)
            return None

        previous = (X, Y, pair["claw"])
        X, Y = next_x, next_y
        if not send_to_board("%d %d %d\n" % (round(X), round(Y), LIFT_Z)):
            print("   伺服: 修正命令发送失败，停在高位", flush=True)
            return None
        time.sleep(SERVO_WAIT)

    print("   伺服: 达到迭代上限，未收敛；保持高位，不下降", flush=True)
    return None


def pick_block(found, frame=None):
    """选画面最左、落在工作区内、且背景是白色传送带的块"""
    best = None
    cand = []
    for c in found:
        X, Y, _, _ = pixel_to_arm(c[1], c[2])
        if not (GRAB_X_MIN <= X <= GRAB_X_MAX and GRAB_Y_MIN <= Y <= GRAB_Y_MAX):
            continue
        if len(c) > 7:
            hw = (c[6] - c[4]) / 2.0
            hh = (c[7] - c[5]) / 2.0
        else:
            hw, hh = 40.0, 40.0
        if not on_belt(frame, c[1], c[2], hw, hh):
            print("   跳过(不在白色带上) %s px(%.0f,%.0f)" % (c[3], c[1], c[2]), flush=True)
            continue
        # 记录候选：角差（相对当前底座方位）+ 图像 x
        _ang = math.degrees(math.atan2(Y, max(X, 1.0)))
        cand.append((abs(_ang - LAST_BASE[0]), c[1], c, _ang))

    if not cand:
        return None

    # 若 VLM 给了建议颜色，且画面里确实有该颜色，则只在其中挑
    _pref = VLM_PREFER[0] if "VLM_PREFER" in globals() else None
    if _pref:
        _sub = [t for t in cand if t[2][3] == _pref]
        if _sub:
            cand = _sub

    # 先按"角度最接近当前方位"排序，再在最小角差 +25 度范围内取最靠前的
    cand.sort(key=lambda t: t[0])
    near = [t for t in cand if t[0] <= cand[0][0] + 25.0]
    near.sort(key=lambda t: t[1])
    best = near[0][2]
    if len(cand) > 1:
        print("   候选 %d 个，选角差最小(%.0f°) 的 %s px(%.0f,%.0f)"
              % (len(cand), near[0][0], best[3], best[1], best[2]), flush=True)
    return best



# ======================= PL 硬件检测（伪装成 YOLO 模型）=======================
class _PLXYXY(object):
    def __init__(self, x1, y1, x2, y2):
        self.v = [float(x1), float(y1), float(x2), float(y2)]

    def tolist(self):
        return list(self.v)

    def __getitem__(self, i):
        return self.v[i]

    def __iter__(self):
        return iter(self.v)


class _PLBox(object):
    __slots__ = ("cls", "conf", "xyxy")

    def __init__(self, cls_id, conf, x1, y1, x2, y2):
        self.cls = int(cls_id)
        self.conf = float(conf)
        self.xyxy = [_PLXYXY(x1, y1, x2, y2)]


class _PLResult(object):
    __slots__ = ("boxes",)

    def __init__(self, boxes):
        self.boxes = boxes


PL_NAMES = {0: "red", 1: "yellow", 2: "green", 3: "blue", 4: "black"}
PL_NAME2ID = {v: k for k, v in PL_NAMES.items()}


class PLDetModel(object):
    """接口与 YOLO 一致：model(frame, conf=...)[0].boxes
       实际是查板端 PL 硬件检测寄存器。
       坐标从显示分辨率(1024x600)换算回摄像头分辨率(1280x720)。"""

    def __init__(self, ip=None, port=None, disp_w=1024, disp_h=600,
                 cam_w=1280, cam_h=720):
        self.ip = ip or BOARD_IP
        self.port = port or BOARD_PORT
        self.sx = float(cam_w) / float(disp_w)
        self.sy = float(cam_h) / float(disp_h)
        self.names = dict(PL_NAMES)
        self.calls = 0
        self.last_ms = 0.0
        self.fb = None          # 指向 FBStream，用于先把当帧推给板子

    def query_raw(self):
        """向板子要一次检测结果，返回 {color_id: [cnt,cx,cy,x0,x1,y0,y1]}"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(3.0)
        try:
            sock.connect((self.ip, self.port))
            sock.sendall(b"DET\n")
            sock.shutdown(socket.SHUT_WR)
            buf = b""
            while True:
                chunk = sock.recv(4096)
                if not chunk:
                    break
                buf += chunk
        finally:
            try:
                sock.close()
            except Exception:
                pass
        out = {}
        for item in buf.decode(errors="ignore").strip().split(";"):
            if ":" not in item:
                continue
            name, vals = item.split(":")
            cid = PL_NAME2ID.get(name.strip())
            if cid is None:
                continue
            try:
                out[cid] = [int(v) for v in vals.split(",")]
            except ValueError:
                continue
        return out

    def __call__(self, frame=None, verbose=False, conf=0.0, **kwargs):
        # 先把“当前这一帧”推给板子，保证检测结果对应当前帧
        if self.fb is not None and frame is not None:
            try:
                self.fb.push(frame)
            except Exception:
                pass
            time.sleep(PL_PUSH_WAIT)
        t0 = time.time()
        raw = self.query_raw()
        self.last_ms = (time.time() - t0) * 1000.0
        self.calls += 1

        boxes = []
        for cid in range(5):
            v = raw.get(cid)
            if not v or len(v) < 7:
                continue
            cnt, cx, cy, x0, x1, y0, y1 = v[:7]
            if cnt < PL_MIN_CNT:
                continue
            bw = max(1.0, float(x1 - x0))
            bh = max(1.0, float(y1 - y0))
            fill = float(cnt) / ((bw * bh) / 16.0)   # 采样步距 4px，面积除以 16
            if fill < PL_MIN_FILL:
                continue
            cf = min(0.99, float(cnt) / PL_FULL_CNT)
            if cf < conf:
                continue
            boxes.append(_PLBox(cid, cf,
                                x0 * self.sx, y0 * self.sy,
                                x1 * self.sx, y1 * self.sy))
        return [_PLResult(boxes)]


def main():
    global BELT_RUNNING
    _install_exit_hooks()
    if USE_PL_DET:
        model = PLDetModel()
        MODEL_REF[0] = model
        print("检测源: PL 硬件（板端 TCP DET）", flush=True)
    else:
        model = YOLO(MODEL_PATH)
        print("检测源: PC YOLO", flush=True)
    cap = cv2.VideoCapture(CAM_ID, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    if not cap.isOpened():
        print("camera open failed"); sys.exit(1)

    # 同时把标注画面推到板子 HDMI
    FB = FBStream(max_fps=(15 if USE_PL_DET else 3)).start()
    print("HDMI 推流已启动 -> %s:%d" % (FB.ip, FB.port), flush=True)
    if MODEL_REF[0] is not None:
        MODEL_REF[0].fb = FB

    if AUTO_RECORD:
        _fn = time.strftime("D:\\share\\record_%Y%m%d_%H%M%S.avi")
        _w = cv2.VideoWriter(_fn, cv2.VideoWriter_fourcc(*"XVID"), 10.0, (FRAME_W, FRAME_H))
        if _w.isOpened():
            REC_WRITER[0] = _w
            REC_PATH[0] = _fn
            RECORD_ON[0] = True
            REC_START[0] = time.time()
            print("=== 自动录像开始: %s ===" % _fn, flush=True)

    cv2.namedWindow("YOLO -> Board", cv2.WINDOW_NORMAL)
    cv2.waitKey(1)
    focus_window()
    cv2.resizeWindow("YOLO -> Board", 960, 540)
    print("board %s:%d ready" % (BOARD_IP, BOARD_PORT))
    # 快速模式：跳过使能阶段（每条运动 1.05s -> 0.3s）；HOME 会重新使能
    try:
        if send_to_board("JOG ON\n"):
            print("快速模式已开启（跳过使能帧）")
    except Exception:
        pass
    print("传送带预测 %s：%.2fmm/s；B 切换预测开/关（不控制电机），ESC 退出"
          % ("ON" if BELT_RUNNING else "OFF", BELT_SPEED), flush=True)

    armed = False
    auto_mode = False
    track_mode = False
    step_ready = False
    home_ready = False
    targets = []
    idx = 0
    t_start = time.time()
    last_send = 0.0
    seq_busy = False
    at_block = False
    last_xy = [0, 0]
    phase = 0
    cur_color = "blue"
    last_focus = 0.0

    def sequence(X, Y):
        """下降 -> 合爪 -> 抬起 ->(可选)移到放置点 -> 开爪；在后台线程跑，不阻塞画面"""
        nonlocal seq_busy
        try:
            steps = [
                ("%d %d %d\n" % (X, Y, GRAB_Z), SEG_DOWN_WAIT),   # 下降
                ("%d %d %d\n" % (X, Y, LIFT_Z),  SEG_DOWN_WAIT),  # 抬起
            ]
            if PLACE_ENABLE:
                steps += [
                    ("%d %d %d\n" % (PLACE_X, PLACE_Y, LIFT_Z), SEG_MOVE_WAIT),  # 移到放置点
                    ("CLAW %d\n" % GRIP_OPEN_US, SEG_CLAW_WAIT),                 # 开爪
                ]
            for msg, wait in steps:
                send_to_board(msg)
                print("[SEQ] %s" % msg.strip())
                time.sleep(wait)
        finally:
            seq_busy = False

    while True:
        for _ in range(3):
            frame_at = time.monotonic()
            cap.grab()
        ok, frame = cap.retrieve()
        frame_raw = frame.copy() if USE_PL_DET else frame
        if not ok:
            continue

        res = model(frame, verbose=False, conf=CONF_TH)[0]
        found = []
        for b in res.boxes:
            cname = model.names[int(b.cls)]
            if cname not in TARGET_CLASSES:
                continue
            conf = float(b.conf)
            x1, y1, x2, y2 = b.xyxy[0].tolist()
            ccx, ccy = (x1 + x2) / 2, (y1 + y2) / 2
            _cc = color_centroid(frame, (x1, y1, x2, y2), cname)
            if _cc is not None:
                ccx, ccy = _cc
            if not (ROI_Y_MIN < ccy < ROI_Y_MAX):
                continue
            found.append((conf, ccx, ccy, cname, x1, y1, x2, y2))
        found.sort(key=lambda t: t[1])
        for _c in found:
            _X, _Y, _, _ = pixel_to_arm(_c[1], _c[2])
            _nm = _c[3]
            _h = hist.setdefault(_nm, [])
            _h.append((time.time(), _X, _Y))
            while _h and time.time() - _h[0][0] > 6.0:
                _h.pop(0)

        for conf, ccx, ccy, cname, x1, y1, x2, y2 in found:
            X, Y, Z, cell = pixel_to_arm(ccx, ccy)
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(frame, "%s (%d,%d)" % (cname, X, Y), (int(x1), int(y1) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

        claw = detect_claw_center(frame)
        if claw is not None:
            for label, (px, py), color in (
                    ("L", claw["left"], (255, 200, 0)),
                    ("R", claw["right"], (255, 200, 0))):
                cv2.circle(frame, (round(px), round(py)), 8, color, 2)
                cv2.putText(frame, label, (round(px) - 6, round(py) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            cv2.drawMarker(frame, (round(claw["center"][0]), round(claw["center"][1])),
                           (0, 0, 255), cv2.MARKER_CROSS, 26, 3)
            cv2.putText(frame, "CLAW CENTER", (round(claw["center"][0]) + 12,
                        round(claw["center"][1]) - 8), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (0, 0, 255), 2)

        now = time.time()
        if now - last_focus > 2.0:
            last_focus = now
            focus_window()

        armed = False  # 每帧复位，禁用旧武装流程
        if armed and not targets and not track_mode:
            armed = False
            print("targets empty -> back to PREVIEW", flush=True)

        # ---------------- PREVIEW ----------------
        if not armed:
            info = "BLOCKS=%d TRIM(%+d,%+d) S=grab q/e=claw B=belt:%s" % (
                len(found), TRIM_X, TRIM_Y, "ON" if BELT_RUNNING else "OFF")
            cv2.rectangle(frame, (0, 0), (frame.shape[1], 46), (0, 0, 0), -1)
            cv2.putText(frame, info, (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 255, 255), 2)
        try:
            _ui = frame_raw if USE_PL_DET else frame
            if _ui.shape[0] != UI_H or _ui.shape[1] != UI_W:
                _ui = cv2.resize(_ui, (UI_W, UI_H), interpolation=cv2.INTER_AREA)
            _ui = draw_dashboard(_ui, fps=_ui_fps(),
                                 target=TARGET_UI[0], center=TARGET_UI[1],
                                 lead=TARGET_UI[2], cycle=TARGET_UI[3],
                                 box_l=BOX_COUNT[0], box_r=BOX_COUNT[1],
                                 board_ok=True)
            FB.push(_ui)
        except Exception:
            pass
        # ---- 录像指示：右上角红点 + 计时 ----
        if RECORD_ON[0]:
            _el = int(time.time() - REC_START[0]) if REC_START[0] else 0
            _h, _w2 = frame.shape[:2]
            cv2.circle(frame, (_w2 - 150, 28), 9, (0, 0, 255), -1)
            cv2.circle(frame, (_w2 - 150, 28), 9, (255, 255, 255), 2)
            cv2.putText(frame, "REC %02d:%02d" % (_el // 60, _el % 60),
                        (_w2 - 132, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
        else:
            _h, _w2 = frame.shape[:2]
            cv2.putText(frame, "NO REC (v)", (_w2 - 160, 36),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (140, 140, 140), 2)
        if RECORD_ON[0] and REC_WRITER[0] is not None:
            REC_WRITER[0].write(frame)
        # 每帧刷新 OSD（限速 2.5Hz，由 osd_push 内部处理）
        try:
            osd_push(state=OSD_STATE[0])
        except Exception:
            pass
        cv2.imshow("YOLO -> Board", cv2.resize(frame, (960, 540), interpolation=cv2.INTER_AREA))

        if True:   # 每帧都读键（原来的 if int(now)%3 是误留，导致丢键+画出 ARMED 0/0）
            k = gk()
            # ---- f 键：全自动循环 ----
            # ---- v 键：录像开关 ----
            if k in (ord("v"), ord("V")):
                if not RECORD_ON[0]:
                    _fn = time.strftime("D:\\share\\record_%Y%m%d_%H%M%S.avi")
                    _w = cv2.VideoWriter(_fn, cv2.VideoWriter_fourcc(*"XVID"), 10.0, (FRAME_W, FRAME_H))
                    if _w.isOpened():
                        REC_WRITER[0] = _w
                        REC_PATH[0] = _fn
                        RECORD_ON[0] = True
                        REC_START[0] = time.time()
                        print("=== 开始录像: %s ===" % _fn, flush=True)
                    else:
                        print("录像失败：无法创建文件", flush=True)
                else:
                    RECORD_ON[0] = False
                    if REC_WRITER[0] is not None:
                        REC_WRITER[0].release()
                        REC_WRITER[0] = None
                    print("=== 停止录像: %s ===" % REC_PATH[0], flush=True)
                continue
            if k in (ord("f"), ord("F")):
                AUTO_RUN[0] = not AUTO_RUN[0]
                AUTO_IDX[0] = 0
                AUTO_NEXT[0] = time.time() + 0.5
                print("=== 全自动 %s ===" % ("开启" if AUTO_RUN[0] else "关闭"), flush=True)
                continue
            if AUTO_RUN[0] and k == -1 and time.time() >= AUTO_NEXT[0]:
                _act = AUTO_SEQ[AUTO_IDX[0] % len(AUTO_SEQ)]
                AUTO_IDX[0] += 1
                if _act == "grab":
                    print("   [自动] 抓取 @%.2f" % time.time(), flush=True)
                    T_MARK[0] = time.time()
                    GRAB_OK[0] = False
                    k = ord("s")
                    AUTO_NEXT[0] = time.time() + 0.1   # 松爪后立刻抓下一个
                elif _act == "close":
                    if not GRAB_OK[0]:
                        print("   [自动] 没抓到块，等待物块", flush=True)
                        _op = _pulse_from_file(GRIP_OPEN_FILE, GRIP_OPEN_US)
                        if CLAW_STATE[0] != _op:
                            send_to_board("CLAW %d\n" % _op)
                            CLAW_STATE[0] = _op
                        AUTO_IDX[0] += 3          # 跳过 place / open / ready
                        AUTO_NEXT[0] = time.time() + 2.0
                        continue

                    # 等爪子在画面里出现（= 已经降到抓取高度）再合爪
                    print("   [自动] 距发S %.2fs -> 合爪" % (time.time() - T_MARK[0]), flush=True)
                    # 自适应：下降命令本身已阻塞一段时间，扣掉它，总时长仍为 AUTO_CLOSE_WAIT
                    _left = max(0.35, AUTO_CLOSE_WAIT - LAST_MOVE_DUR[0])
                    print("   合爪等待 %.2fs（命令耗时 %.2fs）" % (_left, LAST_MOVE_DUR[0]), flush=True)
                    time.sleep(_left)
                    handle_claw(ord("q"))
                    print("   [自动] 合爪完成 @%.2f" % (time.time() - T_MARK[0]), flush=True)
                    OSD_STATS["ok"] += 1
                    try:
                        TARGET_UI[3] = time.time() - T_MARK[0]
                    except Exception:
                        pass
                    try:
                        osd_push(state="GRAB OK")
                    except Exception:
                        pass
                    AUTO_NEXT[0] = time.time() + 0.0   # 合爪后立刻送盒
                elif _act == "place":
                    print("   [自动] 送盒", flush=True)
                    k = ord("s")
                    AUTO_NEXT[0] = time.time() + 0.3
                elif _act == "open":
                    # 等待时间按转角计算（PL 使能阶段占 0.72s）
                    _wt = 0.75 + PLACE_ANGLE[0] / 65.0
                    print("   [自动] 松爪：等底座到位（转 %.0f°，等 %.1fs）"
                          % (PLACE_ANGLE[0], _wt), flush=True)
                    time.sleep(_wt)
                    handle_claw(ord("e"))
                    AUTO_NEXT[0] = time.time() + 0.35

                elif _act == "ready":
                    # 送盒后直接等待下一次抓取，取消新增抬升。
                    last_xy[0], last_xy[1] = BOX_XY[0], BOX_XY[1]
                    AUTO_NEXT[0] = time.time() + 0.30

            if handle_keys(k):
                continue
            if k == 27:
                break
            if k in (ord("b"), ord("B")):
                BELT_RUNNING = not BELT_RUNNING
                print("传送带预测 %s（软件模式，不控制传送带电机）"
                      % ("ON" if BELT_RUNNING else "OFF"), flush=True)
                continue
            if k == 2424832:   # left  -> X-
                globals()["TRIM_X"] -= 5
                print("TRIM", globals()["TRIM_X"], globals()["TRIM_Y"])
                continue
            if k == 2555904:   # right -> X+
                globals()["TRIM_X"] += 5
                print("TRIM", globals()["TRIM_X"], globals()["TRIM_Y"])
                continue
            if k == 2490368:   # up    -> Y+
                globals()["TRIM_Y"] += 5
                print("TRIM", globals()["TRIM_X"], globals()["TRIM_Y"])
                continue
            if k == 2621440:   # down  -> Y-
                globals()["TRIM_Y"] -= 5
                print("TRIM", globals()["TRIM_X"], globals()["TRIM_Y"])
                continue
            if k == ord("j"):      # X-
                globals()["TRIM_X"] -= 5
                print("TRIM", globals()["TRIM_X"], globals()["TRIM_Y"])
                continue
            if k == ord("l"):      # X+
                globals()["TRIM_X"] += 5
                print("TRIM", globals()["TRIM_X"], globals()["TRIM_Y"])
                continue
            if k == ord("i"):      # Y+
                globals()["TRIM_Y"] += 5
                print("TRIM", globals()["TRIM_X"], globals()["TRIM_Y"])
                continue
            if k == ord("k"):      # Y-
                globals()["TRIM_Y"] -= 5
                print("TRIM", globals()["TRIM_X"], globals()["TRIM_Y"])
                continue
            if k in (ord("s"), ord("S")):
                print("S pressed, found=", len(found), "armed=", armed, "phase=", phase, flush=True)
            if k in (ord("h"), ord("H")):
                home_all()
                # 回零同时重置抓取状态
                phase = 0
                LAST_BASE[0] = 0.0
                cur_color = "blue"
                last_xy[0], last_xy[1] = 0, 0
                hist.clear()
                print("=== HOME + 状态重置 (phase=0) ===", flush=True)
            if k in (13, 10) and found and (now - t_start) > 1.0:
                targets = [(*pixel_to_arm(c[1], c[2])[:3], c[3]) for c in found]
                idx = 0
                step_ready = True
                home_ready = False
                armed = True
                auto_mode = True
                print("=== AUTO mode: %d targets ===" % len(targets))
                continue
            if k in (ord("g"), ord("G")):
                track_mode = not track_mode
                armed = track_mode
                targets = []
                idx = 0
                print("=== TRACK %s ===" % ("ON" if track_mode else "OFF"))
                continue
            if k in (ord("f"), ord("F")) and found and (now - t_start) > 1.0:
                targets = [(*pixel_to_arm(c[1], c[2])[:3], c[3]) for c in found]
                idx = 0
                step_ready = True
                home_ready = False
                armed = True
                auto_mode = True
                print("=== F: AUTO all %d targets ===" % len(targets))
                continue
            if k in (ord("s"), ord("S")):
                if phase == 0:
                    pick = pick_block(found, frame)
                    if pick is not None:
                        cur_color = pick[3]
                        X, Y, Z, _ = pixel_to_arm(pick[1], pick[2])
                        vx, vy = belt_velocity()
                        if BELT_RUNNING:
                            print("   不停带 -> 使用一次运动预测", flush=True)
                        else:
                            print("   停带模式 -> 静止视觉定位", flush=True)
                        X1, Y1 = int(X + vx * LEAD1), int(Y + vy * LEAD1)
                        X2, Y2 = int(X + vx * LEAD2), int(Y + vy * LEAD2)
                        _ai, _aa = block_angle(frame, pick[4:8], cur_color)
                        try:
                            osd_push(target=(pick[1], pick[2]), center=(pick[1], pick[2]),
                                     lead=LEAD1, state="TRACK")
                        except Exception:
                            pass
                        # 提交一次 VLM 决策请求（限频 6s，后台异步）
                        try:
                            if VLM is not None:
                                _bl = [(c[3], c[1], c[2]) for c in found]
                                VLM.request(frame, _bl, min_interval=6.0)
                                _c, _r = VLM.latest()
                                if _c:
                                    VLM_PREFER[0] = _c
                                    VLM_REASON[0] = _r or ""
                                    print("   [VLM] 建议先抓 %s：%s" % (_c, _r), flush=True)
                        except Exception:
                            pass
                        TARGET_UI[0] = (pick[1], pick[2])
                        TARGET_UI[1] = (pick[1], pick[2])
                        TARGET_UI[2] = LEAD1
                        print("=== 选中 %s px(%.0f,%.0f)  v=(%+.1f,%+.1f)mm/s  角度: 图像%s 臂%s ==="
                              % (cur_color, pick[1], pick[2], vx, vy,
                                 ("%.0f°" % _ai) if _ai is not None else "-",
                                 ("%.0f°" % _aa) if _aa is not None else "-"), flush=True)
                        _d = ((X2-last_xy[0])**2 + (Y2-last_xy[1])**2) ** 0.5
                        print("=== S1: 高位接近 (%d,%d,%d) [横移%.0fmm] ==="
                              % (X2, Y2, LIFT_Z, _d), flush=True)
                        # 换向角度大时：先转到目标角度（手臂收起），再伸出去下放
                        # 换向角度大时：先转到目标角度（手臂收起），再伸出去下放
                        _ang_now = LAST_BASE[0]
                        _ang_new = math.degrees(math.atan2(Y2, max(X2, 1)))
                        if abs(_ang_new - _ang_now) > SAFE_SWING_DEG:
                            _px, _py = int(X2 * PIVOT_R), int(Y2 * PIVOT_R)
                            send_to_board("%d %d %d\n" % (_px, _py, LIFT_Z))
                            print("   先转底座到 %d°（手臂收起）(%d,%d,%d)"
                                  % (int(_ang_new), _px, _py, LIFT_Z), flush=True)
                            time.sleep(PIVOT_WAIT)
                            LAST_BASE[0] = _ang_new
                        observation = (*pixel_to_arm_float(pick[1], pick[2]), frame_at)
                        GRAB_CTX["extra"] = FIRST_EXTRA_LEAD_S
                        result = refine_target(cap, model, cur_color, X2, Y2, vx, vy,
                                               target_observation=observation)
                        if result is None:
                            print("=== 抓取取消：定位检查未通过，不发送下降命令 ===", flush=True)
                            continue
                        X2, Y2, _ = result
                        GRAB_OK[0] = True
                        send_to_board("%d %d %d\n" % (X2, Y2, GRAB_Z))
                        print("=== S2: 抓取点 (%d,%d,%d) ===  距发S %.2fs" % (X2, Y2, GRAB_Z, time.time() - T_MARK[0]), flush=True)

                        last_xy[0], last_xy[1] = X2, Y2
                        phase = 1
                    else:
                        print("no block", flush=True)
                elif phase == 1:
                    # 按颜色送到对应盒子：蓝->左(+90) 红->右(-90)
                    # 盒子位置：黄色/蓝色 -> 左 45°(+11520)，红色 -> 右 45°(-11520)

                    if cur_color in ("blue", "yellow"):
                        pos = BOX_LEFT_45
                    elif cur_color == "black":
                        pos = BOX_BLACK      # 黑块：底座左转 90°
                    elif cur_color == "green":
                        pos = BOX_GREEN      # 绿块：底座右转 90°（与黑对立）
                    else:
                        pos = BOX_RIGHT_45
                    if not send_motion_and_wait("BOX %d\n" % pos, timeout=30):
                        print("送盒未确认完成，暂停自动流程", flush=True)
                        AUTO_RUN[0] = False
                        continue
                    # 记录盒子位置（板端 PLACE_R 与底座角度决定），供更新软件跟踪
                    _bx = PLACE_R * math.cos(math.radians(pos / 256.0))
                    _by = PLACE_R * math.sin(math.radians(pos / 256.0))
                    BOX_XY[0], BOX_XY[1] = _bx, _by
                    LAST_BASE[0] = pos / 256.0
                    if pos > 0:
                        BOX_COUNT[0] += 1
                    else:
                        BOX_COUNT[1] += 1
                    LAST_BASE[0] = pos / 256.0   # 记录底座角度
                    PLACE_ANGLE[0] = abs(pos) / 256.0
                    phase = 2
                    print("=== S2: 送 %s 到 %s 盒 (BOX %d) ==="
                          % (cur_color, "左" if pos > 0 else "右", pos), flush=True)
                else:
                    pick = pick_block(found, frame)
                    if pick is not None:
                        cur_color = pick[3]
                        X, Y, Z, _ = pixel_to_arm(pick[1], pick[2])
                        vx, vy = belt_velocity()
                        if BELT_RUNNING:
                            print("   不停带 -> 使用一次运动预测", flush=True)
                        else:
                            print("   停带模式 -> 静止视觉定位", flush=True)
                        X1, Y1 = int(X + vx * LEAD_NEXT), int(Y + vy * LEAD_NEXT)
                        X2, Y2 = int(X + vx * LEAD_NEXT), int(Y + vy * LEAD_NEXT)
                        print("=== 下一个 %s  v=(%+.1f,%+.1f) 提前量=%dmm ==="
                              % (cur_color, vx, vy, int(BELT_SPEED * LEAD_NEXT)), flush=True)
                        _d = ((X2-last_xy[0])**2 + (Y2-last_xy[1])**2) ** 0.5
                        print("=== 下一个：高位接近 (%d,%d,%d) [横移%.0fmm] ==="
                              % (X2, Y2, LIFT_Z, _d), flush=True)
                        # 换向角度大时：先转到目标角度（手臂收起），再伸出去下放
                        _ang_now = LAST_BASE[0]
                        _ang_new = math.degrees(math.atan2(Y2, max(X2, 1)))
                        if abs(_ang_new - _ang_now) > SAFE_SWING_DEG:
                            _px, _py = int(X2 * PIVOT_R), int(Y2 * PIVOT_R)
                            send_to_board("%d %d %d\n" % (_px, _py, LIFT_Z))
                            print("   先转底座到 %d°（手臂收起）(%d,%d,%d)"
                                  % (int(_ang_new), _px, _py, LIFT_Z), flush=True)
                            time.sleep(PIVOT_WAIT)
                            LAST_BASE[0] = _ang_new
                        observation = (*pixel_to_arm_float(pick[1], pick[2]), frame_at)
                        GRAB_CTX["extra"] = NEXT_EXTRA_LEAD_S
                        result = refine_target(cap, model, cur_color, X2, Y2, vx, vy,
                                               target_observation=observation)
                        if result is None:
                            print("=== 抓取取消：定位检查未通过，不发送下降命令 ===", flush=True)
                            continue
                        X2, Y2, _ = result
                        GRAB_OK[0] = True
                        send_to_board("%d %d %d\n" % (X2, Y2, GRAB_Z))
                        print("=== 抓取点 (%d,%d,%d) ===  距发S %.2fs" % (X2, Y2, GRAB_Z, time.time() - T_MARK[0]), flush=True)

                        last_xy[0], last_xy[1] = X2, Y2
                        phase = 1
                    else:
                        print("没有块了", flush=True)
                        phase = 0
            continue

        # ---------------- DONE -> wait S -> home ----------------
        if (not track_mode) and targets and idx >= len(targets) and not seq_busy:
            info = "ALL DONE  S=HOME  ESC=quit"
            cv2.rectangle(frame, (0, 0), (frame.shape[1], 46), (0, 0, 0), -1)
            cv2.putText(frame, info, (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 255, 255), 2)
            try:
                _ui = frame_raw if USE_PL_DET else frame
                if _ui.shape[0] != UI_H or _ui.shape[1] != UI_W:
                    _ui = cv2.resize(_ui, (UI_W, UI_H), interpolation=cv2.INTER_AREA)
                _ui = draw_dashboard(_ui, fps=_ui_fps(),
                                     target=TARGET_UI[0], center=TARGET_UI[1],
                                     lead=TARGET_UI[2], cycle=TARGET_UI[3],
                                     box_l=BOX_COUNT[0], box_r=BOX_COUNT[1],
                                     board_ok=True)
                FB.push(_ui)
            except Exception:
                pass
            cv2.imshow("YOLO -> Board", cv2.resize(frame, (960, 540), interpolation=cv2.INTER_AREA))
            k = gk()
            if handle_keys(k):
                continue
            if k == 27:
                break
            if auto_mode or (k in (ord("s"), ord("S")) and not home_ready):
                home_ready = True
                auto_mode = False
                armed = False
                targets = []
                idx = 0
                t_start = time.time()
                print("=== done (no home), back to PREVIEW ===")
            continue

        # ---------------- TRACK mode ----------------
        if track_mode:
            if found and (now - last_send) > 1.0:
                c0 = found[0]
                X, Y, _, _ = pixel_to_arm(c0[1], c0[2])
                if send_to_board("%d %d %d\n" % (X, Y, LIFT_Z)):
                    last_send = now
                    print("[TRACK-HIGH] (%d,%d,%d)" % (X, Y, LIFT_Z))
            info = "TRACK  found=%d  G=off  H=home  ESC" % len(found)
            cv2.rectangle(frame, (0, 0), (frame.shape[1], 46), (0, 0, 0), -1)
            cv2.putText(frame, info, (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 200, 255), 2)
            try:
                FB.push(frame)
            except Exception:
                pass
            cv2.imshow("YOLO -> Board", cv2.resize(frame, (960, 540), interpolation=cv2.INTER_AREA))
            k2 = gk()
            if handle_keys(k2):
                continue
            if k2 == 27:
                break
            if k2 in (ord("g"), ord("G")):
                track_mode = False
                armed = False
                print("=== TRACK OFF ===")
            if k2 in (ord("h"), ord("H")):
                home_all()
                # 回零同时重置抓取状态
                phase = 0
                cur_color = "blue"
                last_xy[0], last_xy[1] = 0, 0
                hist.clear()
                print("=== HOME + 状态重置 (phase=0) ===", flush=True)
            continue

        # ---------------- move to next target ----------------
        if (step_ready or (auto_mode and (now - last_send) > AUTO_DELAY)) and idx < len(targets):
            if not seq_busy:
                X, Y, Z, target_color = targets[idx]
                result = refine_target(cap, model, target_color, X, Y, 0.0, 0.0)
                idx += 1
                step_ready = False
                last_send = now
                if result is None:
                    print("[AUTO] #%d 视觉伺服未收敛，跳过下降" % idx, flush=True)
                else:
                    X, Y, _ = result
                    seq_busy = True
                    threading.Thread(target=sequence, args=(X, Y), daemon=True).start()
                    print("[TX] #%d/%d  (%d, %d) grab->lift%s"
                          % (idx, len(targets), X, Y,
                             " ->place" if PLACE_ENABLE else ""))

        info = "ARMED  target %d/%d   S=next  H=home" % (min(idx + 1, len(targets)), len(targets))
        cv2.rectangle(frame, (0, 0), (frame.shape[1], 46), (0, 0, 0), -1)
        cv2.putText(frame, info, (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 255, 120), 2)
        try:
            FB.push(frame)
        except Exception:
            pass
        cv2.imshow("YOLO -> Board", cv2.resize(frame, (960, 540), interpolation=cv2.INTER_AREA))
        k = gk()
        if handle_keys(k):
            continue
        if k == 27:
            break
        if k in (ord("h"), ord("H")):
            home_all()
            phase = 0
            cur_color = "blue"
            last_xy[0], last_xy[1] = 0, 0
            hist.clear()
            print("=== HOME + 状态重置 (phase=0) ===", flush=True)
        if k in (ord("s"), ord("S")) and not step_ready:
            step_ready = True
            print("--- next ---")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
