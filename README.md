# Zynq-7020 Vision Sorting System

A four-axis robotic arm that picks colour-coded blocks off a conveyor and sorts
them into colour boxes.

A host PC runs object detection and a small task-level vision-language model.
The Zynq-7020 owns everything that has a hard timing requirement: the four-axis
motion command sequencer, the HDMI video overlay and the gripper PWM. The PS
side turns host coordinates into pulses and writes six registers; the PL side
turns those registers into correctly timed UART frames.

> 2026 全国大学生嵌入式芯片与系统设计竞赛 · FPGA 创新设计赛道 · 自主选题赛道

<!-- replace sampsonbrasseux421 with your GitHub user name after the first push -->
[![simulation](https://github.com/sampsonbrasseux421/zynq-vision-sorting/actions/workflows/sim.yml/badge.svg)](https://github.com/sampsonbrasseux421/zynq-vision-sorting/actions/workflows/sim.yml)
[![license](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)


---

## 1. Architecture

```
                 Ethernet (TCP)
  ┌──────────┐   cmds :5000   ┌──────────────────────────────┐
  │  Host PC │ ─────────────► │  Zynq-7020                   │
  │          │   video :5002  │                              │
  │  YOLO    │ ─────────────► │  PS: command server          │
  │  VLM     │                │      inverse kinematics      │
  │  state   │                │      register writer         │
  │  machine │                │                              │
  └──────────┘                │  PL: command sequencer       │
                              │      character OSD overlay   │
                              │      servo PWM               │
                              └───────┬───────────┬──────────┘
                                      │ UART      │ HDMI
                               4 × ZDT drivers   display
                                      │
                                 stepper motors
```

**Why the split is where it is**

| Function | Where | Reason |
|---|---|---|
| Detection | Host | no camera input path into the PL on this board |
| Task decision (VLM) | Host | tens of seconds per inference, must not block control |
| Inverse kinematics | PS | ~5 µs per move, 0.0005 % of the cycle |
| **Frame sequencing** | **PL** | the real bottleneck: 60 ms inter-frame gap plus OS scheduling jitter |
| **Video overlay** | **PL** | must survive a host link failure |
| **Gripper PWM** | **PL** | 50 Hz must not depend on Linux scheduling |

---

## 2. What the PL does

| Feature | Notes |
|---|---|
| Four-axis command sequencer | 12 enable frames + 4 position frames + 1 sync trigger, 60 ms apart |
| Broadcast sync trigger | all axes start within microseconds instead of ~240 ms apart |
| Skip-enable fast mode | `CTRL[4]` skips the 12 enable frames: 1.05 s → 0.22 s per move |
| Frame-interval register | `GAP_MS` settable at runtime, no rebuild needed |
| Hardware character OSD | 16×16 glyph ROM blended per pixel; independent of the CPU and the OS |
| Servo PWM | 50 Hz, 530 µs close / 1150 µs open |
| Status register | busy / done / current axis / frames sent, polled by the PS |

Full register map: [`skill/pl-cmd-engine/register_map.md`](skill/pl-cmd-engine/register_map.md).

---

The complete Vivado project — block design, 30 IP configurations, pin
constraints and the top-level HDMI path — is committed under
[ivado/](vivado/README.md), so the design opens and rebuilds without any
manual reconstruction.

## 3. Repository layout

```
.
├── src/
│   ├── rtl/      Verilog: sequencer, frame TX, AXI4-Lite slave, OSD, PWM
│   ├── ps/       Board side: TCP command server, IK, register writer
│   │   └── legacy/   Superseded implementations, kept for reference
│   └── host/
│       ├── app/  Detection app, gamepad control, VLM advisor
│       ├── deploy/   Launch scripts
│       └── model/    Model provenance and latency
├── sim/          Icarus Verilog testbenches
├── vivado/       Complete Vivado 2023.1 project (open the .xpr, or run scripts/rebuild.tcl)
├── build/        Reports, bootgen configs, hardware handoff
├── board/        Deployment scripts for the board
├── data/         Runtime parameters (gripper pulse widths)
├── skill/        Reusable skill packages (see below)
├── report/       Design report, LLM collaboration record, video script
└── docs/         Architecture, interface and hardware notes
```

---

## 4. Skill packages

`skill/` is the part of this repository intended for **reuse outside this
project**. Each package is self-contained and states its scope, usage, and
**failure conditions** explicitly.

| Package | Purpose |
|---|---|
| [`zdt-driver-timing`](skill/zdt-driver-timing/SKILL.md) | Measure the minimum safe frame interval and decide the enable strategy for ZDT closed-loop stepper drivers |
| [`pl-cmd-engine`](skill/pl-cmd-engine/SKILL.md) | Move a software command sequencer into the FPGA with a reproducible register map |
| [`axi-lite-slave-check`](skill/axi-lite-slave-check/SKILL.md) | Review checklist + testbench template for hand-written AXI4-Lite slaves |
| [`regression-cmd-diff`](skill/regression-cmd-diff/SKILL.md) | Locate a behaviour regression by diffing command logs instead of reading code |
| [`local-vlm-async`](skill/local-vlm-async/SKILL.md) | Attach a small vision-language model to a real-time loop without blocking it |
| [`runtime-param-config`](skill/runtime-param-config/SKILL.md) | Move geometry parameters out of the bitstream into a runtime config file |

Start from [`skill/README.md`](skill/README.md).

---

## 5. Hardware

| Item | Model |
|---|---|
| FPGA board | Zynq-7020 (XC7Z020CLG400-2), PetaLinux 2023.1, kernel 6.1.5 |
| PL clock | FCLK_CLK0 = 50 MHz |
| Arm actuators | 4 × ZDT X42S closed-loop stepper drivers (addresses 1–4; a 5th is installed but not driven) |
| Driver link | TTL UART 115200 8N1 — **3.3 V, not RS485** |
| Gripper | Servo, 50 Hz, 530 µs close / 1150 µs open |
| Camera | USB camera, 1280×720 |
| Display | HDMI, 1024×600 |
| Host | PC over Ethernet, port 5000 (commands) and 5002 (video) |

Wiring, power rails and driver addressing: [`docs/hardware_setup.md`](docs/hardware_setup.md).

---

## 6. Reproduce

```bash
# 1. FPGA bitstream  (Vivado 2023.1, see note below)
vivado -mode batch -source build/build_bitstream.tcl

# 2. Convert to fpga_manager bin
bootgen -image build/bit_only.bif -arch zynq -process_bitstream bin

# 3. PS software (cross compile, static)
arm-linux-gnueabihf-gcc -O2 -static -o ik_zdt_pl src/ps/ik_zdt.c

# 4. Deploy to the board
scp ik_zdt_pl src/ps/arm_net.sh petalinux@192.168.1.10:/usr/bin/
ssh petalinux@192.168.1.10 'chmod 755 /usr/bin/ik_zdt_pl /usr/bin/arm_net.sh'
#    register port 5000 in /etc/inetd.conf, then restart inetd

# 5. Host
python -m pip install -r src/host/requirements.txt
python src/host/app/pc_yolo_send.py      # or pad_arm.py for gamepad
```

Board-side deployment: [`board/`](board/). Step-by-step procedure, calibration
parameters and a troubleshooting table: section 3.7 of the design report.

All seven testbenches run automatically on every push (see
.github/workflows/sim.yml); the badges above show the current state.
Locally: cd sim && sh run_all.sh.

> **Tool version.** This project uses **Vivado 2023.1**. The build script relies
> only on standard AXI4-Lite, clocking-wizard and block-memory IP, so migrating
> to 2025.2 or 2026.1 requires regenerating IP output products and re-running
> the same TCL script — no RTL or constraint changes.

---

## 7. Measured results

| Metric | Before | After |
|---|---|---|
| One four-axis move | 1.49–1.67 s | **1.013 s** (n=10, σ 6.2 ms) |
| Inter-axis start skew | ~240 ms | **µs level** |
| Command timing jitter | ±20 ms | **0** |
| Single move (fast mode) | 1.05 s | **0.229 s** (n=10, σ 4.7 ms) |
| Motion-only pick-place cycle | — | **2.65 s** (n=10) |
| End-to-end cycle per block | ~13 s | **~7–8 s** (includes waiting for feed) |
| PL resources | — | **9,213 LUT (17.32 %)**, 12,987 FF, 5.5 BRAM, 2 DSP |
| Command engine cost | — | **231 LUT (0.43 %)** |
| Timing (50 MHz) | — | **WNS +4.551 ns**, TNS 0, 0 failing paths |
| On-chip power | — | **1.733 W** (est., default toggle rates) |

---

## 8. License

MIT — see [LICENSE](LICENSE).

## PL 逆运动学（ik_solver.v）

四轴逆解、角度—脉冲换算与笛卡尔插补均在 PL 内完成，PS 只写坐标并等待完成位。

- `src/rtl/ik_solver.v` —— CORDIC（16 级）+ 整数开方，13 状态串行；实测 2.4 μs、抖动 0，16 点最大偏差 0.043°
- `src/rtl/zdt_interp.v` —— 笛卡尔直线插补 + 五次 S 曲线（s = 10u³−15u⁴+6u⁵），共线误差 0
- `src/rtl/zdt_axil.v` —— AXI 寄存器 + 角度→脉冲（`pulse = deg × PPR × 46603 >> 32`）+ IK 自动触发
- 机构参数见 `/etc/ik.conf`：LINK1=200、LINK2=175、J2Z=−23040、J3Z=23040、J4Z=1280、ELBOW=−1

### 注意：Vivado module_ref 会缓存 RTL 快照

修改底层 RTL 后，`reset_target` / `generate_target` 都不会重读源文件，
必须换模块名重新 `create_bd_cell`。仓库中保留 `zdt_top8.v`（工程基线）与
`zdt_top21.v`（当前版本），中间迭代版本见 `vivado/scripts/rebuild_zdt*.tcl`。

### 验证

```bash
iverilog -g2005 -o ik.vvp src/rtl/ik_solver.v sim/tb_ik_solver.v && vvp ik.vvp
iverilog -g2005 -o itp.vvp src/rtl/zdt_interp.v sim/tb_zdt_interp.v && vvp itp.vvp
```

## PS 端脚本与验证工具

- `ps/pik.sh` —— 坐标 → 机械臂。只是写坐标、触发、读回关节角，逆解在 PL 内完成。
  用法：`pik X Y Z` / `pik -i X Y Z 3000`（插补）/ `pik -t X Y Z`（只求解不动作）
- `tools/final_ik_verify.py` —— 16 点上板精度验证（实测 16/16，最大偏差 0.043°）
- `tools/extract_ik_params.py` —— 从板端未 strip 的 `ik_zdt_pl` 二进制提取真实机构参数
- `tools/fit_ik_params.py` —— 用样本反推机构参数（结论：二连杆模型套不上整机，以 `/etc/ik.conf` 为准）

细节见 `ps/README.md`。
