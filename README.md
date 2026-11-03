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
├── build/        Vivado TCL script and bootgen configs
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
