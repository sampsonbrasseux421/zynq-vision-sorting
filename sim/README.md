# Simulation & verification

This directory holds the self-checking testbenches used to confirm functional
correctness of the PL design. Every testbench compares the **actual serial byte
stream** captured from the `tx` pin against an expected byte sequence — the
design is verified at the wire level, not by inspecting internal signals.

---

## 1. How to run

Requires [Icarus Verilog](http://iverilog.icarus.com/) 11 or later.

```bash
# one-shot: run everything and write logs to results/
sh run_all.sh                       # Linux / macOS / WSL
pwsh -File run_all.ps1              # Windows PowerShell
```

Single testbench:

```bash
iverilog -g2012 -o sim.out tb_zdt_frame_tx.v ../src/rtl/zdt_frame_tx.v
vvp sim.out
```

> Note the RTL path: the testbenches live in `sim/`, the sources in `src/rtl/`.

---

## 2. What each testbench verifies

| Testbench | DUT | What it proves | Expected byte count |
|---|---|---|---|
| `tb_zdt_frame_tx` | `zdt_frame_tx` | Single 13-byte position frame: header, command, direction, speed, acceleration, position field, checksum, and UART bit timing. `fsel` selects the frame type. | 13 |
| `tb_zdt_tx_engine` | `zdt_tx_engine` | One trigger produces the full 12 enable frames + 4 position frames; per-frame byte comparison for addresses 1–4; `frame_count` = 16. | 124 |
| `tb_zdt_engine3` | `zdt_tx_engine` | Same sequence with a **non-zero frame gap**: checks that `gap_ms` actually inserts the delay and that the ordering is preserved. | 124 |
| `tb_zdt_sync` | `zdt_tx_engine` | `sync_en = 1` adds one broadcast trigger frame at the end; the trigger frame must be exactly `00 FF 66 6B`. | 128 |
| `tb_zdt_axil` | `zdt_axil` | End-to-end register path: write 4 target positions and timing parameters over AXI4-Lite, set `CTRL.start`, poll `STATUS.done`, then compare the emitted byte stream. | 124 |
| `tb_color_blob_detector` | `color_blob_detector` | 640×480 synthetic frame with 4 known colour blobs; compares per-colour count, Σx, Σy against testbench-computed expectations; also checks the pass-through channel is bit-identical. | — |
| `tb_roi_check` | `color_blob_detector_roi` | Full-size 1024×600 colour-bar frame; asserts every bounding box falls inside the ROI window (x 120…1016, y 200…450). | — |

---

## 3. Recorded results

All seven testbenches were run on 2026-11-03 with Icarus Verilog 11.0; the raw
output is committed under [`results/`](results/).

| Testbench | Result | Key output |
|---|---|---|
| `tb_zdt_frame_tx` | **PASS** | `===== 全部通过 =====` |
| `tb_zdt_tx_engine` | **PASS** | `收到 124 字节`, `frame_count = 16`, one trigger = 100 015 ns |
| `tb_zdt_engine3` | **PASS** | `收到 124 字节`, `frame_count = 16` |
| `tb_zdt_sync` | **PASS** | `收到 128 字节`, `frame_count = 17`, trigger frame = `00 FF 66 6B` |
| `tb_zdt_axil` | **PASS** | `STATUS = 00100022`, `收到 124 字节`, `frame_count = 16` |
| `tb_color_blob_detector` | **PASS** | 19 × `[ ok ]`, no `[FAIL]` |
| `tb_roi_check` | **PASS** | `===== ROI 生效 =====`, all bboxes inside the window |

Reproduce with `sh run_all.sh`; each run overwrites the logs in `results/`.

---

## 4. Frame layout (what the tests are checking)

Position frame — 13 bytes:

```
[addr] [0xFD] [dir] [spd_h] [spd_l] [acc] [pos3] [pos2] [pos1] [pos0] [abs] [sync] [0x6B]
```

Enable frame — 6 bytes:

```
[addr] [0xF3] [0xAB] [0x01] [0x00] [0x6B]
```

Synchronised-trigger frame — 4 bytes (broadcast to all axes):

```
0x00  0xFF  0x66  0x6B
```

The engine emits **12 enable frames + 4 position frames + 1 trigger frame**
when `sync_en = 1`, i.e. 72 + 52 + 4 = **128 bytes** ≈ 1.05 s at a 60 ms
inter-frame gap. With `skip_en = 1` the 12 enable frames are omitted, leaving
5 frames ≈ 0.22 s.

---

## 5. Known limitations

| Item | Note |
|---|---|
| Frame-gap accuracy | `tb_zdt_engine3` proves the gap is inserted, but the absolute 60 ms figure was established on hardware with a five-step sweep (60/40/30/20/10 ms), not in simulation. |
| No timing closure check | These testbenches are functional only. Synthesis/implementation timing is covered by the Vivado reports in `build/`. |
| No back-to-back UART traffic | Driver replies arriving during transmission are not modelled, because `uart_rx` is not connected in the current design. |
| Detector tests are standalone | `color_blob_detector` is not part of the final data path; its testbenches are kept for reference only. |
