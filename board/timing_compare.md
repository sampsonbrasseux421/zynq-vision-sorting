# Measured performance — before / after

This file is the source of truth for the numbers quoted in the design report
(part 3.1). Everything here was measured on the physical rig under the
conditions stated. Figures that are derived rather than measured are marked
**(derived)**.

---

## 1. Measurement conditions

| Item | Value |
|---|---|
| Board | Zynq-7020, XC7Z020CLG400-2, PetaLinux 2023.1, kernel 6.1.5 |
| PL clock | FCLK_CLK0 = 50 MHz |
| Driver link | TTL UART 115200 8N1, 3.3 V |
| Axes in motion | 4 (addresses 1–4), 5th installed but not driven |
| Frame gap | 60 ms (factory default used in all "before/after" rows) |
| Load | arm free, no payload, room temperature |
| Command source | host PC over TCP 192.168.1.10:5000 |

---

## 2. Motion timing

| Metric | Before | After | How it was obtained |
|---|---|---|---|---|
| One four-axis move, full mode | 1.49–1.67 s | **1.013 s** (σ 6.2 ms, n=10) | `tools/log_timing.py --moves 10`; timestamps bracket the TCP round trip |
| One four-axis move, fast mode | 1.05 s | **0.229 s** (σ 4.7 ms, n=10) | Same, with `CTRL[4]` (skip-enable) set |
| Motion-only pick-and-place cycle | — | **2.65 s** (σ 0.02, n=10) | `--cycle 10`: grab → close → lift → carry+place → open → lift |
| End-to-end cycle per block | ~13 s | **~7–8 s** | Wall clock in the full pipeline, which also waits for the next block to reach the pick zone |
| Inter-axis start skew | ~240 ms | **µs level** | Logic-analyser capture of the four UART lines during one move |
| Frames per full move | — | **17** (12 enable + 4 position + 1 trigger) | Counted from `STATUS.frame_count` |
| Frames per fast move | — | **5** (4 position + 1 trigger) | Same |

**Derived cross-check** — 17 frames × 60 ms = 1.02 s, plus UART transmission
time (≈221 bytes ≈ 19 ms at 115200) ≈ **1.04 s**, against the measured
**1.013 s**. Fast mode: 5 × 60 ms + 5 bytes ≈ 0.22 s against a measured
**0.229 s**.

**Two different jitters — do not conflate them**

| Layer | Value | What it is |
|---|---|---|
| PL frame timing | **0** | The inter-frame gap comes from a fixed counter, so every burst is identical |
| End-to-end command | **±7 ms** (σ 6.2 over 10 runs) | TCP connect, inetd process spawn, PS register write — none of it inside the PL |

The "±20 ms → 0" claim in the design report refers to the **PL frame timing**
row only. The end-to-end figure is dominated by per-connection process startup
on the PS (~30 ms of the 1013 ms), which is why the fast mode is not
proportionally faster than the frame-count ratio suggests.

**Motion cycle vs. end-to-end cycle** — the 2.65 s figure is the arm moving
through the six commands with no waiting. The 7–8 s per-block number includes
waiting for the next block plus detection latency. Both are real; they answer
different questions.

---

## 3. Why the move got shorter

| Change | Effect |
|---|---|
| Frame sequencing moved from PS to PL | Jitter removed; every move takes the same time |
| Broadcast sync trigger | Axes start together instead of ~240 ms apart |
| Skip-enable mode (`CTRL[4]`) | 12 frames (≈0.72 s) omitted from steady-state moves |
| Runtime `GAP_MS` register | Gap tunable without rebuilding the bitstream |

The enable stage is **not** dropped for `HOME`: after a power cycle or a driver
protection trip, the full 17-frame sequence re-enables the drivers. The drivers
were verified to hold enable for at least 120 s idle, which is what makes the
fast mode safe.

---

## 4. PL resource cost

| Item | LUT | Share | Note |
|---|---|---|---|
| Command engine (`zdt_tx_engine`) | 231 | 0.43 % | Added in this work |
| Character OSD | ≈3,300 | ≈6.2 % | Added in this work, includes the 16×16 glyph ROM |
| Video path / servo PWM / UART mux | ≈5,750 | ≈10.8 % | Pre-existing |
| **Total (slice LUTs)** | **9,213** | **17.32 %** | 8,943 as logic + 270 as memory |

Device total is 53,200 LUT, so the design leaves ~44,000 LUT free.

Full figures from the same report:

| Resource | Used | Available | Utilisation |
|---|---|---|---|
| Slice LUTs | 9,213 | 53,200 | 17.32 % |
| — as logic | 8,943 | 53,200 | 16.81 % |
| — as memory (SRL / distributed RAM) | 270 | 17,400 | 1.55 % |
| Slice registers | 12,987 | 106,400 | 12.21 % |
| Block RAM tiles | 5.5 | 140 | 3.93 % |
| DSP48E1 | 2 | 220 | 0.91 % |
| BUFGCTRL | 5 | 32 | 15.63 % |
| MMCME2_ADV | 1 | 4 | 25.00 % |

> These figures come from the Vivado implementation report. The report file
> itself (`build/utilization.rpt`) should be exported alongside this document —
> see the note at the end of this file.

---

## 4b. Timing and power (implementation report)

| Metric | Value | Meaning |
|---|---|---|
| WNS | **+4.551 ns** | Worst negative slack at 50 MHz — design closes with margin |
| TNS | 0.000 ns | No failing paths out of 27,538 |
| WPWS | +2.448 ns | Pulse-width check clean, 0 failing endpoints |
| Fmax (derived) | ≈ **64.7 MHz** | 1 / (20 ns − 4.551 ns) at the same routing |
| Total on-chip power | 1.733 W | dynamic 1.586 W + static 0.147 W, default toggle rates |
| Junction temperature | 45.0 ℃ | Estimated |

The design is therefore not timing-limited: the 50 MHz PL clock could be raised to
about 64 MHz without re-timing, which is why the 60 ms inter-frame gap — not the
clock — sets the motion latency.

---

## 5. Reproducing these numbers

```bash
python tools/log_timing.py --moves 10 --out measured/move_log.csv
python tools/log_timing.py --moves 10 --fast --out measured/move_log_fast.csv
python tools/log_timing.py --cycle 10 --out measured/cycle_log.csv
```

Recorded on 2026-11-03 (n = 10 each): full mode 1013.3 ms, fast mode
228.5 ms, motion cycle 2.65 s. The CSVs are committed; re-running overwrites
them.

The tool records `t0` immediately before the TCP write and `t1` when the
`OK` line arrives, so the number includes network and PS processing — that is
the figure a user actually experiences, which is why it is the one quoted.

---

## 6. Known gaps

| Gap | Status |
|---|---|
| Vivado utilization / timing `.rpt` | **To be exported** — the LUT figures above are copied from the report but the file must be committed under `build/` |
| Raw per-sample logs | **Committed** under `measured/` — `move_log.csv`, `move_log_fast.csv`, `cycle_log.csv`, all recorded 2026-11-03 |
| Oscilloscope / logic-analyser screenshots | Not committed; the skew figure (~240 ms → µs) is from a single capture |
