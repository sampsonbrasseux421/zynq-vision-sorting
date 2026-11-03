# System architecture

## 1. Partition

```
                 Ethernet (TCP)
  ┌──────────┐   commands :5000    ┌──────────────────────────────┐
  │  Host PC │ ──────────────────► │  Zynq-7020                   │
  │          │   video    :5002    │                              │
  │  YOLO    │ ──────────────────► │  PS: command server          │
  │  VLM     │                     │      inverse kinematics      │
  │  state   │                     │      register writer         │
  │  machine │                     │                              │
  └──────────┘                     │  PL: command engine          │
                                   │      video overlay (OSD)     │
                                   │      servo PWM               │
                                   └───────┬───────────┬──────────┘
                                           │ UART      │ HDMI
                                    4 × ZDT drivers   display
                                           │
                                      stepper motors
```

## 2. Why the split is where it is

| Function | Where | Reason |
|---|---|---|
| Detection | Host | no camera input path into the PL on this board; the host already owns the USB camera |
| Task decision (VLM) | Host | CPU-only inference, tens of seconds, must not block control |
| Inverse kinematics | PS | once per move, ~5 µs, 0.0005 % of the cycle — moving it to PL would cost ~3,000 LUT for no gain |
| Frame sequencing | **PL** | this is the real timing bottleneck: inter-frame gap is fixed by the driver at 60 ms and OS scheduling adds ±20 ms jitter |
| Video overlay | **PL** | must survive a host link failure; character ROM and pixel blending cost ~3,300 LUT |
| Gripper PWM | **PL** | 50 Hz must not depend on Linux scheduling |

## 3. Data flow

### 3.1 Motion command

```
host            PS                        PL                   driver
 │  "245 0 130" │                          │                     │
 ├─────────────►│ ik_solve()               │                     │
 │              │ angle -> pulses          │                     │
 │              ├─ write POS1..POS4 ──────►│                     │
 │              ├─ write CTRL(start) ─────►│                     │
 │              │                          │ 12/0 enable frames ─►│
 │              │                          │ 4 position frames ──►│
 │              │                          │ 1 sync trigger ─────►│
 │              │◄─ poll STATUS.done ──────┤                     │
 │◄─ "OK GOTO" ─┤                          │                     │
```

### 3.2 Display

```
host frame ──TCP:5002──► PS ──/dev/mem──► DDR framebuffer
                                             │
                                     VDMA ───┴──► overlay (PL) ──► HDMI
```

## 4. Time synchronisation

There is no shared clock between host and board. Time enters the system in two
places and both are handled explicitly:

- **Belt prediction.** The host timestamps the frame it detected the block in,
  measures the processing latency itself, and predicts the intercept point from
  that age. The board is not involved.
- **Arrival detection.** The board reports `STATUS.done` when the frame burst
  has been sent, then waits a fixed settle time. The host polls this via the
  `WAIT` command instead of sleeping blindly.

## 5. Failure handling

| Failure | Detection | Response |
|---|---|---|
| Host link lost | TCP timeout | board keeps the last position; OSD keeps showing board state |
| Driver silent | no reply on the UART | status byte not read as OK; homing re-enables |
| Driver protection trip | status bit0 = 0 | full command (homing) clears it |
| Axis overload | repeated failed moves | speed/acceleration reduced |
| VLM unavailable | import or inference error | falls back to the rule-based picker |
