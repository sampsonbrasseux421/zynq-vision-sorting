---
name: zdt-driver-timing
description: Determine the minimum safe frame interval and the enable strategy for ZDT-series closed-loop stepper drivers, using driver replies as the acceptance criterion instead of command return codes.
license: MIT
---

# ZDT driver timing and enable strategy

## 1. When to use

- A ZDT (or protocol-compatible) closed-loop stepper driver **drops frames**
  when several position frames are sent back-to-back.
- You need to know **how long to wait between frames**, and whether the driver
  must be re-enabled before every move.
- Your host sends commands faster than the motors move, and you suspect the
  driver is silently ignoring some of them.

## 2. When NOT to use

- The bus is RS485 with a dedicated transceiver and hardware flow control.
- The driver has an explicit "ready" handshake pin you can sample.
- You only send one command per second — the interval question is moot.

## 3. What you get

| File | Purpose |
|---|---|
| `scripts/probe_axis.sh` | Read one axis status byte through the UART, print the raw reply |
| `scripts/gap_sweep.sh` | Sweep the frame interval and report which values still get replies |
| `SKILL.md` | Method, judgement criterion, failure conditions |

## 4. Method

### 4.1 The acceptance criterion

**Never use "the command returned" as success.** A write into a UART FIFO
returns immediately; the driver may still ignore the frame.

The only reliable criterion is the **driver reply**:

```
[addr][cmd][status][0x6B]
```

| Byte | Meaning |
|---|---|
| `addr` | axis address, echoes the request |
| `cmd` | command code (e.g. `0x3A` = read status) |
| `status` | bit0 = enabled, bit1 = in position, bit2 = stall |
| `0x6B` | checksum |

### 4.2 The sweep

1. Clear the receive FIFO.
2. Send **one** frame.
3. Wait `N` ms.
4. Read the FIFO; a valid 4-byte reply means the frame was accepted.
5. Repeat for `N = 60, 40, 30, 20, 10`.

Typical result on ZDT X42S at 115200 8N1:

| Interval | Reply | Verdict |
|---|---|---|
| 60 ms | yes | **safe** |
| 40 ms | yes | safe |
| 30 ms | intermittent | unsafe |
| 20 ms | no | rejected |
| 10 ms | no | rejected |

### 4.3 The enable question

Drivers keep the enable state until power-off or protection trip. Measure it:

1. Send one enable frame.
2. Send **nothing** for 120 s.
3. Read the status byte.

If `bit0` is still set, the enable can be skipped on subsequent moves.

This is what makes a **fast mode** possible: skip the enable burst and send
only the position frames.

| Mode | Frames | Time @60 ms |
|---|---|---|
| full | 12 enable + 4 position + 1 sync | 1.05 s |
| fast | 4 position + 1 sync | 0.30 s |

**Keep one full command** (e.g. homing) as the recovery path: it re-enables
everything.

## 5. Bitter lessons

- **Symptom looked like a software bug.** The host printed "command sent" for
  every move, yet one axis never moved. Cause: that axis missed the enable
  frames, and nobody read the status byte.
- **The interval is a property of the driver, not of the UART.** Raising the
  baud rate does not help; the frame gap is what matters.
- **Enable frames are the most likely to be dropped** after an idle period —
  they are the first frames on the wire. Sending 3 attempts per axis is a cheap
  guard.

## 6. Failure conditions

This package stops being valid if:

- the driver model or firmware changes (re-measure the sweep);
- the bus becomes RS485 half-duplex with echo (reply parsing changes);
- a driver protection trip occurs — the enable state is cleared and the fast
  mode will silently fail until a full command is sent.
