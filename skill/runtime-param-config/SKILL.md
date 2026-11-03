---
name: runtime-param-config
description: Move geometry and tuning parameters out of the bitstream into a runtime config file shared by the FPGA host and the PC, so field tuning does not require a rebuild.
license: MIT
---

# Runtime parameter configuration

## 1. When to use

- A parameter is **tuned during field调试** (placement height, approach radius,
  dwell time) and every change currently costs a synthesis run.
- Two programs (board side and PC side) both need the same geometry constant,
  and they drift apart.

## 2. When NOT to use

- The parameter affects synthesis (array sizes, pipeline depth).
- The value must be known at reset (clock dividers, FIFO depths).
- Changing it wrongly could damage hardware — those belong in a compiled safety
  limit, not in a text file.

## 3. What you get

| File | Purpose |
|---|---|
| `example_params.conf` | A minimal config file with comments |
| `SKILL.md` | The pattern, the two-sided consistency rule, failure conditions |

## 4. The pattern

```
params.conf  ──(read at boot / per command)──>  board side
      │
      └────────(read at startup)──────────────>  PC side
```

Rules:

1. **One file, two readers.** The PC must read the same file (or a copy synced
   at connect time), never a hard-coded duplicate.
2. **The board reads it per command**, not once at boot, so a change takes
   effect on the next move.
3. **The PC updates its own state model** after the board reports the move. If
   the PC keeps a stale idea of where the arm is, the next predicted target is
   computed from a wrong start point.

## 5. Worked example

Field problem: the collection boxes were lower than the conveyor, so the arm had
to descend before releasing. The descent radius and height changed every time
the boxes were repositioned.

Before: edit RTL/software → synthesis → bitstream → deploy (tens of minutes).

After:

```sh
PLACE_R=200     # descent radius, mm
PLACE_Z=90      # descent height, mm
```

```sh
# board side, evaluated per command
PR=$(grep -E '^PLACE_R=' /etc/app.conf | cut -d= -f2)
PZ=$(grep -E '^PLACE_Z=' /etc/app.conf | cut -d= -f2)
```

Tuning cycle dropped from a rebuild to one command.

## 6. Bitter lesson

- **A duplicated constant will drift.** We had the same radius in the RTL block
  design, in the PC program and in a shell script; after two weeks they
  disagreed and the arm placed blocks 30 mm off. After moving to one file, the
  disagreement became impossible.
- **The PC's idea of the arm position is a separate bug source.** Even with the
  parameters shared, if the PC does not update its `last_position` after a move,
  the next approach distance is computed from a stale point and the safety check
  fires for the wrong reason.

## 7. Failure conditions

- The parameter changes the hardware structure (then it must be a generic).
- Multiple boards share one file and need different values (use per-board files).
- The file is unreadable at boot — the firmware must fall back to safe defaults,
  never to zero.
