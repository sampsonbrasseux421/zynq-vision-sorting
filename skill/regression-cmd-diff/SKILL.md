---
name: regression-cmd-diff
description: Locate a behaviour regression by diffing the command stream against a known-good backup, instead of reading the diff of the source code.
license: MIT
---

# Behaviour-level regression bisect

## 1. When to use

- A refactor changed several files and the machine now behaves differently.
- Reading the source diff is slow because the change spans many commits.
- You have (or can rebuild) a **runnable known-good version**.

## 2. When NOT to use

- There is no known-good baseline (then bisect by reverting single commits).
- The symptom only appears intermittently — a command diff will not show it.

## 3. What you get

| File | Purpose |
|---|---|
| `scripts/diff_cmdlog.py` | Align two command logs per cycle and report the first divergence |
| `SKILL.md` | The method and the case it solved |

## 4. Method

1. **Capture the command stream**, not the source. The board's command log with
   timestamps is ideal (`RX=[...]` lines).
2. **Pick one cycle** — from "grab" to "ready for next grab" — in both versions.
3. **Align** the two streams. Ignore timing columns; compare only the command
   payload sequence.
4. **Find the first divergence.** Everything after it is a consequence.
5. **Map the divergent command back to a source branch**, then decide whether it
   is a *protection* or a *redundancy*.

## 5. The case this solved

Symptom: after a refactor, the arm swept the whole workspace at full extension
right after placing a block.

Source diff: dozens of changed lines, mostly refactoring noise.

Command diff (one cycle):

| Step | Known-good | Regressed |
|---|---|---|
| 1 | `goto 134 -107 200` (retract to 55 % radius) | — **missing** |
| 2 | `goto 243 -156 200` | `goto 243 -156 200` |
| 3 | `goto 243 -156 90` | `goto 243 -156 90` |
| 4 | `BOX 28160` | `BOX 28160` |

One missing line. It looked like a redundant intermediate pose; it was in fact
the **safety retract** before a large base rotation. Restoring it fixed the
symptom.

## 6. Bitter lesson

> An intermediate pose that looks redundant is usually a safety constraint.
> Delete it only after you can explain what it protects.

Corollary: keep a **runnable** known-good build (bitstream + host program +
config) at every milestone. A backup that needs two hours to rebuild is not a
baseline.

## 7. Failure conditions

- The command stream is not deterministic (e.g. depends on wall-clock timing) —
  alignment becomes ambiguous.
- The divergence is in the *timing* of identical commands; compare timestamps
  instead of payloads.
