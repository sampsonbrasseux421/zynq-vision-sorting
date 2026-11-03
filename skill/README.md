# Skill packages

Reusable capability packages extracted from this project. Each package can be
dropped into a **different** project; none of them depends on this repository's
application code.

## Index

| Package | Type | Reuse scenario | Verified on |
|---|---|---|---|
| [zdt-driver-timing](zdt-driver-timing/SKILL.md) | checklist + scripts | Any ZDT / serial stepper driver where frames are dropped | 4-axis arm, 5 interval settings |
| [pl-cmd-engine](pl-cmd-engine/SKILL.md) | design template | Moving a host-side frame sequencer into FPGA logic | 17-frame sequence, 231 LUT |
| [axi-lite-slave-check](axi-lite-slave-check/SKILL.md) | checklist + testbench | Reviewing a hand-written AXI4-Lite slave before integration | caused a blue-screen, fixed |
| [regression-cmd-diff](regression-cmd-diff/SKILL.md) | method + script | Behaviour regression after a refactor, with a known-good backup | located a deleted safety branch |
| [local-vlm-async](local-vlm-async/SKILL.md) | code template | Small VLM/LLM used as a slow advisor in a real-time loop | 256M model, CPU only |
| [runtime-param-config](runtime-param-config/SKILL.md) | pattern | Geometry / tuning parameters that change during field调试 | placement height, radius |

## How each package is written

Every `SKILL.md` follows the same structure, so a new team can read only the
first two sections and decide whether it applies:

1. **When to use / when NOT to use**
2. **What you get** (files, interfaces)
3. **Method** (steps, with the actual measurement or review criteria)
4. **Bitter lessons** — where each rule came from
5. **Failure conditions** — how the package stops being valid

## Extraction rule

A lesson only becomes a skill package if it is:

- **Observable** — it produced a measurable difference, not a preference;
- **Transferable** — it does not mention this project's colours, boxes or arm;
- **Bounded** — its failure conditions can be stated in one sentence.
