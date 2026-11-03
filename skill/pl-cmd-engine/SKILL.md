---
name: pl-cmd-engine
description: Move a host-side serial command sequencer into FPGA logic with a small AXI4-Lite register map, keeping the frame interval configurable at runtime.
license: MIT
---

# PL command engine

## 1. When to use

- The host builds protocol frames in software and sleeps between them; the
  timing jitter of the OS scheduler shows up in the motion.
- You send the **same sequence of frames** on every move (enable burst,
  position frames, sync trigger) and only the payload changes.
- You want the host to do **six register writes** instead of generating a
  protocol.

## 2. When NOT to use

- The protocol needs variable-length payloads or a checksum over arbitrary data.
- The frame rate is below ~10 Hz — jitter of a few ms is invisible and the
  extra logic is not justified.
- You cannot afford any additional LUTs, or the design is already timing
  critical.

## 3. What you get

A three-level structure that has been used as-is in two projects:

```
zdt_axil        AXI4-Lite slave, register file, start/done handshake
  └ zdt_tx_engine   axis sequencing + inter-frame timing + sync trigger
      └ zdt_frame_tx    one frame: assemble bytes, serialize 8N1
```

| File | Purpose |
|---|---|
| `register_map.md` | Register offsets, bit fields, and the read-back quirk |
| `SKILL.md` | Module split, verification method, failure conditions |

## 4. Method

### 4.1 Split by "what changes"

| Concern | Module | Why |
|---|---|---|
| Bytes on the wire | `zdt_frame_tx` | pure serialization, testable with one frame |
| Which frame comes next | `zdt_tx_engine` | sequencing + timing, testable with a counter |
| Host access | `zdt_axil` | register decode, testable with a bus testbench |

### 4.2 Put every timing constant in a register

The frame interval belongs to the driver, not to your design. Expose it:

```
GAP_MS register -> engine counter -> inter-frame delay
```

This single decision made the whole bring-up possible: the interval could be
swept on hardware without rebuilding the bitstream.

### 4.3 Expose a frame counter

Add `frame_count` to the status word. It turns "did the sequence run?" into a
**single read**: after one move the counter must advance by the exact number of
frames (17 in our case).

This catches a whole class of bugs that a waveform would take minutes to find.

### 4.4 Verification order

1. **One frame** — bit-level testbench, check start/stop bits and checksum.
2. **Sequence** — count the frames, check the axis order and the gap.
3. **Bus** — drive the AXI slave with AW and W in different cycles.
4. **Board** — read the frame counter after one move.

## 5. Bitter lessons

- **A `module_ref` IP in a Block Design is cached by module name.** Editing the
  RTL under the same name does not regenerate the IP; you ship the old logic and
  debug a design that is not yours. Renaming the module (or the instance) is the
  cheap workaround.
- **Do not mask a control register by the wrong width.** A write mask of
  `wdata[2:0]` silently cleared bit 3, so the UART source-select never took
  effect and every command went to the wrong path.
- **A counter beats a waveform.** Reviewers and future users can check a counter
  in one second; a waveform needs the right trigger and the right zoom.

## 6. Failure conditions

- The driver needs a different sequence per move (then the sequence belongs in
  software, and only the serializer should stay in hardware).
- The AXI clock and the UART bit clock come from unrelated sources and the
  interval must be exact in absolute time.
- The protocol becomes bidirectional and latency-sensitive (move to a DMA +
  descriptor design instead).
