---
name: axi-lite-slave-check
description: Review checklist and testbench template for hand-written AXI4-Lite slaves, focused on the handshake assumptions that hang the interconnect.
license: MIT
---

# AXI4-Lite slave review

## 1. When to use

- You wrote an AXI4-Lite slave by hand (register file, FIFO control, custom IP).
- The design works in simulation but the board shows a **blue screen, frozen
  video, or a bus that stops responding**.
- You are about to integrate a new slave into an existing interconnect.

## 2. When NOT to use

- You use Xilinx `AXI GPIO` / `axi_bram_ctrl` / generated IP.
- The bus is AXI4 full with bursts — different failure modes.

## 3. What you get

| File | Purpose |
|---|---|
| `tb_axi_handshake.v` | Testbench that drives AW and W in the same cycle, in different cycles, and out of order |
| `SKILL.md` | Checklist and the failure mode that motivated it |

## 4. Checklist

Run through this before every integration:

- [ ] `bvalid` is **not** `awvalid && wvalid` — latch AW and W independently
- [ ] `arready` is driven (a write-only slave must still answer reads, or the
      interconnect returns SLVERR and the CPU takes a bus error)
- [ ] `rvalid` is cleared by `rready`, not by the next `arvalid`
- [ ] `awready` / `wready` do not depend on `bready`
- [ ] every register is readable, even if only for debug
- [ ] write masks cover **all** used bits
- [ ] read `case` selector width matches the address slice width
- [ ] no combinational loop between `*ready` and `*valid`

## 5. The failure that motivated this

A character-overlay module was inserted between the video DMA and the video
output. The screen turned solid blue.

The slave generated `bvalid` from `awvalid && wvalid` in the same cycle. The
interconnect happened to present AW and W in different cycles, so `bvalid`
never asserted; the write channel stalled; the interconnect stopped servicing
the video path.

Fix: latch AW and W independently, then assert `bvalid` when both are held.

```verilog
always @(posedge aclk) begin
    if (!aresetn) begin aw_lat <= 0; w_lat <= 0; end
    else begin
        if (s_axi_awvalid && s_axi_awready) aw_lat <= 1'b1;
        if (s_axi_wvalid  && s_axi_wready)  w_lat  <= 1'b1;
        if (s_axi_bvalid  && s_axi_bready)  begin aw_lat <= 0; w_lat <= 0; end
    end
end
assign s_axi_bvalid = aw_lat && w_lat;
```

## 6. Verification order

1. Same-cycle AW/W.
2. Different-cycle AW then W.
3. Different-cycle W then AW.
4. Read of a write-only register (must not hang).

If all four pass in simulation, the risk of an interconnect hang drops a lot.

## 7. Bitter lesson

A bus hang does not look like a bus problem. In our case it looked like a video
problem, and the first instinct was to debug the video timing. The tell was that
**the whole path stopped**, not just the overlay: when one slave stalls the
interconnect, every master behind it stalls too.

## 8. Failure conditions

- The interconnect is configured with a different protocol (AXI3) and expects
  different ready/valid behaviour.
- Clock domain crossing is involved — this checklist does not cover CDC.
