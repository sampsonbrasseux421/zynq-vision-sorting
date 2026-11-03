# Register map (reference implementation)

Base address: `0x4000_0000`, AXI4-Lite, 32-bit, 6-bit address.

| Offset | Name | R/W | Description |
|---|---|---|---|
| `0x00` | `CTRL` | W | bit0 start, bit1 abs/rel, bit2 sync enable, bit3 uart select, bit4 skip enable |
| `0x04` | `TIMING` | W | reserved / per-move timing |
| `0x08` | `POS1` | W | target pulses, axis 1 |
| `0x0C` | `POS2` | W | axis 2 |
| `0x10` | `POS3` | W | axis 3 |
| `0x14` | `POS4` | W | axis 4 |
| `0x18` | `STATUS` | R | bit0 busy, bit1 done, [5:3] current axis, [31:16] frame count |
| `0x1C` | `CLR` | W | clear done / frame counter |
| `0x20` | `GAP_MS` | W | inter-frame delay in milliseconds |

## Two traps worth copying

### 1. Preserving high control bits

A masked write like

```verilog
reg_ctrl <= {28'd0, s_axi_wdata[3:1], 1'b0};
```

drops any bit above the mask. When a later feature uses bit 4, the write
silently clears it. Use a full-width slice:

```verilog
reg_ctrl <= {27'd0, s_axi_wdata[4:1], 1'b0};
```

### 2. Read-back of a wider address

If the read case selects on `araddr[5:2]` (four bits) the case items must be
`4'dN`, not `3'dN`. A `3'd8` item is truncated and the register at `0x20`
reads back as zero forever, which looks like a write failure.
