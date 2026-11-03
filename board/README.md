# Board-side artifacts and measurements

Everything needed to get the design onto the Zynq-7020 and to reproduce the
numbers quoted in the design report.

| File | Content |
|---|---|
| `deploy_fpga.sh` | Load a bitstream through `fpga_manager` and restart the video service |
| `deploy_ps.sh` | Install `ik_zdt_pl`, `arm_net.sh` and register port 5000 in inetd |
| `timing_compare.md` | Measured before/after comparison, with method and conditions |
| `tools/log_timing.py` | Generate a real per-move timing log (CSV) from the running board |
| `measured/` | Output directory for logs produced by `tools/log_timing.py` |

---

## 1. Bring-up order

```bash
# 1. bitstream  (requires hdmi-video stopped and vdmafb_test unloaded)
sudo sh deploy_fpga.sh /lib/firmware/camera_hdmi_top.bit.bin

# 2. PS binaries and the TCP command service
sudo sh deploy_ps.sh
```

`deploy_fpga.sh` deliberately restarts `hdmi-video` at the end — reloading the
PL without restarting the VTC/VDMA leaves the display black, which is the most
common bring-up mistake.

---

## 2. Verifying the board is alive

```bash
# command channel
echo PING | nc 192.168.1.10 5000          # expect PONG

# register path (write-only slave: reads return a bus error by design)
sudo devmem 0x40000018 32                 # STATUS: bit0 busy, bit1 done

# driver health: read the reply byte, not the software return code
sudo ik_zdt_pl status
```

---

## 3. Producing the timing log

The report quotes per-move and per-cycle timings. To regenerate them on your
own hardware:

```bash
python tools/log_timing.py --moves 10 --out measured/move_log.csv
python tools/log_timing.py --cycle 10 --out measured/cycle_log.csv
```

The tool issues real commands over TCP, timestamps each round trip, and writes
one row per event. It refuses to run if `PING` fails, so a broken link cannot
silently produce an empty log.

---

## 4. What the report quotes, and where it came from

All figures in `timing_compare.md` were measured on the physical rig
(conveyor running, camera live, drivers energised). Conditions and method are
stated next to each number. Where a figure is derived rather than measured —
for example the 1.05 s full-move time computed from 17 frames × 60 ms — it is
labelled as such.

If you re-run the measurements, update `timing_compare.md` rather than editing
the design report, so that the two stay consistent.
