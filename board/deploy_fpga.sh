#!/bin/sh
# deploy_fpga.sh -- load a bitstream through fpga_manager and restart the video path.
#
# The hdmi-video service owns the VTC/VDMA initialisation. Stopping it is
# required before reconfiguring the PL, and starting it again is required
# afterwards -- forgetting the restart leaves a black screen.

set -e

BITBIN=${1:-/lib/firmware/camera_hdmi_top.bit.bin}

/etc/init.d/hdmi-video stop 2>/dev/null || true
rmmod vdmafb_test 2>/dev/null || true
sleep 1

echo "$(basename "$BITBIN")" > /sys/class/fpga_manager/fpga0/firmware
sleep 2
echo -n "fpga state: "; cat /sys/class/fpga_manager/fpga0/state

# MANDATORY: restart the video service, otherwise the screen stays black
/etc/init.d/hdmi-video start camera
sleep 3

echo -n "VDMA: "; devmem 0x43000004 32
