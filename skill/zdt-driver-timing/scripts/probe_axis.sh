#!/bin/sh
# probe_axis.sh <axis_addr> [uart_base]
# Read the status byte of one ZDT axis and print the raw reply.
#
# The base address is the AXI UART Lite register block used by the PS.
# Register map: +0x00 RX FIFO, +0x04 TX FIFO, +0x08 STAT, +0x0C CTRL.

AXIS=${1:-1}
UL=${2:-0x42C00000}

clear_rx() {
    devmem $((UL + 0x0C)) 32 0x02 > /dev/null
    sleep 0.2
    i=0
    while [ $i -lt 32 ]; do
        st=$(devmem $((UL + 0x08)) 32)
        [ $((st & 1)) -eq 0 ] && break
        devmem $UL 32 > /dev/null
        i=$((i + 1))
    done
}

clear_rx
# 0x3A = read status; 0x6B = checksum
printf "\x$(printf %02x $AXIS)\x3A\x6B" > /dev/ttyS0 2>/dev/null || true
# If you drive the UART through /dev/mem instead, write the three bytes to +0x04.
sleep 0.5

echo -n "axis $AXIS reply: "
n=0
while [ $n -lt 8 ]; do
    st=$(devmem $((UL + 0x08)) 32)
    [ $((st & 1)) -eq 0 ] && break
    v=$(devmem $UL 32)
    printf "%02X " $((v & 255))
    n=$((n + 1))
done
[ $n -eq 0 ] && echo "(no reply)" || echo ""
