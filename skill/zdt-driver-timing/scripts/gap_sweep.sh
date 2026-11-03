#!/bin/sh
# gap_sweep.sh
# Sweep the frame interval and report which values still produce a driver reply.
# Assumes the interval is writable through a hardware register (see pl-cmd-engine).

GAP_REG=${GAP_REG:-0x40000020}
UL=${UL:-0x42C00000}

for G in 60 40 30 20 10; do
    devmem $GAP_REG 32 $G > /dev/null
    sleep 0.3
    devmem $((UL + 0x0C)) 32 0x02 > /dev/null    # clear RX FIFO
    sleep 0.2

    # one position frame to axis 1 (adjust for your tool)
    ik_zdt -m -P -r 3000 -A 255 goto 250 0 150 > /dev/null 2>&1

    sleep 0.4
    n=0
    while [ $n -lt 64 ]; do
        st=$(devmem $((UL + 0x08)) 32)
        [ $((st & 1)) -eq 0 ] && break
        devmem $UL 32 > /dev/null
        n=$((n + 1))
    done
    if [ $n -gt 0 ]; then echo "GAP=${G}ms  reply  OK"; else echo "GAP=${G}ms  reply  NONE"; fi
done

devmem $GAP_REG 32 60 > /dev/null    # restore the safe default
