#!/bin/sh
# pik -- 坐标 -> 机械臂（逆解、脉冲换算、组帧全部在 PL 内完成）
#
# 用法:
#   pik X Y Z              点到点移动（快速）
#   pik -i X Y Z [ms]      直线插补 + S 曲线（默认 3000ms）
#   pik -t X Y Z           只求解并打印关节角度
#
# PS 只做三件事：写坐标、触发、等完成。
# 逆运动学（CORDIC）、角度->脉冲、四轴组帧都在 PL 里。

A=0x40000000

MODE=0          # 0=点到点 1=插补
DRY=0
if [ "$1" = "-t" ]; then DRY=1; shift
elif [ "$1" = "-i" ]; then MODE=1; shift
fi

if [ $# -lt 3 ]; then echo "usage: pik [-i|-t] X Y Z [ms]"; exit 2; fi

X=$(( $1 * 10 ))
Y=$(( $2 * 10 ))
Z=$(( $3 * 10 ))
DUR=${4:-3000}

addr() { printf "0x%08X" $(( A + $1 )); }

wr32() {
    v=$(( $2 ))
    if [ "$v" -lt 0 ]; then v=$(( v & 0xFFFFFFFF )); fi
    devmem $(addr $1) 32 $(printf "0x%08X" $v) >/dev/null
}

rd32() {
    raw=$(devmem $(addr $1) 32)
    v=$(( raw ))
    if [ "$v" -ge 2147483648 ]; then v=$(( v - 4294967296 )); fi
    echo $v
}

frames() {
    st=$(devmem $(addr 0x18) 32)
    echo $(( st >> 16 ))
}

# ---- 1. 写坐标（16 位补码）----
wr32 0x24 $(( X & 0xFFFF ))
wr32 0x28 $(( Y & 0xFFFF ))
wr32 0x2C $(( Z & 0xFFFF ))

bx=$(( $(devmem $(addr 0x24) 32) & 0xFFFF ))
by=$(( $(devmem $(addr 0x28) 32) & 0xFFFF ))
bz=$(( $(devmem $(addr 0x2C) 32) & 0xFFFF ))
[ $bx -ge 32768 ] && bx=$(( bx - 65536 ))
[ $by -ge 32768 ] && by=$(( by - 65536 ))
[ $bz -ge 32768 ] && bz=$(( bz - 65536 ))
if [ "$bx" != "$X" ] || [ "$by" != "$Y" ] || [ "$bz" != "$Z" ]; then
    echo "ERR 坐标回读不符: 写($X,$Y,$Z) 读($bx,$by,$bz)"
    exit 3
fi

# ---- 2. 插补模式 ----
if [ $MODE -eq 1 ]; then
    [ $DRY -eq 1 ] && { echo "DRY: 插补 ($1,$2,$3) 时长 ${DUR}ms"; exit 0; }

    wr32 0x20 10          # GAP=10ms（引擎要跟得上）
    wr32 0x40 $DUR        # 插补时长
    wr32 0x00 0x48        # uart_sel | 插补模式
    wr32 0x1C 1           # 清 done
    wr32 0x30 1           # 触发

    # 等插补结束：帧数连续 6 次不变即认为完成
    last=0; same=0; i=0
    while [ $i -lt 400 ]; do
        cur=$(frames)
        if [ "$cur" = "$last" ]; then
            same=$(( same + 1 ))
            [ $same -ge 6 ] && break
        else
            same=0
        fi
        last=$cur
        i=$(( i + 1 ))
        usleep 100000
    done

    wr32 0x00 0x08        # 退出插补
    wr32 0x20 60          # 恢复 GAP
    echo "OK PIK-INTERP ($1,$2,$3) ${DUR}ms frames=$last"
    exit 0
fi

# ---- 3. 点到点：先求解 ----
wr32 0x30 1
i=0
st=0
while [ $i -lt 200 ]; do
    st=$(devmem $(addr 0x3C) 32)
    if [ $(( st & 2 )) -ne 0 ]; then break; fi
    i=$(( i + 1 ))
done
if [ $(( st & 2 )) -eq 0 ]; then echo "ERR 求解超时 st=$st"; exit 4; fi

wr32 0x34 1; J1=$(rd32 0x34)
wr32 0x34 2; J2=$(rd32 0x34)
wr32 0x34 3; J3=$(rd32 0x34)
wr32 0x34 4; J4=$(rd32 0x34)
echo "PL关节 deg256: $J1 $J2 $J3 $J4"

[ $DRY -eq 1 ] && exit 0

wr32 0x00 0x28
wr32 0x1C 1
wr32 0x30 1

i=0
st=0
while [ $i -lt 3000 ]; do
    st=$(devmem $(addr 0x18) 32)
    if [ $(( st & 2 )) -ne 0 ]; then break; fi
    i=$(( i + 1 ))
done
wr32 0x00 0x8
echo "OK PIK frames=$(( st >> 16 ))"
