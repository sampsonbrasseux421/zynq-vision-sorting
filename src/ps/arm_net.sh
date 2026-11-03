#!/bin/sh
# /usr/bin/arm_net.sh —— inetd 在 5000 端口调用的处理脚本
#
# 协议（一行文本，空格分隔，回车结束）：
#   HOME                 四轴回零
#   <X> <Y> [Z]          机械臂 goto，Z 默认 55
#   CLAW <us>            夹爪/舵机：脉宽 500~2500us，PL 硬件持续输出
#   CLAW OFF             停止脉冲（松开、不再受力）
#   PING                 连通性测试，回 PONG
#
# 例（PC 上）：
#   echo "CLAW 1500" | nc 192.168.1.10 5000
#   echo "140 0 65"  | nc 192.168.1.10 5000
#
# 日志：/tmp/arm_net.log  舵机输出：/tmp/servo_pl.log

read LINE
[ -z "$LINE" ] && exit 1

# 解析：第一个词是命令，其余放在 REST
X=${LINE%% *}
if [ "$X" = "$LINE" ]; then
    REST=""; Y=""; Z=""
else
    REST=${LINE#* }
    Y=${REST%% *}
    if [ "$Y" = "$REST" ]; then Z=""; else Z=${REST#* }; Z=${Z%% *}; fi
fi
[ -z "$X" ] && exit 1

echo "$(date '+%m-%d %H:%M:%S') RX=[$X] [$Y] [$Z]" >> /tmp/arm_net.log

# 使用 PL PWM 控制程序
SERVO=""
for p in /usr/bin/servo_pl; do
    if [ -x "$p" ]; then SERVO="$p"; break; fi
done

case "$X" in
    JOG|jog)
        # JOG ON  : 之后的 goto 跳过使能阶段（0.26s/次），适合手柄连续微调
        # JOG OFF : 恢复完整流程（1.05s/次）
        if [ "$Y" = "ON" ] || [ "$Y" = "on" ] || [ "$Y" = "1" ]; then
            touch /tmp/jog_mode
            echo "OK JOG ON"
        else
            rm -f /tmp/jog_mode
            echo "OK JOG OFF"
        fi
        exit 0
        ;;

    READY|ready)
        # 送完盒后回到传送带前方待命位，避免下一次抓取从盒子位置长途奔袭
        RX=$(grep -E '^READY_X=' /etc/ik.conf 2>/dev/null | cut -d= -f2)
        RY=$(grep -E '^READY_Y=' /etc/ik.conf 2>/dev/null | cut -d= -f2)
        RZ=$(grep -E '^READY_Z=' /etc/ik.conf 2>/dev/null | cut -d= -f2)
        RX=${RX:-250}
        RY=${RY:-0}
        RZ=${RZ:-200}
        /usr/bin/ik_zdt_pl -m -P -r 3000 -A 255 goto "$RX" "$RY" "$RZ"
        echo "OK READY $RX $RY $RZ"
        exit 0
        ;;

    LIFT|lift)
        # 原地垂直升到安全高度，避免从盒子高度斜着走
        if [ -f /tmp/ik_box ]; then
            BX=$(awk '{print $1}' /tmp/ik_box)
            BY=$(awk '{print $2}' /tmp/ik_box)
            /usr/bin/ik_zdt_pl -m -P -r 3000 -A 255 goto "$BX" "$BY" 200
            echo "OK LIFT $BX $BY 200"
        else
            echo "OK LIFT NOPOS"
        fi
        exit 0
        ;;

    OSD|osd)
        /usr/bin/osd_text "$REST"
        exit 0
        ;;

    WAIT|wait)
        # 用 PL 状态寄存器判断：STATUS bit0=busy，bit1=done
        # 等 busy 清零后再留一段稳定时间，不依赖软件串口
        i=0
        while [ $i -lt 120 ]; do
            st=$(devmem 0x40000018 32)
            B=$(( (st >> 0) & 1 ))
            if [ "$B" -eq 0 ]; then break; fi
            sleep 0.1
            i=$((i+1))
        done
        # 帧发完后驱动器仍在走，留稳定时间
        sleep 0.9
        echo "OK ARRIVED"
        exit 0
        ;;

    DET|det)
        /usr/bin/pl_det -s
        exit 0
        ;;

    GAP|gap)
        # 在线调整 PL 帧间隔（毫秒），默认 60；JOG 时可降到 25 提速
        G="${Y:-60}"
        devmem 0x40000020 32 "$G" > /dev/null
        echo "OK GAP $G"
        exit 0
        ;;

    PING|ping)
        echo "PONG"
        exit 0
        ;;

    FOLD|fold)
        # 在"当前盒子角度"原地收臂：只折叠，不转底座，避免伸着手臂大范围甩动
        if [ -f /tmp/ik_boxang ]; then
            A=$(cat /tmp/ik_boxang)
            /usr/bin/ik_zdt_pl -m -P -r 3000 -A 255 angle "$A" 0 0 0
            echo "OK FOLD $A"
        else
            /usr/bin/ik_zdt_pl -m -P -r 3000 -A 255 angle 0 0 0 0
            echo "OK FOLD 0"
        fi
        exit 0
        ;;

    HOME|home)
        killall servo_ps 2>/dev/null
        if ! "$SERVO" off >> /tmp/servo_pl.log 2>&1; then
            echo "ERR PL PWM off failed"
            exit 1
        fi
        /usr/bin/ik_zdt_pl -m -P -r 3000 -A 240 angle 0 0 0 0
        echo "OK HOME"
        exit 0
        ;;

    BOX|box)
        # 恢复备份版行为：只转底座 + 收臂，松爪后物块直接落进盒子
        POS="${Y:-23040}"
        /usr/bin/ik_zdt_pl -m -P -r 3000 -A 255 angle "$POS" 0 0 0 0
        # 盒子较低：收臂转到方位后，再沿该方向伸出并下降到盒子高度
        PR=$(grep -E '^PLACE_R=' /etc/ik.conf 2>/dev/null | cut -d= -f2)
        PZ=$(grep -E '^PLACE_Z=' /etc/ik.conf 2>/dev/null | cut -d= -f2)
        PR=${PR:-200}
        PZ=${PZ:-90}
        XY=$(awk -v p="$POS" -v r="$PR" 'BEGIN{
            t = p / 256.0 * 3.14159265358979 / 180.0;
            printf "%d %d", r*cos(t), r*sin(t);
        }')
        echo "$XY" > /tmp/ik_box
        echo "$POS" > /tmp/ik_boxang
        echo "OK BOX $POS"
        exit 0
        ;;

    CLAW|claw)
        if [ "$Y" = "OFF" ] || [ "$Y" = "off" ] || [ "$Y" = "0" ]; then
            killall servo_ps 2>/dev/null
            if ! "$SERVO" off >> /tmp/servo_pl.log 2>&1; then
                echo "ERR PL PWM off failed"
                exit 1
            fi
            echo "OK CLAW OFF"
            exit 0
        fi
        if [ -z "$SERVO" ]; then
            echo "ERR servo_pl not found"
            exit 1
        fi
        case "$Y" in
            ''|*[!0-9]*) echo "ERR bad pulse '$Y'"; exit 1 ;;
        esac
        [ "$Y" -lt 500 ]  && Y=500
        [ "$Y" -gt 2500 ] && Y=2500
        killall servo_ps 2>/dev/null
        if ! "$SERVO" "$Y" >> /tmp/servo_pl.log 2>&1; then
            echo "ERR PL PWM command failed"
            exit 1
        fi
        echo "OK CLAW $Y"
        exit 0
        ;;
esac

# 其它：goto
echo "[RX] X=$X Y=$Y Z=$Z"
rm -f /tmp/ik_pos
if [ -f /tmp/jog_mode ]; then
    /usr/bin/ik_zdt_pl -m -P -J -r 3000 -A 255 goto "$X" "$Y" "$Z"
else
    /usr/bin/ik_zdt_pl -m -P -r 3000 -A 255 goto "$X" "$Y" "$Z"
fi
echo "OK GOTO $X $Y $Z"
