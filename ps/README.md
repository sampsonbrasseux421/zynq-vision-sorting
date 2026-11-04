# PS 端脚本与验证工具

## ps/pik.sh —— 坐标 → 机械臂

PL 逆运动学的 PS 端驱动。整个脚本只有几十行，因为**逆解、脉冲换算、组帧都在 PL 里**，
PS 只负责写坐标、触发、读回关节角。

```bash
# 安装
sudo cp ps/pik.sh /usr/bin/pik && sudo chmod +x /usr/bin/pik

# 点到点移动
pik 250 0 150

# 笛卡尔直线插补（3000 ms）
pik -i 300 0 120 3000

# 只求解、打印关节角，不动作
pik -t 250 0 150
```

### 它做了什么

```
写 IK_X / IK_Y / IK_Z   →  0x24 / 0x28 / 0x2C
写 0x30 = 1             →  触发一次逆解
轮询 0x3C bit1          →  等待 done
读 0x34 索引 1~4        →  取回四个关节角
写 CTRL = 0x28          →  uart_sel + IK 自动模式
写 0x30 = 1             →  PL 自动换脉冲 + 组帧 + 发送
```

### 注意：写负数必须转补码

板端 `devmem` 传负数是**静默失败**的（`devmem 0x... 32 -1558` 实际写入 −1，却返回成功）。
脚本里统一走 `wr32()`：先 `v & 0xFFFFFFFF` 再 `printf "0x%08X"`，并在写坐标后**回读核对**。

```sh
wr32() {
    v=$(( $2 ))
    if [ "$v" -lt 0 ]; then v=$(( v & 0xFFFFFFFF )); fi
    devmem $(addr $1) 32 $(printf "0x%08X" $v) >/dev/null
}
```

---

## tools/ —— 上板验证与参数反推

### final_ik_verify.py —— 16 点精度验证

需要能 SSH 到板卡（默认 `petalinux@192.168.1.10`，密码 `1`）。

```bash
python tools/final_ik_verify.py
```

对 16 个工作空间测试点（四象限覆盖）逐一写入坐标、读回四关节、与浮点参考比对。
实测输出：

```
通过 16 / 16，最大误差 11 deg256 = 0.043 度
```

参考模型与板卡上 `ik_zdt_pl` 等价，参数取自 `/etc/ik.conf`。

### extract_ik_params.py —— 从 ARM 二进制提取真实参数

板端 `ik_zdt_pl` 未 strip，符号 `g_link1 / g_link2 / g_base_h / g_tool_x / g_j2z / g_elbow`
都在。脚本解析 ELF32 的 `.symtab` 与 `.data`，直接读出实际的 double / int 值：

```
g_link1  = 195.0   g_link2 = 115.0   g_base_h = 112.0
g_tool_x = 35.0    g_tool_z = 25.0
g_j2z = -23040  g_j3z = 23040  g_j4z = 10240  g_elbow = -1
```

**注意**：这些是二进制里的**编译期默认值**；运行时会被 `/etc/ik.conf` 覆盖
（LINK1=200、LINK2=175、TOOL_Z=35、J4Z=1280）。以 `ik.conf` 为准。

### fit_ik_params.py —— 用样本反推机构参数

采集 `(X, Y, Z) → 关节角` 样本后用最小二乘反推 `TOOL_X / HZ / L1 / L2 / 偏置`。

实测残差 2.7°、拟合出 L1=833 mm / L2=1188 mm（明显不合理）——
这个结果本身就是个有用的结论：**源码里的二连杆模型套不上现场整机**，
必须直接用 `ik.conf` 的标定值，而不是靠拟合。
样本见 `ik_calib_samples.json`。
