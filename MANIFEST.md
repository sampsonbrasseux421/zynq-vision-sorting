# File manifest

原始工程文件到本仓库的映射。`Source` 为开发期间所在位置，仅供追溯，仓库内不需要该路径。

| Repo path | Source | Description |
|---|---|---|
| `src/rtl/zdt_frame_tx.v` | `zdt_frame_tx.v` | 单帧组帧 + 8N1 串行化 |
| `src/rtl/zdt_tx_engine.v` | `zdt_tx_engine.v` | 四轴调度 + 帧间定时 + 同步触发（含 skip_en） |
| `src/rtl/zdt_axil.v` | `zdt_axil.v` | AXI4-Lite 从机寄存器文件 |
| `src/rtl/zdt_top8.v` | `zdt_top8.v` | 指令引擎顶层（当前版本） |
| `src/rtl/video_osd_text21.v` | `video_osd_text21.v` | 硬件字符 OSD（16x16 字库，当前版本） |
| `src/rtl/font_init.vh` | `font_init.vh` | 字模初始化 |
| `src/rtl/font8x8.vh` | `font8x8.vh` | 8x8 字模 |
| `src/rtl/font8x8.mem` | `font8x8.mem` | 字模数据 |
| `src/rtl/color_blob_detector.v` | `color_blob_detector.v` | PL 色块检测（备选方案，未用于最终版本） |
| `src/rtl/color_blob_detector_roi.v` | `color_blob_detector_roi.v` | PL 色块检测（ROI 版） |
| `src/ps/ik_zdt.c` | `ik_zdt.c` | 逆运动学 + 脉冲换算 + 寄存器写入（含 -P/-J 开关） |
| `src/ps/arm_net.sh` | `arm_net_now.sh` | 板端 TCP 命令服务（inetd 调用） |
| `src/ps/osd_text.c` | `osd_text.c` | OSD 文字推送 |
| `src/ps/legacy/zdt_wait.c` | `zdt_wait.c` | 软件串口到位检测（旧实现，已被 PL 状态位替代） |
| `sim/tb_zdt_frame_tx.v` | `tb_zdt_frame_tx.v` | 单帧格式位级验证 |
| `sim/tb_zdt_tx_engine.v` | `tb_zdt_tx_engine.v` | 四轴调度时序验证 |
| `sim/tb_zdt_engine3.v` | `tb_zdt_engine3.v` | 调度器边界情况 |
| `sim/tb_zdt_axil.v` | `tb_zdt_axil.v` | AXI4-Lite 读写验证 |
| `sim/tb_zdt_sync.v` | `tb_zdt_sync.v` | 同步触发帧验证 |
| `sim/tb_color_blob_detector.v` | `tb_color_blob_detector.v` | 色块检测验证 |
| `sim/tb_roi_check.v` | `tb_roi_check.v` | ROI 边界验证 |
| `build/build_bitstream.tcl` | `build_skipen.tcl` | 最终比特流构建脚本（Vivado TCL） |
| `build/boot.bif` | `boot.bif` | BOOT.BIN 生成配置 |
| `build/bit_only.bif` | `bit_only.bif` | 纯比特流 bin 转换配置 |
| `src/host/app/pc_yolo_send.py` | `pc_yolo_send.py` | 视觉检测 / 状态机 / OSD 推送 |
| `src/host/app/pad_arm.py` | `pad_arm3.py` | Xbox 手柄步进控制 |
| `src/host/app/vlm_advisor.py` | `vlm_advisor.py` | 本地 VLM 任务级决策（异步） |
| `data/grip_close.txt` | `grip_close.txt` | 夹爪闭合脉宽 |
| `data/grip_open.txt` | `grip_open.txt` | 夹爪张开脉宽 |
| `data/grip_backoff.txt` | `grip_backoff.txt` | 夹持回退量 |
