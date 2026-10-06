# HB-2A 正式软件

正式程序按功能拆分。`App` 只负责任务流程和服务编排，硬件访问、协议编码和外设驱动分别放在独立模块中。

GNSS 启动流程、遥测组合层、APRS 下行和 WSPR 编码已经在实板验证通过。SI5351 服务已经接入：检测器件，按固定时隙执行每十分钟 `:02` 标准 WSPR 和 `:04` 编码遥测 WSPR，并按 162 个符号的 1.5 kHz/8192 音调步进实际下发。上电后不额外插入首次 WSPR。当前里程碑加入 APRS UTC 调度，在 `:01` 和 `:06` 各发送两次遥测，间隔 3 秒，并在发送期间继续轮询 GNSS。测试工程 `D:/项目/202602 风信子气球/HyacinthBalloon/code` 保持不变。

启动流程现在先初始化通信总线，按 01～08 设备顺序执行初始化和自检，失败设备最多重试 3 次，设备之间间隔 0.5 秒；8 个设备全部完成后，再统一发送一份包含全部设备状态的 APRS 自检报告。自检完成后才等待 GNSS 的 3D 定位和完整 UTC。

计划的模块边界：

- `app_flow`：上电、等待 GNSS、标准模式、试验模式和高速传输状态机。
- `gnss_service`：UBX 接收、解析、定位状态和 UTC 时间。
- `telemetry_service`：传感器采样和遥测文本拼接。
- `sx1278_radio`：SX1278 初始化、收发和参数切换。
- `camera_service`：摄像头选择、拍照和 JPG 文件交付。
- `aprs_service`：APRS 频点选择、AX.25/1200 bps 下行和转发。
- `uplink_protocol`：16 字节 0x55 前导、CCSDS TC BCH(63,56)、NRZI 和 CRC-16 上行帧解码。
- `transponder_service`：TRANPD 校验、排队和固定 144.925 MHz AX.25 转发。
- `high_speed_service`：CAPTR/REQF 后的高速链路参数协商、重试、确认和取消状态机。
- `gmsk_downlink`：常规 4800 bit/s GMSK 文本下行的传输帧主头、RS(255,223)、PN17 和 CCSDS 1/2 卷积编码。
- `aprs_schedule_service`：APRS UTC 时隙去重、两次重复发送和间隔计时。
- `wspr_service`：WSPR 编码和标准/遥测帧构造。
- `wspr_radio_service`：SI5351 检测、WSPR 162 符号定时和射频输出。
- `experiment_service`：GMSK 常规下行和动态高速 SSDV 传输。
- `storage_service`：Flash 图片目录、编号和最旧图片覆盖。

## 上行接收排查

当前默认用 SX1278 数据包模式接收上行。芯片匹配 `1A CF FC 1D` 后，通过 SPI 从 FIFO 读取原始负载，再交给原有的 NRZI、BCH 和 CRC 解码器。空中格式没有新增长度字节，也没有启用芯片的 CRC、白化或地址过滤；负载长度由命令首个 BCH 块决定。收到完整命令、坏包、溢出或超时后重新等待同步码。

日志中的 `path=FIFO` 和 `pkt=40` 表示启用此路径。`RX FIFO bytes` 是实际读取的字节数，`asm` 是硬件同步匹配次数，`ok` 是校验通过的命令数。DIO1 现在用于 FIFO 水位通知，`irq` 记录通知次数；即使没有通知，主循环也会轮询 FIFO。此时 `clocks=0` 正常，不再表示接收故障。

`APP_UPLINK_RX_FIFO=0` 可以编译原来的 DCLK 接收路径用于对比。APRS 仍用连续模式发射；常规和高速 GMSK 改用 SPI FIFO，保留芯片内置 BT=0.5 高斯整形，不依赖 DIO1/DCLK。两种发射结束后都恢复上行监听。

GMSK 的前导、同步、RS、加扰和卷积编码仍由软件完成。芯片不添加前导、同步、长度字节、CRC 或白化；最后一个字节不足 8 位时，在卷积尾比特之后补 0，最多 7 位。总长度不超过 2047 字节时等待 `PacketSent`，更长的高速分组用无限长度模式，FIFO 空后再等待最后一个字节及高斯滤波器排空。`GMSK FIFO queued=536/536 pad_bits=4 packet_sent=1 drained=0` 表示常规帧已全部写入且芯片报告发完；这仍需地面实际解码验收。`rc=-11` 表示发送中 FIFO 断粮，驱动会停止发射并恢复接收，不把残缺分组当作成功。

转发队列在图传期间也会等待所有可用的发射空档；图传分组结束后，队列中的 `TRANPD` 会优先通过 144.925 MHz 发射，并打印 `TRANSPONDER APRS TX complete freq=144925000`。上行字段和下行 AX.25 路径统一使用 `APZDQ7`。源呼号和目的呼号的基础部分最多 6 个字符，非法呼号或 SSID 会被拒绝。队列满时仍丢弃最新包。

接收启动时按 SX1278 手册的前导码方案启用 AGC/AFC：`RegRxConfig=0x1E`，AFC 捕获带宽使用 `0x0B`，收到前导后记录 AFC 和 FEI。若出现 `pre=1` 但 `asm=0`，等待 250 ms 后会丢弃本次捕获并重新启动接收。日志中的 `afc_hz` 是芯片实际修正量，`fei_est_hz` 是频差估计，均用于判断地面站和气球的频率误差。

## 拍照尺寸

CAPTR 当前仅允许已有且满足 SSDV 对齐要求的预设：128×96、256×192、320×240、480×272、640×480、800×480。地面站在发送前校验，气球相机入口也会拒绝其他尺寸。

正式拍照直接选择原有 OV5640 分辨率预设，640×480 与相机自检使用同一传感器初始化路径，不再调用新增的 SetOutputSize 重写时序。实验性任意尺寸函数保留在驱动中，但业务不调用。日志细分 DCMI 初始化、传感器预设、PLL 和 ISP 阶段；这项修正仍需重新编译烧录并实板验证，不能据主机寄存器测试认定复位已解决。

单张JPEG仍受65532字节接收缓冲区限制；超限会报拍照失败，不自动缩图或降低质量。SSDV传输质量由CAPTR的Quality参数控制，与摄像头JPEG压缩设置是两回事。

## 拍照期间偶发复位排查

CAPTR 只启动协商，收到 CFRM 后才开始摄像头上电和采集。当前代码中主动 MCU 复位只有 GNSS 启动失败及试验模式五分钟超时；摄像头失败路径返回错误，HardFault 处理仍停在循环，未增加自动复位。工程未软件启用 IWDG/WWDG，硬件选项字节状态仍须实板确认。当前链接栈容量是 4096 字节，本次构建的静态最大调用链为 1680 字节，仍不含链接器无法追踪的函数指针和中断嵌套，不能据此保证实机永不溢栈。

采集入口现在先确认旧 DMA 停止再清空缓冲；HAL 的 DCMI_Stop 会忽略内部 DMA_Abort 结果，因此额外核对 DMA 状态。停止失败时不重用缓冲、不返回 JPEG。DMA 剩余字节数先做范围校验再相减，防止异常计数导致无符号下溢、JPEG 扫描越界。新增错误码：`-18` 剩余计数非法、`-19` 停止失败、`-21` DCMI/DMA 未初始化或未关联、`-22` 输出参数为空；原有 `-17` 表示 JPEG 缓冲已满。这些是防护修复，尚未证明是现场偶发复位的根因。

启动串口保留 `HB-2A reset CSR=...`，新增 `RESET BOR=... PIN=... SW=... IWDG=... WWDG=... LPWR=... OBL=... previous=...` 和编译时间。`BOR` 包括正常上电/电源复位，不能仅凭一次开机的 BOR 标志就认定供电不足；故障时应结合复位前日志判断。`previous` 来自 TAMP BKP1，记录 `camera-clock`、`camera-rails`、`camera-config`、`dcmi-init`、`capture`，以及高速拍照后的 `image-store` / `ssdv` 阶段；正常结束回到 `idle`。完整掉电或备份域被清除时记录可能丢失，显示 `unknown`。BKP0 继续用于原有启动计数。预热逐帧打印 `CAM warmup n/20`，最终采集打印 `CAM final capture`。

`Tests/test_camera_capture.c` 用实际驱动配合模拟 HAL 验证空指针、未初始化句柄、计数异常、DMA 停止失败、停止前缓冲不被清空、超时、缺失 JPEG 结束标记、满缓冲及连续 100 次采集。主机测试和 Keil 构建通过不能替代相机上电瞬间供电波形与连续实板拍照测试。本次仅生成固件，没有烧录。

```powershell
gcc -std=c11 -Wall -Wextra -Werror -Wno-pointer-to-int-cast -I HB2A_Balloon/Tests/mocks/camera -I HB2A_Balloon/Target/Core/Inc -I HB2A_Balloon/Target/Drivers/BSP/Components/ov5640 HB2A_Balloon/Tests/test_camera_capture.c HB2A_Balloon/Target/Drivers/BSP/Components/ov5640/ov5640.c HB2A_Balloon/Target/Drivers/BSP/Components/ov5640/ov5640_reg.c -o HB2A_Balloon/Tests/build/test_camera_capture.exe
HB2A_Balloon/Tests/build/test_camera_capture.exe
```

## 主机测试命令

在仓库根目录执行：

```powershell
gcc -std=c11 -Wall -Wextra -Werror -ISoftware/App/Inc -ISoftware/Target/Core/Inc `
  Software/Tests/test_telemetry_service.c Software/App/Src/telemetry_service.c `
  Software/Target/Core/Src/aprs.c Software/Target/Core/Src/sensor_format.c `
  Software/Target/Core/Src/u4b_telemetry.c -o Software/Tests/bin/test_telemetry_service.exe
Software/Tests/bin/test_telemetry_service.exe
```
