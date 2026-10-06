# HB-2A 气球板当前代码核查

日期：2026-10-06。对象：`HB2A_Balloon` 当前工作区源码及未提交改动。本次只审查和验证，没有烧录实板，也没有修改固件实现。

## 结论

目标工程使用 Keil ARMCLANG V6.24 完整重编译通过：0 errors / 0 warnings。主机侧 13 组重新编译测试，以及上行端 Python `unittest` 9 项全部通过。测试通过不能证明真实射频、摄像头、GNSS 天线和供电条件下的端到端行为。

当前仍有以下确定问题，建议按 P1、P2 顺序处理。

### P1：发送等待期间上行命令会丢失

位置：`HB2A_Balloon/App/Src/app_main.c:814`、`:858`；`HB2A_Balloon/App/Src/aprs_schedule_service.c:149`；`HB2A_Balloon/App/Src/camera_ssdv_service.c:92`。

APRS 重复包之间、SSDV I/J/K 子包之间和摄像头预热期间虽然会调用 GNSS poll，但不会调用 `SX1278_APRS_RxPoll()`。发送结束重新启动 RX 时，驱动会重新初始化 FIFO/解码器，间隙中收到的完整命令因此没有进入应用队列。针对 FIFO 和 DCLK 两种接收编译配置的复现均得到 `accepted=0`。

建议把上行接收做成独立的短队列，在所有 RF 空闲等待函数中持续取包；发送开始前定义队列满和模式切换的处理规则。

### P1：SI5351 总线故障时输出关闭不可验证

位置：`HB2A_Balloon/App/Src/wspr_radio_service.c:68`、`:89`、`:110`；`HB2A_Balloon/Target/Core/Src/si5351.c:228`。

I2C 写入失败现在会返回错误并让服务退出，这是进步，但错误路径再次调用同一条失效 I2C 总线关闭输出。若 SI5351 保持上一次输出状态，软件没有 GPIO 复位、供电关闭或可验证的硬件兜底。复现中一次帧在 722 ms 后报错，`initialized=0`，但关闭寄存器写入本身也失败。

建议增加 SI5351 硬件复位/电源控制或明确的外部关断路径；整帧设置最大时间；验证关闭失败时不会继续发射。

### P2：UBX 错误长度会让 GNSS 解析器长时间失步

位置：`HB2A_Balloon/Target/Core/Src/ubx.c:165-174`。

解析器读到任意 16 位长度后进入 payload 状态，只把前 `UBX_MAX_PAYLOAD` 字节保存，但仍按 `s_len` 消费字节。一个损坏的 `FF FF` 长度会吞掉后续约 65 KB 串口数据；在 38400 bps 下可造成十几秒级定位中断，并且环形缓冲区期间持续溢出。复现注入错误头后再注入 60 个有效 NAV-PVT，`accepted=0`。

建议在读完长度时立即拒绝超过协议上限的帧，清空当前帧并重新寻找同步；同时为连续校验错误/环形缓冲溢出增加恢复统计和重配置策略。

### P2：GNSS 失锁后仍无限期使用外推 UTC，并切换到默认 APRS 频点

位置：`HB2A_Balloon/App/Src/gnss_service.c:35-44`、`HB2A_Balloon/App/Src/app_main.c:749-818`、`HB2A_Balloon/App/Src/telemetry_service.c:78-103`。

数据超过 3 秒后只清除 `position_valid` 并设置 `utc_holdover`，UTC 仍被本地时钟无限推进；调度逻辑没有检查 `utc_holdover`，因此长期断流仍会发送 APRS/WSPR。位置结构则继续保留历史坐标，但 APRS 区域选择使用无效位置，回退到 144.640 MHz。复现中伦敦定位从 144.800 MHz/IO91 失锁后变成 144.640 MHz，同时仍携带历史坐标；标准 WSPR 无定位回退网格现已明确设为 `RR73`，不再使用测试位置 `OM68`。

建议区分短时 holdover 和过期状态，超过明确时限后停止依赖 GNSS UTC 的时隙调度或发送无位置状态；区域频点应使用最后可信区域并带年龄，或在无可信区域时明确不发普通位置包。

### P2：WSPR 音调步进不符合 12000/8192 Hz

位置：`HB2A_Balloon/App/Src/wspr_radio_service.c:24-31`。

符号偏移先量化成整数 Hz，四个音调请求偏移为 `0,1,3,4` Hz，而标准步进为约 `0,1.465,2.930,4.395` Hz。每个符号还调用 `si5351_SetupCLK0()`，其中重新配置 PLL 并写 PLL reset。复现确认一帧调用 PLL reset 162 次。

建议固定 PLL，使用定点分频参数生成四个音调，只更新 MultiSynth 所需寄存器；用频谱仪或 SDR 验证频偏、切换瞬态和帧起始时序。

## 功能缺口

- 高速链路收到 `CFRM` 后仍发送 `ERR,CFRM,NOTREADY`，没有完成 SSDV 高速发送：`HB2A_Balloon/App/Src/app_main.c:108-120`。
- 没有 Flash 图片目录、完整图片持久化和 `REQF` 文件读取路径；当前图片只在 RAM 中用于当前传输。
- GNSS 5 分钟等待失败只打印日志，未生成需求文档所述的专用 `GNSS Failed!` APRS 消息。
- 摄像头调度使用 256x192，需求文档中的高速 `CAPTR` 尺寸、SSDV 扩展帧和实际端到端发送仍未闭合。
- Error/Fault Handler 和主流程包含永久循环，工程未发现 IWDG/WWDG 配置；无人值守恢复依赖外部硬件或复位策略。

## 验证

- Keil：`HB2A_Balloon/Target/MDK-ARM/hyacinth_balloon2.uvprojx`，ARMCLANG V6.24，0 errors / 0 warnings；Code=109296，RO-data=7548，RW-data=40，ZI-data=86144。
- 主机 C 测试：app flow、telemetry、WSPR、APRS 调度、AX.25、上行协议、高速状态机、转发、自检、GMSK、GNSS、SX1278 FIFO/DCLK，共 13 组通过。
- 上行端：`python -m unittest discover -s HB2A_Uplink/Tests -v`，9 项通过。环境未安装 `pytest`，因此没有运行 pytest 入口。
- ARM 链接器静态调用图报告最大已知栈使用 1320 bytes，另有未知函数指针/循环；启动文件预留 4096 bytes，仍应在实机中做中断嵌套和故障路径栈水位验证。

## 实板验证清单

先修复上面的 P1，再做 GNSS 断流/恢复、连续多个 APRS/WSPR 周期、发送间隙上行、一次性 SPI/I2C 故障、摄像头超时、低压低温和 SX1278 发射频谱测试。需要记录真实的 DIO、FIFO、AFC/FEI、SI5351 输出关断和摄像头 JPEG 完整性。
