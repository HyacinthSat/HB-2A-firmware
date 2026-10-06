# HB-2A 上行控制台

这套工程把 `USTC_Balloon_Firmware/rx` 的 STM32F103C8T6 + RA-02 板改成地面上行发射器，并配套一个 Windows 图形界面。它使用 USB CDC 收取文本命令，板卡编码 CCSDS BCH(63,56)、NRZI 和 X.25 CRC，再由 SX1278 以 **435.400 MHz、4799.76 bit/s、GMSK BT=0.5、约 1.22 kHz 频偏**发射。硬件前导为 32 字节 0x55，同步字为 `1A CF FC 1D`。

## 烧录

已经生成：

- `Firmware/build/HB2A_Uplink.hex`
- `Firmware/build/HB2A_Uplink.bin`

使用 ST-Link 将文件写入 `0x08000000`。源码也保留了 Keil 工程 `Firmware/MDK-ARM/HB2A_Uplink.uvprojx`；无 Keil 时可在 `Firmware` 目录运行 `python build.py` 重新生成文件。

使用 Keil MDK 时，打开 `Firmware/MDK-ARM/HB2A_Uplink.uvprojx`，确认目标为 `HB2A_Uplink`，点击 **Rebuild**。工程已配置 STM32F103C8、ARM Compiler 6、HEX 输出和 ST-Link 下载器；生成文件在 `Firmware/MDK-ARM/Objects/HB2A_Uplink.hex`。连接 ST-Link 后打开 **Options for Target → Debug** 选择 `ST-Link Debugger`，在 **Utilities** 选择同一个 ST-Link，勾选 `Use Target Driver for Flash Programming`，然后点击 **Download**。第一次下载若询问复位方式，选择 `Connect under Reset`。

板卡时钟为 24 MHz 外部时钟，SPI1 使用 PA5/PA6/PA7，RA-02 的 NSS 为 PA4、RST 为 PB0。网表中的 DIO 为 PA1、PA2、PA3、PB14、PB5、PB6；PB8/PB9 是 RX/TX 指示灯。固件已按网表使用 DIO3=PB14。

## 启动界面

Windows 上运行 `Host/start.cmd`，首次运行会创建 `Host/.venv` 并安装 `pyserial`。连接板卡后选择 USB 串口，点击“连接”，在界面中选择命令、填写字段和功率（2–17 dBm），最后点击“发送一次”。程序不会自动重发；串口在发送期间断开时，界面会报告发送结果未知。

板卡命令行协议如下，每行以换行结束；应用层只负责字段校验和转成十六进制，射频编码在板卡上完成：

```text
STATUS
SEND <2..17> <7 字节字段拼接后的十六进制>
```

支持 `TRANPD`、`EXPR`、`CAPTR`、`REQF`、`HSPAR`、`CHPRAM`、`CFRM` 和 `CANCEL`。每个字段固定 7 个 ASCII 字节，短字段以空格补齐。`TRANPD` 的五个字段是命令、源呼号、目的呼号、`HB-2A`、消息；图形界面会校验呼号、路径和字段范围。

## 气球端行为

当前同步的气球代码中，`EXPR` 在正常模式进入试验模式；`CAPTR`、`REQF` 开始高速图传协商，随后使用 `HSPAR`/`CHPRAM`/`CFRM`/`CANCEL` 管理协商。`TRANPD` 仍按 `HB-2A` 路径排队并通过 144.925 MHz APRS 转发。上行板只会报告“本板已发射”，不会伪造气球执行确认。

## 验证

`python -m unittest discover -s Tests -v` 覆盖协议兼容性、8 类命令、填充 CRC、损坏帧、射频寄存器/FIFO/超时复位、USB 分包/溢出/重连/背压和图形界面连接线程。测试全部通过。`python Tests/qa_gui.py` 已在 900×710 和 740×660 两种尺寸检查控件边界；当前无可抓取桌面，因此未生成截图。

首次实机测试建议先用低功率，将气球置于接收范围内，发送 `EXPR`，观察气球调试日志是否出现 `UPLINK EXPR accepted`，再测试 `TRANPD`。板卡上报 `OK SENT` 只代表 SX1278 报告 PacketSent。

## 收不到上行时怎么看日志

发信机每次发送后会在 `OK SENT` 或 `ERR RADIO` 后附上发射耗时和寄存器读回值。`SNAP=1` 表示读回成功；435.400 MHz、4800 bit/s 对应 `FRF=6CD99A RATE=1A0B DEV=0014 SYNC=33 PKT=40 RAMP=49`。五字段 `TRANPD` 的空中长度为 78 字节，实际发射约 130 ms，`EXPECT=131` 是向上取整后的毫秒数。明显早于预期结束时报 `ERR RADIO -7`。

气球开始监听时打印 `irqtest=PASS`，表示软件触发的 EXTI 能经过中断向量、HAL 和回调。它不能证明 DIO1 连线有时钟，也不会计入 `clocks`。`RX RSSI peak` 是监听期间每 20 ms 采样得到的最强信号，`samples` 是本次统计窗口的采样次数；`samples=0` 时不要使用这个峰值判断信号。

测试时保留同一时段两端的日志，重点对照发送时的 RSSI 峰值、`clocks`、`pre`、`asm` 和 `bch_crc_bad`。RSSI 上升但 `clocks` 始终为零时，继续查位同步和 DIO 时钟；有时钟后，再根据前导、同步和校验计数查数据格式。
