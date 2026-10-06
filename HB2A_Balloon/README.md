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

## 主机测试

在仓库根目录执行：

```powershell
gcc -std=c11 -Wall -Wextra -Werror -ISoftware/App/Inc -ISoftware/Target/Core/Inc `
  Software/Tests/test_telemetry_service.c Software/App/Src/telemetry_service.c `
  Software/Target/Core/Src/aprs.c Software/Target/Core/Src/sensor_format.c `
  Software/Target/Core/Src/u4b_telemetry.c -o Software/Tests/bin/test_telemetry_service.exe
Software/Tests/bin/test_telemetry_service.exe
```

