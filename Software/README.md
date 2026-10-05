# HB-2A 正式软件

正式程序按功能拆分。`App` 只负责任务流程和服务编排，硬件访问、协议编码和外设驱动分别放在独立模块中。

GNSS 启动流程、遥测组合层、APRS 下行和 WSPR 编码已经在实板验证通过。本里程碑接入 SI5351 服务：检测器件、按 GNSS UTC 触发首个偶数分钟标准 WSPR，后续执行 `:02` 标准帧和 `:04` 编码遥测帧，并按 162 个符号的 1.5 kHz/8192 音调步进实际下发。测试工程 `D:/项目/202602 风信子气球/HyacinthBalloon/code` 保持不变。

计划的模块边界：

- `app_flow`：上电、等待 GNSS、标准模式、试验模式和高速传输状态机。
- `gnss_service`：UBX 接收、解析、定位状态和 UTC 时间。
- `telemetry_service`：传感器采样和遥测文本拼接。
- `sx1278_radio`：SX1278 初始化、收发和参数切换。
- `camera_service`：摄像头选择、拍照和 JPG 文件交付。
- `aprs_service`：APRS 频点选择、AX.25/1200 bps 下行和转发。
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

