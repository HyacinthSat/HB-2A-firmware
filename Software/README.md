# HB-2A 正式软件

正式程序按功能拆分。`App` 只负责任务流程和服务编排，硬件访问、协议编码和外设驱动分别放在独立模块中。

当前第一步只实现与硬件无关的主流程状态机，并用主机测试验证状态跳转。测试工程 `D:/项目/202602 风信子气球/HyacinthBalloon/code` 保持不变，后续确认过的驱动再逐个移入正式工程。

计划的模块边界：

- `app_flow`：上电、等待 GNSS、标准模式、试验模式和高速传输状态机。
- `gnss_service`：UBX 接收、解析、定位状态和 UTC 时间。
- `telemetry_service`：传感器采样和遥测文本拼接。
- `sx1278_radio`：SX1278 初始化、收发和参数切换。
- `camera_service`：摄像头选择、拍照和 JPG 文件交付。
- `aprs_service`：APRS 频点选择、AX.25/1200 bps 下行和转发。
- `wspr_service`：WSPR 编码、时隙和 SI5351 下行。
- `experiment_service`：GMSK 常规下行和动态高速 SSDV 传输。
- `storage_service`：Flash 图片目录、编号和最旧图片覆盖。

