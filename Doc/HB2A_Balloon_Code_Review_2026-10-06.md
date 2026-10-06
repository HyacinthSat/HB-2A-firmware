# 气球板代码核查报告

日期：2026-10-06。对象：`HB2A_Balloon` 当前工作区，HEAD `a466b21`，包含未提交修改。

本次仅做审查、构建和复现，没有修改固件源代码，也没有烧录设备。仓库中的需求文档作为业务背景，不作为本次实施新功能的指令。审查期间检测到其他来源对 `app_main.c` 和射频测试等文件的更新，已保留并复核相关路径。

## 结论

当前固件可以完整构建，已有单元测试通过，但异常恢复、GNSS 时间保持和射频错误处理存在确定缺陷。建议先处理下面四项 P1，再做长时间无人值守运行验证。构建成功和模拟测试通过不能代替实板射频、供电及环境验证。

P1 表示可能导致核心任务持续失效，P2 表示具体场景下的功能错误或可靠性缺陷。功能缺口、历史需求差异和待实测风险另列。

## 确定缺陷

### R1 [P1] GNSS 重试没有终止原来的 UART 中断接收

位置：`HB2A_Balloon/Target/Core/Src/ubx.c:70`，`HB2A_Balloon/App/Src/app_main.c:142`。

`Ubx_Init()` 每次都直接调用 `HAL_UART_Receive_IT()`。第一次初始化后，单字节接收回调持续重新挂接接收，UART 通常处于 `HAL_UART_STATE_BUSY_RX`。复位外部 GNSS 不会终止 MCU 的接收状态，因此定位超时后的第二、第三次初始化会得到 `HAL_BUSY`，主循环直接跳过配置和等待；启动配置失败后的自检重试也受影响。

使用遵循 STM32 HAL 状态规则的桩复现：第一次初始化返回 0，第二次返回 -1，RxState 仍为 BUSY_RX。真实 HAL 在 `HAL_UART_Receive_IT()` 中明确拒绝忙状态。

建议：在受控临界区停止/中止旧接收，再清理解析状态和队列，重新挂接；也可以保持唯一接收实例，仅重置解析器和重新配置 GNSS。补测连续超时、配置失败、UART 错误之后的恢复。

### R2 [P1] GNSS 断流后时间冻结，定时遥测可能永久停止

位置：`HB2A_Balloon/App/Src/gnss_service.c:50`、`:94`、`:124`；`HB2A_Balloon/App/Src/app_main.c:399`。

NAV-PVT 更新了 `update_tick`，但没有任何地方使用它检查数据年龄或推进本地 UTC。断流后 `position_valid`、`utc_valid` 和 `fully_resolved` 都保持原值。调度直接使用冻结的年月日时分秒；APRS/WSPR 对同一时隙去重后，无法再进入后续时隙。断流若发生在保护窗口，也可能持续抑制其他发送。

复现：注入一份有效 NAV-PVT，模拟 600000 ms 无数据并继续 poll，位置和 UTC 仍标为有效，年龄为 600000 ms。源码确认没有 GNSS 数据年龄处理路径。

建议：保存最后有效位置与 UTC 锚点，用单调计时推进短期保持时间，单独标记当前定位质量、数据年龄和保持状态；超过策略允许时间后重新配置/复位 GNSS。不要仅清掉 UTC 标志而让所有定时业务无限期停发。

### R3 [P1] SX1278 一次停发 SPI 故障会导致永久失去 APRS 和上行

位置：`HB2A_Balloon/Target/Core/Src/sx1278_aprs.c:261`、`:278`、`:519`。

`stop_radio()` 在待机写入失败时硬件复位芯片并将 `ready=0`，这是必要的停发保护；但 `RxPoll()` 的自动恢复要求 `ready!=0`。应用层又只在启动时初始化射频，`AprsService.initialized` 仍可能保持 1。总线恢复后，接收重试和后续发送都无法自行恢复。

沿用现有 SX1278 测试桩，在 GMSK 结束时仅注入一次 RegOpMode 写失败：发送返回 -10，ready=0；随后恢复 SPI 并轮询 60 秒，RX 仍为 0。

建议：增加有退避的完整射频重新初始化路径，恢复发射参数和上行监听，并同步驱动与服务层状态；保持硬件复位停发保护。

### R4 [P1] SI5351 错误被吞掉，WSPR 可阻塞数分钟且仍报成功

位置：`HB2A_Balloon/Target/Core/Src/si5351.c:240`，`HB2A_Balloon/App/Src/wspr_radio_service.c:68`、`:77`。

`si5351_write()` 超时直接返回 void，并忽略 `HAL_I2C_Mem_Write()` 的结果。每个符号重新写多个寄存器；器件离线时，每次写都重新等待。服务层仍完成 162 次循环、增加 tx_count 并返回成功。符号写入慢于目标时刻时，等待循环里的 GNSS poll 也不会执行。最后关闭输出同样没有可检查的错误结果；若芯片仍供电但总线故障，不能确认载波已关闭。

模拟初始化成功后 I2C 探测持续失败，每次失败增加 50 ms：发送返回 0、tx_count=1，但耗时 616000 ms，而正常帧应为 110592 ms。这是模拟故障时序，并非实板测得的固定耗时。

建议：逐层返回 I2C 错误，第一次失败即终止帧，对整帧设置总时限，执行可验证的关输出/复位策略；自检不能仅依赖器件 ACK。

### R5 [P2] 摄像头预热及采集长时间阻塞，GNSS 队列会溢出

位置：`HB2A_Balloon/App/Src/camera_ssdv_service.c:228`，`HB2A_Balloon/Target/Core/Src/camera.c:455`。

每次拍摄执行 20 次预热采集，每次再延时 300 ms，所有预热错误被忽略；随后再采集一次。这里没有调用 GNSS 或上行服务。即使采集立即完成，仅预热延时就有 6 秒，加上初始化中的 1.5 秒等待；若 DCMI 一直超时，仅 21 次采集就约 105 秒。

UBX 环形队列仅 2048 字节，且配置为每秒同时输出 NAV-PVT 和 NAV-SAT。以 16 颗卫星为例，两种报文约 308 字节/秒，7.5 秒已超过容量。丢字节后又可能破坏 UBX 长度字段的解析边界，延长恢复。上行服务和其他调度也在此期间停顿。

建议：将预热和采集改为可推进的状态机，或在等待中服务接收；限制失败重试总时长，并处理预热采集错误。

### R6 [P2] APRS/SSDV 发射间隔虽恢复 RX，却没有处理上行

位置：`HB2A_Balloon/App/Src/aprs_schedule_service.c:149`，`HB2A_Balloon/App/Src/camera_ssdv_service.c:103`、`:158`。

APRS 两次重复之间、SSDV I/J/K 各段之间均等待 3 秒。此时射频驱动已恢复 RX，但传入的 poll 只处理 GNSS，没有调用 `SX1278_APRS_RxPoll()` 或保存解码命令。下一段发射结束恢复 RX 时会清空 FIFO 和解码器，间隔中收到的命令因而丢失；无限长度 FIFO 接收还可能先溢出。DCLK 路径即使已经收齐，也没有机会取走命令。

建议：在射频空闲间隔持续取出完整上行并进入命令队列；对 EXPR 等模式切换指令定义当前 burst 的安全退出点。不能只以 `RxActive=1` 作为上行业务可用的依据。

### R7 [P2] 一条非法呼号可永久堵塞转发队列

位置：`HB2A_Balloon/App/Src/transponder_service.c:45`、`:68`。

入队只检查命令、块数和 path，没有验证 source/destination 是否能被 AX.25 地址编码器接受。出队时，所有错误均直接返回，既不丢弃永久无效报文，也无重试上限。因此小写呼号、非法 SSID 等 CRC 正确的命令可以长期占住队头，后续合法报文全部不能转发。

复现：先入队 source=`badcall`，再入队合法呼号；第一条被接受，轮询 10 次后队列仍为 2，forwarded=0。

建议：入队前按实际 AX.25 规则验证；区分永久参数错误和暂时硬件错误，对后者设置有界重试。

### R8 [P2] BCH 纠错漏掉最后一个有效校验位

位置：`HB2A_Balloon/Target/Core/Src/uplink_protocol.c:55`。

最后一字节有效位为 bit6..bit0，但纠错循环使用 `1 << (7-bit)` 并只迭代 7 次，实际尝试 bit7..bit1，漏掉 bit0。bit7 是填充位，不属于 63 位码字。

穷举翻转全部 63 个有效码位：只有 byte7/bit0 的单比特错误返回 -2。现有测试仅覆盖部分位，未发现该问题。

建议：校验字节从 bit6 开始尝试，添加 63 位单错穷举，并覆盖 NRZI 后实际空中单比特扰动。

### R9 [P2] 上行编码补齐数据时，CRC 读取了原缓冲区之外的内容

位置：`HB2A_Balloon/Target/Core/Src/uplink_protocol.c:139`。

编码器允许非 7 字节倍数输入，并在局部 `plain` 中填充空格；CRC 却直接对原始 data 读取 `blocks*7` 字节。实际分配仅有 data_length 字节时构成越界读；尾部恰好可读时，也可能生成与已编码空格不一致的 CRC。

复现使用实际分配 7 字节的 `EXPRXYZ`，但传入长度 4，避免测试自身非法访存：返回 30 字节帧，解码拒绝，accepted=0。当前固件接收链不调用此编码接口，影响主要在测试、复用或后续调用，未按飞行主路径故障升级。

建议：CRC 与 BCH 共用同一份补齐数据，或明确拒绝非整块输入。

### R10 [P2] 收到失锁 NAV-PVT 后，APRS 会广播 0,0 位置

位置：`HB2A_Balloon/App/Src/telemetry_service.c:35`、`:74`，`HB2A_Balloon/Target/Core/Src/aprs.c:113`。

GNSS 驱动保留上一次坐标，但把 `position_valid` 清零。遥测层先清零位置结构，随后因无有效定位直接返回，最后 APRS 格式化成正常的 `0000.00N/00000.00E` 位置包，区域频率也回退到默认值；WSPR 标准帧则使用固定默认网格。健康位不能阻止普通 APRS 接收软件显示错误位置。

建议：将“当前 fix 有效”和“持有历史有效位置”分开，按明确策略发送最后位置及年龄，或发送不含位置的状态报文。不要将无定位映射为真实的 0,0 坐标。

### R11 [P2] SX1278/SI5351 的启动三次重试没有重新初始化

位置：`HB2A_Balloon/App/Src/app_main.c:183`、`:216`、`:616`。

两种射频器件在统一自检前各初始化一次。如果这次失败，自检回调仅检查 `initialized==0` 并返回错误，三次调用都不会再次访问初始化流程。一次上电时序或瞬时总线故障会让该射频服务整个启动周期不可用。

建议：把器件初始化放到各自自检 attempt 内，失败时执行必要复位和退避。现有自检单元测试只验证通用回调次数，不验证真实硬件回调是否真的重试。

### R12 [P2] WSPR 音调被量化成整数 Hz，并逐符号复位 PLL

位置：`HB2A_Balloon/App/Src/wspr_radio_service.c:24`，`HB2A_Balloon/Target/Core/Src/si5351.c:72`、`:218`。

标准音调步进应为 12000/8192 Hz。当前 `symbol_frequency()` 四个符号的请求偏移为 0、1、3、4 Hz，实际请求间隔是 1、2、1 Hz，并不是均匀的 1.46484375 Hz。每个符号还经过 `SetupCLK0 -> SetupPLL` 写入相同 PLL 配置并执行 PLL_RESET，不能保证符号切换时相位连续。

整数化和逐符号复位均由源码确定；对当前接收机成功率的影响程度需要 SDR/频谱实测，不能据此断言全部 WSPR 无法解码。

建议：固定 PLL，使用定点/有理数频率计算预生成四组分频参数，符号切换仅更新必要的 MultiSynth 参数；实测音调间距、切换瞬态、帧起始时间和温漂。

## 功能缺口与需求差异

以下记录当前实现边界，不将其等同于新出现的缺陷，也不据此擅自实施历史文档中的功能：

- 高速模式仅有协商状态机。`app_main.c:113` 收到 START_TRANSFER 后发送 `ERR,CFRM,NOTREADY` 并取消。CAPTR/REQF 不能完成高速图片传输；当前 HSSUP 掩码声明的参数集合不代表端到端能力已实现。
- 没有 Flash 图片存储、目录、序号恢复或旧图片覆盖服务。当前 JPEG 保存在 RAM，Flash 只用于启动自检。REQF 没有真实文件查询路径。
- GNSS 五分钟超时只写调试日志，没有发送文档描述的 `GNSS Failed!` APRS 包。
- 当前成像为 256x192，在 :06 APRS 后开始，图像组在 :06/:07/:08/:09/:00 发送；历史整理文档写的是 128x96 和拍摄后每两分钟一组。README 已描述较新的调度，应先确认最终验收基线。
- APRS 区域选择直接使用当前坐标和矩形区域，没有文档所写的“连续三次有效定位后切换”滞回。
- `rst_times` 使用 TAMP 备份寄存器，不能仅靠源码保证完全掉电保存；需要确认 VBAT 供电。它不是 Flash 持久化计数。
- README 主机测试示例仍使用已经不存在的 `Software/...` 路径，照抄不能运行。

## 验证结果

### 完整目标构建

使用本机 Keil uVision/ARMCLANG V6.24 对 `Target/MDK-ARM/hyacinth_balloon2.uvprojx` 执行完整重编译，通过，0 errors / 0 warnings。

链接结果：Code=107900，RO-data=7456，RW-data=40，ZI-data=86144 字节。MDK 启动文件实际预留 4096 字节栈；链接器报告最大已知调用链 1312 字节，但明确包含未知函数指针/循环，不应当作完整中断嵌套栈证明。旁边 GCC 链接脚本仍预留 1024 字节，若迁移工具链必须重新预算；本次没有构建 GCC 目标固件。

### 主机测试

GCC C11，`-Wall -Wextra -Werror`：

| 测试 | 结果 |
| --- | --- |
| app_flow | 通过 |
| telemetry_service | 通过 |
| wspr_service | 通过 |
| aprs_service，包含调度 | 通过 |
| ax25_path | 通过 |
| uplink_protocol | 通过 |
| high_speed_service | 通过 |
| transponder_service | 通过 |
| startup_selftest_service | 通过 |
| gmsk_downlink | 通过 |
| sx1278_gmsk，FIFO=1 和 FIFO=0 | 均通过 |
| gnss_service | 通过，临时提供最小 HAL 类型/宏声明 |

GNSS 原测试的 Receive_IT 桩总是返回 HAL_OK，不模拟 busy 状态；另行用带状态桩复现了 R1。无线测试使用寄存器模型，不能证明真实 DCLK、FIFO acquisition、AFC 或空中解码已成功。

### 补充复现输出

```text
GNSS init first=0 retry=-1 RxState=1
GNSS after 10min silence: position=1 UTC=1 age=600000
BCH single-bit failure: byte=7 bit=0 rc=-2
ENCODE EXPR with length=4: bytes=30 accepted=0
Invalid source enqueue=0 queue after 10 polls=2 forwarded=0
WSPR I2C disconnected: rc=0 count=1 elapsed_ms=616000
Transient standby SPI failure: TX rc=-10 ready=0 RX after 60s=0
```

复现源码、可执行文件及 Keil 日志保存在本机临时目录 `C:/Users/MQSI/AppData/Local/Temp/hb2a-review-20261006/`，没有加入固件工程。

## 覆盖范围与剩余验证

已检查 App 的 12 个服务/主流程源文件，以及自有 Core 中的 GNSS、传感器、SI5351、SX1278、摄像头、DCMI、Flash、自检、APRS/AX.25、上行、GMSK/WSPR 编码和关键外设初始化；交叉检查了 HAL UART 接收状态、启动栈、链接结果和项目源文件清单。厂商 HAL、CMSIS、OV5640、SSDV/RS 的完整逐行审计不在本次已完成范围内，主要核查其调用接口、集成和现有测试覆盖。

剩余风险：

- 未连接实板，没有烧录、RF 回环、示波器或 SDR 采样验证；README 的既有实测记录不计作本次测试。
- 软件未启用 IWDG/WWDG，Error_Handler 和各 Fault Handler 无限循环。需结合 option bytes、外部看门狗和供电设计确定无人值守故障恢复策略；避免在阻塞循环里无条件喂狗掩盖死锁。
- 没有主循环端到端故障注入测试，尤其缺少传感器异常、摄像头超时与 GNSS/上行同时到达的测试。
- 需实测共享 SPI/I2C 总线恢复、SX1278 20 dBm 自检时的限流与供电、低温/低压下的晶振和传感器表现。
- 需做至少跨多个调度周期的连续运行，以及 GNSS 断流/失锁/恢复、日期跨界、HAL tick 回绕、启动失败和一次性总线故障的验证。

建议修复顺序：R1/R2/R3/R4，然后 R5/R6/R7/R8/R10/R11，再完成 R9/R12 及对应回归；高速图传与存储按确认后的里程碑单独实现。
