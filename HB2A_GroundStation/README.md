# HB-2A 自动地面站

SDR 同时接收 4800 / 9600 / 19200 / 38400 / 57600 / 115200 bit/s GMSK。
上行固定 4800 bit/s，通过 IC-9700 的 USB 声卡输入，在 FM-D 模式发射；CAT/PTT 使用 Hamlib。
USB 连接枚举出的音频设备负责波形，COM 设备负责 CI-V/CAT，两者不混用。

## 启动

本机已有 `C:\Users\MQSI\radioconda`，双击本目录 `start.cmd`。首次需要 GCC 编译本项目已有的飞行端协议库；修改气球编码文件后执行 `python build_native.py` 重新构建。

其他机器可以用 conda/mamba 创建 `environment.yml` 中的环境，再运行：

```text
python build_native.py
python station.py
```

`HB2A_PYTHON` 可以指定其他 GNU Radio 环境的 Python。当前实测环境为 GNU Radio 3.8.3.1、gr-satellites、NumPy 1.21.1、SciPy 1.7.0；环境文件面向 GNU Radio 3.10，尚未在本机安装该版本验证。

## IC-9700 USB 接法
 
接收默认使用 Pluto / 兼容 AD936x 设备，URI 为 `ip:192.168.1.10`。此地址来自桌面初始化脚本 `set_eth0_ip.py` 的 Windows 分支；该脚本文件头所写的 `192.168.137.10` 与实际执行代码不一致。地面站不执行串口初始化脚本、不修改网络地址，电脑需能访问设备的 TCP 30431 端口。Linux 分支地址为 `192.168.5.10`，可修改界面的 Pluto URI 或使用 `--pluto-uri ip:192.168.5.10`。

接收复用 GNU Radio 的 gr-osmosdr / SoapyPlutoSDR / libiio，默认设置 521 ksps、400 kHz 模拟带宽及驱动自动 AGC。Pluto 只由 Soapy 单独建立 libiio context，避免重复打开网络设备导致 `no device context found`。521 ksps 仍满足 115.2 kbps 档的 4 倍采样要求，同时原始 IQ 约 15.0 GB/小时；之前的 1.024 Msps 约 29.5 GB/小时，容易造成网络、磁盘和 CPU 压力。六个物理速率常驻，CodeID 和交织深度在收到 `HSCON` 后动态激活，未选中的 CodeID 不创建 Viterbi 解码器；默认只建立 CodeID=1 的相关器，非默认编码链路可用 `--all-codes`，反相链路再加 `--both-polarities`。采样率、AGC、增益不显示为业务输入框，读取的实际设备配置保存进会话日志；其他 SDR 仍可用命令行 `--sdr` 指定。

频谱窗口的 Welch/FFT 计算使用 FFTW 单精度原生 DLL `build/hb2a_dsp.dll`，Python 只负责取 IQ 快照和更新图形。`build_native.py` 会编译该 DLL；若当前地面站仍在运行，协议 DLL 会保持不覆盖，频谱 DLL 可独立更新。滤波、FSK/GMSK 解调、符号同步、相关器和 Viterbi FEC 已由 GNU Radio/gr-satellites 的 C/C++ 块执行，Python 层不重复实现这些 DSP 内核。

发射后端有两个选项：`IC-9700 USB audio` 使用 CAT + USB 声卡；`Hardware transceiver` 使用仓库 `HB2A_Uplink/Host/protocol.py` 的现有 USB CDC 协议，通过 COM 口以 115200 baud 发送 `STATUS` 和 `SEND <power> <hex>`，由 HB2A 上行硬件板完成 GMSK/RF 发射。界面会枚举 COM 口，功率限制为 2--17 dBm；硬件回报 `OK SENT` 后才记录发送完成，串口中断不会自动重发。该后端直接复用上行主机协议，不重复实现 BCH/NRZI/CRC。

1. 安装 Icom USB 驱动，以 USB 线连接电脑和电台。
2. 电台 `Connectors > DATA MOD` 选择 `USB`，调整 `USB MOD Level`；在系统音频设置中禁用增强、空间音效和自动音量，使用 48 kHz。
3. 地面站 `USB audio output` 下拉框选择电台的 `USB Audio CODEC` 播放设备，插拔后点击 `Refresh`。Windows 下直接枚举 GNU Radio 使用的 waveOut 设备，编号区分同名声卡；只有唯一匹配时才自动选中 USB Audio CODEC。每次发射前核对设备，丢失时停止发送，不主动回退到默认扬声器。
4. 使用 Hamlib 管理 CI-V 串口。例如先用 `rigctl -l` 查找本机版本中 IC-9700 的型号编号，然后启动 `rigctld -m <型号编号> -r COMx -s 19200 -t 4532`。波特率必须与电台一致，其他软件不要同时占用这个串口。
5. 点击 `Start automatic` 后，程序通过 CAT 关闭 split，设置 435.400 MHz 和 `PKTFM`（FM-D），发射前检查频率，使用 CAT PTT。请先在电台上配置好 DATA MOD=USB。
6. 用 SDR/频偏计校准 USB 输出电平，使 4800 bit/s 上行的频偏约为 ±1200 Hz。程序电平是归一化音频幅度，不是实际 RF 频偏。先做有衰减的本地 RF 回环，确认电台音频通道没有将低频或高频基带严重削弱。

输出为 BT=0.5 的高斯成形 NRZ 实数基带，经电台 FM 调制得到 GMSK；不附加 1200/2200 Hz AFSK 音调，也不把复数 IQ 送入声卡。程序兼容已有 8 类上行命令，BCH、NRZI、CRC 直接调用飞行端 C 实现。

官方资料：[IC-9700 Advanced Manual](https://www.icomjapan.com/support/manual/2166/)，数据通信见 1-16、USB 连接见 1-17。2.5 mm `[DATA]` 插座的 GPS/D-STAR 串行功能不等于 USB 音频调制。USB FM-D 能否在你的设备、固件与音频设置下完整承载这个 4800 GMSK 波形，仍须实机验证。

## 连续业务

- `Receive` 只接收、录制，不发射。`Start automatic` 启动自动链路和连续图片业务。
- `Receive` 不会发射。`Manual test EXPR` 是一次性手动测试：它先通过 CAT 预检并按所选声卡发射一个 `EXPR` 测试/续约帧，然后回到纯接收状态，不启动连续业务、不发送 `CAPTR` 或 `REQF`。要进行全自动业务，仍需单独点击 `Start automatic`；自动模式会自行发送 `EXPR` 并在成功后开始业务。
- `Receive` 自动打开实时频谱与瀑布图，关闭后可用 `Spectrum` 重开。显示每 250 ms 从当前 IQ 提取最新样本，使用 SciPy Welch 估计功率谱，不依赖成功解包；同时显示累计样本数、IQ 电平以及断流状态。支持暂停显示、自动纵轴和 Matplotlib 缩放/保存。纵轴为相对 IQ 数值的功率谱密度，不是校准后的 dBm，也不是链路协商所用的 Es/N0。暂停或关闭图窗不会停止录制和解调。
- 每轮先 `EXPR` 进入/续约试验模式，等待 `ACK,EXPR`；重试最多三次，失败后等待 15 秒重新连接。
- CAT/音频发送报错时释放PTT、清空旧发送队列，15秒后自动重试；点击停止自动后不再重连。
- 新业务启动时读取下方输入框。默认 `CAPTR`，选择 `REQF` 时执行一次指定文件请求，然后选择器恢复 `CAPTR`。业务执行中修改输入框只影响下一业务。
- 尺寸、质量等业务参数非法时，自动流程进入 `parameters` 等待状态，继续接收，每秒检查修改后的参数，同一错误只记录一次。改正后先重新发送 EXPR 建立链路，再发送已验证的业务请求；一次性 REQF 不会在重连时丢失。点击停止自动则取消恢复。参数错误不再落入 GUI 通用异常处理、悄悄关闭自动模式。
- `CAPTR` 尺寸暂限已有预设：128x96、256x192、320x240、480x272、640x480、800x480。地面站在发射前拒绝其他尺寸。气球固件需同步更新，正式拍照不再额外调用任意尺寸时序配置；预设仍需逐档实板验证。JPEG 仍受约 64 KiB 捕获缓冲限制。
- 收到 `HSSUP` 后自动选择被支持的参数。优先 CodeID=1、交织深度4，并从最高支持速率开始实测，失败向下降档；不向操作员暴露 RateID/CodeID/IntlvID。
- 六档一直同时接收。表格的 `Current probe` 单独显示本轮待探测、等待 HSCON、探测中、超时或选中状态；`Last Es/N0` / `Last Eb/N0` 是最近一次成功解包的测量值，不代表本轮通过。日志显示实际 bit/s，避免只看到 RateID。第一档通过后不会继续测试更低档。
- `HSCON` 必须匹配本次请求；随后用候选速率的 `HSTST` 测量 Es/N0。默认 **Es/N0 >= 3.5 dB** 确认，`--esn0-threshold 3.0` 可设置 3.0–3.5 dB 内的统一门限，各档不加额外速率惩罚。
- 发送 HSPAR/CHPRAM 时提前设置接收配置；HSCON 重复确认相同 CodeID/交织深度时不清空接收缓存，避免中断已到达的高速探测组。仅实际参数改变才重置组装状态。
- 候选帧尝试完当前最大交织深度后立即释放缓存与队列位置，避免失败候选长期占满 16 个位置后阻止新探测帧进入。
- `CFRM` 之后保持该业务的链路参数。重复收到 `HSCON` 时重新等待其探测帧，避免在气球发射探测期间发送确认。
- 图片 EOI 后等待 `HSDONE`，然后进入下一次续约/业务。若已收到完整 EOI 但缺失结束通知，保留已保存图像并记录 `Image complete; HSDONE missing`，随后续约，不再发送会打断已完成业务的 `CANCEL`。
- 新业务开始时保存上一张图的最新部分解码结果，再撤下 GUI 旧图并清空图片包缓存；被动接收时收到新图片的首个有效包也会撤下旧图。接收期间最多每半秒按已有 SSDV 包重建、刷新图片，EOI 到达立即刷新。显示已收包数、已知缺包数、是否收到 EOI；没有 EOI 时总包数未知，不能把“已知缺包为零”当作接收完整。
- 部分解码使用现有 SSDV 库填充尚未收到的图像块；每次从已收包的有序快照重建，避免 `ssdv_dec_get_jpeg` 结束解码状态后无法继续，也支持乱序/迟到包补齐。重复包不重绘、不清空预览。只有 EOI 和全部包均到齐才保存为 `image-*.jpg`，否则保存为 `image-*-partial.jpg`；业务切换、接收停止时也保存最后快照，即使 EOI 丢失。图像解码/显示错误不会关闭自动业务。历史图片与原始 SSDV 包保留。
- 新版气球固件会把 `HSREJ,TRANSFER` 的负结果码附在控制帧后；地面站会记录该码，便于区分相机、存储和 SSDV/RF 发送阶段的失败。
- 流程图的 2.5 秒作为响应预算参考，但本协议两个 4800 控制帧本身约需 1.8 秒；超时还需包括实际控制和探测帧空中时间。业务首次数据允许摄像头启动时间，之后收包将无数据超时续至 30 秒。
- `Stop automatic` 停止后续发射与业务。气球已开始的图片传输不能立即中止，因为当前固件传图期间不监听上行；接收和录制会继续。`Stop all` 停止接收并关闭记录文件。

**需要同步更新气球固件**：本次在 `app_main.c` 加入候选速率 `HSTST`、`HSDONE` 和试验模式 `EXPR` 续约；在 `high_speed_service.c` 支持重复 HSPAR；修正 `gmsk_downlink.c` CodeID=3 的 3/4 穿孔表。旧固件没有探测帧时地面站不会误用 4800 控制帧的质量确认高速。没有烧录任何硬件。

## DSP 与测量定义

原始软判决 RS 解码失败时，卷积编码帧最多重试一次，将归一化鉴频软判决限幅至 ±0.5，减轻异常尖峰的错误权重；仍经过同样的 ASM、RS、帧格式和图像 CRC 校验。每包日志的 `decode_method` 标记 `original` 或 `clipped_soft_retry`，原始 `.soft.f32` 不变，Es/N0 测量和切换门限不变。实录 `20261007-035918-598400` 首段 IQ 在此修正前解出 2 个控制帧，修正后解出 6 个（含 HSSUP），仍存在丢帧，不代表完整图传已通过。

滤波、FM 解调、符号同步使用 GNU Radio / gr-satellites 的 FSK 解调器；软判决 Viterbi 使用 GNU Radio FEC；PSD 使用 SciPy Welch。RS、SSDV、BCH/NRZI、CRC 复用仓库原有实现。项目代码只适配帧格式、去穿孔、交织、PN17、控制流程和测量单位。

Es/N0 在候选帧内部 IQ 上估计：积分 ±0.9Rs 的信号加噪声功率，两侧 1.25–1.75Rs 估计 N0，扣掉带内噪声后计算 `10log10(Psignal / (N0 * Rs))`。Pluto 默认慢速 AGC，其他 SDR 的命令行配置保留固定增益；采样率至少 460800 sample/s。该估计假设附近噪声近似平坦、无强邻频干扰，需校正接收中心偏差；AGC 在测量窗口内剧烈变化或输入削顶会影响估计，仍需实机校验。它不是 RSSI、宽带 SNR 或解调后音频 SNR。

日志中的 Eb/N0 指 **223 字节 TM 信息区域（含头和 CRC）** 的每比特能量，按该组实际编码长度（含 ASM/卷积尾位，不含前导）换算：`Eb/N0 = Es/N0 - 10log10(223*8*码字数 / 编码符号数)`。同时保存换算因子。未能测量时记录 `null`，不会填造一个数值。交织组中的每包共享该组测量窗口。

**门限资格和接收灵敏度是两回事**：3.0–3.5 dB 门限已实现；当前非相干 FSK 解调链在低 Es/N0 下仍可能丢帧。6 dB 的已有噪声诊断样例仍出现 RS 失败，不能宣称 3.5 dB 下已保证可靠接收。确认需要收到本轮有效探测信息，无法解码时会超时降档。后续可替换为成熟的相干 GMSK 解调库，并用保存的 IQ 比较；程序不会因界面显示一个高质量数值就盲发确认。

## 可复盘记录

每个接收会话创建独立 `output/<时间>-<编号>/`：

| 文件 | 内容 |
| --- | --- |
| `iq-*.sigmf-data` / `.sigmf-meta` | 连续原始 SDR IQ，调谐/滤波前 complex float32 little-endian，每60秒分段，不主动删除旧段；包含采样率、中心频率、样本起点 |
| `events.jsonl` | UTC、单调时钟、序号、配置、业务编号、命令、重试、PTT、每包测量、FEC/RS失败、图片缺包情况、程序与原有协议源码SHA-256 |
| `rx-*.soft.f32` | 帧候选的解调软判决；包括不同交织深度尝试的失败记录 |
| `rx-*.tm` | 校验通过的223字节 TM 帧，逐组顺序保存 |
| `tx-*.f32` / `.wav` | 每次发送实际生成的48kHz基带，含发射前后静音；WAV为便于播放的16bit副本，float32文件保留完整值 |
| `tx-*.bits` | 空中编码位流；有效位数在日志中 |
| `image-*.ssdv` / `.jpg` | 标准SSDV重建包和JPEG，缺包情况另外记录 |

默认 521 ksps 原始 IQ 约 **15.0 GB/小时**。文件不循环覆盖，请准备足够空间。日志标出的 IQ 样本位置来自符号时钟换算，属于近似定位，包含滤波/时钟恢复延迟；原始 IQ 才是完整复盘依据。没有找到同步字的射频片段不伪造“数据包”，但仍在连续 IQ 中。

离线接收示例（不发射）：

```text
python station.py --headless --iq-file output/<会话>/iq-000000000000.sigmf-data --sample-rate 1024000
```

分段文件可以按样本起点排序拼接后回放；跨段的帧需要同时提供相邻段。回放默认也生成独立会话记录。

## 验证

`20261007-040837-712500` 实录第一段 IQ 离线回放可解出第二轮 115200、57600、38400、19200、9600 的 `HSTST`，而现场日志在 115200 组尝试深度 1/2/3 后、HSCON 更新配置处中断，随后降到 4800。该会话记录的 `dsp.py` SHA-256 与当前修复版本不同；不能据此认定当时高速射频质量不足。修改后必须退出并重新启动地面站，已运行的 Python 进程不会自动加载磁盘改动。离线解码通过仍需重启后的连续业务实测确认。

```text
python build_native.py
python -m unittest discover -s tests -v
```

测试覆盖四种卷积编码和六种交织深度、RS 16字节纠错、六速率混合IQ、12dB噪声IQ回环、2.5/3/3.5/5dB的功率估计、门限边界、过期/错误探测拒绝、业务续接、重试、连续IQ重组、上行基带记录，以及真实飞行JPEG的SSDV/TM重建。`tests/diagnose_modem.py` 保留6dB弱信号诊断样例。

本机没有完成 SDR 到 IC-9700 到气球的射频实测；无噪声/仿真通过不能替代实机频偏、电平、USB通道带宽、时钟误差和长期录制吞吐验证。第三方 RS/SSDV 的 LGPL/GPL 许可保留在原源码中，发布二进制时需一并遵守。
