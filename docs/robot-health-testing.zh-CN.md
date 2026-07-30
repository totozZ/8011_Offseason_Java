# 机器人健康日志与测试后体检

本文说明 8011 机器人健康日志的使用方法。健康日志只负责观测，不参与电机控制；
配置缺失或设备离线时，机器人原有控制应继续运行。

## 1. 日志类型

系统同时使用两种日志：

- **WPILOG (`.wpilog`)**：由 WPILib `DataLogManager` 写入，包含 Driver Station
  状态、摇杆、机器人模式、整机电源、CAN、子系统、电机健康数据和事件。它可以直接
  用 AdvantageScope 打开，也是桌面健康分析器的输入。
- **Phoenix Hoot (`.hoot`)**：由 CTRE Phoenix 6 `SignalLogger` 写入，适合在
  AdvantageScope/Tuner X 中做更细的 CTRE 设备和控制信号诊断。第一版桌面分析器
  不直接解析 Hoot。

机器人启动时会先开启名为 `RobotBoot` 的初始化会话；第一次进入 Disabled 会结束
它。此后每次 Auto、Teleop 或 Test Enable 会开启一个测试会话，Disable 会结束当前
会话并停止 Hoot 记录。WPILOG 在机器人程序运行期间持续记录，因此一次 WPILOG 可以
包含启动会话和多个 Enable/Disable 测试会话。分析器中的“会话 1”通常是
`RobotBoot`，不应默认理解为第一次 Auto。

## 2. 启动与停止

机器人程序启动后会自动初始化健康日志：

```java
DataLogManager.start();
DriverStation.startDataLog(DataLogManager.getLog(), true);
```

Phoenix 6 日志由同一个健康日志模块统一启动和停止，避免重复创建日志所有者。为捕获
设备初始化，`SignalLogger` 会在 `RobotBoot` 会话标记之前启动；因此 Hoot 的起点与
WPILOG 会话边界不保证完全相同。正常测试不需要在各 Subsystem 或 Command 中手动
启动记录。

代码也提供显式会话 API：

```java
healthLogger.startTestSession("Shooter bench");
healthLogger.markEvent("Operator", "开始 20 RPS 台架测试");
healthLogger.stopTestSession();
```

会话名是日志中的值，不会改变 `/Health/...` 信号路径。

## 3. 日志保存位置

WPILib 会优先把 WPILOG 写到 roboRIO 上已挂载 U 盘的 `logs` 目录；没有 U 盘时写到：

```text
/home/lvuser/logs
```

Phoenix Hoot 通常也优先写入 U 盘，否则写入 roboRIO 本地日志目录。长时间测试前应
确认剩余空间；测试后及时下载并归档日志。

可通过 roboRIO 网页文件浏览器、WPILib 工具或 `scp` 下载。例如：

```powershell
scp "lvuser@roborio-8011-frc.local:/home/lvuser/logs/*.wpilog" .
scp "lvuser@roborio-8011-frc.local:/home/lvuser/logs/*.hoot" .
```

若 mDNS 名称不可用，请改用 Driver Station 显示的 roboRIO IP。下载前先 Disable
机器人，并等待会话停止/日志刷新完成。

## 4. 用 AdvantageScope 查看

1. 打开 AdvantageScope。
2. 选择 `File -> Open Log...`，选中 `.wpilog`。
3. 在信号树中展开 `/Health`。
4. 将电压、总电流、子系统电流、电机电流/速度/Reference、模式和事件拖入时间轴。
5. CTRE 细粒度诊断则单独打开同一次测试生成的 `.hoot`。

WPILOG 的路径保持稳定；测试会话名、Command 名和状态仅作为信号值变化。

## 5. 启动桌面分析器

桌面工具位于独立 Gradle 工程：

```text
tools/health-analyzer
```

它不会被根机器人工程 include，也不会把 JavaFX 或 JSON 依赖打包进 roboRIO 程序。
请使用 WPILib 自带的 Java 17：

```powershell
$env:JAVA_HOME = 'C:\Users\Public\wpilib\2026\jdk'
.\gradlew.bat -p tools\health-analyzer run
```

disable之后将日志下载到本地log目录：
scp "lvuser@roborio-8011-frc.local:/home/lvuser/logs/*.wpilog" ".\log\"

只下载最新目录的日志：

```
$robot = 'lvuser@roborio-8011-frc.local'

$latest = ssh $robot 'ls -1t /home/lvuser/logs/*.wpilog | head -n 1' |
    Where-Object { $_ -match '\.wpilog$' } |
    Select-Object -Last 1

$latest = $latest.Trim()
scp "${robot}:$latest" '.\log\'
```

在窗口中点击“打开 WPILOG”，或把 `.wpilog` 拖入窗口。选择信号后可使用：

- 鼠标滚轮：以鼠标所在时间为中心缩放。
- 左键拖动：平移全局时间范围。
- 鼠标悬停：移动全局游标，并在每条曲线、浮动提示和右侧 Snapshot 中同时显示该
  时刻所有已选信号的具体数值。
- 单击时间线：固定游标到所选时间。
- 单击异常列表：跳转到异常开始时间。
- 导出按钮：生成 JSON、CSV 或 HTML 体检报告。

时间游标的 Snapshot 对连续数值信号只显示在该时刻可信的值：首个样本之前显示
`Unavailable`；连续覆盖范围内可插值；最后一个样本之后或 telemetry gap 内，只在最近
样本距游标不超过 1 秒时暂用该样本，超过 1 秒则显示 `Unavailable (stale)`。DeviceId、
故障位和计数器等阶跃型元数据按最近值保持，不套用连续数值的 stale 规则。

GUI 在任一 JSON、CSV 或 HTML 导出进行中会禁用全部导出按钮并拒绝重叠请求。报告先
写入目标目录内的唯一临时文件，完成后优先原子替换目标；文件系统不支持原子移动时才
安全回退为 replace。已有目标不会被预先截断，失败时会尽量清理临时文件。

若日志包含 `/Health/TestSession/Active`，分析器会默认选择时间上最后一个测试会话；
顶部“分析范围”可切换完整日志或其他会话。摘要、规则检测、时间线和报告导出都只
针对当前选中的范围。没有会话信号时使用完整日志。由于启动时会记录 `RobotBoot`，
`--session 1` 通常选择初始化而不是第一次 Auto；请同时核对会话名称和模式带。

无界面导出默认也选择最后一个会话，可用 `--session full` 或 `--session N` 选择完整
日志或编号为 N 的会话：

```powershell
.\gradlew.bat -p tools\health-analyzer run `
  --args="--analyze path\test.wpilog --session last --export-dir build\report"
```

分析器完全离线工作。首次构建需要由 Gradle 获取桌面依赖；完成 distribution 后，
运行所需依赖会随分发包保存。Loader 会把输入读入内存，并默认拒绝超过 512 MiB 的
WPILOG（实际 heap 需求会高于文件大小）；优先在 AdvantageScope 中裁剪过大的日志。
仅在确认内存足够时，才用 JVM 属性 `healthAnalyzer.maxLogBytes` 调高上限。

对需要连续时间覆盖的数值 telemetry，相邻样本间隔严格大于 1 秒即视为 gap（恰好
1 秒仍连续）。按 selected range 计算电流/能量时，首尾还各允许该序列实际观察到的
最大连续采样间隔再加一个 20 ms 机器人循环（总容忍最多 1 秒）；若首样本或末样本
离所选范围边界超过该容忍，时间加权均值、滚动窗口峰值、阈值持续时间、积分、Ah 和 Wh 保守显示
`Unavailable`，且不会向范围边界外推。raw 最小值/最大值、样本百分位和 raw peak 仍按
实际观测样本计算。

对于 `>1 s` gap，可基于原始阈值评价的规则会分别检查 gap 两侧的真实连续段，但持续
时间不会跨 gap 累积或把两段合并；需要完整连续输入的滚动或多信号规则则不使用缺失
区间作结论。

## 6. 填写 PDH/PDP 与通道映射

实车已确认为 REV PDH；当前按 REV 默认 CAN ID `1` 配置 24 路采集。若该 PDH 曾在
REV Hardware Client 中修改 CAN ID，必须同步修改 `Robot.kPowerDistributionCanId`。
通道接线表尚未确认，因此所有通道暂命名为 `Unknown`/`ChannelXX`，不会猜测实际接线。
这不影响整机总电流、总功率、总能量和每个通道原始电流的采集。

在确认实车接线后，修改：

```text
src/main/java/frc/robot/logging/HealthConfig.java
src/main/java/frc/robot/logging/PdhChannelMap.java
```

至少核对：

1. 模块是 REV PDH 还是 CTRE PDP。
2. CAN ID。
3. 实际通道数（REV PDH 24 路、CTRE PDP 16 路）。
4. 每个通道连接的设备名称和所属子系统。

REV PDH 不提供 WPILib 温度遥测；其 API 返回的 `0` 是不支持的占位值。记录器不会把
它写成真实 `0 °C`，所以 `/Health/Power/Temperature` 保持 `Unavailable`。

示意映射（仅示意，不能直接当作 8011 实车接线）：

```java
PdhChannelMap.builder(24)
    .map(0, "Drive", "FrontLeftDrive")
    .map(1, "Shooter", "FlywheelLeader")
    .build();
```

映射缺失时，子系统 `SupplyCurrent` 会回退为该子系统已注册 TalonFX 的
`SupplyCurrent` 之和，并通过 `/Health/Subsystems/{name}/CurrentSource` 标明来源。整机
`PDH Total Current` 不会用电机电流或 `StatorCurrent` 伪造。

该回退只有在每台已注册电机的 Supply Current 都及时且连续覆盖所选分析范围时才生成
聚合值。覆盖不全时，来源会标为
`PARTIAL[...;motorSeries=已有序列数/登记数;fullCoverage=完整覆盖数/登记数]`，聚合
current/energy/imbalance 均为 `Unavailable`，不会把部分电机之和冒充子系统总电流；
单电机统计仍可按各自实际观测样本显示。

禁用或初始化失败时，日志会明确写入
`/Health/Power/DistributionEnabled=false` 和
`/Health/Power/DistributionType="Unavailable"`。未知的总电流、功率、能量、温度及
通道电流不会写成 `0`，而是没有有限数值样本。分析器界面和 HTML 显示
`Unavailable`，CSV 留空，JSON 写 `null`。缺少数据只表示对应规则无法评价，不能据此
判断设备健康或负载为零。

roboRIO 聚合 CAN 和已注册 TalonFX 所在网络分别记录在：

```text
/Health/Robot/CAN/{Utilization,BusOffCount,ReceiveErrors,TransmitErrors,TxFullCount}
/Health/Robot/CAN/Buses/{bus}/{Utilization,BusOffCount,TxFullCount,
  ReceiveErrorCount,TransmitErrorCount,Connected,Status}
```

`{bus}` 只包含至少注册了一台 TalonFX 的网络，例如 `rio`、`CANivore`。规则会检查
分总线利用率、BusOff、REC/TEC 和 `Connected`；`TxFullCount`、`Status` 也保留给
时间线人工复核。

## 7. 修改异常阈值

桌面分析器默认规则位于：

```text
tools/health-analyzer/src/main/resources/health-rules.json
```

可以复制为测试专用文件后调整全局阈值，以及 Drive、Shooter、Feeder、
GroundIntake 等子系统覆盖值。每次报告应保存所用规则配置，便于复现结论。

外部规则文件按字段深度覆盖程序内置默认值：没有写出的规则、严重程度、持续时间和
阈值继续继承内置配置，不会静默变成零持续时间。若外部文件显式包含 `followers`
数组，则该数组会完整替换内置 follower 列表。归档时应同时保存外部规则文件和分析器
版本，因为有效配置由两者共同决定。

建议先积累多次正常测试日志再收紧阈值。不要只根据一次峰值判断机械故障；重点观察
持续时间、滚动平均、Reference/Measured 误差、速度和电流是否同时异常。

## 8. 三种电流的区别

- **Supply Current**：电机控制器从电池/配电系统一侧吸取的电流。可用于估算电池
  负载和子系统能耗。
- **Stator Current**：电机定子绕组中的电流，和电磁转矩、堵转风险更直接相关。
  它可能明显高于 Supply Current，不能直接相加当作整机电池电流。
- **PDH Total Current**：电源分配模块测得的整机电池侧总电流。整机峰值、Ah、
  Wh 应优先使用这一来源。

报告会明确标注电流类型和子系统电流来源。

## 9. 标准健康测试流程

1. 架车或划定安全区域，检查急停、保险、机构机械限位和周围人员。
2. 确认电池已充电并记录电池编号；确认 U 盘/roboRIO 日志空间充足。
3. 打开 Driver Station，等待 CAN 设备稳定，记录任何启动故障。
4. Enable 前添加测试名称或操作员事件标记。
5. 依次测试 Drive、GroundIntake、Feeder、Shooter；每个机构先低负载，再到目标
   工况。避免多个未知风险机构第一次同时动作。
6. 在关键动作前后添加事件标记，例如球进入、射球、碰撞、卡住、人工停止。
7. 测试结束后立即 Disable，等待日志停止/刷新。
8. 下载 `.wpilog` 和对应 `.hoot`，文件名中记录日期、电池、机器人和测试目的。
9. 用桌面分析器查看摘要与异常，再用 AdvantageScope 对异常时间点做信号级复核。
10. 将报告、规则文件、原始日志和维修结论一起归档。

## 10. 日志开销检查

日志会同时记录完整机器人循环时间 `/Health/Robot/LoopTimeMs` 和健康记录器自身耗时
`/Health/Logger/PeriodicTimeMs`。台架测试时应在 Enable 前后比较 P95/P99，并同时
观察 roboRIO 和 `/Health/Robot/CAN/Buses/*`。

默认配置不调用 Phoenix `setUpdateFrequencyForAll`。健康信号按 CAN 网络分批、非阻塞
刷新；电机数值仅在 Phoenix 的稳定系统时间戳表明信号已更新时写入，时间线中的缺口
表示没有新的设备样本。`motor*SampleHz` 是 logger 的刷新/记录上限，不等于设备状态帧频率；
降低它可以减少 CPU/文件开销，但不保证降低 CAN 流量。任何 Phoenix 频率调整都必须
显式启用，并重新测量每条总线。

`generateSampleLog` 会生成固定 15 秒模拟 WPILOG，打印大小，并回读执行默认规则：

```powershell
.\gradlew.bat -p tools\health-analyzer generateSampleLog
Get-Item tools\health-analyzer\build\samples\robot-health-sample.wpilog
```

模拟文件只能验证格式、算法和 UI，不代表 18 个真实 TalonFX、CANivore 或存储介质
上的数据率，不能用约 50 KB/15 s 外推实车 MB/min 或 CAN 开销。

正式采用前，用最终接线和最终采样配置完成一次可审计的实车验收：

1. 记录构建版本、电池编号、开始/结束文件大小和剩余空间；覆盖 Disabled 基线以及
   至少 10 分钟 Enabled 测试会话负载，计算 MB/min。
2. 测试前定义 `LoopTimeMs`、`PeriodicTimeMs` P95/P99 和 scheduler overrun 的通过
   阈值，并与同一硬件的无健康日志基线比较。
3. 确认 `/Health/Logger/ErrorCount` 不增加、`LastError` 无新错误；所有预期
   `/Health/Robot/CAN/Buses/{bus}/Connected` 为 true，BusOff/REC/TEC 不增加。
4. 确认无意外 brownout/掉线，Phoenix 数值样本间隔与已配置设备频率一致。
5. 确认日志可在默认 512 MiB 限制内加载并成功导出报告；归档 WPILOG、Hoot、规则、
   基线数据和最终结论。

## 11. 当前已知限制

- 配电模块已确认为 REV PDH，并暂按默认 CAN ID `1` 配置；该 ID 仍需在 REV Hardware
  Client 或新日志的 `/Health/Power/DistributionEnabled`、`DistributionModule` 中复核。
  通道接线映射尚未确认；REV PDH 本身不支持温度遥测，因此温度保持 `Unavailable`。
- 活动代码只能确认 TalonFX 控制器，不能从仓库可靠断定所有电机是 Kraken X60、
  X44 或 Falcon。
- 第一版桌面分析器只解析 WPILOG；Hoot 需使用 AdvantageScope/Tuner X。
- Java 迁移分支尚未完成真实机器人 bench validation；CANivore、Pigeon、模块
  offset、归零方向、限流和 Brake/Coast 仍按迁移检查表验证。
- `generated/TunerConstants.java` 当前使用带 Hoot 文件参数的 CANBus 构造方式，
  具有 Phoenix replay 风险。该生成配置应在上机前由 Tuner X/实车流程独立复核，
  本健康日志改动不擅自改变它。
- 自动规则是诊断提示，不替代机械、电气和接线检查。
