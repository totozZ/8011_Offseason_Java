# 8011 机器人健康分析器

这是一个独立于 roboRIO 程序的离线桌面工具，用于读取
`/Health/...` WPILOG、查看时间线、执行可配置规则并导出报告。它不会被根机器人工程
include，也不会改变机器人控制逻辑。

## 环境与启动

使用 WPILib 2026 自带的 Java 17。在仓库根目录执行：

```powershell
$env:JAVA_HOME = 'C:\Users\Public\wpilib\2026\jdk'
.\gradlew.bat -p tools\health-analyzer test
.\gradlew.bat -p tools\health-analyzer run
```

在窗口中点击“打开 WPILOG”，或者把 `.wpilog` 拖入窗口。左侧选择信号后，将鼠标
移入时间线即可同步移动游标，并在曲线、浮动提示和右侧 Snapshot 中查看该时刻所有
已选信号的具体数值；滚轮缩放、左键拖动平移，点击异常可跳转到对应时间。原始日志
不会被修改。

含测试会话的日志默认分析时间上最后一个会话；顶部“分析范围”可切换完整日志或其他
会话。摘要、规则、时间线和导出都会随当前范围重新计算；没有会话时使用完整日志。
摘要中的“起始电压”取所选分析范围内第一条有限电压样本；它可能处于带载状态，
不应直接当作电池静置开路电压。
机器人启动会生成 `RobotBoot` 会话，因此会话 1 通常是初始化而不是第一次 Auto；
应同时核对会话名称与模式带。Hoot 为捕获初始化会更早启动，其边界不保证与 WPILOG
会话完全相同。

时间游标的 Snapshot 在连续数值信号首个样本之前显示 `Unavailable`；连续覆盖范围内
可插值；最后一个样本之后或 telemetry gap 内，最近样本距游标超过 1 秒时显示
`Unavailable (stale)`，不超过 1 秒时暂用该样本。DeviceId、故障位和计数器等阶跃型
元数据继续保持最近值。

需要在 CI/维护电脑上验证 JavaFX 窗口、日志载入、缩放/平移和异常跳转后自动退出时：

```powershell
.\gradlew.bat -p tools\health-analyzer run --args="--smoke-ui build/samples/robot-health-sample.wpilog"
```

## 生成并打开模拟日志

仓库提供纯 Java、无 JNI 的固定样例生成器。它写入真正的 WPILOG 1.0 文件，并在
任务结束前用 `WpiLogLoader` 回读、运行默认规则自检：

```powershell
.\gradlew.bat -p tools\health-analyzer generateSampleLog
```

默认输出：

```text
tools/health-analyzer/build/samples/robot-health-sample.wpilog
```

也可以指定输出文件；相对路径以 `tools/health-analyzer` 为基准：

```powershell
.\gradlew.bat -p tools\health-analyzer generateSampleLog `
  '-PsampleLogOutput=C:\temp\robot-health-sample.wpilog'
```

样例为固定 10 Hz、15 秒的数据，因此相同代码会生成相同字节。时间线包含：

- Disabled → Autonomous → Teleop → Disabled 模式和 8 个事件标记；
- 电池电压、脚本生成的总电流、聚合 CAN 利用率/错误计数及 Drive 子系统汇总；
- Drive/Left、Drive/Right 两个电机的电压、电流、Reference、Velocity、
  ClosedLoopError、Temperature、Connected、限流状态和 int64 `Faults` 位域；
- 人工注入的低压/大电流、brownout、左电机堵转、右电机掉线、CAN 高占用和
  Disabled 后异常耗流。

样例的 `/Health/Power/TotalCurrent` 是独立脚本值，不表示已确认真实 PDH，也不等于
两台示例电机 Supply Current 之和；由它计算的 Ah、Wh、峰值和百分位只用于验证算法。
它只用于熟悉界面、验证规则和演示导出，不是实车测量，也不能作为机器人健康结论。
固定 15 秒、10 Hz、约 50 KB 的结果不能外推实车 MB/min、CAN 开销或 18 台 TalonFX
的数据率。

## 规则配置

默认规则在：

```text
src/main/resources/health-rules.json
```

规则名与用途：

| 规则 | 主要判断 |
| --- | --- |
| `LOW_VOLTAGE` | 电池电压低于阈值并持续指定时间 |
| `BROWNOUT` | `/Health/Robot/BrownedOut` 为 true |
| `HIGH_TOTAL_CURRENT` | PDH 总电流瞬时值或滚动平均过高 |
| `STALL` | 有输出电压、定子电流高，但速度接近零 |
| `MOTOR_IMBALANCE` | 已声明 leader/follower 组的 Supply Current 差异过大 |
| `FOLLOWER_FAULT` | 配置的 follower 在 leader 带载时几乎无电流/速度 |
| `TRACKING_ERROR` | 电机 ClosedLoopError 持续超限 |
| `TEMPERATURE` | 温度或升温速率超限 |
| `CAN_FAULT` | CAN 利用率过高或错误计数增加 |
| `IDLE_DRAW` | 子系统 Idle/Inactive 时仍有明显电流或输出电压 |

建议复制规则文件后再调参，保留每次分析所使用的版本。`defaults` 修改全局阈值，
`subsystems` 可为 Drive、Shooter 等设置覆盖值，`followers` 用于声明 leader/follower
关系。例如：

```json
{
  "subsystems": {
    "Drive": {
      "STALL": {
        "durationSeconds": 0.5,
        "thresholds": {
          "minStatorCurrent": 70.0
        }
      }
    }
  },
  "followers": [
    {
      "subsystem": "Shooter",
      "leader": "Leader",
      "follower": "Follower"
    }
  ]
}
```

外部文件按字段深度覆盖内置默认配置；省略的规则、严重程度、持续时间和阈值仍使用
内置值。若显式写出 `followers`，它会完整替换内置 follower 列表。为使报告可复现，
应归档外部 JSON 和所用分析器版本。

电流不平衡按各电机 Supply Current 的绝对值计算，默认只比较已声明的
leader/follower 组；Pitch、转向和其他独立用途电机不会混入比较。默认差异阈值为
45%。Shooter 的跟踪误差需持续 1.5 秒才告警，以滤除正常飞轮加速瞬态。

阈值告警是排查入口，不等于故障定论。应在时间线中同时核对模式、事件、电压、
Reference/Velocity、Supply Current、Stator Current 和温度。

## 导出报告

加载日志并完成分析后，可从界面导出：

- JSON：适合归档、脚本处理和后续比较；
- CSV：适合在表格工具中筛选统计与异常；
- HTML：单文件中文报告，适合浏览和分享。

任一导出进行时，GUI 会禁用全部导出按钮并拒绝重叠请求。三种格式都先写入目标目录内
的唯一临时文件，完成后优先原子替换目标；文件系统不支持原子移动时才安全回退为
replace。已有报告不会被预先截断，失败时会尽量清理临时文件。

建议把原始 `.wpilog`、规则 JSON、导出报告和测试记录放在同一归档目录，并在文件名中
记录日期、机器人、电池编号和测试目的。若异常涉及 CTRE 设备细节，再用同一测试的
Phoenix Hoot 日志在 AdvantageScope 或 Tuner X 中复核。

无界面批量分析也使用相同的会话语义：默认最后一个会话，`full` 表示完整日志，正整数
表示会话编号。

```powershell
.\gradlew.bat -p tools\health-analyzer run --args="--analyze path\test.wpilog --session last --export-dir build\report"
```

## 已知边界

- 当前 loader 消费 `/Health/...` 下的 boolean、double、int64 和 string 标量；int64
  会无损覆盖常见 CAN/故障计数范围并规范化为数值序列，其他命名空间和不支持的类型
  会被忽略。
- `Unavailable` 不等于零或健康：缺少有限数值样本时，界面和 HTML 显示
  `Unavailable`，CSV 留空，JSON 写 `null`，依赖该信号的规则不作健康结论。
- 连续数值 telemetry 的相邻样本间隔严格大于 1 秒视为 gap。按 selected range 计算
  电流/能量时，首尾各容忍该序列实际观察到的最大连续采样间隔再加一个 20 ms
  机器人循环（总容忍最多 1 秒），以覆盖 roboRIO 调度、CAN 读取及会话启动边界抖动；
  首样本或末样本离范围边界超过该容忍时，时间加权均值、滚动窗口峰值、
  阈值持续时间、积分、Ah/Wh 为 `Unavailable`，不会向边界外推。raw 最小值/最大值、
  样本百分位和 raw peak 仍保留实际观测样本的统计结果。
- 对 `>1 s` gap，可基于原始阈值评价的规则会分别检查两侧真实连续段，但不会跨 gap
  累积持续时间或合并异常；要求完整连续输入的滚动或多信号规则不使用缺失区间作结论。
- 以已注册 TalonFX Supply Current 回退生成子系统聚合电流时，所有登记电机必须及时、
  连续覆盖整个分析范围。覆盖不全会把来源标为
  `PARTIAL[...;motorSeries=已有序列数/登记数;fullCoverage=完整覆盖数/登记数]`，并使
  aggregate current/energy/imbalance 为 `Unavailable`；单电机统计不受此聚合保护影响。
- Loader 会把文件读入内存，默认拒绝超过 512 MiB 的 WPILOG，实际 heap 需求高于
  文件大小。优先裁剪日志；仅确认内存足够后才用 JVM 属性
  `healthAnalyzer.maxLogBytes` 提高限制。
- 分析器不读取 Phoenix Hoot。
- 模拟日志中的 `Connected`、限流和事件用于表达故障；它不模拟真实 CAN 设备或电机。
- 默认 `followers` 已按当前源码登记 Shooter 与 GroundIntake 的已知
  leader/follower 关系；硬件或命名改变时必须同步更新，其他机构只有明确填写关系后
  才会执行 `FOLLOWER_FAULT` 规则。自动匹配使用子系统、规范化 CAN 总线和 DeviceId
  元数据，跨总线的相同设备号不会被当作同一电机。
