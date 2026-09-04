# FRC 8011 Java 代码与基础工程规范

本规范适用于本仓库的 WPILib 2026 Java Command-Based 代码。目标是让新机构代码可以复制、检查、记录和安全中断，
而不是建立一层隐藏 Phoenix 6 或 WPILib 行为的通用电机封装。本基础分支不引入
`project-plan-orchestrator` 契约。

## 1. 代码和硬件真值

底盘现在有两个可编译期选择的硬件 profile，开关集中在
`Constants.DriveConstants.ACTIVE_PROFILE`：

1. `NORMAL` 使用 `src/main/java/frc/robot/generated/TunerConstants.java`。它暂时沿用本项目原有配置，普通底盘
   完成新一轮 Tuner X 标定后应整体替换此文件。
2. `SOCCER_BOT` 使用 `src/main/java/frc/robot/config/SoccerBotDrivetrainConstants.java`。它保存从
   `C:\Users\95833\Desktop\sum` 迁入的当前生效参数和左前驱动单独反转修正。
3. Tuner X 导出的 JSON 是生成存档。当前 `docs/archive/tuner-project.DO_NOT_USE.json` 与普通底盘不一致，
   只能用于历史追踪，禁止重新生成代码。
4. `src/main/deploy/pathplanner/settings.json` 是 PathPlanner 的自动轨迹物理模型，不配置 CAN 设备。目前它只与
   `NORMAL` 匹配；`SOCCER_BOT` 未标定对应模型前不得运行 PathPlanner 自动。

更换底盘或模块时，必须重新完成 Tuner X 向导和架空验证，并成对替换“新导出的 JSON”与整个
`TunerConstants.java`。生成文件不得局部重排或套用项目格式化；参考
[CTRE Tuner X swerve 生成流程](https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/generating-running-project.html)。

当前 `NORMAL` 占位配置锁定项：

- CAN bus：`CANivore`；Pigeon 2：ID 33。
- Front Left：drive 2、steer 1、CANcoder 3。
- Front Right：drive 5、steer 4、CANcoder 6。
- Back Left：drive 8、steer 7、CANcoder 9。
- Back Right：drive 11、steer 10、CANcoder 12。
- drive ratio `6.746031746031747`，steer ratio `21.428571428571427`，coupling ratio
  `3.5714285714285716`，轮半径 `2.008 in`。
- drive supply/stator 限流 40 A/90 A，steer supply/stator 限流 20 A/60 A，均启用。

编码器偏移、模块反向和模块位置由各 profile 的配置类及自动测试锁定，不在文档中复制第二份可编辑真值。

## 2. 工程职责边界

- `Robot`：只处理生命周期、`CommandScheduler`、DataLog、系统电源遥测和自动命令调度。
- `RobotContainer`：构造 subsystem、设置驾驶绑定、创建自动 chooser。不得放周期机构状态机。
- `Subsystem`：拥有硬件、传感器刷新、控制接口、局部安全状态和本 subsystem 的 NT 发布者。
- `Command`：组织动作时序和 requirements；被中断时必须让输出进入明确的安全状态。
- `Constants`：保存团队维护的常量和唯一底盘 profile 开关。Tuner X 生成文件仍须整体替换，禁止手改。
- `Telemetry`：只发布 swerve typed topics，不拥有底盘控制逻辑。
- `vision/LimelightIO`：只读写 Limelight 厂商表并解析数据；是否接受测量由 `VisionSubsystem` 决定。

禁止恢复 WayiMotor 式“万能电机”封装。新机构直接使用 Phoenix 6 request/config API；如果两处代码确实共享业务语义，
优先提取小型 command factory 或纯函数。

## 3. Java 格式与命名

- 使用 4 个空格缩进，禁止 tab。
- 一般代码保持 100–120 列；长泛型、URL 和 Tuner X 生成代码可例外。
- 类型、record、enum：`PascalCase`。
- 方法、字段、局部变量：`camelCase`。
- 常量：`UPPER_SNAKE_CASE`。
- package 全小写；一个顶层 public 类型一个文件。
- import 不使用项目自建类的通配符；静态单位 import 可在不造成歧义时使用。
- 字段默认 `private`；只在确有跨类 API 时提高可见性。
- 用 `final` 表达不重新赋值的引用，用 `Optional` 表达确实允许不存在的对象，不用 `null` 充当运行模式。
- 提交前运行 `git diff --check`；`.editorconfig` 是编辑器基础约束，不依赖联网格式化插件。

### 3.1 单位必须进入名称

所有裸 `double`/`int` 目标值必须从名称看出单位或比例，例如：

- `positionRotations`、`velocityRotationsPerSecond`；
- `distanceMeters`、`angularVelocityRadiansPerSecond`；
- `latencyMilliseconds`、`timeoutSeconds`；
- `supplyCurrentAmps`、`batteryVoltageVolts`；
- 无单位比例使用 `ratio`、`coefficient`、`dutyCycle`。

优先使用 WPILib units 类型；在 Phoenix、PathPlanner 或 NT API 需要裸数值时，变量名仍必须携带单位。禁止用一个变量在
电机转数、机构转数和角度之间隐式切换。

### 3.2 常量、可调参数和注释

- CAN ID、端口、机械极限、控制增益和阈值集中放置，不在 command 中散落 magic number。
- 每个可调参数旁用一句话说明单位、来源或调参前置条件。
- 示例参数必须写明“按实际机构重新计算”，不得描述为通用安全值。
- 注释说明“为什么、约束和危险”，不重复代码字面含义。
- 临时禁用的危险功能要说明重新启用条件，例如 SysId 必须架空全部车轮。
- 改变单位、齿比、传感器方向或 request 类型时，同一提交更新测试与 NT 数据字典。

## 4. Command-Based 安全规则

- 会操作 subsystem 的 command 必须声明 requirement。
- 持续运行命令从 supplier 读取目标值，避免构造时缓存摇杆或实时参数。
- `end(boolean interrupted)` 必须 Neutral/stop；不能假设下一条 command 会覆盖输出。
- 默认命令只能维持安全、可预测的人工控制。
- `Commands.parallel` 中不得有两个 command 要求同一个 subsystem。
- 自动命令失败、chooser 为空或 AutoBuilder 未配置时必须回退到 `Commands.none()`/`Do Nothing`。
- 危险测试命令不绑定到默认驾驶布局；临时绑定需要显眼的安全注释并在测试后移除。

`ExampleCommand` 是最小模板：使用 `DoubleSupplier`、声明 `ExampleSubsystem` requirement、持续发送
`VelocityVoltage`，并在 `end()` 中调用 `stop()`。

## 5. NetworkTables、Elastic 与 AdvantageScope

团队自有 topic 必须位于 `/FRC8011`。使用 typed publisher 和 WPILib struct topic；禁止用
`SmartDashboard.putNumber/putString/putBoolean` 写标量。

唯一团队例外是 Sendable chooser：`/SmartDashboard/FRC8011/Auto/Chooser`。它用于 Elastic 的 chooser 控件，
不授权在 `/SmartDashboard` 下新增其他团队标量。

Limelight 的 `/limelight-left`、`/limelight-back`、`/limelight-right` 以及 PathPlanner 自有表继续使用厂商命名，
不得搬进 `/FRC8011` 或由团队代码重新发布全部副本。NetworkTables 基础概念见
[WPILib NetworkTables](https://docs.wpilib.org/en/latest/docs/software/networktables/networktables-intro.html)。

### 5.1 团队 topic 数据字典

| 完整路径 | NT 类型 | 发布频率 | 发布者/含义 |
| --- | --- | --- | --- |
| `/FRC8011/Robot/BatteryVoltageV` | double | 10 Hz | `Robot`，roboRIO 电池电压 |
| `/FRC8011/Robot/PDHTotalCurrentA` | double | 10 Hz | `Robot`，REV PDH ID 1 总电流 |
| `/FRC8011/Robot/BrownoutVoltageV` | double | 10 Hz | `Robot`，配置的 7.0 V 阈值 |
| `/FRC8011/Robot/BrownedOut` | boolean | 10 Hz | `Robot`，roboRIO brownout 状态 |
| `/FRC8011/Robot/Mode` | string | 10 Hz | `Robot`，Disabled/Auto/Teleop/Test |
| `/FRC8011/Drive/Pose` | `Pose2d` struct | 约 50 Hz | `Telemetry`，融合后机器人姿态 |
| `/FRC8011/Drive/Speeds` | `ChassisSpeeds` struct | 约 50 Hz | `Telemetry`，机器人相对底盘速度 |
| `/FRC8011/Drive/ModuleStates` | `SwerveModuleState[]` struct | 约 50 Hz | 实测模块状态 |
| `/FRC8011/Drive/ModuleTargets` | `SwerveModuleState[]` struct | 约 50 Hz | 模块控制目标 |
| `/FRC8011/Drive/ModulePositions` | `SwerveModulePosition[]` struct | 约 50 Hz | 模块里程计位置 |
| `/FRC8011/Drive/TimestampS` | double | 约 50 Hz | CTRE 状态时间戳，秒 |
| `/FRC8011/Drive/OdometryFrequencyHz` | double | 约 50 Hz | CTRE 里程计实际频率 |
| `/FRC8011/Vision/<camera>/Accepted` | boolean | robot loop | 本周期测量是否融合 |
| `/FRC8011/Vision/<camera>/RejectReason` | string | robot loop | `None` 或拒绝原因组合 |
| `/FRC8011/Vision/<camera>/TagCount` | integer | robot loop | MegaTag2 标签数量 |
| `/FRC8011/Vision/<camera>/AverageDistanceM` | double | robot loop | 平均标签距离，米 |
| `/FRC8011/Vision/<camera>/LatencyMs` | double | robot loop | Limelight pipeline 延迟，毫秒 |
| `/FRC8011/Vision/<camera>/Pose` | `Pose2d` struct | robot loop | MegaTag2 原始平面姿态 |
| `/FRC8011/Vision/<camera>/FailureCount` | integer | robot loop | 捕获的相机处理异常累计值 |
| `/FRC8011/LED/State` | string | 状态变化时 | 有效 LED 状态或 `SOLID` |
| `/FRC8011/Example/PositionRot` | double | subsystem loop | 示例机构位置，机构转数 |
| `/FRC8011/Example/VelocityRps` | double | subsystem loop | 示例机构速度，机构转/秒 |
| `/FRC8011/Example/SupplyCurrentA` | double | subsystem loop | 示例电机 supply 电流 |
| `/FRC8011/Example/StatorCurrentA` | double | subsystem loop | 示例电机 stator 电流 |
| `/FRC8011/Example/ControlMode` | string | subsystem loop | 当前 Phoenix request 名称 |
| `/FRC8011/Tuning/Example/EnableVelocityControl` | boolean entry | 事件/每周期读取 | 默认 false；为 true 会立即允许运动 |
| `/FRC8011/Tuning/Example/VelocityTargetRps` | double entry | 事件/每周期读取 | 示例机构目标转/秒 |
| `/SmartDashboard/FRC8011/Auto/Chooser` | Sendable chooser | 事件 | 唯一 SmartDashboard 例外 |

`<camera>` 只能是 `limelight-left`、`limelight-back`、`limelight-right`。

`DataLogManager.start()` 与 `DriverStation.startDataLog(..., true)` 在启动时开启 NT4、DriverStation 和 joystick
记录。AdvantageScope 可直接连接 NT4，也可离线打开 `.wpilog`；参见
[WPILib DataLog](https://docs.wpilib.org/en/latest/docs/software/telemetry/datalog.html)。运行日志、`.hoot` 和本地
`simgui-ds.json` 均被 git 忽略。

## 6. Phoenix 6 TalonFX 使用规范

每个 subsystem 直接拥有 `TalonFX` 和需要的 request 对象。request 作为字段复用，不在 `periodic()` 中反复创建。
`ExampleSubsystem` 展示以下选择：

| Request/API | 使用场景 | 本项目示例方法 |
| --- | --- | --- |
| `NeutralOut` | 明确停止输出；command 结束时使用 | `stop()` |
| `setNeutralMode(Brake)` | Neutral 时电机主动抵抗转动 | `setBrakeMode()` |
| `setNeutralMode(Coast)` | Neutral 时允许机构滑行 | `setCoastMode()` |
| `DutyCycleOut` | -1 到 +1 的开环占空比 | `setDutyCycle(...)` |
| `VoltageOut` | 以伏特表达的开环输出、简单测试/SysId 辅助 | `setVoltage(...)` |
| `VelocityVoltage` | 电压闭环速度，使用 Slot0 PID/FF | `setVelocityRotationsPerSecond(...)` |
| `PositionVoltage` | 电压闭环位置，使用 Slot0 | `setPositionRotations(...)` |
| `MotionMagicVoltage` | 受速度/加速度/jerk 约束的位置运动 | `setMotionMagicPositionRotations(...)` |
| `TorqueCurrentFOC` | 以 stator 安培请求扭矩电流 | `setTorqueCurrentAmps(...)` |
| `Follower` | 同 CAN bus 跟随 leader，可同向/反向 | `follow(...)` |

### 6.1 配置检查顺序

1. 确认 CAN bus、CAN ID、固件和电机型号。
2. 确认 `Inverted` 与 Brake/Coast；架空机构验证正方向。
3. 设置 `SensorToMechanismRatio`，并用手动转动验证“机构转数”单位。
4. 根据机械范围设置正反软限位；软限位不是硬件止挡的替代品。
5. 先限制正反 duty cycle，再从低输出开始测试。
6. 设置 supply/stator 电流限制，再按实际负载记录电流和温升。
7. 调整 Slot0 `kS/kV/kA/kG/kP/kI/kD`；不复制其他机构的数值。
8. 最后设置 Motion Magic 速度、加速度和 jerk，并验证急停/中断。

### 6.2 三类电流值不能混用

- Supply current 是电池/断路器侧电流，主要保护供电和总功耗。
- Stator current 是电机相电流，直接关联扭矩和电机热负载。
- `TorqueCurrentFOC` 的 output 与 `PeakForward/ReverseTorqueCurrent` 使用 stator 安培语义。

电流限制要结合断路器、线径、散热、传动和允许堵转时间计算。roboRIO 不会替代 TalonFX 完成每台电机的电流保护。
FOC request 还取决于设备支持与 CTRE 授权；未确认前优先使用 Voltage request。`NeutralOut` 与 Brake/Coast 是两个维度：
前者发送当前控制请求，后者决定 Neutral 时的物理行为。

## 7. roboRIO 电源与 brownout

- brownout 阈值固定为 7.0 V，并发布到 `/FRC8011/Robot/BrownoutVoltageV`。
- REV PDH 使用 rio CAN ID 1；基础工程发布电池电压和 PDH 总电流。
- brownout 时 LED fault 红色优先于正常模式颜色；恢复后 fault latch 由 `Robot` 清除。
- 新机构必须同时配置电机控制器限流，不能只观察 PDH 总电流。
- 真机测试记录静态电压、最大总电流、最低电压、brownout 状态和每个关键电机的 supply/stator 电流。

## 8. Swerve、PathPlanner 和 SysId

PathPlanner 当前只对应 `NORMAL`，使用非 FOC `krakenX60` 模型、40 A supply limit、与 `TunerConstants`
一致的轮半径/drive ratio 和模块位置。下列模型值保留自当前基线，但都必须通过真机加速度、滑移和轨迹误差日志复测：

- 质量 55 kg；转动惯量 6.883 kg·m²；
- 最高速度 4.1 m/s；轮胎 COF 1.2；
- robot width/length 0.86 m。

PathPlanner 配置含义见
[PathPlanner Robot Config](https://pathplanner.dev/robot-config.html)。`driveCurrentLimit` 是供给侧电流限制，
不得填入 stator 限流值。

自动目录默认只保留 `Do Nothing.auto`。`AutoBuilder` 配置或 chooser 创建失败时必须向 DriverStation 报错并使用
`Commands.none()`。不得在基础分支注册赛季 `NamedCommands` 或恢复自制动态路径工厂。若以后选用 Choreo，应先记录
坐标系、镜像和 feedforward 的架构决定，再替换自动入口；不要让 PathPlanner 与 Choreo 同时拥有同一自动控制链。

底盘保留 translation、steer、rotation 三套 SysIdRoutine。`configureSysIdBindings()` 默认不调用。启用流程：

1. 固定机器人，架空全部车轮，清空周围人员与物品；
2. 一次只绑定并核对一种 routine；
3. 先 quasistatic、后 dynamic，从正确方向开始；
4. 现场人员保持急停权限并观察电流、方向和机械干涉；
5. 完成后立即注释绑定，再分析 `.hoot`/SysId 日志。

## 9. Limelight MegaTag2

- 相机名固定为 `limelight-left`、`limelight-back`、`limelight-right`。
- 每周期向每台相机写入 Pigeon yaw，并读取 `botpose_orb_wpiblue`。
- 不读取 MegaTag1，不混合视觉航向；只融合 MegaTag2 平移。
- heading 标准差固定为极大值 `10_000_000 rad`，防止视觉改变 gyro 航向。
- 无标签、角速度超过 100 deg/s、平均距离小于 0.26 m 或大于 4.50 m 时拒绝。
- 平移标准差为 `0.01 * distanceMeters^1.2`；阈值和模型均需按真机日志复测。
- 异常必须增加 `FailureCount` 并节流报告 DriverStation，禁止空 catch。

调试时同时查看 Accepted、RejectReason、TagCount、AverageDistanceM、LatencyMs、Pose 和时间戳。先确认相机安装姿态、
AprilTag 地图与 Pigeon 方向，再调整噪声参数。

## 10. CANdle LED

CANdle 位于 rio CAN ID 26。状态和颜色：

- `OFF` 黑；`DISABLED` 蓝；`TELEOP` 绿；
- `AUTONOMOUS` 紫；`TEST` 黄；`FAULT` 红。

Fault 优先级最高。`setState(...)` 设置正常模式，`setFault(...)` 控制 fault，`setSolid(...)` 供临时调试，`off()`
关闭；对应 command factory 只做一次状态变更。NT 状态只在变化时发布，禁止每周期重新配置 CANdle 动画或重复发送同色。

## 11. ExampleSubsystem 启用流程

示例默认关闭，`ENABLE_EXAMPLE_SUBSYSTEM=false` 时 `RobotContainer` 不构造 TalonFX，因此不会因占位 CAN ID 产生离线告警。
需要复制或试用时：

1. 在 `Constants.ExampleConstants` 填入真实 CAN bus 和唯一 CAN ID；
2. 重新计算 `createMotorConfiguration()` 中所有 PID/FF、齿比、软限位、输出和电流值；
3. 机械架空后把 `ENABLE_EXAMPLE_SUBSYSTEM` 改为 true；
4. 在 `RobotContainer` 中通过 `getExampleSubsystem().ifPresent(...)` 添加明确的临时绑定；
5. 如使用 `ExampleCommand`，传入单位为机构 rotations/s 的 `DoubleSupplier`；
6. 先验证 Neutral、Brake/Coast、正方向和低占空比，再测试闭环；
7. `/FRC8011/Tuning/Example/EnableVelocityControl` 默认 false。改为 true 会立即按 NT 目标运动；
8. 完成教学/调试后移除绑定并恢复默认关闭。

不要把示例数值复制到正式机构后直接上电。正式 subsystem 应使用自己的名字、常量、NT 路径和测试。

## 12. 构建、测试与交付检查

GradleRIO 2026 使用 WPILib JDK 17。本机 Windows 命令：

```powershell
$env:JAVA_HOME = 'C:\Users\Public\wpilib\2026\jdk'
.\gradlew.bat clean test build --no-daemon
git diff --check
```

系统默认 JDK 26 会与当前 GradleRIO 不兼容。基础测试必须覆盖 TunerConstants、PathPlanner、Limelight 解析和拒绝逻辑、
LED fault 优先级、Example 配置以及 NT namespace。

真机合并前架空检查：

- CANivore 正常，Pigeon 2 ID 33，四模块 ID/方向/零位正确；
- 三台 Limelight 名称、安装姿态、时间戳和 MegaTag2 接受/拒绝符合预期；
- rio CANdle 26 的六种状态和 fault 优先级正确；
- PathPlanner 起始朝向、红蓝镜像、Do Nothing 回退和模块 feedforward 正确；
- 7.0 V brownout、PDH ID 1 电压/总电流、DataLog 和 joystick 日志可在 AdvantageScope 中读取；
- Example TalonFX 保持未构造，除非本次测试明确完成了启用清单。
