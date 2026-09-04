# FRC 8011 BabyAuto Classroom Branch

这是面向 90 分钟自动程序活动的 WPILib 2026 Java Command-Based 工程。运行时只启用：

- CTRE Phoenix 6 Swerve，以及可一行切换的普通底盘/SoccerBot 两套配置；
- 官方 2026 KitBot 两电机吸球/射球机构；
- BabyAuto 限速命令接口、`Do Nothing` / `Baby Auto` chooser；
- Swerve 手动驾驶、系统电源遥测和 DataLog。

学生只修改
[`src/main/java/frc/robot/auto/Auto.java`](src/main/java/frc/robot/auto/Auto.java)。坐标系、单位、
`sequence` / `parallel`、电机 `.speed()` / `.stop()`、完整示例和真机安全提醒都写在该文件中。

## 构建

GradleRIO 2026 必须使用 WPILib 自带 JDK 17：

```powershell
$env:JAVA_HOME = 'C:\Users\Public\wpilib\2026\jdk'
.\gradlew.bat clean test build --no-daemon
```

## 教师准备

- 使用 REV Hardware Client 更新两台 SPARK MAX 固件，并设置 intake/launcher 为 rio CAN ID 5、feeder 为
  rio CAN ID 6；两台控制器连接有刷电机。
- 底盘开关只改
  [`Constants.DriveConstants.ACTIVE_PROFILE`](src/main/java/frc/robot/Constants.java)：`NORMAL` 使用当前
  `TunerConstants.java`，`SOCCER_BOT` 使用从 `C:\Users\95833\Desktop\sum` 迁入的参数。
- 普通底盘重新完成 Tuner X 标定后，完整替换
  [`TunerConstants.java`](src/main/java/frc/robot/generated/TunerConstants.java)，不要局部复制参数。选择层不需要改。
- SoccerBot 配置保留了左前驱动电机的单独反转。每次切换后都必须重新构建、部署，并架空验证四个模块。
- `Do Nothing` 始终是默认自动；部署后必须在 Dashboard 明确选择 `Baby Auto`。
- 第一次运行必须架空车轮和机构，验证方向与停止行为后，再到清空的场地进行低速测试。

Vision、CANdle、ExampleSubsystem 和 PathPlanner 自动不会在本分支的运行入口中构造或显示；其基础代码仍保留，
便于以后从 `java-base` 对照或复用。
