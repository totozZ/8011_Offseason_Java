# FRC 8011 Java Foundation

这是 FRC 8011 的 2026 Java Command-Based 基础工程。它只保留可复用的底层能力：

- CTRE Phoenix 6 swerve 与当前现役底盘参数；
- 三台 Limelight 的 MegaTag2 平移融合；
- PathPlanner 自动与安全的 `Do Nothing` 默认项；
- rio CANdle 状态灯；
- 默认关闭的 `ExampleSubsystem` / `ExampleCommand` Phoenix 6 教学样例；
- NT4 typed topics、Elastic、AdvantageScope 和 WPILib DataLog。

## 构建

GradleRIO 2026 必须使用 WPILib 自带 JDK 17。Windows PowerShell 示例：

```powershell
$env:JAVA_HOME = 'C:\Users\Public\wpilib\2026\jdk'
.\gradlew.bat clean test build --no-daemon
```

系统默认 JDK 26 与当前 GradleRIO 不兼容。不要用“能启动 Gradle”代替完整的 `clean test build`。

## 使用前必读

- 团队代码规范、NT4 数据字典和硬件流程见
  [docs/JAVA_CODE_STANDARD.zh-CN.md](docs/JAVA_CODE_STANDARD.zh-CN.md)。
- 运行时底盘硬件真值是
  [TunerConstants.java](src/main/java/frc/robot/generated/TunerConstants.java)。
- [PathPlanner settings](src/main/deploy/pathplanner/settings.json) 只描述自动轨迹物理模型，其中质量、转动惯量、
  最高速度和摩擦系数仍需真机复测。
- [归档 Tuner JSON](docs/archive/tuner-project.DO_NOT_USE.json) 与现役底盘不一致，禁止用它重新生成代码。
- `ExampleSubsystem` 默认关闭。启用前必须填写真实 CAN bus/ID，并重新计算齿比、软限位、PID/前馈和电流限制。
- SysId 绑定默认不启用；只有固定机器人并架空全部车轮后才可临时打开。

部署或合并到比赛代码前，按规范中的真机清单检查 CANivore、Pigeon 2、四个模块、三台 Limelight、
CANdle、PathPlanner 朝向和 7.0 V brownout 遥测。
