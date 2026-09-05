# FRC 8011 BabyAuto Classroom Branch

这是面向 90 分钟自动程序活动的 WPILib 2026 Java Command-Based 工程。运行时只启用：

- CTRE Phoenix 6 Swerve，以及可一行切换的普通底盘/SoccerBot 两套配置；
- CAN 20 TalonFX intake 与 CAN 21 TalonFX shooter；
- BabyAuto 限速命令接口、`Do Nothing` / `Baby Auto` chooser；
- 两台车统一 RobotCentric 手动驾驶、系统电源遥测和 DataLog。

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

- 使用 Phoenix Tuner X 检查两台 TalonFX 固件和 rio CAN 接线：intake 为 ID 20，shooter 为 ID 21。
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

## 两台车的驾驶与 BabyAuto

两套 profile 都固定使用 **RobotCentric（机器人坐标系）**：左摇杆前推始终向车头走，左推向车身左侧走，
右摇杆控制转弯；X 保持车轮交叉锁定。红蓝联盟和 Pigeon 航向都不改变方向，Start 不再重置航向。
只需选择对应的 `ACTIVE_PROFILE`，无需根据是否安装 Pigeon 切换驾驶模式。

手动机构控制使用 Xbox 扳机：按住左扳机 Intake，按住右扳机 Shoot，松开立即停止。两者同时按下时 Shoot
优先；松开右扳机但继续按住左扳机时会恢复 Intake。

手动驾驶和 BabyAuto 都使用驱动电机速度闭环、转向位置闭环；Pigeon 不参与这些请求的轮速/转角计算。
CTRE 的底盘类仍会创建 Pigeon 对象，未安装时可能有设备离线/里程计采集告警；这不等于需要安装第二个 Pigeon。
无 Pigeon 时不要使用 Dashboard 的 Pose/航向判断实际位置。本分支不配置 PathPlanner AutoBuilder，也不使用航向保持。

两台车使用相同的摇杆响应、平移/旋转死区和物理速度上限。当前最大平移速度为 `4.59 × 0.75 = 3.4425 m/s`，
最大转速为 `1.9π × 0.95 ≈ 5.67 rad/s`，斜推摇杆不会让平移合速度额外增加。各车的齿比、编码器偏移、反转、
电流限制及 PID/前馈仍保留各自标定值，不能为统一手感而照抄另一台的硬件参数。
大幅平移与转弯叠加时，轮速饱和会受底盘尺寸影响；不同重量、轮胎、电池与调参也会造成实际响应差异。

BabyAuto 写法不变，例如 `robot.drive().setVx(0.6).forSeconds(2.0)`：以车头方向的 0.6 m/s 目标速度走 2 秒。
`setRotation(45).forSeconds(2)` 是以 45°/s 的目标转速转 2 秒，不是陀螺仪闭环转到 90°。
边走边转时方向跟随车身旋转，轨迹可能是弧线。平移 1 m/s、转速 90°/s、总时长 15 秒的保护继续生效。
自动等待及 `Do Nothing` 不接受手柄驾驶输入，动作结束或中断会停止输出。

两车验收：分别使用对应 profile，在安装与未安装 Pigeon（以及中途失去 Pigeon 数据）条件下检查前进、横移、
旋转、X 锁定和 Disable 停止；转动车头后再次前推，仍应沿新车头方向前进。
随后在相同地面、电池状态下运行同一段低速 BabyAuto，比较距离和转角。定时自动只能尽量接近，不能保证绝对距离/角度；
若差异明显，检查各车轮径、齿比和速度闭环标定，实测通过后再调整自动动作时间。
