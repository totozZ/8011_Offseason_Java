# FRC8011 2026 Template

本模板以 CTRE Phoenix6 Swerve + PathPlanner 为底盘基础，保留 Vision 子系统与 ComplexCommand 框架，作为 2026 赛季的起始代码。

## 0. 项目概览
- 入口：`src/main/cpp/Robot.cpp`
- 子系统装配与按键绑定：`src/main/cpp/RobotContainer.cpp`
- Swerve 底盘实现：`src/main/include/subsystems/CommandSwerveDrivetrain.h`
- 视觉与定位：`src/main/include/subsystems/VisionSubsystem.h`
- 复杂流程编排：`src/main/include/commands/ComplexCommand.h`
- PathPlanner 数据：`src/main/deploy/pathplanner/`

## 1. 使用 Tuner X 校准底盘并生成 constants
1) 打开 CTRE Tuner X，完成底盘硬件校准与配置。
2) 导出/生成的内容应同步到本仓库：
   - `tuner-project.json`（根目录）
   - `src/main/include/generated/TunerConstants.h`
   - `src/main/cpp/generated/TunerConstants.cpp`
3) 生成文件为 CTRE 输出，**不要手动改**。若底盘硬件变化，应重新生成。

## 2. 按 ExampleSubsystem 创建新子系统（Wayimotor）
参考文档：`docs/ExampleSubsystem.md`

推荐流程：
1) 新建子系统头文件与实现文件：
   - `src/main/include/subsystems/<SubsystemName>.h`
   - `src/main/cpp/subsystems/<SubsystemName>.cpp`
2) 常量写入 `src/main/include/Constants.h` 对应命名空间。
3) 使用 Wayimotor 封装（参考 `src/main/include/frc8011/Wayimotor.h`）：
   - 配置 CANBus、ID、减速比、控制模式等
   - 通过 ExampleSubsystem 的写法完成初始化与周期更新

## 3. Command + ComplexCommand + 手柄绑定
建议路径：
1) 子系统提供最小可复用动作（`frc2::CommandPtr` 工厂函数）。
2) 在 `ComplexCommand` 中编排多子系统组合动作。
3) 在 `RobotContainer::ConfigureBindings()` 中绑定手柄。

相关文件：
- `src/main/include/commands/ComplexCommand.h`
- `src/main/cpp/commands/ComplexCommand.cpp`
- `src/main/cpp/RobotContainer.cpp`

## 4. Limelight 使用与全场定位
视觉核心文件：
- `src/main/include/subsystems/VisionSubsystem.h`
- `src/main/cpp/subsystems/VisionSubsystem.cpp`
- `src/main/include/LimelightHelpers.h`

关键步骤：
1) 在 Limelight UI 中完成标定与相机外参设置。
2) 在代码中设置 Limelight 名称与 IMU 模式：
   - 例如 `limelight_left_name_ = "limelight-left"`
3) VisionSubsystem 使用 MegaTag2 数据更新位姿：
   - `LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2(...)`
4) 根据距离/角速度进行置信度筛选，避免高速旋转导致错误融合。

## 5. PathPlanner 路径生成与 Auto 调用
路径文件目录：`src/main/deploy/pathplanner/`
- Autos: `autos/*.auto`
- Paths: `paths/*.path`
- Navgrid: `navgrid.json`

Auto 选择器：
- `RobotContainer` 中：`AutoBuilder::buildAutoChooser("offseason_right")`
- 选择入口：**Glass / SmartDashboard** 中的 `Auto Mode`

说明：
- 若 Auto 中包含事件标记，需要在代码里注册对应命名指令。
- 本模板已移除旧的 Autocommand 状态机，优先使用 PathPlanner 原生 auto。

## 6. 仿真与可视化
- 安装仿真依赖
- 仿真入口：`gradlew.bat simulate`,template可实现auto中对生成路径的跟踪3d仿真
- AdvantageScope 连接 NT4 观察位姿：
  - `DriveState/Pose`

## 7. 在线资料与教程（官方推荐）
- CTRE Phoenix6 / Tuner X：`https://v6.docs.ctr-electronics.com/`
- WPILib 官方文档：`https://docs.wpilib.org/`
- PathPlanner：`https://pathplanner.dev/`
- Limelight：`https://docs.limelightvision.io/`
- AdvantageScope：`https://www.advantagescope.org/`

> 说明：以上链接为官方参考入口，具体使用方式请结合本模板代码结构学习。
