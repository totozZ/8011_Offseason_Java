# Example Subsystem 示例子系统开发文档

本文件用于规范 FRC（WPILib C++）项目中自定义子系统的创建流程，便于复用与迭代。

## 目标与范围
- 适用：FRC WPILib C++ Command-Based 结构（含 CTRE、PathPlanner 等外部库）。
- 目标：形成标准化的子系统开发步骤与最小可复用模板。

## 基本目录约定
- 头文件：`src/main/include/subsystems/<SubsystemName>.h`
- 实现文件：`src/main/cpp/subsystems/<SubsystemName>.cpp`
- 命令：`src/main/include/commands/` 与 `src/main/cpp/commands/`

## 标准化创建流程
1. 需求与模型
   - 明确子系统职责与边界（只负责一个硬件/功能域）。
   - 建立 Y Model（系统模型/控制模型）：用于估算、控制参数设计、仿真或 SysId。
2. 常量与参数
   - 在 `Constants.h` 中建立独立命名空间，避免混杂赛季特有数据。
   - 参数包含：电机 ID、传感器 ID、控制参数（PID/FF）、安全极限等。
3. 子系统头文件
   - 继承 `frc2::SubsystemBase`。
   - 声明硬件成员、状态缓存、公开 API、Command 工厂函数（`frc2::CommandPtr`）。
4. 子系统实现
   - 构造函数中完成硬件初始化与配置（如中立模式、电流限制、传感器校准）。
   - `Periodic()` 更新状态与遥测。
5. 基础 Command
   - 为常用动作提供最小命令（例如 `RunOnce`、`StartEnd`、`Run`）。
   - 提供“安全停止”或“复位”命令，便于复用与自动化。
6. RobotContainer 接线
   - 在 `RobotContainer` 中实例化子系统并设置默认命令。
   - 绑定手柄或触发条件到 Command。
7. 验证与仿真
   - 如需仿真/标定，提供 SysId 或基础模拟逻辑。

## 最小模板示例

### 子系统头文件
```cpp
#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>

class ExampleSubsystem : public frc2::SubsystemBase {
 public:
  ExampleSubsystem();
  void Periodic() override;

  frc2::CommandPtr StopCommand();
  frc2::CommandPtr RunCommand();

 private:
  // TODO: hardware handles, state caches
};
```

### 子系统实现
```cpp
#include "subsystems/ExampleSubsystem.h"

ExampleSubsystem::ExampleSubsystem() {
  // TODO: hardware init and configuration
}

void ExampleSubsystem::Periodic() {
  // TODO: telemetry and state updates
}

frc2::CommandPtr ExampleSubsystem::StopCommand() {
  return frc2::cmd::RunOnce([this] {
    // TODO: stop hardware
  });
}

frc2::CommandPtr ExampleSubsystem::RunCommand() {
  return frc2::cmd::Run([this] {
    // TODO: run control loop
  });
}
```

### RobotContainer 接线
```cpp
ExampleSubsystem exampleSub;

exampleSub.SetDefaultCommand(exampleSub.RunCommand());
joystick.A().OnTrue(exampleSub.StopCommand());
```

## 关键注意事项
- 子系统只暴露“可复用的动作接口”，避免业务逻辑堆在 `RobotContainer`。
- 关键参数必须集中配置并可追溯（常量/模型/校准数据）。
- 任何赛季特有逻辑应隔离，便于模板化与复用。
