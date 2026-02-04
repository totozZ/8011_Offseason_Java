// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <memory>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/geometry/Pose2d.h>
#include "Constants.h"
#include "frc8011/AprilTags.h"
#include "subsystems/CommandSwerveDrivetrain.h"
#include "LimelightHelpers.h"
#include "frc8011/LEDSubsystem.h"
#include "frc8011/LinearServo.h"
namespace subsystems {
    class CommandSwerveDrivetrain;
    class GPDetection;
} // 前向声明
namespace frc2 {
    class CommandXboxController;
}
namespace nt {
    class NetworkTable;
}

namespace subsystems{

class VisionSubsystem : public frc2::SubsystemBase {
 public:

    explicit VisionSubsystem(CommandSwerveDrivetrain* drivetrain_,
                             LEDSubsystem* ledsub_,
                             GPDetection* gpdetection_,
                             frc2::CommandXboxController* joystick_);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void SetLinearServoLeftPositionMm(double position_mm);
  void SetLinearServoRightPositionMm(double position_mm);
  void UpdateLinearServoPositions();
  double GetLinearServoLeftPositionMm() const;
  double GetLinearServoRightPositionMm() const;
  bool IsLinearServoLeftAtTarget() const;
  bool IsLinearServoRightAtTarget() const;



 private:
  static constexpr int kLinearServoLeftPwm = 5;
  static constexpr int kLinearServoRightPwm = 8;
  static constexpr double kLinearServoLengthMm = 140.0;
  static constexpr double kLinearServoSpeedMmPerS = 30.0;    // update to actuator spec
  LinearServo linear_servo_left_{kLinearServoLeftPwm, kLinearServoLengthMm,
                                 kLinearServoSpeedMmPerS};
  LinearServo linear_servo_right_{kLinearServoRightPwm, kLinearServoLengthMm,
                                  kLinearServoSpeedMmPerS};
  double linear_servo_left_target_mm_ = 0.0;
  double linear_servo_right_target_mm_ = 0.0;
  double last_servo_update_s_ = 0.0;

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // 依赖的子系统
    CommandSwerveDrivetrain* drivetrain_;
    LEDSubsystem* ledsub_;
    GPDetection* gpdetection_;
    frc2::CommandXboxController* joystick_;

  // 当前机器人角速度
  double currentAngularVelocity_ = 0.0;

  // Limelight 位姿数据
  LimelightHelpers::PoseEstimate mt1_left_pose_;
  LimelightHelpers::PoseEstimate mt2_left_pose_;

  int vision_mode_ = 1; // 0视觉不更新，1更新mt2, 2更新混合
  double switch_distance_ = 1.3; // 1米，距离阈值，低于此距离使用混合模式

   // IMU 模式
    enum class LimelightIMUMode {
        ExternalIMU = 0,
        SeedingMode = 1,
        InternalIMU = 2,
        FusedIMU = 4
    };

  LimelightIMUMode currentIMUMode = LimelightIMUMode::ExternalIMU;
  
  std::string limelight_left_name_ = "limelight-front"; // Limelight 的名称
    
  // 信任度阈值常量
  const double kMaxAngularVelocity_ = 360.0; // 超过 360度/秒 则不信任视觉

  /**
   * @brief 设置limelight工作模式，一共有四种，具体见LimelightIMUMode枚举
   * 
   */
  void SetLimelightIMUMode(std::string, LimelightIMUMode mode);

  /**
   * @brief 判断mt1和mt2传来的poseestimate是否应该被拒绝，检查是否有tag、tag距离是、机器人是否高速旋转、模糊度
   * 
   */
  bool ShouldRejectMetatagPose(const LimelightHelpers::PoseEstimate &poseEstimate);

  /**
   * @brief 根据距离更新视觉模式，使用mt2还是使用混合mt
   * 
   */
  void UpdateVisionMode();

  // 全场定位相关变量和方法
  /**
   * @brief 主处理函数，读取 Limelight 数据并更新里程计
   */
  void LimelightMeasurement();

  /**
   * @brief 更新当前的角速度，用于判断是否处于快速旋转中
   */
  void UpdateAngularVelocity();
  std::shared_ptr<nt::NetworkTable> vision_table_;

  void Test(LimelightHelpers::PoseEstimate mt1, frc::Pose2d mt2);


  public:
    bool disable_mix = 0;

    void LED_control();
};
}
