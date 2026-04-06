// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <cmath>
#include <algorithm>

#include "Constants.h"
#include "frc8011/Wayimotor.h"
#include "subsystems/ExampleSubsystem.h"

namespace subsystems {
class FeederSubsystem;
class CommandSwerveDrivetrain;

class ShooterSubsystem : public ExampleSubsystem {
 public:
  ShooterSubsystem(frc2::CommandXboxController& joystick_)
      : ExampleSubsystem(joystick_) {
    Initialization();
  }

  void Periodic() override;
  void SetFeederSubsystem(FeederSubsystem* feeder_subsystem);
  void SetDrivetrainSubsystem(CommandSwerveDrivetrain* drivetrain_subsystem);

  void Stop();
  frc2::CommandPtr StopCommandPtr();

  // --- 旧代码保留的外部接口 ---
  void SetAngleOffset(double angleInDeg) {
    angleOffsetFromDrive = angleInDeg;
    frc::SmartDashboard::PutNumber("angleOffsetFromDrive", angleOffsetFromDrive);
  }

  void SetSpeedOffset(double speed) {
    velOffsetFromDrive = speed;
    frc::SmartDashboard::PutNumber("velOffsetFromDrive", velOffsetFromDrive);
  }

  void getFinalVel();
  double CalculateShooterSpeedRegression(double dis);
  double CalculatePitchAngleAboveHub(double dis);

  frc2::CommandPtr EnableShooter() {
    return frc2::cmd::RunOnce([this] { shooting = true; });
  }
  frc2::CommandPtr DisableShooter() {
    return frc2::cmd::RunOnce([this] { shooting = false; });
  }
  void DisableShooterNonCmd() { shooting = false; }

  void setPassTarget(frc::Translation2d target) { passTarget = target; }
  void startPassing() { /* isPassing = true; 保留旧代码结构 */ }
  void stopPassing() { /* isPassing = false; */ }
  bool isPassingEnabled() { return isPassing; }
  // SysId 方法
  frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction) {
    return m_sysIdRoutine.Quasistatic(direction);
  }
  frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction) {
    return m_sysIdRoutine.Dynamic(direction);
  }
  frc2::CommandPtr DisableDefaultShoot() {
    return frc2::cmd::RunOnce([this] { onlyDefaultShoot = false; });
  }
  frc2::CommandPtr EnableDefaultShoot() {
    return frc2::cmd::RunOnce([this] { onlyDefaultShoot = true; });
  }

  // --- 全局状态变量 ---
  double vel = 0;
  double Tangle = 0; 
  double angle = 0;
  double realShootVelocity = 0; 
  double GetShootVelocity();
 private:
  void Initialization();
  frc2::sysid::SysIdRoutine m_sysIdRoutine{
      frc2::sysid::Config{std::nullopt,  // 默认斜坡率 (1 V/s)
                          4_V,           // 动态电压
                          std::nullopt,  // 默认超时 (10 s)
                          nullptr},
      frc2::sysid::Mechanism{
          [this](units::volt_t output) { shooter_right_up_.setVoltage(output); },
          [this](frc::sysid::SysIdRoutineLog* log) {
            log->Motor("shooter")
                .voltage(shooter_right_up_.Getmotor().GetMotorVoltage().GetValue())
                .position(shooter_right_down_.Getmotor().GetPosition().GetValue())
                .velocity(shooter_left_up_.Getmotor().GetVelocity().GetValue());
          },
          this}};
  // --- 新代码的新马达配置 (4个飞轮电机 + 1个角度电机) ---
  Wayimotor shooter_left_down_{ShooterConstants::ShooterLeftDownMotorID, kCANBus};
  Wayimotor shooter_left_up_{ShooterConstants::ShooterLeftUpMotorID, kCANBus};
  Wayimotor shooter_right_up_{ShooterConstants::ShooterRightUpMotorID, kCANBus}; // 主控电机
  Wayimotor shooter_right_down_{ShooterConstants::ShooterRightDownMotorID, kCANBus};
  Wayimotor shooter_pitch_{ShooterConstants::ShooterPitchMotorID, kCANBus};

  FeederSubsystem* feeder_sub_ = nullptr;
  CommandSwerveDrivetrain* drivetrain_sub_ = nullptr;

  // --- 旧代码的控制变量 ---
  bool isPassing = false;
  frc::Translation2d passTarget;
  double angleOffsetFromDrive = 0;
  double velOffsetFromDrive = 0;
  bool shooting = false;
  bool onlyDefaultShoot = false;

  // --- 新代码的 Pitch 归零逻辑 ---
  void ShootPitch_Reset();
  void CalculateShooterPitch();
  //void SetShootPitchNormPosition(double norm);
  bool shooter_pitch_reset_flag_ = false;
  int shooter_pitch_reset_counter_ = 0;
  void SetShootPitchNormPosition(double norm);
  void SetShootPitchAngle(double target_angle);

  // --- 旧代码的物理几何常量 ---
  double ball_error1 = 9.1; 
  double ball_error2 = 9.1; 
  double hoodradis = 177.6; 
  double ballradis = 75; 
  double flywheelradis = 50; 
  double max_hood_angle = 18.58; 
  double min_hood_angle = 51.03;
  double thetaMiddleLine = 0; // 在 Initialization 中计算
};

#ifndef M_PI 
#define M_PI 3.14159265358979323846
#endif

#define shooter_height_approx 0.46932
#define shooter_max_composite 13.5

#define shooter_vel_quadratic_regression_a 2.8
#define shooter_vel_quadratic_regression_b 25.0
#define shooter_vel_quadratic_regression_c 2.2
#define shooter_vel_quadratic_regression_d 26.5
// #define shooter_vel_quadratic_regression_e 6.4
// #define shooter_vel_quadratic_regression_f 32.6



#define pass_m 5
#define pass_b 8

}  // namespace subsystems