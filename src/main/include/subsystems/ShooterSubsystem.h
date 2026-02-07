// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"
#include "frc8011/LinearServo.h"
#include "frc8011/Wayimotor.h"
#include "rev/ServoChannel.h"
#include "rev/ServoHub.h"
#include "subsystems/ExampleSubsystem.h"

namespace subsystems
{

class ShooterSubsystem : public ExampleSubsystem
{
public:
  ShooterSubsystem(frc2::CommandXboxController& joystick_) : ExampleSubsystem(joystick_)
  {
    Initialization();
  }
  void Periodic() override;

  frc2::CommandPtr SetShootVelocityCommandPtr(double velocity);
  void SetShootVelocity(double velocity);
  double GetShootVelocity();
  void Stop();

  void SetLinearServoLeftPositionMm(double position_mm);
  void SetLinearServoRightPositionMm(double position_mm);

  // SysId 方法
  frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction)
  {
    return m_sysIdRoutine.Quasistatic(direction);
  }
  frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction)
  {
    return m_sysIdRoutine.Dynamic(direction);
  }

private:
  void Initialization();

  static constexpr int kServoHubCanId = 3;
  static constexpr auto kLinearServoLeftChannel =
      rev::servohub::ServoChannel::ChannelId::kChannelId0;
  static constexpr auto kLinearServoRightChannel =
      rev::servohub::ServoChannel::ChannelId::kChannelId3;
  static constexpr double kLinearServoLengthMm = 129.0;
  static constexpr double kLinearServoMaxPositionMm = (LinearServoConstants::MaxPositionMm < kLinearServoLengthMm) ?
                                                          LinearServoConstants::MaxPositionMm :
                                                          kLinearServoLengthMm;
  static constexpr double kLinearServoSpeedMmPerS = 10.0;
  static constexpr double kLinearServoInitialPositionMm = 100.0;

  rev::servohub::ServoHub servo_hub_{ kServoHubCanId };
  LinearServo linear_servo_left_{ servo_hub_, kLinearServoLeftChannel, kLinearServoLengthMm,
                                  kLinearServoSpeedMmPerS };
  LinearServo linear_servo_right_{ servo_hub_, kLinearServoRightChannel, kLinearServoLengthMm,
                                   kLinearServoSpeedMmPerS };
  double linear_servo_left_target_mm_ = kLinearServoInitialPositionMm;
  double linear_servo_right_target_mm_ = kLinearServoInitialPositionMm;
  double last_servo_update_s_ = 0.0;

  Wayimotor shooter_left_front_{ ShooterConstants::ShooterLeftFrontMotorID, kCANBus };  // 发射左电机
  Wayimotor shooter_left_back_{ ShooterConstants::ShooterLeftBackMotorID, kCANBus };    // 发射左电机
  Wayimotor shooter_right_{ ShooterConstants::ShooterRightMotorID, kCANBus };           // 发射右电机

  // SysId routine for shooter
  frc2::sysid::SysIdRoutine m_sysIdRoutine{
      frc2::sysid::Config{
          std::nullopt,  // 默认斜坡率 (1 V/s)
          4_V,           // 动态电压
          std::nullopt,  // 默认超时 (10 s)
          nullptr },
      frc2::sysid::Mechanism{
          [this](units::volt_t output) { shooter_right_.setVoltage(output); },
          [this](frc::sysid::SysIdRoutineLog* log) {
            log->Motor("shooter")
                .voltage(shooter_right_.Getmotor().GetMotorVoltage().GetValue())
                .position(shooter_right_.Getmotor().GetPosition().GetValue())
                .velocity(shooter_right_.Getmotor().GetVelocity().GetValue());
          },
          this }
  };
};

}  // namespace subsystems
