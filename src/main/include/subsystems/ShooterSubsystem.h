// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"
#include "frc8011/LinearServo.h"
#include "frc8011/Wayimotor.h"
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

private:
  void Initialization();

  static constexpr int kLinearServoLeftPwm = 5;
  static constexpr int kLinearServoRightPwm = 8;
  static constexpr double kLinearServoLengthMm = 129.0;
  static constexpr double kLinearServoMaxPositionMm = (LinearServoConstants::MaxPositionMm < kLinearServoLengthMm) ?
                                                          LinearServoConstants::MaxPositionMm :
                                                          kLinearServoLengthMm;
  static constexpr double kLinearServoSpeedMmPerS = 10.0;

  LinearServo linear_servo_left_{ kLinearServoLeftPwm, kLinearServoLengthMm, kLinearServoSpeedMmPerS };
  LinearServo linear_servo_right_{ kLinearServoRightPwm, kLinearServoLengthMm, kLinearServoSpeedMmPerS };
  double linear_servo_left_target_mm_ = 0.0;
  double linear_servo_right_target_mm_ = 0.0;
  double last_servo_update_s_ = 0.0;

  Wayimotor shooter_left_front_{ ShooterConstants::ShooterLeftFrontMotorID, kCANBus };  // 发射左电机
  Wayimotor shooter_left_back_{ ShooterConstants::ShooterLeftBackMotorID, kCANBus };    // 发射左电机
  Wayimotor shooter_right_{ ShooterConstants::ShooterRightMotorID, kCANBus };           // 发射右电机
};

}  // namespace subsystems
