// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterSubsystem.h"

#include <algorithm>

#include <frc/Timer.h>

using namespace subsystems;

void ShooterSubsystem::Initialization()
{
  configs::TalonFXConfiguration shooter_left_front_config{};
  shooter_left_front_config.MotorOutput.Inverted = 0;
  /* slot0 PID槽*/
  configs::Slot0Configs& shooter_left_front_slot0 = shooter_left_front_config.Slot0;
  shooter_left_front_slot0.kG = 0.;          // Gear ratio of 1:2, 0.5 rotations per rotor rotation
  shooter_left_front_slot0.kS = 0.12;        // Add 0.25 V output to overcome static friction
  shooter_left_front_slot0.kV = 0.12;        // A velocity target of 1 rps results in 0.12 V output
  shooter_left_front_slot0.kA = 0;           // An acceleration of 1 rps/s requires 0.01 V output
  shooter_left_front_slot0.kP = 0.03;        // A position error of 0.2 rotations results in 12 V output
  shooter_left_front_slot0.kI = 0;           // No output for integrated error
  shooter_left_front_slot0.kD = 0.;          // A velocity error of 1 rps results in 0.5 V output
  shooter_left_front_slot0.GravityType = 0;  // elevator重力补偿

  /* Retry config apply up to 5 times, report if failure */
  ctre::phoenix::StatusCode shooter_left_front_status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i)
  {
    shooter_left_front_status = shooter_left_front_.Applyconfig(shooter_left_front_config);
    if (shooter_left_front_status.IsOK())
      break;
  }
  shooter_left_front_.setgearRatio(1.3);

  shooter_right_.setfollowControl(shooter_left_front_.Getdata().deviceId, false);
  shooter_right_.setgearRatio(1.3);

  frc::SmartDashboard::PutNumber("Shooter/Target X Distance:", 0.);
  frc::SmartDashboard::PutNumber("Shooter/Speed_conversion_eff:", 0.);
}

void ShooterSubsystem::Periodic()
{
  double left_axis = -joystick_.GetLeftY();
  double right_axis = left_axis;
  double now_s = frc::Timer::GetFPGATimestamp().value();
  double dt_s = (last_servo_update_s_ > 0.0) ? (now_s - last_servo_update_s_) : 0.0;
  last_servo_update_s_ = now_s;

  linear_servo_left_target_mm_ = std::clamp(linear_servo_left_target_mm_ + left_axis * kLinearServoSpeedMmPerS * dt_s,
                                            0.0, kLinearServoMaxPositionMm);
  linear_servo_right_target_mm_ = std::clamp(
      linear_servo_right_target_mm_ + right_axis * kLinearServoSpeedMmPerS * dt_s, 0.0, kLinearServoMaxPositionMm);

  SetLinearServoLeftPositionMm(linear_servo_left_target_mm_);
  SetLinearServoRightPositionMm(linear_servo_right_target_mm_);

  frc::SmartDashboard::PutNumber("linear_servo_left_cmd_mm", linear_servo_left_target_mm_);
  frc::SmartDashboard::PutNumber("linear_servo_right_cmd_mm", linear_servo_right_target_mm_);
}

void ShooterSubsystem::SetLinearServoLeftPositionMm(double position_mm)
{
  linear_servo_left_.SetPositionMm(position_mm);
}

void ShooterSubsystem::SetLinearServoRightPositionMm(double position_mm)
{
  linear_servo_right_.SetPositionMm(position_mm);
}

void ShooterSubsystem::SetShootVelocity(double velocity)
{
  velocity /= shooter_left_front_.Getdata().gearRatio;
  shooter_left_front_.setvelocity(velocity);
}

frc2::CommandPtr ShooterSubsystem::SetShootVelocityCommandPtr(double velocity)
{
  return frc2::cmd::RunOnce([this, velocity] { SetShootVelocity(velocity); });
}

double ShooterSubsystem::GetShootVelocity()
{
  return shooter_left_front_.Getdata().currentVelocity;
}

void ShooterSubsystem::Stop()
{
  SetShootVelocity(0.);
}