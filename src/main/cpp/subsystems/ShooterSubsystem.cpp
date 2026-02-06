// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterSubsystem.h"

#include <algorithm>

#include <frc/Timer.h>

using namespace subsystems;

ShooterSubsystem::ShooterSubsystem(frc2::CommandXboxController& joystick)
    : joystick_(joystick) {}

void ShooterSubsystem::Periodic() {
  double left_axis = -joystick_.GetLeftY();
  double right_axis = left_axis;
  double now_s = frc::Timer::GetFPGATimestamp().value();
  double dt_s = (last_servo_update_s_ > 0.0) ? (now_s - last_servo_update_s_) : 0.0;
  last_servo_update_s_ = now_s;

  linear_servo_left_target_mm_ = std::clamp(
      linear_servo_left_target_mm_ + left_axis * kLinearServoSpeedMmPerS * dt_s,
      0.0, kLinearServoMaxPositionMm);
  linear_servo_right_target_mm_ = std::clamp(
      linear_servo_right_target_mm_ + right_axis * kLinearServoSpeedMmPerS * dt_s,
      0.0, kLinearServoMaxPositionMm);

  SetLinearServoLeftPositionMm(linear_servo_left_target_mm_);
  SetLinearServoRightPositionMm(linear_servo_right_target_mm_);

  frc::SmartDashboard::PutNumber("linear_servo_left_cmd_mm",
                                 linear_servo_left_target_mm_);
  frc::SmartDashboard::PutNumber("linear_servo_right_cmd_mm",
                                 linear_servo_right_target_mm_);
}

void ShooterSubsystem::SetLinearServoLeftPositionMm(double position_mm) {
  linear_servo_left_.SetPositionMm(position_mm);
}

void ShooterSubsystem::SetLinearServoRightPositionMm(double position_mm) {
  linear_servo_right_.SetPositionMm(position_mm);
}
