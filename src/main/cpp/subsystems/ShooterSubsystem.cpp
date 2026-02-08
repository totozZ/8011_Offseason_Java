// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterSubsystem.h"

#include <algorithm>
#include <cmath>

#include <frc/Timer.h>

using namespace subsystems;

void ShooterSubsystem::Initialization()
{
  
  configs::TalonFXConfiguration shooter_right_config{};
  shooter_right_config.MotorOutput.Inverted = 0; 

  // Slot0 PID 
  configs::Slot0Configs& shooter_right_slot0 = shooter_right_config.Slot0;
  shooter_right_slot0.kG = 0.;
  shooter_right_slot0.kS = 7;
  shooter_right_slot0.kV = 0.06;
  shooter_right_slot0.kA = 3.5;
  shooter_right_slot0.kP = 5;
  shooter_right_slot0.kI = 0;
  shooter_right_slot0.kD = 0.;
  shooter_right_slot0.GravityType = 0;


  ctre::phoenix::StatusCode shooter_right_status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i)
  {
    shooter_right_status = shooter_right_.Applyconfig(shooter_right_config);
    if (shooter_right_status.IsOK())
      break;
  }

  // follow
  shooter_left_front_.setfollowControl(shooter_right_.Getdata().deviceId, true);
  shooter_left_back_.setfollowControl(shooter_right_.Getdata().deviceId, true);

  // Power-on default actuator position
  SetLinearServoLeftPositionMm(linear_servo_left_target_mm_);
  SetLinearServoRightPositionMm(linear_servo_right_target_mm_);
}
void ShooterSubsystem::Periodic()
{
  double left_trigger = joystick_.GetLeftTriggerAxis();
  double right_trigger = joystick_.GetRightTriggerAxis();
  double now_s = frc::Timer::GetFPGATimestamp().value();
  double dt_s = (last_servo_update_s_ > 0.0) ? (now_s - last_servo_update_s_) : 0.0;
  last_servo_update_s_ = now_s;

  double servo_command = left_trigger - right_trigger;
  if (std::abs(servo_command) < 0.02) {
    servo_command = 0.0;
  }

  linear_servo_left_target_mm_ =
      std::clamp(linear_servo_left_target_mm_ + servo_command * kLinearServoSpeedMmPerS * dt_s, 0.0,
                 kLinearServoMaxPositionMm);
  linear_servo_right_target_mm_ =
      std::clamp(linear_servo_right_target_mm_ + servo_command * kLinearServoSpeedMmPerS * dt_s, 0.0,
                 kLinearServoMaxPositionMm);

  SetLinearServoLeftPositionMm(linear_servo_left_target_mm_);
  SetLinearServoRightPositionMm(linear_servo_right_target_mm_);

  

      frc::SmartDashboard::PutNumber("linear_servo_left_cmd_mm", linear_servo_left_target_mm_);
  frc::SmartDashboard::PutNumber("linear_servo_right_cmd_mm", linear_servo_right_target_mm_);
  shooter_right_.Control();
  shooter_left_back_.Control();
  shooter_left_front_.Control();
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
  shooter_right_.setvelocitytorquecurrent(velocity);  // 使用VelocityTorqueCurrentFOC
}

frc2::CommandPtr ShooterSubsystem::SetShootVelocityCommandPtr(double velocity)
{
  return frc2::cmd::RunOnce([this, velocity] { SetShootVelocity(velocity); });
}

double ShooterSubsystem::GetShootVelocity()
{
  return shooter_right_.Getdata().currentVelocity;  // 从主电机读取
}

void ShooterSubsystem::Stop()
{
  SetShootVelocity(0.0);
}
