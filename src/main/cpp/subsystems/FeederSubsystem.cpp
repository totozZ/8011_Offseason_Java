// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/FeederSubsystem.h"

using namespace subsystems;

void FeederSubsystem::Initialization()
{
  configs::TalonFXConfiguration backward_feeder_config{};
  backward_feeder_config.MotorOutput.Inverted = 1;
  /* slot0 PID槽*/
  configs::Slot0Configs& backward_feeder_slot0 = backward_feeder_config.Slot0;
  backward_feeder_slot0.kG = 0.;          // Gear ratio of 1:2, 0.5 rotations per rotor rotation
  backward_feeder_slot0.kS = 0.12;        // Add 0.25 V output to overcome static friction
  backward_feeder_slot0.kV = 0.12;        // A velocity target of 1 rps results in 0.12 V output
  backward_feeder_slot0.kA = 0;           // An acceleration of 1 rps/s requires 0.01 V output
  backward_feeder_slot0.kP = 0.03;        // A position error of 0.2 rotations results in 12 V output
  backward_feeder_slot0.kI = 0;           // No output for integrated error
  backward_feeder_slot0.kD = 0.;          // A velocity error of 1 rps results in 0.5 V output
  backward_feeder_slot0.GravityType = 0;  // elevator重力补偿

  /* Retry config apply up to 5 times, report if failure */
  ctre::phoenix::StatusCode backward_feeder_status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i)
  {
    backward_feeder_status = backward_feeder_.Applyconfig(backward_feeder_config);
    if (backward_feeder_status.IsOK())
      break;
  }

  configs::TalonFXConfiguration upward_feeder_config{};
  upward_feeder_config.MotorOutput.Inverted = 0;
  /* slot0 PID槽*/
  configs::Slot0Configs& upward_feeder_slot0 = upward_feeder_config.Slot0;
  upward_feeder_slot0.kG = 0.;          // Gear ratio of 1:2, 0.5 rotations per rotor rotation
  upward_feeder_slot0.kS = 0.12;        // Add 0.25 V output to overcome static friction
  upward_feeder_slot0.kV = 0.12;        // A velocity target of 1 rps results in 0.12 V output
  upward_feeder_slot0.kA = 0;           // An acceleration of 1 rps/s requires 0.01 V output
  upward_feeder_slot0.kP = 0.03;        // A position error of 0.2 rotations results in 12 V output
  upward_feeder_slot0.kI = 0;           // No output for integrated error
  upward_feeder_slot0.kD = 0.;          // A velocity error of 1 rps results in 0.5 V output
  upward_feeder_slot0.GravityType = 0;  // elevator重力补偿

  /* Retry config apply up to 5 times, report if failure */
  ctre::phoenix::StatusCode upward_feeder_status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i)
  {
    upward_feeder_status = upward_feeder_.Applyconfig(upward_feeder_config);
    if (upward_feeder_status.IsOK())
      break;
  }
}

void FeederSubsystem::SetBackwardFeederVelocity(double duty)
{
  backward_feeder_.setNormalizedDutyCircle(duty);  // 使用占空比，范围-1到1
}

void FeederSubsystem::SetUpwardFeederVelocity(double duty)
{
  upward_feeder_.setNormalizedDutyCircle(duty);  // 使用占空比，范围-1到1
}

frc2::CommandPtr FeederSubsystem::SetBackwardFeederVelocityCommandPtr(double velocity)
{
  return frc2::cmd::RunOnce([this, velocity] { SetBackwardFeederVelocity(velocity); });
}

frc2::CommandPtr FeederSubsystem::SetUpwardFeederVelocityCommandPtr(double velocity)
{
  return frc2::cmd::RunOnce([this, velocity] { SetUpwardFeederVelocity(velocity); });
}

double FeederSubsystem::GetBackwardFeederVelocity()
{
  return backward_feeder_.Getdata().currentVelocity;
}

double FeederSubsystem::GetUpwardFeederVelocity()
{
  return upward_feeder_.Getdata().currentVelocity;
}

void FeederSubsystem::Stop()
{
  SetBackwardFeederVelocity(0.);
  SetUpwardFeederVelocity(0.);
}