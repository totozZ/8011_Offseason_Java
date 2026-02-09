// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/FeederSubsystem.h"

#include <cmath>

using namespace subsystems;

void FeederSubsystem::Initialization()
{
  configs::TalonFXConfiguration backward_feeder_config{};
  backward_feeder_config.MotorOutput.Inverted = 1;
  /* slot0 PID妲?*/
  configs::Slot0Configs &backward_feeder_slot0 = backward_feeder_config.Slot0;
  backward_feeder_slot0.kG = 0.;         // Gear ratio of 1:2, 0.5 rotations per rotor rotation
  backward_feeder_slot0.kS = 0.12;       // Add 0.25 V output to overcome static friction
  backward_feeder_slot0.kV = 0.12;       // A velocity target of 1 rps results in 0.12 V output
  backward_feeder_slot0.kA = 0;          // An acceleration of 1 rps/s requires 0.01 V output
  backward_feeder_slot0.kP = 0.03;       // A position error of 0.2 rotations results in 12 V output
  backward_feeder_slot0.kI = 0;          // No output for integrated error
  backward_feeder_slot0.kD = 0.;         // A velocity error of 1 rps results in 0.5 V output
  backward_feeder_slot0.GravityType = 0; // elevator閲嶅姏琛ュ伩

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
  /* slot0 PID妲?*/
  configs::Slot0Configs &upward_feeder_slot0 = upward_feeder_config.Slot0;
  upward_feeder_slot0.kG = 0.;         // Gear ratio of 1:2, 0.5 rotations per rotor rotation
  upward_feeder_slot0.kS = 7;          // Add 0.25 V output to overcome static friction
  upward_feeder_slot0.kV = 0.1;        // A velocity target of 1 rps results in 0.12 V output
  upward_feeder_slot0.kA = 3;          // An acceleration of 1 rps/s requires 0.01 V output
  upward_feeder_slot0.kP = 3.4;        // A position error of 0.2 rotations results in 12 V output
  upward_feeder_slot0.kI = 1.5;        // No output for integrated error
  upward_feeder_slot0.kD = 0.;         // A velocity error of 1 rps results in 0.5 V output
  upward_feeder_slot0.GravityType = 0; // elevator閲嶅姏琛ュ伩

  /* Retry config apply up to 5 times, report if failure */
  ctre::phoenix::StatusCode upward_feeder_status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i)
  {
    upward_feeder_status = upward_feeder_.Applyconfig(upward_feeder_config);
    if (upward_feeder_status.IsOK())
      break;
  }
  upward_feeder_.setPhysicalLimits(0, 360000, 100, 40); // 璁剧疆鐗╃悊闄愬埗锛屼綅缃?-360搴︼紝閫熷害100搴?s锛岀數娴?0A
  upward_feeder_.setgearRatio(1.4); // 璁剧疆鍑忛€熸瘮
}

void FeederSubsystem::Periodic()
{

  backward_feeder_.Control();
  upward_feeder_.Control();

  backward_feeder_.Receive();
  upward_feeder_.Receive();



  if (joystick_.X().Get())
  {
  SetUpperVelocitycombo(combo_target_velocity_);
  }
  else
  {
    Stop();
  }
}

void FeederSubsystem::SetBackwardFeederVelocity(double duty)
{
  backward_feeder_.setNormalizedDutyCircle(duty); // 浣跨敤鍗犵┖姣旓紝鑼冨洿-1鍒?
  // backward_feeder_.setVelocityTorqueCurrent(duty);
}

void FeederSubsystem::SetUpwardFeederVelocity(double duty)
{
  // upward_feeder_.setvelocitytorquecurrent(duty); // 浣跨敤鍗犵┖姣旓紝鑼冨洿-1鍒?
  upward_feeder_.setvelocitytorquecurrent(duty);
}

frc2::CommandPtr FeederSubsystem::SetBackwardFeederVelocityCommandPtr(double velocity)
{
  return frc2::cmd::RunOnce([this, velocity]
                            { SetBackwardFeederVelocity(velocity); });
}

frc2::CommandPtr FeederSubsystem::SetUpwardFeederVelocityCommandPtr(double velocity)
{
  return frc2::cmd::RunOnce([this, velocity]
                            { SetUpwardFeederVelocity(velocity); });
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
  upper_velocity_reached_once_ = false;
  //combo_target_velocity_ = 0.0;
}

void FeederSubsystem::SetUpwardDuty(double duty)
{
  upward_feeder_.setNormalizedDutyCircle(duty);
}

void FeederSubsystem::SetUpperVelocityBANGBANG(double velocity)
{
  double current_velocity = GetUpwardFeederVelocity();
  frc::SmartDashboard::PutNumber("Feeder Upward Velocity", current_velocity);

  if (current_velocity < velocity)
  {
    SetUpwardDuty(1.0); // 鍏ㄩ€熷墠杩?
  }
  else
  {
    SetUpwardDuty(0.0); // 鍋滄
  }
}

frc2::CommandPtr FeederSubsystem::SetUpperVelocityBANGBANGCommandPtr(double velocity)
{
  return frc2::cmd::RunOnce([this, velocity]
                            { SetUpperVelocityBANGBANG(velocity); });
}


void FeederSubsystem::SetUpperVelocitycombo(double velocity)
{
  if (velocity <= 0.0)
  {
    SetUpwardFeederVelocity(0.0);
    upper_velocity_reached_once_ = false;
    combo_target_velocity_ = 0.0;
    return;
  }

  if (std::abs(velocity - combo_target_velocity_) > 1e-6)
  {
    combo_target_velocity_ = velocity;
    upper_velocity_reached_once_ = false;
  }

  const double current_velocity = GetUpwardFeederVelocity();
  frc::SmartDashboard::PutNumber("Feeder Combo Target Velocity", velocity);
  frc::SmartDashboard::PutNumber("Feeder Combo ReachedOnce", upper_velocity_reached_once_);
  frc::SmartDashboard::PutNumber("Feeder Combo current Velocity", current_velocity);

  if (!upper_velocity_reached_once_)
  {
    SetUpperVelocityBANGBANG(velocity);
    if (current_velocity >= (velocity - kUpperVelocityReachTolerance))
    {
      upper_velocity_reached_once_ = true;
    }
    return;
  }

  // Reached once: hold by FOC velocity control.
  SetUpwardFeederVelocity(velocity);
  frc::SmartDashboard::PutNumber("targetvelocity", upward_feeder_.Getdata().targetVelocity);
  frc::SmartDashboard::PutNumber("Veloutput", upward_feeder_.Getdata().Veloutput.value());
}
