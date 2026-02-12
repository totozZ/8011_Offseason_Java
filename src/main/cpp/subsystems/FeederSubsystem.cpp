// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/FeederSubsystem.h"

#include <cmath>
#include <frc/Timer.h>

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
  static constexpr double kPeriodicOverrunMs = 5.0;
  static int periodic_overrun_count = 0;
  static double periodic_ms_max = 0.0;
  const double t_start_s = frc::Timer::GetFPGATimestamp().value();

  backward_feeder_.Control();
  upward_feeder_.Control();

  backward_feeder_.Receive();
  upward_feeder_.Receive();



  const bool x_pressed = joystick_.X().Get();
  if (x_pressed) {
    SetUpperVelocitycombo(combo_target_velocity_);
    x_combo_was_active_ = true;
  } else if (x_combo_was_active_) {
    // Only stop once when exiting X-hold mode, do not override other commands.
    Stop();
    x_combo_was_active_ = false;
  }

  const double periodic_ms =
      (frc::Timer::GetFPGATimestamp().value() - t_start_s) * 1000.0;
  if (periodic_ms > periodic_ms_max) {
    periodic_ms_max = periodic_ms;
  }
  if (periodic_ms > kPeriodicOverrunMs) {
    ++periodic_overrun_count;
  }
  frc::SmartDashboard::PutNumber("Perf/FeederPeriodicMs", periodic_ms);
  frc::SmartDashboard::PutNumber("Perf/FeederPeriodicMsMax", periodic_ms_max);
  frc::SmartDashboard::PutNumber("Perf/FeederPeriodicOverrunCount",
                                 periodic_overrun_count);
}

void FeederSubsystem::SetBackwardFeederVelocity(double duty)
{
  backward_feeder_.setNormalizedDutyCircle(duty); // 浣跨敤鍗犵┖姣旓紝鑼冨洿-1鍒?
  // backward_feeder_.setVelocityTorqueCurrent(duty);
}

void FeederSubsystem::SetBackwardFeederDuty(double duty) {
  backward_feeder_.setNormalizedDutyCircle(duty);
}

void FeederSubsystem::SetUpwardFeederVelocity(double duty)
{
  // upward_feeder_.setNormalizedDutyCircle(duty);
  upward_feeder_.setvelocitytorquecurrent(duty);
}

void FeederSubsystem::SetUpwardFeederCurrent(double current,
                                             double max_abs_duty_cycle) {
  upward_feeder_.setCurrent_Speed(max_abs_duty_cycle);
  upward_feeder_.setcurrent(current);
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

double FeederSubsystem::GetUpwardFeederCurrent() {
  return upward_feeder_.Getdata().currentCurrent;
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
  static double last_debug_publish_s = -1.0;
  static constexpr double kDebugPublishPeriodS = 0.1;
  const double now_s = frc::Timer::GetFPGATimestamp().value();
  const bool publish_debug =
      (last_debug_publish_s < 0.0) ||
      ((now_s - last_debug_publish_s) >= kDebugPublishPeriodS);
  if (publish_debug) {
    last_debug_publish_s = now_s;
  }

  double current_velocity = GetUpwardFeederVelocity();
  if (publish_debug) {
    frc::SmartDashboard::PutNumber("Feeder Upward Velocity", current_velocity);
  }

  if (current_velocity < velocity)
  {
    SetUpwardDuty(1.0); // 全速
  }
  else
  {
    SetUpwardDuty(0.0); // 停
  }
}

frc2::CommandPtr FeederSubsystem::SetUpperVelocityBANGBANGCommandPtr(double velocity)
{
  return frc2::cmd::RunOnce([this, velocity]
                            { SetUpperVelocityBANGBANG(velocity); });
}

frc2::CommandPtr FeederSubsystem::SetBackwardFeederDutyCommandPtr(double duty) {
  return frc2::cmd::RunOnce([this, duty] { SetBackwardFeederDuty(duty); });
}

frc2::CommandPtr FeederSubsystem::SetUpwardFeederCurrentCommandPtr(
    double current, double max_abs_duty_cycle) {
  return frc2::cmd::RunOnce([this, current, max_abs_duty_cycle] {
    SetUpwardFeederCurrent(current, max_abs_duty_cycle);
  });
}


void FeederSubsystem::SetUpperVelocitycombo(double velocity)
{
  static double last_debug_publish_s = -1.0;
  static constexpr double kDebugPublishPeriodS = 0.1;
  const double now_s = frc::Timer::GetFPGATimestamp().value();
  const bool publish_debug =
      (last_debug_publish_s < 0.0) ||
      ((now_s - last_debug_publish_s) >= kDebugPublishPeriodS);
  if (publish_debug) {
    last_debug_publish_s = now_s;
  }

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
  if (publish_debug) {
    frc::SmartDashboard::PutNumber("Feeder Combo Target Velocity", velocity);
    frc::SmartDashboard::PutNumber("Feeder Combo ReachedOnce",
                                   upper_velocity_reached_once_);
    frc::SmartDashboard::PutNumber("Feeder Combo current Velocity",
                                   current_velocity);
  }

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
  if (publish_debug) {
    frc::SmartDashboard::PutNumber("targetvelocity",
                                   upward_feeder_.Getdata().targetVelocity);
    frc::SmartDashboard::PutNumber("Veloutput",
                                   upward_feeder_.Getdata().Veloutput.value());
  }
}

void FeederSubsystem::SetPreload() {
  SetUpwardFeederCurrent(13.0, 0.25);
  SetBackwardFeederDuty(0.3);
}

frc2::CommandPtr FeederSubsystem::SetPreloadCommandPtr() {
  return this->RunOnce([this] { SetPreload(); });
}
