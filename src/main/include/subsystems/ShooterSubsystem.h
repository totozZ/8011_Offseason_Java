// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANBus.hpp>

#include "subsystems/ExampleSubsystem.h"
#include "frc8011/Wayimotor.h"
#include "frc8011/BallSolver.h"
#include "Constants.h"

namespace subsystems
{
class ShooterSubsystem : public ExampleSubsystem
{
public:
  ShooterSubsystem(frc2::CommandXboxController& m_joystick) : ExampleSubsystem(m_joystick)
  {
    Initialization();
  }

  frc2::CommandPtr TeleopControlCommand();
  frc2::CommandPtr SetPitchPositionCommand(double position);
  void SetPitchPosition(double position);
  void SetShootVelocity(double velocity);
  double GetShootVelocity();
  void Shoot(double shoot_vel, double shoot_angle);
  void Stop();
  void Periodic() override;

private:
  void Initialization();

  BallSolver ball_solver_;
  Wayimotor shooter_left_{ ShooterConstants::ShooterLeftMotorID, kCANBus };    // 发射左电机
  Wayimotor shooter_right_{ ShooterConstants::ShooterRightMotorID, kCANBus };  // 发射右电机
  Wayimotor pitch_{ ShooterConstants::PitchMotorID, kCANBus };                 // 发射机构俯仰电机
  double speed_conversion_efficiency_ = 0;

  double shoot_vel_, shoot_pitch_angle_, shoot_yaw_angle_ = 0.;
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
};
}  // namespace subsystems