// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"
#include "frc8011/Wayimotor.h"
#include "subsystems/ExampleSubsystem.h"

namespace subsystems
{
class FeederSubsystem : public ExampleSubsystem
{
public:
  FeederSubsystem(frc2::CommandXboxController& joystick_) : ExampleSubsystem(joystick_)
  {
    Initialization();
  }
  void Periodic() override;

  frc2::CommandPtr SetBackwardFeederVelocityCommandPtr(double velocity);
  frc2::CommandPtr SetUpwardFeederVelocityCommandPtr(double velocity);
  void SetBackwardFeederVelocity(double velocity);
  void SetUpwardFeederVelocity(double velocity);
  double GetBackwardFeederVelocity();
  double GetUpwardFeederVelocity();
  void Stop();

private:
  void Initialization();

  Wayimotor backward_feeder_{ ShooterConstants::BackwardFeederMotorID, kCANBus };  // 反进料电机
  Wayimotor upward_feeder_{ ShooterConstants::UpwardFeederMotorID, kCANBus };      // 上进料电机
};
}  // namespace subsystems
