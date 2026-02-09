// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/button/CommandXboxController.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"
#include "frc8011/Wayimotor.h"
#include "subsystems/ExampleSubsystem.h"

namespace subsystems
{

class GroundIntakeSubsystem : public ExampleSubsystem
{
public:
  explicit GroundIntakeSubsystem(frc2::CommandXboxController& joystick_) : ExampleSubsystem(joystick_)
  {
    Initialization();
  }

  void Periodic() override;
  void SetRollerVelocity(double velocity);
  void SetPivotPosition(double position);
  void Stop();

private:
  void Initialization();

  // Intake roller (spin to intake/eject game pieces).
  Wayimotor intake_roller_{ GroundIntakeConstants::IntakeRollerMotorID, kCANBus };
  // Intake pivot (position-loop for intake arm angle).
  Wayimotor intake_pivot_{ GroundIntakeConstants::IntakePivotMotorID, kCANBus };
};

}  // namespace subsystems
