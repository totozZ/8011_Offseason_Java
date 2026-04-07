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
  
  // --- 保留的旧版旧接口 ---
  void SetRollerVelocity(double velocity);
  frc2::CommandPtr SetRollerVelocityCommandPtr(double velocity);
  
  void SetRollerDutyCycle(double dutyCycle);
  frc2::CommandPtr SetRollerDutyCycleCommandPtr(double dutyCycle);
  
  void SetPitchPosition(double position);
  
  void Stop();
  frc2::CommandPtr StopCommandPtr();
  void SetTeleopRollerCurrentLimit();
  void SetPitchNormPosition(double norm);
  frc2::CommandPtr SetPitchNormPositionCommandPtr(double norm);

  double GetPitchCurrent();
  double GetPitchNormPosition();
  void BrakePitch();
  frc2::CommandPtr BrakePitchCommandPtr();
  bool getPitchResetFlag(){
    return pitch_reset_flag_;
  }
private:
  void Initialization();

  // --- 新版马达配置：由 2 个变为 3 个 ---
  Wayimotor intake_roller_left_{ GroundIntakeConstants::IntakeRollerLeftMotorID, kCANBus };
  Wayimotor intake_roller_right_{ GroundIntakeConstants::IntakeRollerRightMotorID, kCANBus }; // 主控
  Wayimotor intake_pitch_{ GroundIntakeConstants::IntakePivotMotorID, kCANBus };
  
  bool pitch_reset_flag_ = false;
  int pitch_reset_counter_ = 0;
  void GroundIntakeReset();
};

}  // namespace subsystems