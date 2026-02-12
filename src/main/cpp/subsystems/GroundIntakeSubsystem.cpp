// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/GroundIntakeSubsystem.h"

using namespace subsystems;

void GroundIntakeSubsystem::Initialization() {
  configs::TalonFXConfiguration intake_roller_config{};
  intake_roller_config.MotorOutput.Inverted = 0;
  configs::Slot0Configs& intake_roller_slot0 = intake_roller_config.Slot0;
  intake_roller_slot0.kG = 0.;
  intake_roller_slot0.kS = 0.;
  intake_roller_slot0.kV = 0.;
  intake_roller_slot0.kA = 0.;
  intake_roller_slot0.kP = 0.;
  intake_roller_slot0.kI = 0.;
  intake_roller_slot0.kD = 0.;
  intake_roller_slot0.GravityType = 0;

  ctre::phoenix::StatusCode intake_roller_status =
      ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i) {
    intake_roller_status = intake_roller_.Applyconfig(intake_roller_config);
    if (intake_roller_status.IsOK()) {
      break;
    }
  }

  configs::TalonFXConfiguration intake_pivot_config{};
  intake_pivot_config.MotorOutput.Inverted = 0;
  configs::Slot0Configs& intake_pivot_slot0 = intake_pivot_config.Slot0;
  intake_pivot_slot0.kG = 0.;
  intake_pivot_slot0.kS = 0.;
  intake_pivot_slot0.kV = 0.;
  intake_pivot_slot0.kA = 0.;
  intake_pivot_slot0.kP = 0.03;
  intake_pivot_slot0.kI = 0.;
  intake_pivot_slot0.kD = 0.;
  intake_pivot_slot0.GravityType = 0;

  ctre::phoenix::StatusCode intake_pivot_status =
      ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i) {
    intake_pivot_status = intake_pivot_.Applyconfig(intake_pivot_config);
    if (intake_pivot_status.IsOK()) {
      break;
    }
  }
}

void GroundIntakeSubsystem::Periodic() {
  intake_roller_.Control();
  intake_pivot_.Control();
  intake_roller_.Receive();
  intake_pivot_.Receive();

  frc::SmartDashboard::PutNumber("ground_intake_roller_velocity",
                                 intake_roller_.Getdata().currentVelocity);
  frc::SmartDashboard::PutNumber("ground_intake_pivot_position",
                                 intake_pivot_.Getdata().currentPosition);
}

void GroundIntakeSubsystem::SetRollerVelocity(double velocity) {
  intake_roller_.setvelocitytorquecurrent(velocity);
}

void GroundIntakeSubsystem::SetPivotPosition(double position) {
  intake_pivot_.setmode(12);
  intake_pivot_.Getdata().targetPosition = position;
}

void GroundIntakeSubsystem::SetRollerDutyCycle(double dutyCycle) {
  intake_roller_.setNormalizedDutyCircle(dutyCycle);
}

void GroundIntakeSubsystem::Stop() { SetRollerVelocity(0.0); }
