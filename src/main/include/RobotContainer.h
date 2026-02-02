// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"
#include "Telemetry.h"
#include "commands/ComplexCommand.h"
#include "frc8011/ClientSubsystem.h"
#include "subsystems/CommandSwerveDrivetrain.h"
#include "frc8011/GPDetection.h"
#include "frc8011/LEDSubsystem.h"
#include "subsystems/VisionSubsystem.h"

class RobotContainer {
 private:
  units::meters_per_second_t MaxSpeed = TunerConstants::kSpeedAt12Volts;
  units::radians_per_second_t MaxAngularRate = 0.75_tps;

  swerve::requests::FieldCentric drive = swerve::requests::FieldCentric{}
      .WithDeadband(MaxSpeed * 0.05)
      .WithRotationalDeadband(MaxAngularRate * 0.05)
      .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage);

  Telemetry logger{MaxSpeed};

 public:
  frc2::CommandXboxController joystick{0};
  LEDSubsystem m_ledsubsystem;

  subsystems::CommandSwerveDrivetrain drivetrain{TunerConstants::CreateDrivetrain()};
  subsystems::VisionSubsystem visionSub;
  subsystems::ClientSubsystem clientSub;
  subsystems::GPDetection gpdetection;
  ComplexCommand complexcommand;

 private:
  frc::SendableChooser<frc2::Command *> autoChooser;

 public:
  RobotContainer();
  frc2::Command *GetAutonomousCommand();

 private:
  void ConfigureBindings();
};

