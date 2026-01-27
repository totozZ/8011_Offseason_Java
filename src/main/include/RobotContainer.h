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
#include "subsystems/ClientSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/CommandSwerveDrivetrain.h"
#include "subsystems/GPDetection.h"

class RobotContainer
{
private:
  units::meters_per_second_t MaxSpeed = TunerConstants::kSpeedAt12Volts;
  units::radians_per_second_t MaxAngularRate = 0.75_tps;

  /* Setting up bindings for necessary control of the swerve drive platform */
  swerve::requests::FieldCentric drive = swerve::requests::FieldCentric{}
                                             .WithDeadband(MaxSpeed * 0.05)
                                             .WithRotationalDeadband(MaxAngularRate * 0.05)
                                             .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage);
  swerve::requests::SwerveDriveBrake brake{};
  swerve::requests::PointWheelsAt point{};
  swerve::requests::RobotCentric forwardStraight =
      swerve::requests::RobotCentric{}.WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage);

  Telemetry logger{ MaxSpeed };

  frc2::CommandXboxController joystick{ 0 };
  LEDSubsystem m_ledsubsystem;

public:
  subsystems::CommandSwerveDrivetrain drivetrain{ TunerConstants::CreateDrivetrain() };
  subsystems::VisionSubsystem visionSub;
  subsystems::ClientSubsystem clientSub;
  subsystems::GPDetection gpdetection;
  subsystems::ShooterSubsystem shooter;
  ComplexCommand complexcommand;

private:
  /* Path follower */
  frc::SendableChooser<frc2::Command*> autoChooser;

  std::optional<frc2::CommandPtr> pid_align_command;  // 当前正在执行的PID对齐Command

public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

private:
  void ConfigureBindings();
};
