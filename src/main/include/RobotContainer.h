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
#include "subsystems/FeederSubsystem.h"
#include "subsystems/GroundIntakeSubsystem.h"
#include "frc8011/LEDSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/VisionSubsystem.h"

class RobotContainer
{
private:
  units::meters_per_second_t MaxSpeed = TunerConstants::kSpeedAt12Volts;
  units::radians_per_second_t MaxAngularRate = 0.75_tps;

  /* Setting up bindings for necessary control of the swerve drive platform */
  swerve::requests::FieldCentric drive = swerve::requests::FieldCentric{}
                                             .WithDeadband(MaxSpeed * 0.1)
                                             .WithRotationalDeadband(MaxAngularRate * 0.1)
                                             .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage);

  swerve::requests::FieldCentric driveClosed = swerve::requests::FieldCentric{}
                                                   .WithDeadband(MaxSpeed * 0.1)
                                                   .WithRotationalDeadband(MaxAngularRate * 0.1)
                                                   .WithDriveRequestType(swerve::DriveRequestType::Velocity)
                                                   .WithSteerRequestType(swerve::SteerRequestType::Position);

  bool useClosedLoop = false;

  Telemetry logger{ MaxSpeed };

  frc2::CommandXboxController joystick{ 0 };
  LEDSubsystem m_ledsubsystem;

public:
  subsystems::CommandSwerveDrivetrain drivetrain{ TunerConstants::CreateDrivetrain() };
  subsystems::VisionSubsystem visionSub;
  subsystems::ShooterSubsystem shooterSub;
  subsystems::FeederSubsystem feederSub;
  subsystems::ClientSubsystem clientSub;
  subsystems::GroundIntakeSubsystem groundIntakeSub;
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
