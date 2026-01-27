// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

RobotContainer::RobotContainer()
  : visionSub(&drivetrain, &m_ledsubsystem, &gpdetection)
  , clientSub(&drivetrain)
  , gpdetection("GPDetection", &drivetrain)
  , shooter(joystick)
  , complexcommand(&drivetrain, &visionSub)

{
  drivetrain.SetGPDetection(&gpdetection);

  // 注册auto中events命令
  // EventTrigger("Reset").OnTrue(ResetTranslationCommand());
  autoChooser = pathplanner::AutoBuilder::buildAutoChooser("offseason_right");
  frc::SmartDashboard::PutData("Auto Mode", &autoChooser);

  ConfigureBindings();
}

void RobotContainer::ConfigureBindings()
{
  // Note that X is defined as forward according to WPILib convention,
  // and Y is defined as to the left according to WPILib convention.
  drivetrain.SetDefaultCommand(drivetrain.ApplyRequest([this]() -> auto&& {
    return drive.WithVelocityX(-joystick.GetLeftY() * MaxSpeed * OperatorConstants::SpeedRate)
        .WithVelocityY(-joystick.GetLeftX() * MaxSpeed * OperatorConstants::SpeedRate)
        .WithRotationalRate(-joystick.GetRightX() * MaxAngularRate * OperatorConstants::AngularSpeedRate);
  }));

  joystick.LeftBumper().ToggleOnTrue(frc2::cmd::DeferredProxy(
      [this]() { return drivetrain.followPathCommand(visionSub.GetTarget_posleft(), frc2::cmd::None()); }));

  joystick.RightBumper().ToggleOnTrue(frc2::cmd::DeferredProxy(
      [this]() { return drivetrain.followPathCommand(visionSub.GetTarget_posright(), frc2::cmd::None()); }));

  drivetrain.RegisterTelemetry([this](auto const& state) { logger.Telemeterize(state); });
}

frc2::Command* RobotContainer::GetAutonomousCommand()
{
  return autoChooser.GetSelected();
}
