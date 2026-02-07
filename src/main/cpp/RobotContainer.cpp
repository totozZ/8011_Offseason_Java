// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <frc2/command/WaitCommand.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

RobotContainer::RobotContainer()
  : visionSub(&drivetrain, &m_ledsubsystem, &gpdetection)
  , shooterSub(joystick)
  , feederSub(joystick)
  , clientSub(&drivetrain)
  , gpdetection("GPDetection", &drivetrain)
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
    auto& request = useClosedLoop ? driveClosed : drive;
    return request.WithVelocityX(-joystick.GetLeftY() * MaxSpeed * OperatorConstants::SpeedRate)
        .WithVelocityY(-joystick.GetLeftX() * MaxSpeed * OperatorConstants::SpeedRate)
        .WithRotationalRate(-joystick.GetRightX() * MaxAngularRate * OperatorConstants::AngularSpeedRate);
  }));

  joystick.LeftBumper().OnTrue(frc2::cmd::RunOnce([this] {
    useClosedLoop = !useClosedLoop;
    frc::SmartDashboard::PutBoolean("Drive ClosedLoop", useClosedLoop);
  }));

  // joystick.A().WhileTrue(
  //     frc2::cmd::Sequence(
  //         frc2::cmd::RunOnce([this] { shooterSub.SetShootVelocity(45); }),
  //         frc2::WaitCommand(3_s).ToPtr(),
  //         frc2::cmd::Run([this] {
  //           feederSub.SetBackwardFeederVelocity(0.54);
  //           feederSub.SetUpwardFeederVelocity(0.7);
  //         }))
  //         .FinallyDo([this] {
  //           shooterSub.Stop();
  //           feederSub.Stop();
  //         }));

  joystick.B().WhileTrue(
      frc2::cmd::Run([this] { shooterSub.SetShootVelocity(39); }))
      .WhileFalse(frc2::cmd::RunOnce([this] { shooterSub.Stop(); }));

          
  // //底盘 SysId
  // joystick.A().WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kForward));
  // joystick.B().WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kReverse));
  // joystick.X().WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kForward));
  // joystick.Y().WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kReverse));

  // // Shooter SysId
  // joystick.A().WhileTrue(shooterSub.SysIdQuasistatic(frc2::sysid::Direction::kForward));
  // joystick.B().WhileTrue(shooterSub.SysIdQuasistatic(frc2::sysid::Direction::kReverse));
  // joystick.X().WhileTrue(shooterSub.SysIdDynamic(frc2::sysid::Direction::kForward));
  // joystick.Y().WhileTrue(shooterSub.SysIdDynamic(frc2::sysid::Direction::kReverse));

  // // Feeder SysId
  // joystick.A().WhileTrue(feederSub.SysIdQuasistatic(frc2::sysid::Direction::kForward));
  // joystick.B().WhileTrue(feederSub.SysIdQuasistatic(frc2::sysid::Direction::kReverse));
  // joystick.X().WhileTrue(feederSub.SysIdDynamic(frc2::sysid::Direction::kForward));
  // joystick.Y().WhileTrue(feederSub.SysIdDynamic(frc2::sysid::Direction::kReverse));

  drivetrain.RegisterTelemetry([this](auto const& state) { logger.Telemeterize(state); });
}

frc2::Command* RobotContainer::GetAutonomousCommand()
{
  return autoChooser.GetSelected();
}
