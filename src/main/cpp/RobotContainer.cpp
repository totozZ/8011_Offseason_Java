// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <frc2/command/WaitCommand.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

RobotContainer::RobotContainer()
    : visionSub(&drivetrain, nullptr),
      shooterSub(joystick),
      feederSub(joystick),
      clientSub(&drivetrain),
      groundIntakeSub(joystick),
      complexcommand(&drivetrain, &visionSub, &shooterSub, &feederSub,
                     &groundIntakeSub)

{
  shooterSub.SetFeederSubsystem(&feederSub);
  shooterSub.SetDrivetrainSubsystem(&drivetrain);

  // 濞夈劌鍞絘uto娑撶挱vents閸涙垝鎶?
  // EventTrigger("Reset").OnTrue(ResetTranslationCommand());
  autoChooser = pathplanner::AutoBuilder::buildAutoChooser();
  frc::SmartDashboard::PutData("Auto Mode", &autoChooser);

  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Input ownership note:
  // Periodic-owned inputs: LeftTrigger, RightTrigger.
  // Check docs/controller-input-map.md before adding new button bindings.
  // Note that X is defined as forward according to WPILib convention,
  // and Y is defined as to the left according to WPILib convention.
  drivetrain.SetDefaultCommand(drivetrain.ApplyRequest([this]() -> auto&& {
    auto& request = useClosedLoop ? driveClosed : drive;
    return request
        .WithVelocityX(-joystick.GetLeftY() * MaxSpeed *
                       OperatorConstants::SpeedRate)
        .WithVelocityY(-joystick.GetLeftX() * MaxSpeed *
                       OperatorConstants::SpeedRate)
        .WithRotationalRate(-joystick.GetRightX() * MaxAngularRate *
                            OperatorConstants::AngularSpeedRate);
  }));

  joystick.LeftBumper().OnTrue(frc2::cmd::RunOnce([this] {
    useClosedLoop = !useClosedLoop;
    frc::SmartDashboard::PutBoolean("Drive ClosedLoop", useClosedLoop);
  }));

  // joystick.B().WhileTrue(
  //     frc2::cmd::StartEnd(銆併€戙€愩€戙€愩€?
  //         [this]
  //         { shooterSub.SetShootVelocity(44
  //         ); }, // 瀵偓婵妞傞幍褑顢?
  //         [this]
  //         { shooterSub.Stop(); }, //
  //         缂佹挻娼敍鍫熸緱瀵偓閿涘妞傞幍褑顢?
  //         {&shooterSub}           // 閸忔娊鏁敍姘紣閺勫酣娓跺Ч?
  //         ));

  // //鎼存洜娲?SysId閵嗘垯鈧降鈧?
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

  drivetrain.RegisterTelemetry(
      [this](auto const& state) { logger.Telemeterize(state); });

  joystick.RightBumper().WhileTrue(drivetrain.DriveAimingCommand(
      [this]() { return -joystick.GetLeftY(); },
      [this]() { return -joystick.GetLeftX(); }, MaxSpeed * 0.6));
  joystick.A().WhileTrue(
      frc2::cmd::Either(complexcommand.ShootWithFeederCommand(),
                        frc2::cmd::None(),
                        [this] { return shooterSub.IsAutoPitchValid(); }));
  joystick.A().OnFalse(complexcommand.StopShootWithFeederCommand());

  joystick.POVDown().WhileTrue(complexcommand.FollowAndShootCommand2(frc::Pose2d{units::meter_t{1.8}, units::meter_t{4.034},
                  frc::Rotation2d{units::degree_t{0.0}}}));
  joystick.POVDown().OnFalse(complexcommand.StopShootWithFeederCommand());

  // X閿? 鍦伴潰intake灞曞紑骞跺惛鍙?
  joystick.Y().OnTrue(
      frc2::cmd::Either(
          complexcommand.GroundintakeresetCommand(),
          complexcommand.GroundintakeprepareCommand(),
          [this] { return ground_intake_prepared_; })
          .AndThen(frc2::cmd::RunOnce(
              [this] { ground_intake_prepared_ = !ground_intake_prepared_; })));

  // B閿? intake杈呭姪鍔ㄤ綔(鐭椂鍙嶈浆)鍚庡洖鍒皃repare鐘舵€?
  joystick.B().OnTrue(
      complexcommand.GroundintakeassistCommand()
          .AndThen(frc2::cmd::RunOnce(
              [this] { ground_intake_prepared_ = true; })));
  
  // Start閿細棰勮杞?
  joystick.Start().OnTrue(complexcommand.PreloadCommand());
  joystick.POVUp().OnTrue(complexcommand.FollowAndShootCommand(
      frc::Pose2d{units::meter_t{1.8}, units::meter_t{4.034},
                  frc::Rotation2d{units::degree_t{0.0}}}));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return autoChooser.GetSelected();
}
