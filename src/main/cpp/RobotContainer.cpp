#include "RobotContainer.h"

#include <algorithm>
#include <cmath>
#include <numbers>

#include <frc/DriverStation.h>
#include <frc/RobotController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>

#include "commands/PassBallCommand.h"
#include "commands/RealTimeAimDrive.h"
#include "commands/ShootWithTableCommand.h"
#include "commands/intakeNextToHub.h"
#include "commands/intakeNextToSide.h"
#include "commands/intakeNextToWall.h"

using namespace units::literals;

RobotContainer::RobotContainer()
    : visionSub(&drivetrain, nullptr),
      complexcommand(&drivetrain, &feederSub, &groundIntakeSub) {
  rotation_controller_.EnableContinuousInput(-180, 180);
  frc::RobotController::SetBrownoutVoltage(6.5_V);

  ConfigureNamedCommands();
  auto_chooser_ = pathplanner::AutoBuilder::buildAutoChooser("Do Nothing");
  auto_chooser_.SetDefaultOption("Do Nothing (Safe)",
                                 do_nothing_command_.get());
  frc::SmartDashboard::PutData("Auto Mode", &auto_chooser_);
  ConfigureBindings();
}

void RobotContainer::UpdateDriverPerspective() {
  const auto alliance = frc::DriverStation::GetAlliance();
  if (!alliance.has_value()) {
    return;
  }
  drivetrain.SetOperatorPerspectiveForward(
      alliance.value() == frc::DriverStation::Alliance::kRed
          ? frc::Rotation2d{180_deg}
          : frc::Rotation2d{0_deg});
}

void RobotContainer::ConfigureNamedCommands() {
  pathplanner::NamedCommands::registerCommand(
      "Intake", complexcommand.GroundintakeprepareCommand());
  pathplanner::NamedCommands::registerCommand(
      "ShootTower",
      ShootWithTableCommand(
          &shooterSub, &feederSub,
          [] { return shooting::ShotTable::Tower(); }, [] { return true; })
          .ToPtr()
          .WithTimeout(3_s));
  pathplanner::NamedCommands::registerCommand(
      "StopAll",
      frc2::cmd::RunOnce(
          [this] {
            feederSub.Stop();
            shooterSub.SetIdle();
            groundIntakeSub.Stop();
          },
          {&shooterSub, &feederSub, &groundIntakeSub}));
}

frc2::CommandPtr RobotContainer::MakeHubShootCommand() {
  return frc2::cmd::Sequence(
      complexcommand.GroundintakeresetCommand(),
      frc2::cmd::RunOnce([this] { ground_intake_prepared_ = false; }),
      frc2::cmd::Parallel(
          RealTimeAimDrive(&drivetrain).ToPtr(),
          ShootWithTableCommand(
              &shooterSub, &feederSub,
              [this] {
                return shooting::ShotTable::Hub(
                    units::meter_t{drivetrain.GetDistanceToHub()});
              },
              [this] { return drivetrain.SOMangleDiff <= 3.0; })
              .ToPtr(),
          complexcommand.GroundintakeassistCommand().Repeatedly()));
}

frc2::CommandPtr RobotContainer::MakePassShootCommand() {
  return frc2::cmd::Sequence(
      complexcommand.GroundintakeresetCommand(),
      frc2::cmd::RunOnce([this] { ground_intake_prepared_ = false; }),
      frc2::cmd::Parallel(
          PassBallCommand(&drivetrain, &shooterSub, &feederSub).ToPtr(),
          complexcommand.GroundintakeassistCommand().Repeatedly()));
}

frc2::CommandPtr RobotContainer::MakeFallbackShootCommand(
    shooting::ShotSetpoint setpoint) {
  return frc2::cmd::Sequence(
      complexcommand.GroundintakeresetCommand(),
      frc2::cmd::RunOnce([this] { ground_intake_prepared_ = false; }),
      frc2::cmd::Parallel(
          frc2::cmd::Run(
              [this] {
                drivetrain.SetControl(
                    swerve::requests::SwerveDriveBrake{});
              },
              {&drivetrain}),
          ShootWithTableCommand(
              &shooterSub, &feederSub, [setpoint] { return setpoint; },
              [] { return true; }, true)
              .ToPtr()));
}

bool RobotContainer::IsHubShootingRegion() {
  const auto alliance = frc::DriverStation::GetAlliance();
  if (!alliance.has_value()) {
    return false;
  }
  const auto side = alliance.value() == frc::DriverStation::Alliance::kRed
                        ? shooting::AllianceSide::kRed
                        : shooting::AllianceSide::kBlue;
  return shooting::ShotTable::IsHubRegion(side,
                                           drivetrain.GetState().Pose.X());
}

bool RobotContainer::UseOpponentRoute() {
  const auto alliance = frc::DriverStation::GetAlliance();
  if (!alliance.has_value()) {
    return false;
  }
  const units::meter_t x = drivetrain.GetState().Pose.X();
  const units::meter_t midfield = FieldConstants::kFieldLength / 2.0;
  return alliance.value() == frc::DriverStation::Alliance::kRed
             ? x >= midfield
             : x <= midfield;
}

void RobotContainer::ConfigureBindings() {
  drivetrain.SetDefaultCommand(drivetrain.ApplyRequest([this]() -> auto&& {
    const double raw_vx = -joystick.GetLeftY();
    const double raw_vy = -joystick.GetLeftX();
    auto vx = raw_vx * max_speed_ * OperatorConstants::SpeedRate;
    auto vy = raw_vy * max_speed_ * OperatorConstants::SpeedRate;

    if (drive_like_tank_) {
      double rotation_rate = 0.0;
      if (std::hypot(raw_vx, raw_vy) > 0.05) {
        const auto alliance = frc::DriverStation::GetAlliance();
        const bool is_red =
            alliance.value_or(frc::DriverStation::Alliance::kBlue) ==
            frc::DriverStation::Alliance::kRed;
        const double field_x = is_red ? -raw_vx : raw_vx;
        const double field_y = is_red ? -raw_vy : raw_vy;
        const double target_degrees =
            std::atan2(field_y, field_x) / std::numbers::pi * 180.0;
        rotation_rate = rotation_controller_.Calculate(
            drivetrain.GetState().Pose.Rotation().Degrees().value(),
            target_degrees);
        const double maximum =
            (max_angular_rate_ * OperatorConstants::AngularSpeedRate /
             std::numbers::pi * 180)
                .value();
        rotation_rate = std::clamp(rotation_rate, -maximum, maximum);
      }
      return drive_closed_.WithVelocityX(vx)
          .WithVelocityY(vy)
          .WithRotationalRate(units::degrees_per_second_t{rotation_rate});
    }

    return drive_closed_.WithVelocityX(vx)
        .WithVelocityY(vy)
        .WithRotationalRate(-joystick.GetRightX() * max_angular_rate_ *
                            OperatorConstants::AngularSpeedRate);
  }));

  joystick.Start().OnTrue(frc2::cmd::Sequence(
      complexcommand.GroundintakeresetCommand(),
      groundIntakeSub.SetPitchNormPositionCommandPtr(0.07)));

  auto main_shot = frc2::cmd::Either(MakeHubShootCommand(),
                                     MakePassShootCommand(),
                                     [this] { return IsHubShootingRegion(); })
                       .OnlyIf([] {
                         const bool known =
                             frc::DriverStation::GetAlliance().has_value();
                         if (!known) {
                           frc::SmartDashboard::PutString(
                               "Shooting/BlockedReason", "Alliance unknown");
                         }
                         return known;
                       });
  joystick.RightTrigger()
      .WhileTrue(std::move(main_shot))
      .OnFalse(complexcommand.GroundintakeresetCommand());

  drivetrain.RegisterTelemetry(
      [this](auto const& state) { logger_.Telemeterize(state); });

  joystick.RightBumper().WhileTrue(frc2::cmd::Sequence(
      complexcommand.GroundintakeresetCommand(),
      frc2::cmd::Either(complexcommand.PassBump(false),
                        complexcommand.PassBump(true),
                        [this] { return UseOpponentRoute(); })));

  joystick.POVUp()
      .WhileTrue(MakeFallbackShootCommand(
          shooting::ShotTable::ZeroPitchFallback()))
      .OnFalse(complexcommand.GroundintakeresetCommand());

  joystick.POVDown()
      .WhileTrue(MakeFallbackShootCommand(shooting::ShotTable::Tower()))
      .OnFalse(complexcommand.GroundintakeresetCommand());

  joystick.POVRight()
      .OnTrue(complexcommand.GroundintakeantiCommand())
      .OnFalse(complexcommand.GroundintakeresetCommand());

  joystick.A()
      .WhileTrue(intakeNextToSide(&drivetrain, &groundIntakeSub, &joystick,
                                  true)
                     .ToPtr())
      .OnFalse(frc2::cmd::RunOnce(
          [this] { ground_intake_prepared_ = false; }));

  joystick.LeftTrigger()
      .WhileTrue(frc2::cmd::Sequence(
          frc2::cmd::RunOnce(
              [this] { ground_intake_prepared_ = true; }),
          complexcommand.GroundintakeprepareCommand()))
      .OnFalse(frc2::cmd::Sequence(
          complexcommand.GroundintakeresetCommand(),
          frc2::cmd::RunOnce(
              [this] { ground_intake_prepared_ = false; })));

  joystick.Y().WhileTrue(frc2::cmd::Sequence(
      frc2::cmd::RunOnce(
          [this] {
            shooterSub.SetIdle();
            feederSub.Stop();
          },
          {&shooterSub, &feederSub}),
      frc2::cmd::Either(complexcommand.PassTrench(false),
                        complexcommand.PassTrench(true),
                        [this] { return UseOpponentRoute(); })));

  joystick.B()
      .WhileTrue(frc2::cmd::Either(
          intakeNextToHub(&drivetrain, &groundIntakeSub, &joystick, true)
              .ToPtr(),
          intakeNextToWall(&drivetrain, &groundIntakeSub, &joystick, true)
              .ToPtr(),
          [this] {
            const auto x = drivetrain.GetState().Pose.X();
            return x > FieldConstants::kHubPassBlueBoundaryX &&
                   x < FieldConstants::kHubPassRedBoundaryX;
          }))
      .OnFalse(frc2::cmd::RunOnce(
          [this] { ground_intake_prepared_ = false; }));

  joystick.X()
      .WhileTrue(frc2::cmd::Either(
          intakeNextToHub(&drivetrain, &groundIntakeSub, &joystick, false)
              .ToPtr(),
          intakeNextToWall(&drivetrain, &groundIntakeSub, &joystick, false)
              .ToPtr(),
          [this] {
            const auto x = drivetrain.GetState().Pose.X();
            return x > FieldConstants::kHubPassBlueBoundaryX &&
                   x < FieldConstants::kHubPassRedBoundaryX;
          }))
      .OnFalse(frc2::cmd::RunOnce(
          [this] { ground_intake_prepared_ = false; }));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return auto_chooser_.GetSelected();
}
