#include "commands/ComplexCommand.h"

#include <cmath>

#include <frc/DriverStation.h>
#include <frc2/command/Commands.h>
#include <frc2/command/DeferredCommand.h>

#include "Constants.h"
#include "commands/AutoMoveCircle.h"
#include "commands/AutoMoveClosed.h"
#include "commands/AutoMoveOpen.h"

using namespace units::literals;

ComplexCommand::ComplexCommand(
    subsystems::CommandSwerveDrivetrain* driveSubsystem,
    subsystems::FeederSubsystem* feederSubsystem,
    subsystems::GroundIntakeSubsystem* groundIntakeSubsystem)
    : drive_(driveSubsystem),
      feeder_(feederSubsystem),
      ground_intake_(groundIntakeSubsystem) {}

frc2::CommandPtr ComplexCommand::GroundintakeprepareCommand() {
  return frc2::cmd::Sequence(
      ground_intake_->SetRollerDutyCycleCommandPtr(0.8),
      ground_intake_->SetPitchNormPositionCommandPtr(
          GroundIntakeConstants::PitchNormPosition));
}

frc2::CommandPtr ComplexCommand::GroundintakeassistCommand() {
  return frc2::cmd::Sequence(
      ground_intake_->SetRollerVelocityCommandPtr(30),
      ground_intake_->SetPitchNormPositionCommandPtr(0.6),
      frc2::cmd::Wait(0.4_s),
      ground_intake_->SetPitchNormPositionCommandPtr(0.90),
      frc2::cmd::Wait(0.4_s));
}

frc2::CommandPtr ComplexCommand::GroundintakeantiCommand() {
  return ground_intake_->SetRollerVelocityCommandPtr(-20.0);
}

frc2::CommandPtr ComplexCommand::GroundintakeresetCommand() {
  return frc2::cmd::Parallel(ground_intake_->StopCommandPtr(),
                             feeder_->SetBackwardFeederDutyCommandPtr(0.0));
}

frc2::CommandPtr ComplexCommand::PassBump(bool atOppo) {
  auto* drive = drive_;
  return frc2::DeferredCommand(
             [drive, atOppo]() {
               double inner_x = 3.43;
               constexpr double inner_y = 5.5;
               double inner_rotation = 45;
               double outer_x = 6.0;
               constexpr double outer_y = 5.5;
               double outer_rotation = 45;

               const auto alliance = frc::DriverStation::GetAlliance();
               const double x = drive->GetState().Pose.X().value();
               const double y = drive->GetState().Pose.Y().value();
               const double target_speed =
                   0.7 * TunerConstants::kSpeedAt12Volts.value();
               const bool invert_y = y <= FieldConstants::kFieldWidth.value() / 2.0;
               bool invert_alliance =
                   alliance.has_value() &&
                   alliance.value() == frc::DriverStation::Alliance::kRed;
               if (atOppo) {
                 invert_alliance = !invert_alliance;
               }

               if (x > FieldConstants::kHubPassBlueBoundaryX.value() &&
                   x < FieldConstants::kHubPassRedBoundaryX.value()) {
                 inner_x -= 0.5;
                 inner_rotation = 135;
                 outer_rotation = 135;
                 return frc2::cmd::Sequence(
                     AutoMoveOpen(drive, outer_x, outer_y, outer_rotation,
                                  target_speed, invert_alliance, invert_y)
                         .ToPtr(),
                     AutoMoveClosed(drive, inner_x, inner_y, inner_rotation,
                                    target_speed, invert_alliance, invert_y)
                         .ToPtr());
               }

               outer_x += 0.5;
               return frc2::cmd::Sequence(
                   AutoMoveOpen(drive, inner_x, inner_y, inner_rotation,
                                target_speed, invert_alliance, invert_y)
                       .ToPtr(),
                   AutoMoveClosed(drive, outer_x, outer_y, outer_rotation,
                                  target_speed, invert_alliance, invert_y)
                       .ToPtr());
             },
             {drive})
      .ToPtr();
}

frc2::CommandPtr ComplexCommand::PassTrench(bool atOppo) {
  auto* drive = drive_;
  return frc2::DeferredCommand(
             [drive, atOppo]() {
               double inner_x = 3.1;
               double inner_y = 7.4;
               double inner_rotation = 0;
               double outer_x = 6.0;
               double outer_y = 7.4;
               double outer_rotation = 0;

               const double current_rotation =
                   drive->GetState().Pose.Rotation().Degrees().value();
               if (current_rotation > 90 || current_rotation < -90) {
                 inner_rotation = 180;
                 outer_rotation = 180;
               }

               const auto alliance = frc::DriverStation::GetAlliance();
               const double x = drive->GetState().Pose.X().value();
               const double y = drive->GetState().Pose.Y().value();
               const double target_speed =
                   0.7 * TunerConstants::kSpeedAt12Volts.value();
               const bool invert_y = y <= FieldConstants::kFieldWidth.value() / 2.0;
               bool invert_alliance =
                   alliance.has_value() &&
                   alliance.value() == frc::DriverStation::Alliance::kRed;
               if (atOppo) {
                 invert_alliance = !invert_alliance;
               }
               if (invert_alliance) {
                 inner_rotation -= 180;
                 outer_rotation -= 180;
               }

               const double virtual_x =
                   invert_alliance ? FieldConstants::kFieldLength.value() - x : x;
               const double virtual_y =
                   invert_y ? FieldConstants::kFieldWidth.value() - y : y;
               constexpr double offset_distance = 0.5;

               if (x > FieldConstants::kHubPassBlueBoundaryX.value() &&
                   x < FieldConstants::kHubPassRedBoundaryX.value()) {
                 outer_x += 0.2;
                 const frc::Rotation2d reverse{180_deg};
                 const frc::Rotation2d approach{units::radian_t{std::atan2(
                     outer_y - virtual_y, outer_x - virtual_x)}};
                 if (std::abs((reverse - approach).Degrees().value()) >= 35) {
                   outer_x -= offset_distance * approach.Cos();
                   outer_y -= offset_distance * approach.Sin();
                   return frc2::cmd::Sequence(
                       AutoMoveOpen(drive, outer_x, outer_y, outer_rotation,
                                    target_speed, invert_alliance, invert_y)
                           .ToPtr(),
                       AutoMoveCircle(drive, outer_x - 0.6, inner_y - 0.6,
                                      0.3, 90, true, target_speed, false,
                                      outer_rotation, false, invert_alliance,
                                      invert_y, 0)
                           .ToPtr(),
                       AutoMoveClosed(drive, inner_x - 1.0, inner_y,
                                      inner_rotation, target_speed,
                                      invert_alliance, invert_y)
                           .ToPtr());
                 }
                 return frc2::cmd::Sequence(
                     AutoMoveOpen(drive, outer_x, outer_y, outer_rotation,
                                  target_speed, invert_alliance, invert_y)
                         .ToPtr(),
                     AutoMoveClosed(drive, inner_x - 1.0, inner_y,
                                    inner_rotation, target_speed,
                                    invert_alliance, invert_y)
                         .ToPtr());
               }

               inner_x -= 0.2;
               const frc::Rotation2d forward{0_deg};
               const frc::Rotation2d approach{units::radian_t{std::atan2(
                   inner_y - virtual_y, inner_x - virtual_x)}};
               if (std::abs((forward - approach).Degrees().value()) >= 35) {
                 inner_x -= offset_distance * approach.Cos();
                 inner_y -= offset_distance * approach.Sin();
                 return frc2::cmd::Sequence(
                     AutoMoveOpen(drive, inner_x, inner_y, inner_rotation,
                                  target_speed, invert_alliance, invert_y)
                         .ToPtr(),
                     AutoMoveCircle(drive, inner_x + 0.6, outer_y - 0.6,
                                    0.2, 90, false, target_speed, false,
                                    outer_rotation, false, invert_alliance,
                                    invert_y, 0)
                         .ToPtr(),
                     AutoMoveClosed(drive, outer_x + 1.0, outer_y,
                                    outer_rotation, target_speed,
                                    invert_alliance, invert_y)
                         .ToPtr());
               }
               return frc2::cmd::Sequence(
                   AutoMoveOpen(drive, inner_x, inner_y, inner_rotation,
                                target_speed, invert_alliance, invert_y)
                       .ToPtr(),
                   AutoMoveClosed(drive, outer_x + 1.0, outer_y,
                                  outer_rotation, target_speed,
                                  invert_alliance, invert_y)
                       .ToPtr());
             },
             {drive})
      .ToPtr();
}
