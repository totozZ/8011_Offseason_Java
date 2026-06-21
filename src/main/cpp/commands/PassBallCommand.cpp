#include "commands/PassBallCommand.h"

#include <cmath>

#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"
#include "shooting/ShotTable.h"

using namespace units::literals;

PassBallCommand::PassBallCommand(
    subsystems::CommandSwerveDrivetrain* drive,
    subsystems::ShooterSubsystem* shooter,
    subsystems::FeederSubsystem* feeder,
    units::degree_t shooterFacingOffset)
    : drive_(drive),
      shooter_(shooter),
      feeder_(feeder),
      shooter_facing_offset_(shooterFacingOffset) {
  AddRequirements({drive, shooter, feeder});
}

void PassBallCommand::Initialize() {
  timer_started_ = false;
  feeding_ = false;
  shot_timer_.Stop();
  shot_timer_.Reset();
  feeder_->Stop();

  if (shooter_->GetPitchHomeState() ==
          subsystems::ShooterSubsystem::PitchHomeState::kUnhomed ||
      shooter_->GetPitchHomeState() ==
          subsystems::ShooterSubsystem::PitchHomeState::kFault) {
    shooter_->BeginPitchHoming();
  }

  drive_->SOMangleDiff = 180.0;
  facing_request_.WithHeadingPID(9, 0, 0.1)
      .WithDeadband(max_speed_ * 0.05)
      .WithRotationalDeadband(0.1_rad_per_s)
      .WithMaxAbsRotationalRate(3.14_rad_per_s)
      .WithDriveRequestType(swerve::DriveRequestType::Velocity)
      .WithSteerRequestType(swerve::SteerRequestType::Position);
}

void PassBallCommand::Execute() {
  const auto alliance = frc::DriverStation::GetAlliance();
  if (!alliance.has_value()) {
    feeder_->Stop();
    shooter_->SetIdle();
    return;
  }

  const bool is_red = alliance.value() == frc::DriverStation::Alliance::kRed;
  const frc::Pose2d pose = drive_->GetState().Pose;
  frc::Translation2d target =
      pose.Y() > 4_m ? blue_left_target_ : blue_right_target_;
  if (is_red) {
    target = frc::Translation2d{FieldConstants::kFieldLength - target.X(),
                                target.Y()};
  }

  const auto target_direction =
      frc::Rotation2d{units::math::atan2(target.Y() - pose.Y(),
                                         target.X() - pose.X())} -
      frc::Rotation2d{shooter_facing_offset_};
  const double angle_error =
      std::abs((target_direction - pose.Rotation()).Degrees().value());
  drive_->SOMangleDiff = angle_error;

  if (angle_error < 2.0) {
    drive_->SetControl(brake_request_);
  } else {
    drive_->SetControl(facing_request_.WithVelocityX(0_mps)
                           .WithVelocityY(0_mps)
                           .WithTargetDirection(target_direction));
  }

  if (!shooter_->IsPitchHomed()) {
    feeder_->SetBackwardFeederDuty(0.0);
    feeder_->SetUpwardDuty(0.0);
    return;
  }

  const auto distance = pose.Translation().Distance(target);
  const auto setpoint = shooting::ShotTable::Pass(distance);
  shooter_->ApplyShotSetpoint(setpoint);
  feeder_->SetUpwardFeederVelocity(setpoint.upperFeeder.value());

  if (!timer_started_) {
    shot_timer_.Restart();
    timer_started_ = true;
  }

  const bool clear_of_hub = pose.Y() <= 3.5_m || pose.Y() >= 4.5_m;
  const bool ready = clear_of_hub && angle_error <= 3.0 &&
                     shooter_->IsFlywheelReady(0.7_tps) &&
                     shooter_->IsPitchReady(0.75_deg) &&
                     std::abs(feeder_->GetUpwardFeederVelocity() -
                              setpoint.upperFeeder.value()) <= 2.0;

  if (shooting::ShotTable::FeedAllowed(
          shooter_->IsPitchHomed(), ready,
          shot_timer_.HasElapsed(1.5_s))) {
    feeding_ = true;
  }
  feeder_->SetBackwardFeederDuty(feeding_ ? 1.0 : 0.0);

  frc::SmartDashboard::PutNumber("Shooting/PassDistanceM", distance.value());
  frc::SmartDashboard::PutBoolean("Shooting/PassReady", ready);
  frc::SmartDashboard::PutBoolean("Shooting/PassForcedFeed",
                                  feeding_ && !ready);
}

void PassBallCommand::End(bool interrupted) {
  shot_timer_.Stop();
  feeder_->Stop();
  shooter_->SetIdle();
  drive_->SOMangleDiff = 180.0;

  const auto speeds = drive_->GetState().Speeds;
  drive_->SetControl(drive_->m_safeCoastRequest.WithVelocityX(speeds.vx)
                         .WithVelocityY(speeds.vy)
                         .WithRotationalRate(speeds.omega));
}

bool PassBallCommand::IsFinished() { return false; }
