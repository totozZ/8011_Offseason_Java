#include "commands/RealTimeAimDrive.h"

#include <cmath>

#include <frc/smartdashboard/SmartDashboard.h>

using namespace units::literals;

RealTimeAimDrive::RealTimeAimDrive(
    subsystems::CommandSwerveDrivetrain* drive,
    units::degree_t shooterFacingOffset)
    : drive_(drive), shooter_facing_offset_(shooterFacingOffset) {
  AddRequirements({drive});
}

void RealTimeAimDrive::Initialize() {
  drive_->SOMangleDiff = 180.0;
  facing_request_.WithHeadingPID(8, 0, 0.1)
      .WithDeadband(max_speed_ * 0.05)
      .WithRotationalDeadband(0.1_rad_per_s)
      .WithMaxAbsRotationalRate(4.71_rad_per_s)
      .WithDriveRequestType(swerve::DriveRequestType::Velocity)
      .WithSteerRequestType(swerve::SteerRequestType::Position);
}

void RealTimeAimDrive::Execute() {
  const frc::Pose2d pose = drive_->GetState().Pose;
  const frc::Translation2d hub = drive_->GetHubPosition();
  const auto target_direction =
      frc::Rotation2d{units::math::atan2(hub.Y() - pose.Y(),
                                         hub.X() - pose.X())} -
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

  frc::SmartDashboard::PutNumber("Shooting/AimErrorDeg", angle_error);
  frc::SmartDashboard::PutBoolean("Shooting/DrivetrainLocked",
                                  angle_error < 2.0);
}

void RealTimeAimDrive::End(bool interrupted) {
  drive_->SOMangleDiff = 180.0;
  const auto speeds = drive_->GetState().Speeds;
  drive_->SetControl(drive_->m_safeCoastRequest.WithVelocityX(speeds.vx)
                         .WithVelocityY(speeds.vy)
                         .WithRotationalRate(speeds.omega));
}

bool RealTimeAimDrive::IsFinished() { return false; }
