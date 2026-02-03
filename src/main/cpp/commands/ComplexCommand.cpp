#include "commands/ComplexCommand.h"

ComplexCommand::ComplexCommand(
    subsystems::CommandSwerveDrivetrain* driveSubsystem,
    subsystems::VisionSubsystem* visionSubsystem)
    : m_drivesubsystem(driveSubsystem), m_visionSubsystem(visionSubsystem) {}

frc2::CommandPtr ComplexCommand::FollowPathCommand(frc::Pose2d targetPos) {
  return m_drivesubsystem->followPathCommand(targetPos);
}

frc2::CommandPtr ComplexCommand::FollowPathCommand(
    std::vector<frc::Pose2d> const& targetPoses) {
  return m_drivesubsystem->followPathCommand(targetPoses);
}

frc2::CommandPtr ComplexCommand::AutoFollowPathCommand(
    frc::Pose2d targetPos, double maxspeed, double maxacc) {
  return m_drivesubsystem->AutofollowPathCommand(targetPos, maxspeed, maxacc);
}

frc2::CommandPtr ComplexCommand::autoFollow(frc::Pose2d targetPos) {
  return FollowPathCommand(targetPos);
}
