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

frc2::CommandPtr ComplexCommand::VisionLeftFollow() {
  return FollowPathCommand(m_visionSubsystem->GetTarget_posleft());
}

frc2::CommandPtr ComplexCommand::VisionRightFollow() {
  return FollowPathCommand(m_visionSubsystem->GetTarget_posright());
}

frc2::CommandPtr ComplexCommand::VisionMiddleFollow() {
  return FollowPathCommand(m_visionSubsystem->GetTarget_posmiddle());
}

frc2::CommandPtr ComplexCommand::AutoVisionLeftFollow() {
  return AutoFollowPathCommand(m_visionSubsystem->GetTarget_posleft(), 3, 3);
}

frc2::CommandPtr ComplexCommand::AutoVisionRightFollow() {
  return AutoFollowPathCommand(m_visionSubsystem->GetTarget_posright(), 3, 3);
}

frc2::CommandPtr ComplexCommand::autoFollow(frc::Pose2d targetPos) {
  return FollowPathCommand(targetPos);
}
