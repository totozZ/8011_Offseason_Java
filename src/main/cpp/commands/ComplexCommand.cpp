#include "commands/ComplexCommand.h"

#include <frc2/command/Commands.h>

ComplexCommand::ComplexCommand(
    subsystems::CommandSwerveDrivetrain* driveSubsystem,
    subsystems::VisionSubsystem* visionSubsystem,
    subsystems::ShooterSubsystem* shooterSubsystem,
    subsystems::FeederSubsystem* feederSubsystem,
    subsystems::GroundIntakeSubsystem* groundIntakeSubsystem)
    : m_drivesubsystem(driveSubsystem),
      m_visionSubsystem(visionSubsystem),
      m_shooterSubsystem(shooterSubsystem),
      m_feederSubsystem(feederSubsystem),
      m_groundIntakeSubsystem(groundIntakeSubsystem) {}

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

frc2::CommandPtr ComplexCommand::GroundintakeprepareCommand() {
  return frc2::cmd::Sequence(
          m_groundIntakeSubsystem->SetPitchNormPositionCommandPtr(1),
          m_groundIntakeSubsystem->SetRollerDutyCycleCommandPtr(0.6),
      m_feederSubsystem->SetBackwardFeederDutyCommandPtr(0.1)

        );
}

frc2::CommandPtr ComplexCommand::GroundintakeassistCommand() {
  return frc2::cmd::Sequence(
      m_groundIntakeSubsystem->SetPitchNormPositionCommandPtr(0.7),
      m_groundIntakeSubsystem->SetRollerDutyCycleCommandPtr(0.4),
      frc2::cmd::Wait(units::second_t{0.5}),
      GroundintakeprepareCommand());
}

frc2::CommandPtr ComplexCommand::GroundintakeresetCommand() {
  return m_groundIntakeSubsystem->SetRollerDutyCycleCommandPtr(0.0);
}

frc2::CommandPtr ComplexCommand::PreloadCommand() {
  static constexpr double kPreloadCurrentThresholdA = 12.5;
  static constexpr units::second_t kPostThresholdDelay = units::second_t{0.6};

  return frc2::cmd::Sequence(
      m_feederSubsystem->SetUpwardFeederCurrentCommandPtr(13.0, 0.25),
      m_feederSubsystem->SetBackwardFeederDutyCommandPtr(0.3),
      frc2::cmd::WaitUntil([this] {
        return m_feederSubsystem->GetUpwardFeederCurrent() >
               kPreloadCurrentThresholdA;
      }),
      frc2::cmd::Wait(kPostThresholdDelay),
      m_feederSubsystem->SetUpwardFeederCurrentCommandPtr(0.0, 0.25),
      m_feederSubsystem->SetBackwardFeederDutyCommandPtr(0.1));
}

frc2::CommandPtr ComplexCommand::autoFollow(frc::Pose2d targetPos) {
  return FollowPathCommand(targetPos);
}
