#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandPtr.h>
#include <frc/geometry/Pose2d.h>
#include <functional>
#include "subsystems/CommandSwerveDrivetrain.h"
#include "subsystems/FeederSubsystem.h"
#include "subsystems/GroundIntakeSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/VisionSubsystem.h"
#include "subsystems/WeidaiSub.h"
#include "commands/AutoMoveOpen.h"
#include "commands/AutoMoveClosed.h"
#include "commands/AutoMoveCircle.h"

class ComplexCommand
    : public frc2::CommandHelper<frc2::Command, ComplexCommand> {
 public:
  explicit ComplexCommand(subsystems::CommandSwerveDrivetrain* driveSubsystem,
                          subsystems::VisionSubsystem* visionSubsystem,
                          subsystems::ShooterSubsystem* shooterSubsystem,
                          subsystems::FeederSubsystem* feederSubsystem,
                          subsystems::WeidaiSub* weidaiSub,
                          subsystems::GroundIntakeSubsystem* groundIntakeSubsystem);                                                       

  frc2::CommandPtr FollowPathCommand(frc::Pose2d targetPos);
  frc2::CommandPtr FollowPathCommand(
  std::vector<frc::Pose2d> const& targetPoses);
  frc2::CommandPtr MoveOnShoot(std::function<int()> supplier);
  frc2::CommandPtr AutoFollowPathCommand(frc::Pose2d targetPos, double maxspeed,
                                         double maxacc);
  frc2::CommandPtr GroundintakeprepareCommand();
  frc2::CommandPtr GroundintakeassistCommand();
  frc2::CommandPtr GroundintakeresetCommand();
  frc2::CommandPtr PreloadCommand();
  frc2::CommandPtr ShootWithFeederCommand();
  frc2::CommandPtr StopShootWithFeederCommand();

  frc2::CommandPtr FollowAndShootCommand(frc::Pose2d targetPos);
  frc2::CommandPtr FollowAndShootCommand2(frc::Pose2d targetPos);

  frc2::CommandPtr autoFollow(frc::Pose2d targetPos);
  frc2::CommandPtr PassBump(bool atOppo);
  frc2::CommandPtr PassTrench(bool atOppo);
  frc2::CommandPtr GoToClimb();

  frc2::CommandPtr assistPassing();
  frc2::CommandPtr StartFeederCommand();

  frc2::CommandPtr StartStorageCommand();
  frc2::CommandPtr CloseStorageCommand();

 private:
  subsystems::CommandSwerveDrivetrain* m_drivesubsystem;
  subsystems::VisionSubsystem* m_visionSubsystem;
  subsystems::ShooterSubsystem* m_shooterSubsystem;
  subsystems::FeederSubsystem* m_feederSubsystem;
  subsystems::WeidaiSub* m_weidaiSub;
  subsystems::GroundIntakeSubsystem* m_groundIntakeSubsystem;

};
