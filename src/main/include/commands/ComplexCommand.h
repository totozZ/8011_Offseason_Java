#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandPtr.h>
#include <frc/geometry/Pose2d.h>

#include "subsystems/CommandSwerveDrivetrain.h"
#include "subsystems/VisionSubsystem.h"

class ComplexCommand
    : public frc2::CommandHelper<frc2::Command, ComplexCommand> {
 public:
  explicit ComplexCommand(subsystems::CommandSwerveDrivetrain* driveSubsystem,
                          subsystems::VisionSubsystem* visionSubsystem);

  frc2::CommandPtr FollowPathCommand(frc::Pose2d targetPos);
  frc2::CommandPtr FollowPathCommand(
      std::vector<frc::Pose2d> const& targetPoses);
  frc2::CommandPtr AutoFollowPathCommand(frc::Pose2d targetPos, double maxspeed,
                                         double maxacc);

  frc2::CommandPtr autoFollow(frc::Pose2d targetPos);

 private:
  subsystems::CommandSwerveDrivetrain* m_drivesubsystem;
  subsystems::VisionSubsystem* m_visionSubsystem;
};
