#pragma once

#include <frc2/command/CommandPtr.h>

#include "subsystems/CommandSwerveDrivetrain.h"
#include "subsystems/FeederSubsystem.h"
#include "subsystems/GroundIntakeSubsystem.h"

class ComplexCommand {
 public:
  ComplexCommand(subsystems::CommandSwerveDrivetrain* driveSubsystem,
                 subsystems::FeederSubsystem* feederSubsystem,
                 subsystems::GroundIntakeSubsystem* groundIntakeSubsystem);

  frc2::CommandPtr GroundintakeprepareCommand();
  frc2::CommandPtr GroundintakeantiCommand();
  frc2::CommandPtr GroundintakeassistCommand();
  frc2::CommandPtr GroundintakeresetCommand();

  // Driver-requested tactical movement. These are not autonomous routines.
  frc2::CommandPtr PassBump(bool atOppo);
  frc2::CommandPtr PassTrench(bool atOppo);

 private:
  subsystems::CommandSwerveDrivetrain* drive_;
  subsystems::FeederSubsystem* feeder_;
  subsystems::GroundIntakeSubsystem* ground_intake_;
};
