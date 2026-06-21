#pragma once

#include <functional>

#include <frc/Timer.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "shooting/ShotTable.h"
#include "subsystems/FeederSubsystem.h"
#include "subsystems/ShooterSubsystem.h"

class ShootWithTableCommand
    : public frc2::CommandHelper<frc2::Command, ShootWithTableCommand> {
 public:
  ShootWithTableCommand(
      subsystems::ShooterSubsystem* shooter,
      subsystems::FeederSubsystem* feeder,
      std::function<shooting::ShotSetpoint()> setpointSupplier,
      std::function<bool()> alignmentReadySupplier,
      bool forcePitchHome = false);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

 private:
  subsystems::ShooterSubsystem* shooter_;
  subsystems::FeederSubsystem* feeder_;
  std::function<shooting::ShotSetpoint()> setpoint_supplier_;
  std::function<bool()> alignment_ready_supplier_;
  bool force_pitch_home_;
  bool shot_timer_started_ = false;
  bool feeding_ = false;
  frc::Timer shot_timer_;
};
