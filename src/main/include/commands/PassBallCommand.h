#pragma once

#include <frc/Timer.h>
#include <frc/geometry/Translation2d.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/CommandSwerveDrivetrain.h"
#include "subsystems/FeederSubsystem.h"
#include "subsystems/ShooterSubsystem.h"

class PassBallCommand
    : public frc2::CommandHelper<frc2::Command, PassBallCommand> {
 public:
  PassBallCommand(subsystems::CommandSwerveDrivetrain* drive,
                  subsystems::ShooterSubsystem* shooter,
                  subsystems::FeederSubsystem* feeder,
                  units::degree_t shooterFacingOffset = 180_deg);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

 private:
  subsystems::CommandSwerveDrivetrain* drive_;
  subsystems::ShooterSubsystem* shooter_;
  subsystems::FeederSubsystem* feeder_;
  units::degree_t shooter_facing_offset_;
  bool timer_started_ = false;
  bool feeding_ = false;
  frc::Timer shot_timer_;

  units::meters_per_second_t max_speed_ = TunerConstants::kSpeedAt12Volts;
  swerve::requests::FieldCentricFacingAngle facing_request_{};
  swerve::requests::SwerveDriveBrake brake_request_{};

  const frc::Translation2d blue_left_target_{3.0_m, 5.5_m};
  const frc::Translation2d blue_right_target_{3.0_m, 2.5_m};
};
