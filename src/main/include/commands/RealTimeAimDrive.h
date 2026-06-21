#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <ctre/phoenix6/swerve/SwerveRequest.hpp>

#include "subsystems/CommandSwerveDrivetrain.h"

class RealTimeAimDrive
    : public frc2::CommandHelper<frc2::Command, RealTimeAimDrive> {
 public:
  explicit RealTimeAimDrive(
      subsystems::CommandSwerveDrivetrain* drive,
      units::degree_t shooterFacingOffset = 180_deg);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

 private:
  subsystems::CommandSwerveDrivetrain* drive_;
  units::degree_t shooter_facing_offset_;
  units::meters_per_second_t max_speed_ = TunerConstants::kSpeedAt12Volts;
  swerve::requests::FieldCentricFacingAngle facing_request_{};
  swerve::requests::SwerveDriveBrake brake_request_{};
};
