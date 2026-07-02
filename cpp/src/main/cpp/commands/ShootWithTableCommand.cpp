#include "commands/ShootWithTableCommand.h"

#include <cmath>

#include <frc/smartdashboard/SmartDashboard.h>

using namespace units::literals;

ShootWithTableCommand::ShootWithTableCommand(
    subsystems::ShooterSubsystem* shooter,
    subsystems::FeederSubsystem* feeder,
    std::function<shooting::ShotSetpoint()> setpointSupplier,
    std::function<bool()> alignmentReadySupplier, bool forcePitchHome)
    : shooter_(shooter),
      feeder_(feeder),
      setpoint_supplier_(std::move(setpointSupplier)),
      alignment_ready_supplier_(std::move(alignmentReadySupplier)),
      force_pitch_home_(forcePitchHome) {
  AddRequirements({shooter, feeder});
}

void ShootWithTableCommand::Initialize() {
  shot_timer_.Stop();
  shot_timer_.Reset();
  shot_timer_started_ = false;
  feeding_ = false;
  feeder_->Stop();

  if (force_pitch_home_ ||
      shooter_->GetPitchHomeState() ==
          subsystems::ShooterSubsystem::PitchHomeState::kUnhomed ||
      shooter_->GetPitchHomeState() ==
          subsystems::ShooterSubsystem::PitchHomeState::kFault) {
    shooter_->BeginPitchHoming();
  }
}

void ShootWithTableCommand::Execute() {
  if (!shooter_->IsPitchHomed()) {
    feeder_->SetBackwardFeederDuty(0.0);
    feeder_->SetUpwardDuty(0.0);
    return;
  }

  const shooting::ShotSetpoint setpoint = setpoint_supplier_();
  shooter_->ApplyShotSetpoint(setpoint);
  feeder_->SetUpwardFeederVelocity(setpoint.upperFeeder.value());

  if (!shot_timer_started_) {
    shot_timer_.Restart();
    shot_timer_started_ = true;
  }

  const bool ready =
      shooter_->IsFlywheelReady(0.7_tps) &&
      shooter_->IsPitchReady(0.75_deg) &&
      std::abs(feeder_->GetUpwardFeederVelocity() -
               setpoint.upperFeeder.value()) <= 2.0 &&
      alignment_ready_supplier_();

  if (shooting::ShotTable::FeedAllowed(
          shooter_->IsPitchHomed(), ready,
          shot_timer_.HasElapsed(1.5_s))) {
    feeding_ = true;
  }
  feeder_->SetBackwardFeederDuty(feeding_ ? 1.0 : 0.0);

  frc::SmartDashboard::PutBoolean("Shooting/Ready", ready);
  frc::SmartDashboard::PutBoolean("Shooting/ForcedFeed",
                                  feeding_ && !ready);
  frc::SmartDashboard::PutBoolean("Shooting/Feeding", feeding_);
}

void ShootWithTableCommand::End(bool interrupted) {
  shot_timer_.Stop();
  feeder_->Stop();
  shooter_->SetIdle();
}

bool ShootWithTableCommand::IsFinished() { return false; }
