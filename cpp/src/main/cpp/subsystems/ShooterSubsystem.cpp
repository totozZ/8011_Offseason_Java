#include "subsystems/ShooterSubsystem.h"

#include <algorithm>
#include <cmath>

#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace subsystems;
using namespace units::literals;

void ShooterSubsystem::Initialization() {
  configs::TalonFXConfiguration flywheel_config{};
  flywheel_config.MotorOutput.Inverted = 0;
  flywheel_config.MotorOutput.NeutralMode = 0;
  flywheel_config.CurrentLimits.StatorCurrentLimit = 120_A;
  flywheel_config.CurrentLimits.StatorCurrentLimitEnable = true;
  flywheel_config.CurrentLimits.SupplyCurrentLimit = 50_A;
  flywheel_config.CurrentLimits.SupplyCurrentLimitEnable = true;

  configs::Slot0Configs& flywheel_slot0 = flywheel_config.Slot0;
  flywheel_slot0.kS = 4.875;
  flywheel_slot0.kP = 9;

  shooter_right_up_.Applyconfig(flywheel_config);
  shooter_right_up_.setinvert(-1);

  shooter_left_down_.setfollowControl(shooter_right_up_.Getdata().deviceId,
                                       true);
  shooter_left_up_.setfollowControl(shooter_right_up_.Getdata().deviceId,
                                     true);
  shooter_right_down_.setfollowControl(shooter_right_up_.Getdata().deviceId,
                                        false);
  shooter_left_down_.Control();
  shooter_left_up_.Control();
  shooter_right_down_.Control();

  configs::TalonFXConfiguration pitch_config{};
  pitch_config.MotorOutput.Inverted = 0;
  pitch_config.MotorOutput.NeutralMode = 1;
  pitch_config.CurrentLimits.StatorCurrentLimit = 60_A;
  pitch_config.CurrentLimits.StatorCurrentLimitEnable = true;
  pitch_config.CurrentLimits.SupplyCurrentLimit = 20_A;
  pitch_config.CurrentLimits.SupplyCurrentLimitEnable = true;

  configs::Slot0Configs& pitch_slot0 = pitch_config.Slot0;
  pitch_slot0.kP = 7;
  pitch_slot0.kI = 0.8;
  pitch_slot0.kD = 0.02;

  configs::MotionMagicConfigs& pitch_motion_magic = pitch_config.MotionMagic;
  pitch_motion_magic.MotionMagicCruiseVelocity = 0_tps;
  pitch_motion_magic.MotionMagicExpo_kV = 0.1_V / 1_tps;
  pitch_motion_magic.MotionMagicExpo_kA = 0.01_V / 1_tr_per_s_sq;

  shooter_pitch_.Applyconfig(pitch_config);
  shooter_pitch_.setgearRatio(18.67);
  shooter_pitch_.setinvert(-1);
  shooter_pitch_.setPhysicalLimits(
      0, ShooterConstants::kPitchMotorMaxposition / 18.67, 120 / 18.67, 40);
  shooter_pitch_.setCurrent_Speed(0.1);
  shooter_pitch_.SetStatusSignalUpdateFrequency(50_Hz);
}

void ShooterSubsystem::Periodic() {
  shooter_right_up_.ReceiveVelocity();

  if (!frc::DriverStation::IsEnabled()) {
    shooter_right_up_.setcoast();
    shooter_right_up_.Control();
    shooter_pitch_.setbrake();
    shooter_pitch_.Control();
    pitch_home_timer_.Stop();
    pitch_home_current_counter_ = 0;
    pitch_home_state_ = PitchHomeState::kUnhomed;
    return;
  }

  if (pitch_home_state_ == PitchHomeState::kUnhomed) {
    BeginPitchHoming();
  }

  if (pitch_home_state_ == PitchHomeState::kHoming) {
    RunPitchHoming();
  } else if (pitch_home_state_ == PitchHomeState::kHomed) {
    SetShootPitchAngle(target_setpoint_.pitch);
    shooter_pitch_.Control();
  } else {
    shooter_pitch_.setbrake();
    shooter_pitch_.Control();
  }

  shooter_right_up_.setvelocitytorquecurrent(
      shot_active_ ? target_setpoint_.flywheel.value() : kIdleSpeed.value());
  shooter_right_up_.Control();

  frc::SmartDashboard::PutNumber("Shooting/FlywheelTargetRps",
                                 shot_active_ ? target_setpoint_.flywheel.value()
                                              : kIdleSpeed.value());
  frc::SmartDashboard::PutNumber("Shooting/FlywheelActualRps",
                                 GetShootVelocity());
  frc::SmartDashboard::PutNumber("Shooting/PitchTargetDeg",
                                 target_setpoint_.pitch.value());
  frc::SmartDashboard::PutNumber("Shooting/PitchActualDeg", GetPitchAngle());
  frc::SmartDashboard::PutNumber(
      "Shooting/PitchHomeState", static_cast<int>(pitch_home_state_));
}

void ShooterSubsystem::ApplyShotSetpoint(
    const shooting::ShotSetpoint& setpoint) {
  target_setpoint_ = setpoint;
  shot_active_ = true;
}

void ShooterSubsystem::SetIdle() {
  shot_active_ = false;
  target_setpoint_.flywheel = kIdleSpeed;
  target_setpoint_.pitch = 0.2_deg;
}

void ShooterSubsystem::Stop() {
  shot_active_ = false;
  shooter_right_up_.setcoast();
  shooter_right_up_.Control();
}

frc2::CommandPtr ShooterSubsystem::StopCommandPtr() {
  return this->RunOnce([this] { Stop(); });
}

void ShooterSubsystem::BeginPitchHoming() {
  if (!frc::DriverStation::IsEnabled()) {
    pitch_home_state_ = PitchHomeState::kUnhomed;
    return;
  }
  if (frc::RobotBase::IsSimulation()) {
    target_setpoint_.pitch = 0_deg;
    pitch_home_state_ = PitchHomeState::kHomed;
    return;
  }
  pitch_home_state_ = PitchHomeState::kHoming;
  pitch_home_current_counter_ = 0;
  pitch_home_timer_.Restart();
}

void ShooterSubsystem::RunPitchHoming() {
  shooter_pitch_.setcurrent(-10.0);
  shooter_pitch_.Control();

  if (shooter_pitch_.GetCurrent() < -8.0) {
    ++pitch_home_current_counter_;
  } else {
    pitch_home_current_counter_ = 0;
  }

  if (pitch_home_current_counter_ >= 3) {
    shooter_pitch_.Reset(shooter_pitch_.GetAbsPosition());
    shooter_pitch_.setbrake();
    shooter_pitch_.Control();
    target_setpoint_.pitch = 0_deg;
    pitch_home_state_ = PitchHomeState::kHomed;
    pitch_home_timer_.Stop();
    return;
  }

  if (pitch_home_timer_.HasElapsed(kPitchHomeTimeout)) {
    shooter_pitch_.setbrake();
    shooter_pitch_.Control();
    pitch_home_state_ = PitchHomeState::kFault;
    pitch_home_timer_.Stop();
  }
}

double ShooterSubsystem::GetShootVelocity() {
  return shooter_right_up_.GetVelocity();
}

double ShooterSubsystem::GetPitchAngle() {
  const double normalized = shooter_pitch_.GetNormalizedPosition();
  return normalized *
         (ShooterConstants::kMaxPitchAngle - ShooterConstants::kMinPitchAngle) +
         ShooterConstants::kMinPitchAngle;
}

bool ShooterSubsystem::IsFlywheelReady(
    units::turns_per_second_t tolerance) {
  const double actual = shooter_right_up_.GetVelocity();
  return shot_active_ &&
         std::abs(actual - target_setpoint_.flywheel.value()) <=
             tolerance.value();
}

bool ShooterSubsystem::IsPitchReady(units::degree_t tolerance) {
  return IsPitchHomed() &&
         std::abs(GetPitchAngle() - target_setpoint_.pitch.value()) <=
             tolerance.value();
}

void ShooterSubsystem::SetShootPitchAngle(units::degree_t target_angle) {
  const double clamped =
      std::clamp(target_angle.value(), ShooterConstants::kMinPitchAngle,
                 ShooterConstants::kMaxPitchAngle);
  const double normalized =
      (clamped - ShooterConstants::kMinPitchAngle) /
      (ShooterConstants::kMaxPitchAngle - ShooterConstants::kMinPitchAngle);
  shooter_pitch_.setNormalizedMotionPosition(normalized);
}
