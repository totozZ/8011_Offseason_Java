// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Timer.h>
#include <frc2/command/sysid/SysIdRoutine.h>

#include "Constants.h"
#include "frc8011/Wayimotor.h"
#include "shooting/ShotTable.h"
#include "subsystems/ExampleSubsystem.h"

namespace subsystems {

class ShooterSubsystem : public ExampleSubsystem {
 public:
  enum class PitchHomeState { kUnhomed, kHoming, kHomed, kFault };

  ShooterSubsystem() { Initialization(); }

  void Periodic() override;

  void ApplyShotSetpoint(const shooting::ShotSetpoint& setpoint);
  void SetIdle();
  void Stop();
  frc2::CommandPtr StopCommandPtr();

  void BeginPitchHoming();
  PitchHomeState GetPitchHomeState() const { return pitch_home_state_; }
  bool IsPitchHomed() const {
    return pitch_home_state_ == PitchHomeState::kHomed;
  }

  double GetShootVelocity();
  double GetPitchAngle();
  bool IsFlywheelReady(units::turns_per_second_t tolerance);
  bool IsPitchReady(units::degree_t tolerance);
  const shooting::ShotSetpoint& GetTargetSetpoint() const {
    return target_setpoint_;
  }

  // SysId 方法
  frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction) {
    return m_sysIdRoutine.Quasistatic(direction);
  }
  frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction) {
    return m_sysIdRoutine.Dynamic(direction);
  }
 private:
  void Initialization();
  void RunPitchHoming();
  void SetShootPitchAngle(units::degree_t target_angle);

  Wayimotor shooter_left_down_{ShooterConstants::ShooterLeftDownMotorID, kCANBus};
  Wayimotor shooter_left_up_{ShooterConstants::ShooterLeftUpMotorID, kCANBus};
  Wayimotor shooter_right_up_{ShooterConstants::ShooterRightUpMotorID, kCANBus};
  Wayimotor shooter_right_down_{ShooterConstants::ShooterRightDownMotorID, kCANBus};
  Wayimotor shooter_pitch_{ShooterConstants::ShooterPitchMotorID, kCANBus};

  frc2::sysid::SysIdRoutine m_sysIdRoutine{
      frc2::sysid::Config{std::nullopt, 4_V, std::nullopt, nullptr},
      frc2::sysid::Mechanism{
          [this](units::volt_t output) { shooter_right_up_.setVoltage(output); },
          [this](frc::sysid::SysIdRoutineLog* log) {
            log->Motor("shooter")
                .voltage(shooter_right_up_.Getmotor().GetMotorVoltage().GetValue())
                .position(shooter_right_down_.Getmotor().GetPosition().GetValue())
                .velocity(shooter_left_up_.Getmotor().GetVelocity().GetValue());
          },
          this}};

  shooting::ShotSetpoint target_setpoint_{15_tps, 0_tps, 0.2_deg};
  bool shot_active_ = false;
  PitchHomeState pitch_home_state_ = PitchHomeState::kUnhomed;
  int pitch_home_current_counter_ = 0;
  frc::Timer pitch_home_timer_;

  static constexpr units::turns_per_second_t kIdleSpeed = 15_tps;
  static constexpr units::second_t kPitchHomeTimeout = 2_s;
};

}  // namespace subsystems
