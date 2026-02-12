// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"
#include "frc8011/Wayimotor.h"
#include "subsystems/ExampleSubsystem.h"

namespace subsystems
{
class FeederSubsystem : public ExampleSubsystem
{
public:
  FeederSubsystem(frc2::CommandXboxController& joystick_) : ExampleSubsystem(joystick_)
  {
    Initialization();
  }
  void Periodic() override;

  frc2::CommandPtr SetBackwardFeederVelocityCommandPtr(double velocity);
  frc2::CommandPtr SetUpwardFeederVelocityCommandPtr(double velocity);
  void SetBackwardFeederVelocity(double velocity);
  void SetUpwardFeederVelocity(double velocity);
  void SetBackwardFeederDuty(double duty);
  void SetUpwardFeederCurrent(double current, double max_abs_duty_cycle);
  void SetUpwardDuty(double duty);
  double GetBackwardFeederVelocity();
  double GetUpwardFeederVelocity();
  double GetUpwardFeederCurrent();
  void Stop();

  frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction)
  {
    return m_sysIdRoutine.Quasistatic(direction);
  }
  frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction)
  {
    return m_sysIdRoutine.Dynamic(direction);
  }
  
  double GetComboTargetVelocity() const {
    return combo_target_velocity_;
  }
  void ChangeComboTargetVelocity(double delta) {
    combo_target_velocity_ = delta;
  }

  void SetUpperVelocityBANGBANG(double velocity);

  frc2::CommandPtr SetUpperVelocityBANGBANGCommandPtr(double velocity);
  frc2::CommandPtr SetBackwardFeederDutyCommandPtr(double duty);
  frc2::CommandPtr SetUpwardFeederCurrentCommandPtr(
      double current, double max_abs_duty_cycle);

  void SetUpperVelocitycombo(double velocity);
  void SetPreload();
  frc2::CommandPtr SetPreloadCommandPtr();

private:
  void Initialization();
  static constexpr double kUpperVelocityReachTolerance = 1.0;

  Wayimotor backward_feeder_{ FeederConstants::BackwardFeederMotorID, kCANBus };
  Wayimotor upward_feeder_{ FeederConstants::UpwardFeederMotorID, kCANBus };
  bool upper_velocity_reached_once_ = false;
  bool x_combo_was_active_ = false;
  double combo_target_velocity_ = 40.0;

  // SysId routine for feeder (测试backward_feeder)
  frc2::sysid::SysIdRoutine m_sysIdRoutine{ frc2::sysid::Config{ std::nullopt,  // 默认斜坡率 (1 V/s)
                                                                 4_V,           // 动态电压
                                                                 std::nullopt,  // 默认超时 (10 s)
                                                                 nullptr },
                                            frc2::sysid::Mechanism{
                                                [this](units::volt_t output) { backward_feeder_.setVoltage(output); },
                                                [this](frc::sysid::SysIdRoutineLog* log) {
                                                  log->Motor("feeder")
                                                      .voltage(backward_feeder_.Getmotor().GetMotorVoltage().GetValue())
                                                      .position(backward_feeder_.Getmotor().GetPosition().GetValue())
                                                      .velocity(backward_feeder_.Getmotor().GetVelocity().GetValue());
                                                },
                                                this } };
};
}  // namespace subsystems
