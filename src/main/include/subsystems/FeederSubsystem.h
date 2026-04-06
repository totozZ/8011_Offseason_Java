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

  // --- 基础控制与动作指令 (混合新老接口) ---
  frc2::CommandPtr StopCommandPtr();
  void Stop();
  
  void SetBackwardFeederCurrent(double current, double max_abs_duty_cycle);
  frc2::CommandPtr SetBackwardFeederCurrentCommandPtr(double current, double max_abs_duty_cycle);
  
  void SetUpwardFeederCurrent(double current, double max_abs_duty_cycle);
  frc2::CommandPtr SetUpwardFeederCurrentCommandPtr(double current, double max_abs_duty_cycle);

  void SetUpwardFeederVelocity(double velocity);
  frc2::CommandPtr SetUpwardFeederVelocityCommandPtr(double velocity);

  void SetBackwardFeederVelocity(double duty); 
  frc2::CommandPtr SetBackwardFeederVelocityCommandPtr(double velocity);

  void setduty(double backward_duty, double upward_duty);
  frc2::CommandPtr setdutyCommandPtr(double backward_duty, double upward_duty);

  void SetBackwardFeederDuty(double duty);
  frc2::CommandPtr SetBackwardFeederDutyCommandPtr(double duty);

  void SetUpwardDuty(double duty);

  frc2::CommandPtr HoldFeederVelocityCommandPtr(double backward_current,
                                              
                                                double upward_velocity);

  // --- 旧版战术逻辑接口 ---
  void SetPreload();
  frc2::CommandPtr SetPreloadCommandPtr();

  void SetUpperVelocityBANGBANG(double velocity);
  frc2::CommandPtr SetUpperVelocityBANGBANGCommandPtr(double velocity);
  
  void SetUpperVelocitycombo(double velocity);

  // --- 状态获取 (保留旧版接口) ---
  double GetBackwardFeederVelocity();
  double GetUpwardFeederVelocity();
  double GetUpwardFeederCurrent();
  
  double GetComboTargetVelocity() const { return combo_target_velocity_; }
  void ChangeComboTargetVelocity(double delta) { combo_target_velocity_ = delta; }

  // --- SysId ---
  frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction) {
    return m_sysIdRoutine.Quasistatic(direction);
  }
  frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction) {
    return m_sysIdRoutine.Dynamic(direction);
  }

private:
  void Initialization();

  Wayimotor backward_feeder_{ FeederConstants::BackwardFeederMotorID, kCANBus };
  Wayimotor upward_feeder_{ FeederConstants::UpwardFeederMotorID, kCANBus };

  // --- 战术状态变量 (从旧版移植) ---
  static constexpr double kUpperVelocityReachTolerance = 1.0;
  bool upper_velocity_reached_once_ = false;
  double combo_target_velocity_ =70;

  // SysId routine for feeder 
  frc2::sysid::SysIdRoutine m_sysIdRoutine{ frc2::sysid::Config{ std::nullopt, 
                                                                 4_V,           
                                                                 std::nullopt,  
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
