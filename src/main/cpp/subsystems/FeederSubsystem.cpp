// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/FeederSubsystem.h"
#include <cmath>
#include <frc/DriverStation.h>
#include <frc/Timer.h>

using namespace subsystems;

void FeederSubsystem::Initialization()
{
  // ==========================================
  // 新版电机配置: Backward Feeder
  // ==========================================
  configs::TalonFXConfiguration backward_feeder_config{};
  backward_feeder_config.MotorOutput.Inverted = 1;
  configs::Slot0Configs &backward_feeder_slot0 = backward_feeder_config.Slot0;
  backward_feeder_slot0.kG = 0.;         
  backward_feeder_slot0.kS = 0.12;       
  backward_feeder_slot0.kV = 0.12;       
  backward_feeder_slot0.kA = 0;          
  backward_feeder_slot0.kP = 0.03;       
  backward_feeder_slot0.kI = 0;          
  backward_feeder_slot0.kD = 0.;         
  backward_feeder_slot0.GravityType = 0; 

  ctre::phoenix::StatusCode backward_feeder_status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i) {
    backward_feeder_status = backward_feeder_.Applyconfig(backward_feeder_config);
    if (backward_feeder_status.IsOK()) break;
  }
  backward_feeder_.setinvert(-1);
  backward_feeder_.setCurrent_Speed(0.2);

  // ==========================================
  // 新版电机配置: Upward Feeder
  // ==========================================
  configs::TalonFXConfiguration upward_feeder_config{};
  upward_feeder_config.MotorOutput.Inverted = 0;
  configs::Slot0Configs &upward_feeder_slot0 = upward_feeder_config.Slot0;
  upward_feeder_slot0.kG = 0.;         
  upward_feeder_slot0.kS = 10;          
  upward_feeder_slot0.kV = 0;        
  upward_feeder_slot0.kA = 0;          
  upward_feeder_slot0.kP = 9;        
  upward_feeder_slot0.kI = 1.2;        
  upward_feeder_slot0.kD = 0.;         
  upward_feeder_slot0.GravityType = 0; 

  upward_feeder_config.CurrentLimits.SupplyCurrentLimit = 40_A;
  upward_feeder_config.CurrentLimits.SupplyCurrentLimitEnable = true;

  ctre::phoenix::StatusCode upward_feeder_status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i) {
    upward_feeder_status = upward_feeder_.Applyconfig(upward_feeder_config);
    if (upward_feeder_status.IsOK()) break;
  }
  upward_feeder_.setinvert(-1);
  upward_feeder_.SetStatusSignalUpdateFrequency(50_Hz);
  upward_feeder_.setPhysicalLimits(0, 360000, 100, 40); 
  upward_feeder_.setgearRatio(1.4); 

  configs::TalonFXConfiguration storage_config{};
  storage_config.MotorOutput.NeutralMode = 1;
  storage_config.MotorOutput.Inverted = 0;
  configs::Slot0Configs& storage_slot0 = storage_config.Slot0;
  storage_slot0.kG = 0.;
  storage_slot0.kS = 0.;
  storage_slot0.kV = 0.;
  storage_slot0.kA = 0.;
  storage_slot0.kP = 1.5;
  storage_slot0.kI = 0;
  storage_slot0.kD = 0.;
  storage_slot0.GravityType = 0;

  configs::MotionMagicConfigs& mm_storage = storage_config.MotionMagic;
  mm_storage.MotionMagicCruiseVelocity = 0_tps;
  mm_storage.MotionMagicExpo_kV = 0.11_V / 1_tps;
  mm_storage.MotionMagicExpo_kA = 0.07_V / 1_tr_per_s_sq;

  storage_config.CurrentLimits.SupplyCurrentLimit = 20_A;
  storage_config.CurrentLimits.SupplyCurrentLimitEnable = true;

  ctre::phoenix::StatusCode storage_status =
      ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i) {
    storage_status = storage_.Applyconfig(storage_config);
    if (storage_status.IsOK()) {
      break;
    }
  }
  storage_.setgearRatio(22.5);
  storage_.setinvert(-1);
  storage_.setPhysicalLimits(0, 5.7 / storage_.Getdata().gearRatio,
                             120 / storage_.Getdata().gearRatio, 40);
  storage_.setCurrent_Speed(0.07);
}

void FeederSubsystem::Periodic()
{
  if (!storage_reset_flag_) {
    StorageReset();
  }
  else{
  }

  backward_feeder_.Control();
  upward_feeder_.Control();
  storage_.Control();

  // frc::SmartDashboard::PutBoolean("feeder/storage_reset_done",
  //                                 storage_reset_flag_);
  // frc::SmartDashboard::PutNumber("feeder/storage_norm_pos",
  //                                storage_.GetNormalizedPosition());
  // frc::SmartDashboard::PutNumber("feeder/storage_current",
  //                                storage_.GetCurrent());


  //   frc::SmartDashboard::PutNumber("storage__wayiconfig.offset",
  //                                  storage_.Getdata().offset);
  //   frc::SmartDashboard::PutNumber("storage__currentPosition",
  //                                  storage_.GetPosition());
  //   frc::SmartDashboard::PutNumber("storage__targetnormalizedPosition",
  //                                  storage_.Getdata().normalizedPosition);

}

// ==========================================
// 基础控制接口 (完美融合新旧逻辑)
// ==========================================

void FeederSubsystem::SetBackwardFeederCurrent(double current, double max_abs_duty_cycle) {
  backward_feeder_.setCurrent_Speed(max_abs_duty_cycle);
  backward_feeder_.setcurrent(current);
}

void FeederSubsystem::SetUpwardFeederVelocity(double velocity) {
  upward_feeder_.setvelocitytorquecurrent(velocity);
}

void FeederSubsystem::SetUpwardFeederCurrent(double current, double max_abs_duty_cycle) {
  upward_feeder_.setCurrent_Speed(max_abs_duty_cycle);
  upward_feeder_.setcurrent(current);
}

void FeederSubsystem::setduty(double backward_duty, double upward_duty) {
  backward_feeder_.setNormalizedDutyCircle(backward_duty);
  upward_feeder_.setNormalizedDutyCircle(upward_duty);
}

void FeederSubsystem::SetBackwardFeederVelocity(double duty) {
  // 旧版代码里这个方法其实传的是 duty
  backward_feeder_.setNormalizedDutyCircle(duty); 
}

void FeederSubsystem::SetBackwardFeederDuty(double duty) {
  backward_feeder_.setNormalizedDutyCircle(duty);
}

void FeederSubsystem::SetUpwardDuty(double duty) {
  upward_feeder_.setNormalizedDutyCircle(duty);
}

void FeederSubsystem::SetStorageNormPosition(double norm) {
  storage_.setNormalizedMotionPosition(norm);
}

void FeederSubsystem::Stop() {
  // 使用新版安全的 coast 停止方式
  backward_feeder_.setcoast();
  upward_feeder_.setcoast();
  //setduty(0,0);
  // 重置 Combo 状态机
  upper_velocity_reached_once_ = false;
}

// ==========================================
// 状态获取接口 (获取新版电机的实时数据)
// ==========================================

double FeederSubsystem::GetUpwardFeederVelocity() {
  return upward_feeder_.GetVelocity();
}

double FeederSubsystem::GetBackwardFeederVelocity() {
  return backward_feeder_.GetVelocity();
}

double FeederSubsystem::GetUpwardFeederCurrent() {
  return upward_feeder_.GetCurrent();
}

double FeederSubsystem::GetStorageCurrent() {
  return storage_.GetCurrent();
}

double FeederSubsystem::GetStorageNormPosition() {
  return storage_.GetNormalizedPosition();
}

// ==========================================
// Command 包装层
// ==========================================

frc2::CommandPtr FeederSubsystem::StopCommandPtr() {
  return this->RunOnce([this] { Stop(); });
}

frc2::CommandPtr FeederSubsystem::SetBackwardFeederCurrentCommandPtr(double current, double max_abs_duty_cycle) {
  return frc2::cmd::RunOnce([this, current, max_abs_duty_cycle] {
    SetBackwardFeederCurrent(current, max_abs_duty_cycle);
  });
}

frc2::CommandPtr FeederSubsystem::SetUpwardFeederCurrentCommandPtr(double current, double max_abs_duty_cycle) {
  return frc2::cmd::RunOnce([this, current, max_abs_duty_cycle] {
    SetUpwardFeederCurrent(current, max_abs_duty_cycle);
  });
}

frc2::CommandPtr FeederSubsystem::setdutyCommandPtr(double backward_duty, double upward_duty) {
  return frc2::cmd::RunOnce([this, backward_duty, upward_duty] {
    setduty(backward_duty, upward_duty);
  });
}

frc2::CommandPtr FeederSubsystem::SetBackwardFeederDutyCommandPtr(double duty) {
  return frc2::cmd::RunOnce([this, duty] { SetBackwardFeederDuty(duty); });
}

frc2::CommandPtr FeederSubsystem::SetBackwardFeederVelocityCommandPtr(double velocity) {
  return frc2::cmd::RunOnce([this, velocity] { SetBackwardFeederVelocity(velocity); });
}

frc2::CommandPtr FeederSubsystem::SetUpwardFeederVelocityCommandPtr(double velocity) {
  return frc2::cmd::RunOnce([this, velocity] { SetUpwardFeederVelocity(velocity); });
}

frc2::CommandPtr FeederSubsystem::SetStorageNormPositionCommandPtr(double norm) {
  return this->RunOnce([this, norm] { SetStorageNormPosition(norm); });
}


frc2::CommandPtr FeederSubsystem::HoldFeederVelocityCommandPtr(
    double backward_velocity, double upward_velocity) {
  return this->Run([this, backward_velocity, upward_velocity] {
    SetBackwardFeederVelocity(backward_velocity);
    SetUpwardFeederVelocity(upward_velocity);
  });
}
// ==========================================
// 战术特定逻辑 (保留自旧版)
// ==========================================

void FeederSubsystem::SetPreload() {
  SetUpwardFeederCurrent(13.0, 0.25);
  SetBackwardFeederDuty(0.3);
}

frc2::CommandPtr FeederSubsystem::SetPreloadCommandPtr() {
  return this->RunOnce([this] { SetPreload(); });
}

void FeederSubsystem::SetUpperVelocityBANGBANG(double velocity) {
  double current_velocity = GetUpwardFeederVelocity();
  if (current_velocity < velocity) {
    SetUpwardDuty(1.0); // 没达到目标前满功率加速
  } else {
    SetUpwardDuty(0.0); // 达到目标后切断动力
  }
}

frc2::CommandPtr FeederSubsystem::SetUpperVelocityBANGBANGCommandPtr(double velocity) {
  return frc2::cmd::RunOnce([this, velocity] { SetUpperVelocityBANGBANG(velocity); });
}

void FeederSubsystem::SetUpperVelocitycombo(double velocity) {
  if (velocity <= 0.0) {
    Stop(); 
    return;
  }

  // 检测目标速度是否改变
  if (std::abs(velocity - combo_target_velocity_) > 1e-6) {
    combo_target_velocity_ = velocity;
    upper_velocity_reached_once_ = false;
  }

  const double current_velocity = GetUpwardFeederVelocity();

  if (!upper_velocity_reached_once_) {
    SetUpperVelocityBANGBANG(velocity); // 第一阶段：BangBang 狂暴起步
    if (current_velocity >= (velocity - kUpperVelocityReachTolerance)) {
      upper_velocity_reached_once_ = true;
    }
    return;
  }

  // 第二阶段：到达一次后，切入平滑的 FOC 闭环稳速
  SetUpwardFeederVelocity(velocity);
}

void FeederSubsystem::StorageReset() {
  // 初始复位仅在 Enable 状态推进
  if (!frc::DriverStation::IsEnabled()) {
    storage_reset_counter_ = 0;
    storage_reset_flag_ = false;
    return;
  }

  storage_.setcurrent(-9);
  if (storage_.GetCurrent() < -7) {
    ++storage_reset_counter_;
  } else {
    storage_reset_counter_ = 0;
  }

  if (storage_reset_counter_ >= 3) {
    storage_.Reset(storage_.GetAbsPosition());
    storage_reset_flag_ = true;
  }
}
