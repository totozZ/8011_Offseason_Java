// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/GroundIntakeSubsystem.h"

#include <frc/DriverStation.h>
#include <frc/Timer.h>

using namespace subsystems;

void GroundIntakeSubsystem::Initialization() {
  // ==========================================
  // 1. 配置主控滚筒 (右滚筒)
  // ==========================================
  configs::TalonFXConfiguration intake_roller_right_config{};
  intake_roller_right_config.MotorOutput.Inverted = 0;
  configs::Slot0Configs& intake_roller_right_slot0 = intake_roller_right_config.Slot0;
  intake_roller_right_slot0.kG = 0.;
  intake_roller_right_slot0.kS = 9.;
  intake_roller_right_slot0.kV = 0.;
  intake_roller_right_slot0.kA = 0.;
  intake_roller_right_slot0.kP = 2;
  intake_roller_right_slot0.kI = 0;
  intake_roller_right_slot0.kD = 0;
  intake_roller_right_slot0.GravityType = 0;

  ctre::phoenix::StatusCode intake_roller_right_status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i) {
    intake_roller_right_status = intake_roller_right_.Applyconfig(intake_roller_right_config);
    if (intake_roller_right_status.IsOK()) break;
  }

  // ==========================================
  // 2. 配置从动滚筒 (左滚筒跟随右滚筒)
  // ==========================================
  intake_roller_left_.setfollowControl(intake_roller_right_.Getdata().deviceId, true);
  intake_roller_left_.Control();

  // ==========================================
  // 3. 配置 Pitch 俯仰电机
  // ==========================================
  configs::TalonFXConfiguration intake_pitch_config{};
  intake_pitch_config.MotorOutput.NeutralMode = 1; // Brake 刹车模式
  intake_pitch_config.MotorOutput.Inverted = 0;
  configs::Slot0Configs& intake_pitch_slot0 = intake_pitch_config.Slot0;
  intake_pitch_slot0.kG = 0.;
  intake_pitch_slot0.kS = 0.;
  intake_pitch_slot0.kV = 0.;
  intake_pitch_slot0.kA = 0.;
  intake_pitch_slot0.kP = 1.1;
  intake_pitch_slot0.kI = 0;
  intake_pitch_slot0.kD = 0.;
  intake_pitch_slot0.GravityType = 0;

  /* Configure Motion Magic Expo */
  configs::MotionMagicConfigs& mm_pitch = intake_pitch_config.MotionMagic;
  mm_pitch.MotionMagicCruiseVelocity = 0_tps; 
  mm_pitch.MotionMagicExpo_kV = 0.07_V / 1_tps;           
  mm_pitch.MotionMagicExpo_kA = 0.02_V / 1_tr_per_s_sq;  

  ctre::phoenix::StatusCode intake_pitch_status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i) {
    intake_pitch_status = intake_pitch_.Applyconfig(intake_pitch_config);
    if (intake_pitch_status.IsOK()) break;
  }
  
  // 应用新版机械参数
  intake_pitch_.setgearRatio(18.67);
  intake_pitch_.setinvert(1);
  intake_pitch_.setPhysicalLimits(0, 7 / intake_pitch_.Getdata().gearRatio,
                                  120 / intake_pitch_.Getdata().gearRatio, 40);
  intake_pitch_.setCurrent_Speed(0.1);
}

void GroundIntakeSubsystem::Periodic() {
  static double last_debug_publish_s = -1.0;
  static constexpr double kDebugPublishPeriodS = 0.1;
  const double t_start_s = frc::Timer::GetFPGATimestamp().value();
  const bool publish_debug =
      (last_debug_publish_s < 0.0) ||
      ((t_start_s - last_debug_publish_s) >= kDebugPublishPeriodS);
  if (publish_debug) {
    last_debug_publish_s = t_start_s;
  }

  // 维持通信心跳
  intake_roller_right_.Control();
  intake_roller_left_.Control(); // 左侧跟随电机也需要 Control
  intake_pitch_.Control();

  if (pitch_reset_flag_ == 0) {
    GroundIntakeReset();
  }

  if (publish_debug) {
    frc::SmartDashboard::PutNumber("ground_intake_pitch_currentnormalizedPosition",
                                   intake_pitch_.GetNormalizedPosition());
    frc::SmartDashboard::PutNumber("ground_intake_pitch_wayiconfig.offset",
                                   intake_pitch_.Getdata().offset);
    frc::SmartDashboard::PutNumber("ground_intake_pitch_currentPosition",
                                   intake_pitch_.GetPosition());
  }
}

// ==========================================
// 保留的各种控制接口
// ==========================================

void GroundIntakeSubsystem::SetRollerVelocity(double velocity) {
  intake_roller_right_.setvelocitytorquecurrent(velocity);
}

frc2::CommandPtr GroundIntakeSubsystem::SetRollerVelocityCommandPtr(double velocity) {
  return this->RunOnce([this, velocity] { SetRollerVelocity(velocity); });
}

void GroundIntakeSubsystem::SetPitchPosition(double position) {
  intake_pitch_.setmode(12);
  intake_pitch_.Getdata().targetPosition = position;
}

void GroundIntakeSubsystem::SetRollerDutyCycle(double dutyCycle) {
  intake_roller_right_.setNormalizedDutyCircle(dutyCycle);
}

frc2::CommandPtr GroundIntakeSubsystem::SetRollerDutyCycleCommandPtr(double dutyCycle) {
  return this->RunOnce([this, dutyCycle] { SetRollerDutyCycle(dutyCycle); });
}

void GroundIntakeSubsystem::Stop() { 
  intake_roller_right_.setcoast(); 
  intake_roller_left_.setcoast(); // 确保两个电机一起释放
}

frc2::CommandPtr GroundIntakeSubsystem::StopCommandPtr() {
  return this->RunOnce([this] { Stop(); });
}

void GroundIntakeSubsystem::SetPitchNormPosition(double norm) {
  intake_pitch_.setNormalizedMotionPosition(norm);
}

frc2::CommandPtr GroundIntakeSubsystem::SetPitchNormPositionCommandPtr(double norm) {
  return this->RunOnce([this, norm] { SetPitchNormPosition(norm); });
}

double GroundIntakeSubsystem::GetPitchCurrent() {
  return intake_pitch_.GetCurrent();
}

double GroundIntakeSubsystem::GetPitchNormPosition() {
  return intake_pitch_.GetNormalizedPosition();
}

void GroundIntakeSubsystem::BrakePitch() { 
  intake_pitch_.setbrake(); 
}

frc2::CommandPtr GroundIntakeSubsystem::BrakePitchCommandPtr() {
  return this->RunOnce([this] { BrakePitch(); });
}

void GroundIntakeSubsystem::GroundIntakeReset() {
  if (!frc::DriverStation::IsEnabled()) {
    pitch_reset_counter_ = 0;
    frc::SmartDashboard::PutBoolean("pitch_reset_flag", pitch_reset_flag_);
    return;
  }

  // 采用新版归零电流，因为减速比变小了，需要更大的电流来确保触底
  intake_pitch_.setcurrent(-60);

  frc::SmartDashboard::PutNumber("intake_pitch Current", intake_pitch_.GetCurrent());
  frc::SmartDashboard::PutNumber("intake_pitch Position", intake_pitch_.GetPosition());

  if (intake_pitch_.GetCurrent() < -56) {
    ++pitch_reset_counter_;
  } else {
    pitch_reset_counter_ = 0;
  }

  // 新版采用连续 3 次达标判定，更稳健
  if (pitch_reset_counter_ >= 3) {
    intake_pitch_.Reset(intake_pitch_.GetAbsPosition());
    pitch_reset_flag_ = true;
  }
}