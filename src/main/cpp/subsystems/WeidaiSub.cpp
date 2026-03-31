// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/WeidaiSub.h"

#include <frc/DriverStation.h>

using namespace subsystems;

void WeidaiSub::Initialization() {
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

void WeidaiSub::Periodic() {
  if (!storage_reset_flag_) {
    StorageReset();
  }

  storage_.Control();
}

void WeidaiSub::SetStorageNormPosition(double norm) {
  storage_.setNormalizedMotionPosition(norm);
}

frc2::CommandPtr WeidaiSub::SetStorageNormPositionCommandPtr(double norm) {
  return this->RunOnce([this, norm] { SetStorageNormPosition(norm); });
}

double WeidaiSub::GetStorageCurrent() {
  return storage_.GetCurrent();
}

double WeidaiSub::GetStorageNormPosition() {
  return storage_.GetNormalizedPosition();
}

void WeidaiSub::StorageReset() {
  // Disabled 状态不推进reset。
  if (!frc::DriverStation::IsEnabled()) {
    storage_reset_counter_ = 0;
    storage_reset_flag_ = false;
    return;
  }

  storage_.setcurrent(-8);
  if (storage_.GetCurrent() < -6) {
    ++storage_reset_counter_;
  } else {
    storage_reset_counter_ = 0;
  }

  if (storage_reset_counter_ >= 3) {
    storage_.Reset(storage_.GetAbsPosition());
    storage_reset_flag_ = true;
  }
}
