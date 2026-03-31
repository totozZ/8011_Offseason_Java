// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"
#include "frc8011/Wayimotor.h"
#include "subsystems/ExampleSubsystem.h"

namespace subsystems {

class WeidaiSub : public ExampleSubsystem {
 public:
  explicit WeidaiSub(frc2::CommandXboxController& joystick_)
      : ExampleSubsystem(joystick_) {
    Initialization();
  }

  void Periodic() override;

  void SetStorageNormPosition(double norm);
  frc2::CommandPtr SetStorageNormPositionCommandPtr(double norm);
  double GetStorageCurrent();
  double GetStorageNormPosition();
  bool IsStorageResetDone() const { return storage_reset_flag_; }

 private:
  void Initialization();
  void StorageReset();

  Wayimotor storage_{WeidaiConstants::StorageMotorID, kCANBus};
  bool storage_reset_flag_ = false;
  int storage_reset_counter_ = 0;
};

}  // namespace subsystems
