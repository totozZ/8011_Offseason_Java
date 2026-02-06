// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Servo.h>

class LinearServo : public frc::Servo {
 public:
  LinearServo(int channel, double length_mm, double speed_mm_per_s);

  void SetPositionMm(double setpoint_mm);

 private:
  double m_length_mm = 0.0;
  double m_setPos_mm = 0.0;
};
