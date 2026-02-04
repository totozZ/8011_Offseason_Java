// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/time.h>

#include <frc/Servo.h>

class LinearServo : public frc::Servo {
 public:
  LinearServo(int channel, double length_mm, double speed_mm_per_s);

  void SetPositionMm(double setpoint_mm);
  void UpdateCurPos();
  double GetPositionMm() const;
  bool IsFinished() const;

 private:
  double m_speed_mm_per_s = 0.0;
  double m_length_mm = 0.0;
  double m_setPos_mm = 0.0;
  double m_curPos_mm = 0.0;
  units::second_t m_lastTime_s{0.0};
};
