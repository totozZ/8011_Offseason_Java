// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc8011/LinearServo.h"

#include <algorithm>

using namespace units::literals;

LinearServo::LinearServo(int channel, double length_mm, double speed_mm_per_s)
    : frc::Servo(channel),
      m_length_mm(length_mm) {
  (void)speed_mm_per_s;
  SetBounds(2_ms, 1.8_ms, 1.5_ms, 1.2_ms, 1.0_ms);
}

void LinearServo::SetPositionMm(double setpoint_mm) {
  if (m_length_mm <= 0.0) {
    return;
  }
  m_setPos_mm = std::clamp(setpoint_mm, 0.0, m_length_mm);
  SetSpeed((m_setPos_mm / m_length_mm) * 2.0 - 1.0);
}
