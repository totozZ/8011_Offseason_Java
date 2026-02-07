// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <rev/ServoChannel.h>
#include <rev/ServoHub.h>

class LinearServo {
 public:
  LinearServo(rev::servohub::ServoHub& hub,
              rev::servohub::ServoChannel::ChannelId channel_id,
              double length_mm,
              double speed_mm_per_s);

  void SetPositionMm(double setpoint_mm);

 private:
  rev::servohub::ServoHub& hub_;
  rev::servohub::ServoChannel& channel_;
  double m_length_mm = 0.0;
  double m_setPos_mm = 0.0;

  static constexpr int kMinPulseUs = 1000;
  static constexpr int kCenterPulseUs = 1500;
  static constexpr int kMaxPulseUs = 2000;
};
