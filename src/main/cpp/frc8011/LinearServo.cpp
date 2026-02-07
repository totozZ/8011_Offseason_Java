// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc8011/LinearServo.h"

#include <algorithm>
#include <cmath>

#include <rev/config/ServoChannelConfig.h>
#include <rev/config/ServoHubConfig.h>

LinearServo::LinearServo(rev::servohub::ServoHub& hub,
                         rev::servohub::ServoChannel::ChannelId channel_id,
                         double length_mm,
                         double speed_mm_per_s)
    : hub_(hub),
      channel_(hub.GetServoChannel(channel_id)),
      m_length_mm(length_mm) {
  (void)speed_mm_per_s;
  rev::servohub::ServoHubConfig hub_config;
  rev::servohub::ServoChannelConfig channel_config(channel_id);
  channel_config.PulseRange(kMinPulseUs, kCenterPulseUs, kMaxPulseUs);
  hub_config.Apply(channel_id, channel_config);
  hub_.Configure(hub_config, rev::ResetMode::kNoResetSafeParameters);
  channel_.SetPowered(true);
  channel_.SetEnabled(true);
}

void LinearServo::SetPositionMm(double setpoint_mm) {
  if (m_length_mm <= 0.0) {
    return;
  }
  m_setPos_mm = std::clamp(setpoint_mm, 0.0, m_length_mm);
  double fraction = m_setPos_mm / m_length_mm;
  int pulse_us = static_cast<int>(
      std::lround(kMinPulseUs + fraction * (kMaxPulseUs - kMinPulseUs)));
  pulse_us = std::clamp(pulse_us, kMinPulseUs, kMaxPulseUs);
  channel_.SetPulseWidth(pulse_us);
}
