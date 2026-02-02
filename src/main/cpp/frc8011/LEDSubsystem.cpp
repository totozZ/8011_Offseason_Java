// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc8011/LEDSubsystem.h"

void LEDSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  /* if the selection for slot 0 changes, change animations */

  switch (m_anim0State) {
    case AnimationType::Red:

      m_candle.SetControl(
          controls::SolidColor{kSlot0StartIdx, kSlot0EndIdx}.WithColor(kRed));
      m_candle.SetControl(
          controls::SolidColor{kSlot1StartIdx, kSlot1EndIdx}.WithColor(kRed));

      break;
    case AnimationType::Blue:
      m_candle.SetControl(
          controls::SolidColor{kSlot0StartIdx, kSlot0EndIdx}.WithColor(kBlue));
      m_candle.SetControl(
          controls::SolidColor{kSlot1StartIdx, kSlot1EndIdx}.WithColor(kBlue));
      break;
    case AnimationType::Green:
      m_candle.SetControl(
          controls::SolidColor{kSlot0StartIdx, kSlot0EndIdx}.WithColor(kGreen));
      m_candle.SetControl(
          controls::SolidColor{kSlot1StartIdx, kSlot1EndIdx}.WithColor(kGreen));
      break;

    case AnimationType::Purple:
      m_candle.SetControl(
          controls::SolidColor{kSlot0StartIdx, kSlot0EndIdx}.WithColor(
              kPurple));
      m_candle.SetControl(
          controls::SolidColor{kSlot1StartIdx, kSlot1EndIdx}.WithColor(
              kPurple));
      break;

    default:
      break;
      // }
  }
}

void LEDSubsystem::LED_Init() {
  /* Configure CANdle */
  configs::CANdleConfiguration cfg{};
  /* set the LED strip type and brightness */
  cfg.LED.StripType = signals::StripTypeValue::RGB;
  cfg.LED.BrightnessScalar = 1.0;
  /* disable status LED when being controlled */
  cfg.CANdleFeatures.StatusLedWhenActive =
      signals::StatusLedWhenActiveValue::Disabled;

  m_candle.GetConfigurator().Apply(cfg);

  /* clear all previous animations */
  for (int i = 0; i < 8; ++i) {
    m_candle.SetControl(controls::EmptyAnimation{i});
  }
  /* set the onboard LEDs to a solid color */
  m_candle.SetControl(controls::SolidColor{0, 3}.WithColor(kRed));
  m_candle.SetControl(controls::SolidColor{4, 7}.WithColor(kRed));

  m_candle.SetControl(
      controls::SolidColor{kSlot0StartIdx, kSlot0EndIdx}.WithColor(kRed));
  m_candle.SetControl(
      controls::SolidColor{kSlot1StartIdx, kSlot1EndIdx}.WithColor(kRed));
}

