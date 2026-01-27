#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/CANBus.hpp>
#include <ctre/phoenix6/CANdle.hpp>

#include "frc/AddressableLED.h"

using namespace ctre::phoenix6;

enum {
  DISABLE,
  LOW_BATTERY,
  AIMED,
  READY_TO_SHOOT,
  LOSE_TARGET,
  AUTO_MODE,
};
class LEDSubsystem : public frc2::SubsystemBase {
 private:
  static constexpr int kLength = 54;
  // PWM port 9
  frc::AddressableLED *m_led;

  // Reuse the buffer
  std::array<frc::AddressableLED::LEDData, kLength> m_ledBuffer;

  // Store what the last hue of the first pixel is
  int firstPixelHue = 0;
  bool FlashState = 0;
  int TimePr_ms = 0;

  double led_state = 0;
  double led_brightness = 0;

  using RGBWColor = ctre::phoenix6::signals::RGBWColor;

  /* color can be constructed from RGBW, a WPILib Color/Color8Bit, HSV, or hex
   */
  static constexpr RGBWColor kWhite =
      RGBWColor{frc::Color::kWhite} * 0.5; /* half brightness */
  static constexpr RGBWColor kRed = RGBWColor::FromHSV(0_deg, 1, 1);  // 0° 是红
  static constexpr RGBWColor kGreen =
      RGBWColor::FromHSV(120_deg, 1, 1);  
  static constexpr RGBWColor kBlue =
      RGBWColor::FromHSV(240_deg, 1, 1);  
  static constexpr RGBWColor kPurple =
      RGBWColor::FromHSV(300_deg, 1, 1);  
  static constexpr RGBWColor kYellow =
      RGBWColor::FromHSV(60_deg, 1, 1);  

  static constexpr RGBWColor kCyan =
      RGBWColor::FromHSV(180_deg, 1, 1);  // 180° 是青

  // static constexpr RGBWColor kRed{255, 0, 0}; // 纯红色，R=255, G=0, B=0, W=0
  /*
   * Start and end index for LED animations.
   * 0-7 are onboard, 8-399 are an external strip.
   * CANdle supports 8 animation slots (0-7).
   */
  static constexpr int kSlot0StartIdx = 0;
  static constexpr int kSlot0EndIdx = 23;

  static constexpr int kSlot1StartIdx = 38;
  static constexpr int kSlot1EndIdx = 53;

  ctre::phoenix6::CANBus m_canBus{"rio"};
  ctre::phoenix6::hardware::CANdle m_candle{26, m_canBus};

 public:
  LEDSubsystem() { LED_Init(); }
  ~LEDSubsystem() {}

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  enum class AnimationType {
    None,
    Blue,
    Green,
    Red,
    Purple,
    ColorFlow,
    Fire,
    Larson,
    Rainbow,
    RgbFade,
    SingleFade,
    Strobe,
    Twinkle,
    TwinkleOff,
  };

  AnimationType m_anim0State{AnimationType::None};

  frc::SendableChooser<AnimationType> m_anim0Chooser;

  void LED_Init();

  void SetLEDState(AnimationType _state) { m_anim0State = _state; }
};
