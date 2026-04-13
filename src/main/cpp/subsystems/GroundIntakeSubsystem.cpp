// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/GroundIntakeSubsystem.h"

#include <frc/DriverStation.h>
#include <frc/Timer.h>

using namespace subsystems;

void GroundIntakeSubsystem::Initialization() {
  configs::TalonFXConfiguration intake_roller_right_config{};
  intake_roller_right_config.MotorOutput.Inverted = 0;
  configs::Slot0Configs& intake_roller_right_slot0 = intake_roller_right_config.Slot0;
  intake_roller_right_slot0.kG = 0.;
  intake_roller_right_slot0.kS = 9.;
  intake_roller_right_slot0.kV = 0.;
  intake_roller_right_slot0.kA = 0.;
  intake_roller_right_slot0.kP = 1.2;
  intake_roller_right_slot0.kI = 0;
  intake_roller_right_slot0.kD = 0;
  intake_roller_right_slot0.GravityType = 0;

  intake_roller_right_config.CurrentLimits.SupplyCurrentLimit = 40_A;
  intake_roller_right_config.CurrentLimits.SupplyCurrentLimitEnable = true;
  intake_roller_right_config.CurrentLimits.SupplyCurrentLowerLimit = 40_A;

  ctre::phoenix::StatusCode intake_roller_right_status =
      ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i) {
    intake_roller_right_status = intake_roller_right_.Applyconfig(intake_roller_right_config);
    if (intake_roller_right_status.IsOK()) {
      break;
    }
  }
  intake_roller_right_.setinvert(-1);

  intake_roller_left_.setfollowControl(intake_roller_right_.Getdata().deviceId, true);
  intake_roller_left_.Control();

  configs::TalonFXConfiguration intake_pitch_config{};
  intake_pitch_config.MotorOutput.NeutralMode = 1;
  intake_pitch_config.MotorOutput.Inverted = 0;
  configs::Slot0Configs& intake_pitch_slot0 = intake_pitch_config.Slot0;
  intake_pitch_slot0.kG = 0.;
  intake_pitch_slot0.kS = 0.;
  intake_pitch_slot0.kV = 0.;
  intake_pitch_slot0.kA = 0.;
  intake_pitch_slot0.kP = 2;
  intake_pitch_slot0.kI = 0;
  intake_pitch_slot0.kD = 0.;
  intake_pitch_slot0.GravityType = 0;

    /* Configure Motion Magic */
  configs::MotionMagicConfigs& mm_pitch = intake_pitch_config.MotionMagic;
  // expo鎵€闇€鍙傛暟
  mm_pitch.MotionMagicCruiseVelocity =
      0_tps;  // 5 (mechanism) rotations per second cruise
  mm_pitch.MotionMagicExpo_kV = 0.1_V / 1_tps;           // 0.12
  mm_pitch.MotionMagicExpo_kA = 0.3_V / 1_tr_per_s_sq;  // 0.1

  intake_pitch_config.CurrentLimits.SupplyCurrentLimit = 20_A;
  intake_pitch_config.CurrentLimits.SupplyCurrentLimitEnable = true;


  ctre::phoenix::StatusCode intake_pitch_status =
      ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i) {
    intake_pitch_status = intake_pitch_.Applyconfig(intake_pitch_config);
    if (intake_pitch_status.IsOK()) {
      break;
    }
  }
  intake_pitch_.setgearRatio(18.67);
  intake_pitch_.setinvert(1);
  intake_pitch_.setPhysicalLimits(0, 7 / intake_pitch_.Getdata().gearRatio,
                                  120 / intake_pitch_.Getdata().gearRatio, 40);
  intake_pitch_.setCurrent_Speed(0.07);
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

  if (pitch_reset_flag_ == 0) {
    GroundIntakeReset();
  }
  else {  
  // Pitch manual test: RightY top -> max pitch, bottom -> min pitch.
  // const double right_y = joystick_.GetRightY();
  // const double normalized = std::clamp((1.0 - right_y) / 2.0, 0.0, 1.0);
  // SetPitchNormPosition(normalized);
  }

  intake_roller_right_.Control();
  intake_pitch_.Control();
  frc::SmartDashboard::PutBoolean("GI_pitch_reset", getPitchResetFlag());
  if (publish_debug) {

    // frc::SmartDashboard::PutNumber(
    //     "ground_intake_pitch_currentnormalizedPosition",
    //     intake_pitch_.GetNormalizedPosition());
    // frc::SmartDashboard::PutNumber("ground_intake_pitch_wayiconfig.offset",
    //                                intake_pitch_.Getdata().offset);
    // frc::SmartDashboard::PutNumber("ground_intake_pitch_currentPosition",
    //                                intake_pitch_.GetPosition());
    // frc::SmartDashboard::PutNumber("ground_intake_pitch_targetnormalizedPosition",
    //                                intake_pitch_.Getdata().normalizedPosition);

  }
}

void GroundIntakeSubsystem::SetRollerVelocity(double velocity) {
  intake_roller_right_.setvelocitytorquecurrent(velocity);
}

frc2::CommandPtr GroundIntakeSubsystem::SetRollerVelocityCommandPtr(
    double velocity) {
  return this->RunOnce([this, velocity] { SetRollerVelocity(velocity); });
}

void GroundIntakeSubsystem::Stop() { intake_roller_right_.setcoast(); }

frc2::CommandPtr GroundIntakeSubsystem::StopCommandPtr() {
  return this->RunOnce([this] { Stop(); });
}

void GroundIntakeSubsystem::SetPitchNormPosition(double norm) {
  //frc::SmartDashboard::PutNumber("SetPitchNormPosition", norm);
  intake_pitch_.setNormalizedMotionPosition(norm);
}

frc2::CommandPtr GroundIntakeSubsystem::SetPitchNormPositionCommandPtr(
    double norm) {
  return this->RunOnce([this, norm] { SetPitchNormPosition(norm); });
}

double GroundIntakeSubsystem::GetPitchCurrent() {
  return intake_pitch_.GetCurrent();
}

double GroundIntakeSubsystem::GetPitchNormPosition() {
  return intake_pitch_.GetNormalizedPosition();
}

void GroundIntakeSubsystem::GroundIntakeReset() {
  // Disabled state should not advance homing.
  if (!frc::DriverStation::IsEnabled()) {
    pitch_reset_counter_ = 0;
    //frc::SmartDashboard::PutBoolean("pitch_reset_flag", pitch_reset_flag_);
    return;
  }

  intake_pitch_.setcurrent(-60);

  //frc::SmartDashboard::PutNumber("intake_pitch Current",intake_pitch_.GetCurrent());

  //frc::SmartDashboard::PutNumber("intake_pitch Position",intake_pitch_.GetPosition());

  if (intake_pitch_.GetCurrent() < -20) {
    ++pitch_reset_counter_;}
  // } else {
  //   pitch_reset_counter_ = 0;
  // }
    //frc::SmartDashboard::PutBoolean("pitch_reset_counter_", pitch_reset_counter_);

  if (pitch_reset_counter_ >= 1) {
    //frc::SmartDashboard::PutBoolean("pitch_reset_flag", pitch_reset_flag_);
    intake_pitch_.Reset(intake_pitch_.GetAbsPosition());
    pitch_reset_flag_ = true;

  }

}

void GroundIntakeSubsystem::SetTeleopRollerCurrentLimit() {
  
  configs::CurrentLimitsConfigs current_limits{};

  current_limits.SupplyCurrentLimit = 30_A;
  current_limits.SupplyCurrentLimitEnable = true;
  current_limits.SupplyCurrentLowerLimit = 30_A;
  ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i) {
    status = intake_roller_right_.Getmotor().GetConfigurator().Apply(current_limits);
    
    if (status.IsOK()) {
      break;
    }
  }
}
                                            