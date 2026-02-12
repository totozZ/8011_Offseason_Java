// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/GroundIntakeSubsystem.h"
#include <frc/Timer.h>

using namespace subsystems;

void GroundIntakeSubsystem::Initialization() {
  configs::TalonFXConfiguration intake_roller_config{};
  intake_roller_config.MotorOutput.Inverted = 0;
  configs::Slot0Configs& intake_roller_slot0 = intake_roller_config.Slot0;
  intake_roller_slot0.kG = 0.;
  intake_roller_slot0.kS = 0.;
  intake_roller_slot0.kV = 0.;
  intake_roller_slot0.kA = 0.;
  intake_roller_slot0.kP = 0.85;
  intake_roller_slot0.kI = 0.08;
  intake_roller_slot0.kD = 0.;
  intake_roller_slot0.GravityType = 0;



  configs::TalonFXConfiguration intake_pitch_config{};
  intake_pitch_config.MotorOutput.Inverted = 0;
  configs::Slot0Configs& intake_pitch_slot0 = intake_pitch_config.Slot0;
  intake_pitch_slot0.kG = 0.;
  intake_pitch_slot0.kS = 0.;
  intake_pitch_slot0.kV = 0.;
  intake_pitch_slot0.kA = 0.;
  intake_pitch_slot0.kP = 0.85;
  intake_pitch_slot0.kI = 0.08;
  intake_pitch_slot0.kD = 0.;
  intake_pitch_slot0.GravityType = 0;

  ctre::phoenix::StatusCode intake_pitch_status =
      ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i) {
    intake_pitch_status = intake_pitch_.Applyconfig(intake_pitch_config);
    if (intake_pitch_status.IsOK()) {
      break;
    }
  }

    
  /* Configure Motion Magic */
  configs::MotionMagicConfigs &mm_pitch = intake_pitch_config.MotionMagic;
  // expo所需参数
  mm_pitch.MotionMagicCruiseVelocity = 0_tps; // 5 (mechanism) rotations per second cruise
  mm_pitch.MotionMagicExpo_kV=0.1_V / 1_tps; // 0.12
  mm_pitch.MotionMagicExpo_kA=0.04_V / 1_tr_per_s_sq; // 0.1

  ctre::phoenix::StatusCode intake_roller_status =
      ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i) {
    intake_roller_status = intake_roller_.Applyconfig(intake_roller_config);
    if (intake_roller_status.IsOK()) {

      break;
    }
  }
  intake_pitch_.setgearRatio(126.56);
  intake_pitch_.setinvert(-1);
  intake_pitch_.setPhysicalLimits(0, 23.4 / intake_pitch_.Getdata().gearRatio,120 / intake_pitch_.Getdata().gearRatio, 40);
  intake_pitch_.setCurrent_Speed(0.1);
}

void GroundIntakeSubsystem::Periodic() {
  static constexpr double kPeriodicOverrunMs = 5.0;
  static int periodic_overrun_count = 0;
  static double periodic_ms_max = 0.0;
  static double last_debug_publish_s = -1.0;
  static constexpr double kDebugPublishPeriodS = 0.1;
  const double t_start_s = frc::Timer::GetFPGATimestamp().value();
  const bool publish_debug =
      (last_debug_publish_s < 0.0) ||
      ((t_start_s - last_debug_publish_s) >= kDebugPublishPeriodS);
  if (publish_debug) {
    last_debug_publish_s = t_start_s;
  }

  intake_roller_.Control();
  intake_pitch_.Control();
  intake_roller_.Receive();
  intake_pitch_.Receive();

  GroundIntakeReset();

  if (joystick_.POVUp().Get()) 
  {
    SetPitchNormPosition(0.9);}

  if (joystick_.POVDown().Get()) 
  {
    SetPitchNormPosition(1);}


  if (publish_debug) {
    frc::SmartDashboard::PutNumber("ground_intake_roller_velocity",
                                   intake_roller_.Getdata().currentVelocity);
    frc::SmartDashboard::PutNumber(
        "ground_intake_pitch_currentnormalizedPosition",
        intake_pitch_.Getdata().currentnormalizedPosition);
    frc::SmartDashboard::PutNumber("ground_intake_pitch_wayiconfig.offset",
                                   intake_pitch_.Getdata().offset);
    frc::SmartDashboard::PutNumber("ground_intake_pitch_currentPosition",
                                   intake_pitch_.Getdata().currentPosition);
    frc::SmartDashboard::PutNumber("target_pos",
                                   intake_pitch_.Getdata().normalizedPosition);
    frc::SmartDashboard::PutNumber("target_pos2",
                                   intake_pitch_.Getdata().targetPosition);
    frc::SmartDashboard::PutNumber("target_pos3",
                                   intake_pitch_.Getdata().motionoutput.value());
    frc::SmartDashboard::PutNumber("mode", intake_pitch_.Getdata().mode);
    frc::SmartDashboard::PutNumber(
        "motor.GetPosition().GetValueAsDouble() ",
        intake_pitch_.Getmotor().GetPosition().GetValueAsDouble());
    frc::SmartDashboard::PutNumber("motor.gearRatio ",
                                   intake_pitch_.Getdata().gearRatio);
  }

  const double periodic_ms =
      (frc::Timer::GetFPGATimestamp().value() - t_start_s) * 1000.0;
  if (periodic_ms > periodic_ms_max) {
    periodic_ms_max = periodic_ms;
  }
  if (periodic_ms > kPeriodicOverrunMs) {
    ++periodic_overrun_count;
  }
  frc::SmartDashboard::PutNumber("Perf/GroundIntakePeriodicMs", periodic_ms);
  frc::SmartDashboard::PutNumber("Perf/GroundIntakePeriodicMsMax",
                                 periodic_ms_max);
  frc::SmartDashboard::PutNumber("Perf/GroundIntakePeriodicOverrunCount",
                                 periodic_overrun_count);
}

void GroundIntakeSubsystem::SetRollerVelocity(double velocity) {
  intake_roller_.setvelocitytorquecurrent(velocity);
}



void GroundIntakeSubsystem::SetPitchPosition(double position) {
  intake_pitch_.setmode(12);
  intake_pitch_.Getdata().targetPosition = position;
}

void GroundIntakeSubsystem::SetRollerDutyCycle(double dutyCycle) {
  intake_roller_.setNormalizedDutyCircle(dutyCycle);
}

frc2::CommandPtr GroundIntakeSubsystem::SetRollerDutyCycleCommandPtr(
    double dutyCycle) {
  return this->RunOnce(
      [this, dutyCycle] { SetRollerDutyCycle(dutyCycle); });
}

void GroundIntakeSubsystem::Stop() { SetRollerVelocity(0.0); }


void GroundIntakeSubsystem::SetPitchNormPosition(double norm) {
  intake_pitch_.setNormalizedMotionPosition(norm);
}


frc2::CommandPtr GroundIntakeSubsystem::SetPitchNormPositionCommandPtr(double norm) {

    // 设置intake电机的占空比
    return this->RunOnce(
        [this, norm] {
            SetPitchNormPosition(norm);
        }
    );

}

void GroundIntakeSubsystem::GroundIntakeReset() {
  static int reset_counter_pitch = 0;
  static bool pitch_reset_flag = 0;

  
      if (!pitch_reset_flag) {
        intake_pitch_.setcurrent(-30); // 设置pitch电机电流为-28A
        if (intake_pitch_.Getdata().currentCurrent < -28) {
            reset_counter_pitch++;
            if (reset_counter_pitch >= 3) {
                intake_pitch_.Reset(intake_pitch_.Getmotor().GetPosition().GetValueAsDouble());
                pitch_reset_flag = 1;
            }
        }
    }
}
