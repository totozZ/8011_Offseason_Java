// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterSubsystem.h"

#include <frc/Timer.h>

#include <algorithm>
#include <cmath>

#include "subsystems/FeederSubsystem.h"

using namespace subsystems;

void ShooterSubsystem::SetFeederSubsystem(FeederSubsystem* feeder_subsystem) {
  feeder_sub_ = feeder_subsystem;
}

void ShooterSubsystem::Initialization() {
  configs::TalonFXConfiguration shooter_right_config{};
  shooter_right_config.MotorOutput.Inverted = 0;     // 不反转
  shooter_right_config.MotorOutput.NeutralMode = 0;  // 刹车模式

  // Slot0 PID
  configs::Slot0Configs& shooter_right_slot0 = shooter_right_config.Slot0;
  shooter_right_slot0.kG = 0.;
  shooter_right_slot0.kS = 7;
  shooter_right_slot0.kV = 0.06;
  shooter_right_slot0.kA = 3.5;
  shooter_right_slot0.kP = 5;
  shooter_right_slot0.kI = 0;
  shooter_right_slot0.kD = 0.;
  shooter_right_slot0.GravityType = 0;

  ctre::phoenix::StatusCode shooter_right_status =
      ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i) {
    shooter_right_status = shooter_right_.Applyconfig(shooter_right_config);
    if (shooter_right_status.IsOK()) break;
  }

  // follow
  shooter_left_front_.setfollowControl(shooter_right_.Getdata().deviceId, true);
  shooter_left_back_.setfollowControl(shooter_right_.Getdata().deviceId, true);

  // Power-on default actuator position
  SetLinearServoLeftPositionMm(linear_servo_left_target_mm_);
  SetLinearServoRightPositionMm(linear_servo_right_target_mm_);
}
void ShooterSubsystem::Periodic() {
  static constexpr double kPeriodicOverrunMs = 5.0;
  static int periodic_overrun_count = 0;
  static double periodic_ms_max = 0.0;
  const double t_start_s = frc::Timer::GetFPGATimestamp().value();

  shooter_right_.Control();
  shooter_left_back_.Control();
  shooter_left_front_.Control();
  shooter_right_.Receive();
  shooter_left_back_.Receive();
  shooter_left_front_.Receive();

  // 使用左右trigger控制电推杆
  LinearServoControl();

  CalculateShooterVelocity();

  // 射击
  double now_shoot = frc::Timer::GetFPGATimestamp().value();
  if (joystick_.A().Get()) {
    if (!is_shooting_) {
      shoot_start_time_ = now_shoot;
      is_shooting_ = true;
    }

    SetShootVelocity(ShooterConstants::kShootVelocity);

    // 设置超过kFeederDelayTime启动feeder
    if (now_shoot - shoot_start_time_ >= ShooterConstants::kFeederDelayTime &&
        feeder_sub_ != nullptr) {
      feeder_sub_->SetBackwardFeederVelocity(
          ShooterConstants::kBackwardFeederVelocity);
      feeder_sub_->SetUpwardFeederVelocity(
          ShooterConstants::kUpwardFeederVelocity);
    }
  } else {
    if (is_shooting_) {
      // A键松开，停止所有
      SetShootVelocity(0.0);
      if (feeder_sub_ != nullptr) {
        feeder_sub_->Stop();
      }
      is_shooting_ = false;
    }
  }

  const double periodic_ms =
      (frc::Timer::GetFPGATimestamp().value() - t_start_s) * 1000.0;
  if (periodic_ms > periodic_ms_max) {
    periodic_ms_max = periodic_ms;
  }
  if (periodic_ms > kPeriodicOverrunMs) {
    ++periodic_overrun_count;
  }
  frc::SmartDashboard::PutNumber("Perf/ShooterPeriodicMs", periodic_ms);
  frc::SmartDashboard::PutNumber("Perf/ShooterPeriodicMsMax", periodic_ms_max);
  frc::SmartDashboard::PutNumber("Perf/ShooterPeriodicOverrunCount",
                                 periodic_overrun_count);
}

void ShooterSubsystem::CalculateShooterVelocity() {
  static double last_debug_publish_s = -1.0;
  static constexpr double kDebugPublishPeriodS = 0.1;
  const double now_s = frc::Timer::GetFPGATimestamp().value();
  const bool publish_debug =
      (last_debug_publish_s < 0.0) ||
      ((now_s - last_debug_publish_s) >= kDebugPublishPeriodS);
  if (publish_debug) {
    last_debug_publish_s = now_s;
  }

  double feeder_upward_velocity = 0.0;
  double feeder_upward_velocity_difference = 0.0;
  if (feeder_sub_ != nullptr) {
    feeder_upward_velocity = feeder_sub_->GetUpwardFeederVelocity();
    feeder_upward_velocity_difference =
        -feeder_upward_velocity + feeder_sub_->GetComboTargetVelocity();
    if (feeder_upward_velocity_difference >
        ShooterConstants::kMaxFeederVelocityDifference) {
      feeder_upward_velocity_difference =
          ShooterConstants::kMaxFeederVelocityDifference;
    }
    if (feeder_upward_velocity_difference <
        -ShooterConstants::kMaxFeederVelocityDifference) {
      feeder_upward_velocity_difference =
          -ShooterConstants::kMaxFeederVelocityDifference;
    }
  }
  shooter_velocity_target = ShooterConstants::kUpwardVelocityTarget +
                            feeder_upward_velocity_difference;

  if (publish_debug) {
    frc::SmartDashboard::PutNumber("shooter_feeder_upward_velocity",
                                   feeder_upward_velocity);
    frc::SmartDashboard::PutNumber("shooter_velocity_target",
                                   shooter_velocity_target);
  }
}

void ShooterSubsystem::CalculateLinearServoTarget() {
  // pitch angle映射关系：
  linear_servo_left_target_mm_ =
      sin((90 - shooter_pitch_angle_) * M_PI / 180) * 206.17 - 48.64858705;
  linear_servo_right_target_mm_ = linear_servo_left_target_mm_;
  // shooter_pitch_angle_ = 90.0 - asin((linear_servo_left_target_mm_
  // + 48.64858705) / 206.17) * 180.0 /M_PI;
}

void ShooterSubsystem::SetLinearServoLeftPositionMm(double position_mm) {
  linear_servo_left_.SetPositionMm(position_mm);
}

void ShooterSubsystem::SetLinearServoRightPositionMm(double position_mm) {
  linear_servo_right_.SetPositionMm(position_mm);
}

void ShooterSubsystem::SetShootVelocity(double velocity) {
  shooter_right_.setmode(9);
  shooter_right_.Getdata().targetVelocity = velocity;
}

void ShooterSubsystem::SetBangBangShootVelocity(double velocity) {
  shooter_right_.setBangBangVelocity(velocity, true);  // 使用BangBang控制
}

frc2::CommandPtr ShooterSubsystem::SetShootVelocityCommandPtr(double velocity) {
  return frc2::cmd::RunOnce([this, velocity] { SetShootVelocity(velocity); });
}

frc2::CommandPtr ShooterSubsystem::SetBangBangShootVelocityCommandPtr(
    double velocity) {
  return frc2::cmd::RunOnce(
      [this, velocity] { SetBangBangShootVelocity(velocity); });
}

double ShooterSubsystem::GetShootVelocity() {
  return shooter_right_.Getdata().currentVelocity;  // 从主电机读取
}

void ShooterSubsystem::Stop() { SetShootVelocity(0.0); }

void ShooterSubsystem::LinearServoControl() {
  static double last_debug_publish_s = -1.0;
  static constexpr double kDebugPublishPeriodS = 0.1;
  double left_trigger = joystick_.GetLeftTriggerAxis();
  double right_trigger = joystick_.GetRightTriggerAxis();
  double now_s = frc::Timer::GetFPGATimestamp().value();
  const bool publish_debug =
      (last_debug_publish_s < 0.0) ||
      ((now_s - last_debug_publish_s) >= kDebugPublishPeriodS);
  if (publish_debug) {
    last_debug_publish_s = now_s;
  }
  double dt_s =
      (last_servo_update_s_ > 0.0) ? (now_s - last_servo_update_s_) : 0.0;
  last_servo_update_s_ = now_s;

  double servo_command = left_trigger - right_trigger;
  if (std::abs(servo_command) < 0.02) {
    servo_command = 0.0;
  }

  linear_servo_left_target_mm_ =
      std::clamp(linear_servo_left_target_mm_ +
                     servo_command * kLinearServoSpeedMmPerS * dt_s,
                 0.0, kLinearServoMaxPositionMm);
  linear_servo_right_target_mm_ =
      std::clamp(linear_servo_right_target_mm_ +
                     servo_command * kLinearServoSpeedMmPerS * dt_s,
                 0.0, kLinearServoMaxPositionMm);

  SetLinearServoLeftPositionMm(linear_servo_left_target_mm_);
  SetLinearServoRightPositionMm(linear_servo_right_target_mm_);

  if (publish_debug) {
    frc::SmartDashboard::PutNumber("linear_servo_left_cmd_mm",
                                   linear_servo_left_target_mm_);
    frc::SmartDashboard::PutNumber("linear_servo_right_cmd_mm",
                                   linear_servo_right_target_mm_);
  }
}
