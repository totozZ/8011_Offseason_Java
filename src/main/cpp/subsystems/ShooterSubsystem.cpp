// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterSubsystem.h"
#include <frc/Timer.h>
#include <algorithm>
#include <cmath>

#include "subsystems/CommandSwerveDrivetrain.h"
#include "subsystems/FeederSubsystem.h"

using namespace subsystems;

void ShooterSubsystem::SetFeederSubsystem(FeederSubsystem* feeder_subsystem) {
  feeder_sub_ = feeder_subsystem;
}

void ShooterSubsystem::SetDrivetrainSubsystem(
    CommandSwerveDrivetrain* drivetrain_subsystem) {
  drivetrain_sub_ = drivetrain_subsystem;
}

void ShooterSubsystem::Initialization() {
  // --- 1. 旧代码的几何模型预计算 ---
  const double compensateballradis1 = ballradis - ball_error1;
  const double compensateballradis2 = ballradis - ball_error2;
  const double CoscompensateAngleRad = std::clamp(
      (((compensateballradis2 + flywheelradis) * (compensateballradis2 + flywheelradis) +
        hoodradis * hoodradis - compensateballradis1 * compensateballradis1) /
       (2.0 * (compensateballradis2 + flywheelradis) * hoodradis)),
      -1.0, 1.0);
  const double compensateAngleRad = std::acos(CoscompensateAngleRad);
  const double linemiddle = std::sqrt((flywheelradis * flywheelradis) +
                                      hoodradis * hoodradis -
                                      2.0 * flywheelradis * hoodradis * CoscompensateAngleRad);
  const double costhetamiddleline = std::clamp((linemiddle * linemiddle + hoodradis * hoodradis -
                                                flywheelradis * flywheelradis) /
                                                   (2.0 * linemiddle * hoodradis),
                                               -1.0, 1.0);
  thetaMiddleLine = std::acos(costhetamiddleline);

  // --- 2. 新代码的主控飞轮配置 (右上) ---
  configs::TalonFXConfiguration shooter_right_up_config{};
  shooter_right_up_config.MotorOutput.Inverted = 0;
  shooter_right_up_config.MotorOutput.NeutralMode = 0; // Coast
  shooter_right_up_config.CurrentLimits.StatorCurrentLimit = 120_A;
  shooter_right_up_config.CurrentLimits.StatorCurrentLimitEnable = true;
  shooter_right_up_config.CurrentLimits.SupplyCurrentLimit = 40_A;
  shooter_right_up_config.CurrentLimits.SupplyCurrentLimitEnable = true;

  configs::Slot0Configs& shooter_right_up_slot0 = shooter_right_up_config.Slot0;
  shooter_right_up_slot0.kG = 0.;
  shooter_right_up_slot0.kS = 4.875;
  shooter_right_up_slot0.kV = 0;
  shooter_right_up_slot0.kA = 0;
  shooter_right_up_slot0.kP = 9;
  shooter_right_up_slot0.kI = 0;
  shooter_right_up_slot0.kD = 0;

  shooter_right_up_.Applyconfig(shooter_right_up_config);
  shooter_right_up_.setinvert(-1);

  // --- 3. 配置其他三个飞轮跟随右上 ---
  shooter_left_down_.setfollowControl(shooter_right_up_.Getdata().deviceId, true);
  shooter_left_up_.setfollowControl(shooter_right_up_.Getdata().deviceId, true);
  shooter_right_down_.setfollowControl(shooter_right_up_.Getdata().deviceId, false);
  shooter_left_down_.Control();
  shooter_left_up_.Control();
  shooter_right_down_.Control();

  // --- 4. 新代码的 Pitch 角度电机配置 ---
  configs::TalonFXConfiguration shooter_pitch_config{};
  shooter_pitch_config.MotorOutput.Inverted = 0;
  shooter_pitch_config.MotorOutput.NeutralMode = 1; // Brake
  shooter_pitch_config.CurrentLimits.StatorCurrentLimit = 60_A;
  shooter_pitch_config.CurrentLimits.StatorCurrentLimitEnable = true;
  shooter_pitch_config.CurrentLimits.SupplyCurrentLimit = 20_A;
  shooter_pitch_config.CurrentLimits.SupplyCurrentLimitEnable = true;

  configs::Slot0Configs& shooter_pitch_slot0 = shooter_pitch_config.Slot0;
  shooter_pitch_slot0.kP = 7;
  shooter_pitch_slot0.kI = 0.8;
  shooter_pitch_slot0.kD = 0.02;

  configs::MotionMagicConfigs& mm_pitch = shooter_pitch_config.MotionMagic;
  mm_pitch.MotionMagicCruiseVelocity = 0_tps;
  mm_pitch.MotionMagicExpo_kV = 0.1_V / 1_tps;
  mm_pitch.MotionMagicExpo_kA = 0.01_V / 1_tr_per_s_sq;

  shooter_pitch_.Applyconfig(shooter_pitch_config);
  shooter_pitch_.setgearRatio(18.67);
  shooter_pitch_.setinvert(-1);
  shooter_pitch_.setPhysicalLimits(0, ShooterConstants::kPitchMotorMaxposition / 18.67,
                                   120 / 18.67, 40);
  shooter_pitch_.setCurrent_Speed(0.1);
  frc::SmartDashboard::SetDefaultNumber("shootVelWant",40);
  frc::SmartDashboard::SetDefaultBoolean("shootUseDash",false);
}

void ShooterSubsystem::Periodic() {
  // 必须调用以维持封装好的 Wayimotor 的通讯
  shooter_right_up_.Control();
  shooter_pitch_.Control();
  shooter_right_up_.Receive();


  frc::SmartDashboard::PutNumber("shooter_pitch_currentPosition", shooter_pitch_.GetPosition());

  // 1. 判断是否需要归零
  if (shooter_pitch_reset_flag_ == 0) {
       ShootPitch_Reset();
       return;
  } 

  if (drivetrain_sub_ != nullptr) {
      const double hub_distance_m = drivetrain_sub_->GetDistanceToHub();
      const double pass_distance_m = drivetrain_sub_->GetState().Pose.Translation().Distance(passTarget).value();

      frc::SmartDashboard::PutNumber("shooter_hub_distance_m", hub_distance_m);
      auto alliance = frc::DriverStation::GetAlliance();
      bool isRed=alliance.has_value()&&alliance.value()==frc::DriverStation::Alliance::kRed;
      double x = drivetrain_sub_->GetState().Pose.X().value();
      isPassing=(!isRed&&x>=5.0) || (isRed&&x<=11.54);
      frc::SmartDashboard::PutBoolean("shootOnMove/isPassing", isPassing);

        CalculatePitchAngleAboveHub(hub_distance_m);
        if (!isPassing) {
          CalculateShooterSpeedRegression(hub_distance_m);
        } else {
          CalculateShooterSpeedRegression(pass_distance_m);
        }
        getFinalVel();
        frc::SmartDashboard::PutNumber("shoot_velocity_expected", realShootVelocity);
        if (shooting) {
          shooter_right_up_.setvelocitytorquecurrent(realShootVelocity);
          SetShootPitchAngle(90-angle);
        } else {
          Stop();
          SetShootPitchAngle(0.2);
        }
      

      
    }
}

// ==========================================
// 旧代码核心：速度与角度算法
// ==========================================

double ShooterSubsystem::CalculatePitchAngleAboveHub(double dis) {
  if (onlyDefaultShoot) {
    angle = 80;
    return 80;
  } else if (isPassing) {
    Tangle = 60; 
    angle = Tangle + angleOffsetFromDrive;
    return angle;
  } else {
    
    dis = dis * 1.5/ 8.0;
    const double height = 1.8288 + 1.8 - shooter_height_approx;
    constexpr double kRad2Deg = 180.0 / M_PI;
    
    angle = std::atan2(height, dis) * kRad2Deg;
    if (angle > 85) angle = 85;
    if (angle < 75) angle = 75;
    
    Tangle = angle;
    angle += angleOffsetFromDrive;
    frc::SmartDashboard::PutNumber("shootIdealAngle",angle);
    return angle;
  }
}

double ShooterSubsystem::CalculateShooterSpeedRegression(double dis) {
  bool s=frc::SmartDashboard::GetBoolean("shootUseDash",false);
  double sp=frc::SmartDashboard::GetNumber("shootVelWant",40);
  if(!s){
  if (onlyDefaultShoot) {
    vel = 40.9;
  } else if (isPassing) {
    vel = pass_m * dis + pass_b;
  } else if (dis < 2.5) {
    vel = shooter_vel_quadratic_regression_a * dis + shooter_vel_quadratic_regression_b;
  } else { 
    vel = shooter_vel_quadratic_regression_c * dis + shooter_vel_quadratic_regression_d;
  }}
  else{
    vel=sp;
  }

  return vel;
}

double ShooterSubsystem::GetShootVelocity() {
  // 返回主控电机 (右上) 的当前真实转速
  double sh=shooter_right_up_.GetVelocity();
  frc::SmartDashboard::PutNumber("shootRealVelo", sh);
  return sh;
}
void ShooterSubsystem::getFinalVel() {
  double feeder_upward_target_velocity = 0.0;
  double feeder_upward_velocity_difference = 0.0;
  double add = 0;
  
  if (feeder_sub_ != nullptr) {
    feeder_upward_target_velocity = feeder_sub_->GetComboTargetVelocity();
    // 防零除保护
    if (feeder_upward_target_velocity > 0.01) {
      feeder_upward_velocity_difference = -feeder_sub_->GetUpwardFeederVelocity() + feeder_upward_target_velocity;
      add = feeder_upward_velocity_difference / feeder_upward_target_velocity * shooter_max_composite;
    }
  }
  realShootVelocity = vel + velOffsetFromDrive + add;
}

void ShooterSubsystem::ShootPitch_Reset() {
  if (!frc::DriverStation::IsEnabled()) {
    shooter_pitch_reset_flag_ = false;
    return;
  }

  // 给定一个负电流让机构压到底部
  shooter_pitch_.setcurrent(-10);

  if (shooter_pitch_.GetCurrent() < -8) {
    ++shooter_pitch_reset_counter_;
  } else {
    shooter_pitch_reset_counter_ = 0;
  }

  if (shooter_pitch_reset_counter_ >= 3) {
    shooter_pitch_.Reset(shooter_pitch_.GetAbsPosition());
    shooter_pitch_reset_flag_ = true;
  }
}

void ShooterSubsystem::Stop() {
  shooter_right_up_.setcoast(); 
}

frc2::CommandPtr ShooterSubsystem::StopCommandPtr() {
  return this->RunOnce([this] { Stop(); });
}
void ShooterSubsystem::SetShootPitchNormPosition(double norm) {
  shooter_pitch_.setNormalizedMotionPosition(norm);
}

void ShooterSubsystem::SetShootPitchAngle(double target_angle) {
  double clamped_angle = std::clamp(target_angle, ShooterConstants::kMinPitchAngle,
        ShooterConstants::kMaxPitchAngle);

  double target_norm_position = (clamped_angle - ShooterConstants::kMinPitchAngle) /
      (ShooterConstants::kMaxPitchAngle - ShooterConstants::kMinPitchAngle);

  SetShootPitchNormPosition(target_norm_position);
}

// frc2::CommandPtr ShooterSubsystem::SetShootPitchAngleCommandPtr(double target_angle) {
//   return this->RunOnce([this, target_angle] { SetShootPitchAngle(target_angle); });
// }
