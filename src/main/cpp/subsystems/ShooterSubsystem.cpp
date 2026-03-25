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
  shooter_right_up_config.CurrentLimits.StatorCurrentLimit = 100_A;
  shooter_right_up_config.CurrentLimits.StatorCurrentLimitEnable = true;
  shooter_right_up_config.CurrentLimits.SupplyCurrentLimit = 25_A;
  shooter_right_up_config.CurrentLimits.SupplyCurrentLimitEnable = true;

  configs::Slot0Configs& shooter_right_up_slot0 = shooter_right_up_config.Slot0;
  shooter_right_up_slot0.kG = 0.;
  shooter_right_up_slot0.kS = 4.875;
  shooter_right_up_slot0.kV = 0;
  shooter_right_up_slot0.kA = 0;
  shooter_right_up_slot0.kP = 8;
  shooter_right_up_slot0.kI = 0;
  shooter_right_up_slot0.kD = 0;

  shooter_right_up_.Applyconfig(shooter_right_up_config);
  shooter_right_up_.setinvert(-1);

  // --- 3. 配置其他三个飞轮跟随右上 ---
  shooter_left_down_.setfollowControl(shooter_right_up_.Getdata().deviceId, true);
  shooter_left_up_.setfollowControl(shooter_right_up_.Getdata().deviceId, true);
  shooter_right_down_.setfollowControl(shooter_right_up_.Getdata().deviceId, false);

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
  const double hub_distance_m = drivetrain_sub_->GetDistanceToHub();
  frc::SmartDashboard::PutNumber("shooter_hub_distance_m", hub_distance_m);

  frc::SmartDashboard::PutNumber("shooter_pitch_currentPosition", shooter_pitch_.GetPosition());

  // 1. 判断是否需要归零
  if (shooter_pitch_reset_flag_ == 0) {
    ShootPitch_Reset();
  } else {
    // 2. 如果已归零，获取底盘数据并计算
    if (drivetrain_sub_ != nullptr) {
      const double hub_distance_m = drivetrain_sub_->GetDistanceToHub();
      const double pass_distance_m = drivetrain_sub_->GetState().Pose.Translation().Distance(passTarget).value();
      
      isPassing = drivetrain_sub_->GetState().Pose.X().value() <= 11.4 && 
                  drivetrain_sub_->GetState().Pose.X().value() >= 5;
      
      frc::SmartDashboard::PutBoolean("shootOnMove/isPassing", isPassing);

      // 旧算法：计算角度
      CalculatePitchAngleAboveHub(hub_distance_m);
      
      // 旧算法：计算速度
      if (!isPassing) {
        CalculateShooterSpeedRegression(hub_distance_m);
      } else {
        CalculateShooterSpeedRegression(pass_distance_m);
      }
      
      frc::SmartDashboard::PutNumber("shoot_velocity_expected", vel);
    }

    // 3. 将理论角度转换为新版电机的运动指令
    SetShootPitchAngle(90-angle);

  }

  // 4. 合成速度并下发给飞轮
  getFinalVel();
  if (shooting) {
    shooter_right_up_.setvelocitytorquecurrent(realShootVelocity);
  } else {
    Stop();
  }
}

// ==========================================
// 旧代码核心：速度与角度算法
// ==========================================

double ShooterSubsystem::CalculatePitchAngleAboveHub(double dis) {
  if (onlyDefaultShoot) {
    angle = 70;
    return 70;
  } else if (isPassing) {
    Tangle = 50; 
    angle = Tangle + angleOffsetFromDrive;
    return angle;
  } else {
    
    dis = dis * 2 / 8.0;
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
  return shooter_right_up_.Getdata().currentVelocity;
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

// ==========================================
// 结合点：旧物理模型 -> 新马达控制
// ==========================================

void ShooterSubsystem::CalculateShooterPitch() {
  // 1. 旧代码的物理几何映射：理论射出角度 -> 内部 Hood 角度
  const double hood_target = (90.0 - angle) + thetaMiddleLine * 180.0 / M_PI;
  
  // 2. 将 Hood 角度转换为归一化的位置 [0, 1] 供新系统使用
  // 当 hood_target == min 时，norm = 0； 当 hood_target == max 时，norm = 1
  double norm_target = (hood_target - min_hood_angle) / (max_hood_angle - min_hood_angle);
  norm_target = std::clamp(norm_target, 0.0, 1.0);

  // 3. 下发给新版的 Motion Magic 控制器
  SetShootPitchNormPosition(norm_target);
  
  frc::SmartDashboard::PutNumber("shooter_calculated_hood_target_deg", hood_target);
  frc::SmartDashboard::PutNumber("shooter_norm_target_pitch", norm_target);
}

// ==========================================
// 新代码核心：Pitch 电流归零与停止指令
// ==========================================

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
