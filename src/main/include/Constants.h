// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/geometry/Translation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>

#include <array>
#include <string>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *\[]
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace OperatorConstants {

// inline constexpr int kDriverControllerPort = 0;
inline double SpeedRate = 0.7;
inline double AngularSpeedRate = 0.95;

}  // namespace OperatorConstants

namespace DriveConstants {
constexpr double kPTranslation = 1.0;  // 位置到速度的比例系数
constexpr double kPRotation = 1.0;     // 角度到角速度的比例系数
}  // namespace DriveConstants

namespace posConstants {

static constexpr auto pos_x1 = 3.483_m;
static constexpr auto pos_x2 = 3.980_m;
static constexpr auto pos_x3 = 4.976_m;
static constexpr auto pos_x4 = 5.463_m;
static constexpr auto pos_y1 = 4.023_m;
static constexpr auto pos_y2 = 3.165_m;
static constexpr auto pos_y3 = 4.881_m;

// 都是左边的目标
static constexpr auto pos_22_targetx = 4.991144_m;
static constexpr auto pos_22_targety = 2.688769_m;

static constexpr auto pos_17_targetx = 3.575_m;
static constexpr auto pos_17_targety = 2.913_m;

static constexpr auto pos_20_targetx = 5.380856;
static constexpr auto pos_20_targety = 5.132231;

}  // namespace posConstants

namespace SolverConstants {
inline constexpr double ResistanceCoefficient = 0.01;
inline constexpr double G = 9.81;
inline constexpr double DT = 0.1;
inline constexpr double Timeout = 5.0;
inline constexpr double Delay = 0.2;
inline constexpr double MaxIterations = 30;
inline constexpr double ErrorTolerance = 0.001;
inline constexpr double ShooterMinSpeed = 3.0;
inline constexpr double ShooterMaxSpeed = 10.0;
inline constexpr double MaxFlightTime = 2.0;
inline constexpr double AdjustSpeed = 0.4;
}  // namespace SolverConstants

namespace FeederConstants {
inline constexpr int BackwardFeederMotorID = 17;
inline constexpr int UpwardFeederMotorID = 18;
inline constexpr double kBackwardHoldCurrent = 60.0;
inline constexpr double kBackwardHoldCurrentSpeed = 0.1;
inline constexpr double kBackwardShootCurrent = 80.0;
inline constexpr double kBackwardShootCurrentSpeed = 0.6;
inline constexpr double kUpwardVelocityTarget = 80.0;
}  // namespace FeederConstants

namespace WeidaiConstants {
inline constexpr int StorageMotorID = 22;
}  // namespace WeidaiConstants

namespace ShooterConstants {
inline constexpr int ShooterLeftDownMotorID = 12;
inline constexpr int ShooterLeftUpMotorID = 13;
inline constexpr int ShooterRightUpMotorID = 14;
inline constexpr int ShooterRightDownMotorID = 15;
inline constexpr int ShooterPitchMotorID = 16;
inline constexpr double LinearServoInitialPositionMm = 0.0;

inline constexpr double SpeedConversionEfficiency = 0.3;
inline constexpr double ShootWheelRadius = 0.1;
inline constexpr double TargetHeight = 1.82;
inline constexpr double ShooterHeight = 0.66;

// Shoot velocity constants
inline constexpr double kShootVelocity = 46;
inline constexpr double kUpwardVelocityTarget = 44.0;
inline constexpr double kFarUpwardVelocityTarget =
    54.0;  // 远段电机基础转速(3.5~4.0m)
inline constexpr double kFarUpwardVelocityDistanceThresholdM =
    4.0;  // 超过此距离电机加速
inline constexpr double kFarUpwardVelocityIncrementPerM =
    14.0;  // 超过阈值每米增加转速
inline constexpr double kFeederDelayTime = 1.0;
inline constexpr double kMaxFeederVelocityDifference = 10.0;

// Feeder velocity constants
inline constexpr double kUpwardFeederVelocity = 40.0;
inline constexpr double kDistanceSplitThresholdM = 3.333;

//pitch constants
inline constexpr double kMaxPitchAngle = 34.65;
inline constexpr double kMinPitchAngle = 0.0;
inline constexpr double kPitchMotorMaxposition = 12.005;
}  // namespace ShooterConstants

namespace GroundIntakeConstants {
inline constexpr int IntakeRollerLeftMotorID = 19;
inline constexpr int IntakeRollerRightMotorID = 20;
inline constexpr int IntakePivotMotorID = 21;

// Auto intake assist: 当 intake pitch 力矩电流超过此阈值时触发 assist
inline constexpr double kAssistPitchCurrentThreshold = 17.0;
// assist 触发后的冷却时间(秒)
inline constexpr double kAssistCooldownS = 1.2;
inline constexpr double PitchNormPosition =0.94;
}  // namespace GroundIntakeConstants

namespace LinearServoConstants {
// 0-12.5 125-60
//  70mm  100mm
inline constexpr double MaxPositionMm = 58.0;
}  // namespace LinearServoConstants

namespace DriveAimingConstants {
// 蓝方Hub坐标
//  static constexpr frc::Translation2d BlueHubPosition{4.625467_m, 4.034536_m};
static constexpr frc::Translation2d BlueHubPosition{4.625594_m, 4.034536_m};
static constexpr frc::Translation2d RedHubPosition{11.915394_m, 4.034536_m};

// DriveAiming pid
constexpr double kPDriveAiming = 0.135;
constexpr double kIDriveAiming = 0.0;
constexpr double kDDriveAiming = 0.0;

// 旋转限制
static constexpr units::radians_per_second_t MaxDriveAimingOmega = 3.5_rad_per_s;
constexpr double DriveAimingAngleTolerance = 0.5;  // 度
}  // namespace DriveAimingConstants

namespace VisionConstants {
inline const std::array<std::string, 2> limelightNames = {"limelight-left", "limelight-back"};
}  // namespace VisionConstants

namespace ClimbConstants {
inline constexpr int ClimbUpMotorID = 30;
inline constexpr int ClimbUpMinPos = 0.;
inline constexpr int ClimbUpMaxPos = 150.;
inline constexpr double ClimbUpMaxCurrent = 40.0;

}  // namespace ClimbConstants
