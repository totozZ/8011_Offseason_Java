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
inline double SpeedRate = 0.75;
inline double AngularSpeedRate = 0.95;

}  // namespace OperatorConstants

namespace DriveConstants {
constexpr double kPTranslation = 1.0;  // 位置到速度的比例系数
constexpr double kPRotation = 1.0;     // 角度到角速度的比例系数
}  // namespace DriveConstants

namespace FieldConstants {
inline constexpr auto kFieldLength = 16.54_m;
inline constexpr auto kFieldWidth = 8.07_m;
inline constexpr auto kHubPassBlueBoundaryX = 5.0_m;
inline constexpr auto kHubPassRedBoundaryX =
    kFieldLength - kHubPassBlueBoundaryX;
}  // namespace FieldConstants

namespace FeederConstants {
inline constexpr int BackwardFeederMotorID = 17;
inline constexpr int UpwardFeederMotorID = 18;
}  // namespace FeederConstants

namespace ShooterConstants {
inline constexpr int ShooterLeftDownMotorID = 12;
inline constexpr int ShooterLeftUpMotorID = 13;
inline constexpr int ShooterRightUpMotorID = 14;
inline constexpr int ShooterRightDownMotorID = 15;
inline constexpr int ShooterPitchMotorID = 16;

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
inline constexpr double kIntakePitchMotorMaxposition = 8.2;
inline constexpr double PitchNormPosition = 0.923;
}  // namespace GroundIntakeConstants

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
static constexpr units::radians_per_second_t MaxDriveAimingOmega =
    3.5_rad_per_s;
constexpr double DriveAimingAngleTolerance = 0.5;  // 度
}  // namespace DriveAimingConstants

namespace VisionConstants {
inline const std::array<std::string, 2> limelightNames = {"limelight-left",
                                                          "limelight-back"};
}  // namespace VisionConstants
