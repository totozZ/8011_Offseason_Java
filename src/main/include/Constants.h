// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/angle.h>
#include <units/length.h>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *\[]
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace OperatorConstants
{

// inline constexpr int kDriverControllerPort = 0;
inline double SpeedRate = 0.6;
inline double AngularSpeedRate = 0.6;

}  // namespace OperatorConstants

namespace DriveConstants
{
constexpr double kPTranslation = 1.0;  // 位置到速度的比例系数
constexpr double kPRotation = 1.0;     // 角度到角速度的比例系数
}  // namespace DriveConstants

namespace posConstants
{

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

namespace auto_Con
{

// frc::Pose2d pos_22_ready = frc::Pose2d(pos_22_rex,pos_22_rey,frc::Rotation2d(pos_22_angle));

// 805 1755
/* left */

// pos17
static constexpr auto pos_blue_17right_rex = 3.921_m;
static constexpr auto pos_blue_17right_rey = 2.74_m;
static constexpr auto pos_blue_17right_angle = 60_deg;

// bluestartpoint_left
static constexpr auto pos_blue_start_rex = 7.114_m;
static constexpr auto pos_blue_start_rey = 7.558_m;
static constexpr auto pos_blue_start_angle = 180_deg;

// bluestartpoint_right
static constexpr auto pos_blue_start_rex_right = 7.114_m;
static constexpr auto pos_blue_start_rey_right = 8.05_m - pos_blue_start_rey;
static constexpr auto pos_blue_start_angle_right = 180_deg;

// bluestartpoint_middle
static constexpr auto pos_blue_start_rex_middle = 7.114_m;
static constexpr auto pos_blue_start_rey_middle = 4.025_m;
static constexpr auto pos_blue_start_angle_middle = 180_deg;

// redstartpoint:right
static constexpr auto pos_red_start_rex_right = 17.55_m - pos_blue_start_rex_right;
static constexpr auto pos_red_start_rey_right = 8.05_m - pos_blue_start_rey_right;
static constexpr auto pos_red_start_angle_right = 0_deg;

// redstartpoint:left
static constexpr auto pos_red_start_rex = 17.55_m - pos_blue_start_rex;
static constexpr auto pos_red_start_rey = 8.05_m - pos_blue_start_rey;
static constexpr auto pos_red_start_angle = 0_deg;

// redstartpoint:middle
static constexpr auto pos_red_start_rex_middle = pos_red_start_rex_right;
static constexpr auto pos_red_start_rey_middle = 4.025_m;
static constexpr auto pos_red_start_angle_middle = 0_deg;

// 20ready
static constexpr auto pos_20_rex = 5.618_m;
static constexpr auto pos_20_rey = 5.815_m;
static constexpr auto pos_20_angle = -120_deg;

// 22ready
static constexpr auto pos_22_rex = pos_20_rex;
static constexpr auto pos_22_rey = 8.05_m - pos_20_rey;
static constexpr auto pos_22_angle = 120_deg;

// 19ready
static constexpr auto pos_19_rex = 3.496_m;
static constexpr auto pos_19_rey = 5.709_m;
static constexpr auto pos_19_angle = -60_deg;

// 17ready
static constexpr auto pos_17_rex = pos_19_rex;
static constexpr auto pos_17_rey = 8.05_m - pos_19_rey;
static constexpr auto pos_17_angle = 60_deg;

// 18ready
static constexpr auto pos_18_rex = 2.713_m;
static constexpr auto pos_18_rey = 4.045_m;
static constexpr auto pos_18_angle = 0_deg;

// 21ready
static constexpr auto pos_21_rex = 6.497_m;
static constexpr auto pos_21_rey = 4.025_m;
static constexpr auto pos_21_angle = 180_deg;

// 10ready
static constexpr auto pos_10_rex = 17.55_m - pos_21_rex;
static constexpr auto pos_10_rey = 4.025_m;
static constexpr auto pos_10_angle = 0_deg;

// 6ready
static constexpr auto pos_6_rex = 17.55_m - pos_19_rex;
static constexpr auto pos_6_rey = 8.05_m - pos_19_rey;
static constexpr auto pos_6_angle = 120_deg;

// 7ready
static constexpr auto pos_7_rex = 17.55_m - pos_18_rex;
static constexpr auto pos_7_rey = pos_18_rey;
static constexpr auto pos_7_angle = 180_deg;

// 8ready
static constexpr auto pos_8_rex = pos_6_rex;
static constexpr auto pos_8_rey = 8.05_m - pos_6_rey;
static constexpr auto pos_8_angle = -120_deg;

// blue_leftCoral
static constexpr auto pos_blue_leftcoral_rex = 1.223_m;
static constexpr auto pos_blue_leftcoral_rey = 7.232_m;
static constexpr auto pos_blue_leftcoral_angle = -55_deg;

// red_leftCoral
static constexpr auto pos_red_leftcoral_rex = 17.55_m - pos_blue_leftcoral_rex;
static constexpr auto pos_red_leftcoral_rey = 8.05_m - pos_blue_leftcoral_rey;
static constexpr auto pos_red_leftcoral_angle = 180_deg + pos_blue_leftcoral_angle;

// 11ready
static constexpr auto pos_11_rex = 17.55_m - pos_20_rex;
static constexpr auto pos_11_rey = 8.05_m - pos_20_rey;
static constexpr auto pos_11_angle = 60_deg;

// Coral参考位置
static constexpr auto x_Coral_blue = 1.2192_m;
// blueleft
static constexpr auto y_Coralleft_blue = 5.8547_m;
static constexpr auto angle_Coral1_blue = -45_deg;
// bluemiddle
static constexpr auto y_Coralmiddle_blue = 4.0259_m;
static constexpr auto angle_Coral2_blue = 0_deg;
// blueright
static constexpr auto y_Coralright_blue = 2.1844_m;
static constexpr auto angle_Coral3_blue = 45_deg;

// redleft
static constexpr auto x_Coral_red = 17.55_m - x_Coral_blue;
static constexpr auto y_Coralleft_red = 8.05_m - y_Coralleft_blue;
static constexpr auto angle_Coral1_red = 135_deg;
// redmiddle
static constexpr auto y_Coralmiddle_red = 8.05_m - y_Coralmiddle_blue;
static constexpr auto angle_Coral2_red = 0_deg;
// redright
static constexpr auto y_Coralright_red = 8.05_m - y_Coralright_blue;
static constexpr auto angle_Coral3_red = -135_deg;

// prepare，地吸coral准备
// blueright
static constexpr auto x_Coralright_blue_pre = 2.225_m;
static constexpr auto y_Coralright_blue_pre = 2.826_m;
static constexpr auto angle_Coral3_blue_pre = 28_deg;

// bluemiddle
static constexpr auto x_Coralmiddle_blue_pre = 2.500_m;
static constexpr auto y_Coralmiddle_blue_pre = 4.025_m;
static constexpr auto angle_Coral2_blue_pre = 0_deg;

// blueleft
static constexpr auto x_Coralleft_blue_pre = 2.225_m;
static constexpr auto y_Coralleft_blue_pre = 8.05_m - y_Coralright_blue_pre;
static constexpr auto angle_Coral1_blue_pre = -28_deg;

// redright
static constexpr auto x_Coralright_red_pre = 17.55_m - x_Coralright_blue_pre;
static constexpr auto y_Coralright_red_pre = 8.055_m - y_Coralright_blue_pre;
static constexpr auto angle_Coral3_red_pre = -180_deg + angle_Coral3_blue_pre;

// redmiddle
static constexpr auto x_Coralmiddle_red_pre = 17.55_m - x_Coralmiddle_blue_pre;
static constexpr auto y_Coralmiddle_red_pre = 8.05_m - y_Coralmiddle_blue_pre;
static constexpr auto angle_Coral2_red_pre = 180_deg;

// redleft
static constexpr auto x_Coralleft_red_pre = 17.55_m - x_Coralleft_blue_pre;
static constexpr auto y_Coralleft_red_pre = 8.05_m - y_Coralleft_blue_pre;
static constexpr auto angle_Coral1_red_pre = 180_deg + (-28_deg);

// 3.153 3.85 0
}  // namespace auto_Con

namespace ShooterConstants
{
inline constexpr int ShooterLeftFrontMotorID = 11;
inline constexpr int ShooterLeftBackMotorID = 12;
inline constexpr int ShooterRightMotorID = 13;

inline constexpr double SpeedConversionEfficiency = 0.3;
inline constexpr double ShootWheelRadius = 0.1;
inline constexpr double TargetHeight = 1.8796;
inline constexpr double ShooterHeight = 0.75;
}  // namespace ShooterConstants

namespace SolverConstants
{
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