#pragma once

#include <frc/geometry/Pose2d.h>
#include <units/length.h>
#include <iostream>
#include "generated/TunerConstants.h"

namespace LocalizationConstants
{
static constexpr int MAX_ANGULAR_VELOCITY = 360;     // MT2 的最大角速度，单位为度每秒，超出该值则拒绝 MT2 结果
static constexpr double MIN_AVG_TAG_AREA = 0.1;      // MT2 的最小平均 Tag 面积，单位为平方像素，低于该值则拒绝 MT2 结果
static constexpr double MAX_AMBIGUITY = 0.7;         // MT1 的最大模糊度，超出该值则拒绝 MT1 结果
static constexpr double MAX_DISTANCE_TO_CAMERA = 3;  // MT1 的最大距离，单位为米，超出该值则拒绝 MT1 结果
static constexpr double DISTANCE_SWITCH_TO_MT1_YAW =
    1;                                     // 距离小于该值时，单位为米，使用 MT2 Pose + MT1 Yaw，不然使用 MT2 Pose + Yaw
static constexpr double XY_DEV = 0.01;     // X 和 Y 的权重
static constexpr double THETA_DEV = 0.03;  // Yaw 的权重
}  // namespace LocalizationConstants

namespace GPDetectionConstants
{
static constexpr double ALGAE_RADIUS = 0.413;         // Algae 的半径
static constexpr double HEIGHT_OF_CORAL = 0.3;        // Coral 的高度
static constexpr double HEIGHT_OF_CAMERA = 0.148;     // LL 的高度
static constexpr double HEIGHT_OF_LOLLIPOP = 0.5001;  // 棒棒糖的高度（地面到 Algae 的中心）
static constexpr double PIXELS_OF_CAMERA = 1280;      // LL 的像素宽度
static constexpr double HORIZONTAL_FOV = 81.022;      // LL 的水平视野
static constexpr double YAW_OF_CAMERA = 24.5;         // LL 的水平偏航角
}  // namespace GPDetectionConstants

namespace RobotConstants
{
static constexpr units::meter_t CENTER_TO_BUMPER = 0.33_m;  // 底盘中心到 Bumper 最外侧的长度
}

namespace TagConstants
{
static constexpr std::array<frc::Pose2d, 6> RED_TAG_POS{
  frc::Pose2d{ 530.49_in, 129.97_in, 120_deg },   // ID6
  frc::Pose2d{ 546.87_in, 158.30_in, 180_deg },   // ID7
  frc::Pose2d{ 530.49_in, 186.63_in, -120_deg },  // ID8
  frc::Pose2d{ 497.77_in, 186.63_in, -60_deg },   // ID9
  frc::Pose2d{ 481.39_in, 158.30_in, 0_deg },     // ID10
  frc::Pose2d{ 497.77_in, 129.97_in, 60_deg }     // ID11
};  // 全部红色Tag位置

static constexpr std::array<frc::Pose2d, 6> BLUE_TAG_POS{
  frc::Pose2d{ 3.980_m, 3.165_m, 60_deg },        // ID17
  frc::Pose2d{ 144.00_in, 158.30_in, 0_deg },     // ID18
  frc::Pose2d{ 160.39_in, 186.63_in, -60_deg },   // ID19
  frc::Pose2d{ 193.10_in, 186.63_in, -120_deg },  // ID20
  frc::Pose2d{ 209.49_in, 158.30_in, 180_deg },   // ID21
  frc::Pose2d{ 193.10_in, 129.97_in, 120_deg }    // ID22
};  // 全部蓝色Tag位置

static constexpr std::array<frc::Pose2d, 12> ALL_TAG_POS{
  frc::Pose2d{ 530.49_in, 129.97_in, 120_deg },   // ID6
  frc::Pose2d{ 546.87_in, 158.30_in, 180_deg },   // ID7
  frc::Pose2d{ 530.49_in, 186.63_in, -120_deg },  // ID8
  frc::Pose2d{ 497.77_in, 186.63_in, -60_deg },   // ID9
  frc::Pose2d{ 481.39_in, 158.30_in, 0_deg },     // ID10
  frc::Pose2d{ 497.77_in, 129.97_in, 60_deg },    // ID11
  frc::Pose2d{ 3.980_m, 3.165_m, 60_deg },        // ID17
  frc::Pose2d{ 144.00_in, 158.30_in, 0_deg },     // ID18
  frc::Pose2d{ 160.39_in, 186.63_in, -60_deg },   // ID19
  frc::Pose2d{ 193.10_in, 186.63_in, -120_deg },  // ID20
  frc::Pose2d{ 209.49_in, 158.30_in, 180_deg },   // ID21
  frc::Pose2d{ 193.10_in, 129.97_in, 120_deg }    // ID22
};  // 全部蓝色Tag位置

static constexpr std::array<int, 12> ALL_VALID_TAGS = {
  6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22
};  // 全部有效的 Tag ID
static constexpr std::array<int, 6> RED_VALID_TAGS = { 6, 7, 8, 9, 10, 11 };       // 红色有效的 Tag ID
static constexpr std::array<int, 6> BLUE_VALID_TAGS = { 17, 18, 19, 20, 21, 22 };  // 蓝色有效的 Tag ID

static constexpr units::meter_t LEFT_TO_TAG_POS = 0.19_m;      // Reef 每一侧左边到 Tag 的距离
static constexpr units::meter_t RIGHT_TO_TAG_POS = -0.165_m;   // Reef 每一侧右边到 Tag 的距离
static constexpr units::meter_t CENTER_TO_TAG_POS = 0.0075_m;  // Reef 每一侧中间到 Tag 的距离
}  // namespace TagConstants

namespace AutoAlignConstants
{
static constexpr double X_KP = 7;            // X 方向 P 值
static constexpr double X_KI = 0.0;          // X 方向 I 值
static constexpr double X_KD = 0.15;         // X 方向 D 值
static constexpr double X_TOLERANCE = 0.01;  // X 方向容差

static constexpr double Y_KP = 7.1;          // Y 方向 P 值
static constexpr double Y_KI = 0.0;          // Y 方向 I 值
static constexpr double Y_KD = 0.15;         // Y 方向 D 值
static constexpr double Y_TOLERANCE = 0.01;  // Y 方向容差d

static constexpr double THETA_KP = 6;         // 角度 P 值
static constexpr double THETA_KI = 0.0001;    // 角度 I 值
static constexpr double THETA_KD = 0.0;       // 角度 D 值
static constexpr double THETA_TOLERANCE = 1;  // 角度容差

static constexpr double MAX_LINEAR_SPEED = TunerConstants::kSpeedAt12Volts.value();  // 最大线性速度
static constexpr double MAX_ANGULAR_SPEED = 460.0;                                   // 最大角速度
static constexpr double MAX_DECELERATION = 2.5;                                      // 最大减速度

static constexpr frc::Transform2d TEST_TRANSFORM = { 1_m, 1_m, 90_deg };  // 测试用
}  // namespace AutoAlignConstants

namespace LLConstants
{
static constexpr std::array<std::string_view, 1> LOCALIZATION_LL_NAMES = { "limelight-left" };
static constexpr std::array<std::string_view, 1> DETECTION_LL_NAMES = { "limelight-back" };
static constexpr frc::Transform2d ROBOT_TO_DETECTION_LL{ frc::Translation2d{ -0.32559_m, 0_m },
                                                         frc::Rotation2d{ 180_deg } };
}  // namespace LLConstants

namespace ShooterConstants
{
inline constexpr int ShooterLeftMotorID = 11;
inline constexpr int ShooterRightMotorID = 12;
inline constexpr int PitchMotorID = 13;

inline constexpr double PitchMinPosition = 0.0;
inline constexpr double ExitHorizontalPitchPosition = 0.0;
inline constexpr double PitchMaxPosition = 7.0;
inline constexpr double PitchMinAngle = 20.0;
inline constexpr double PitchMaxAngle = 60;

inline constexpr double PitchDisplacementPerDegree =
    (PitchMaxPosition - PitchMinPosition) / (PitchMaxAngle - PitchMinAngle);
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