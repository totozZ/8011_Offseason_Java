#pragma once

#include <frc/geometry/Pose2d.h>
#include <units/length.h>
#include <iostream>

namespace AutoAlignConstants
{
    static constexpr std::array<frc::Pose2d, 6> RED_TAG_POS{
        frc::Pose2d{530.49_in, 129.97_in, 120_deg},  // ID6
        frc::Pose2d{546.87_in, 158.30_in, 180_deg},  // ID7
        frc::Pose2d{530.49_in, 186.63_in, -120_deg}, // ID8
        frc::Pose2d{497.77_in, 186.63_in, -60_deg},  // ID9
        frc::Pose2d{481.39_in, 158.30_in, 0_deg},    // ID10
        frc::Pose2d{497.77_in, 129.97_in, 60_deg}    // ID11
    }; // 全部红色Tag位置

    static constexpr std::array<frc::Pose2d, 6> BLUE_TAG_POS{
        frc::Pose2d{160.39_in, 129.97_in, 60_deg},   // ID17
        frc::Pose2d{144.00_in, 158.30_in, 0_deg},    // ID18
        frc::Pose2d{160.39_in, 186.63_in, -60_deg},  // ID19
        frc::Pose2d{193.10_in, 186.63_in, -120_deg}, // ID20
        frc::Pose2d{209.49_in, 158.30_in, 180_deg},  // ID21
        frc::Pose2d{193.10_in, 129.97_in, 120_deg}   // ID22
    }; // 全部蓝色Tag位置

    static constexpr units::meter_t LEFT_TO_TAG_POS = 0.19_m;     // Reef每一侧左边到Tag的距离
    static constexpr units::meter_t RIGHT_TO_TAG_POS = -0.165_m;  // Reef每一侧右边到Tag的距离
    static constexpr units::meter_t CENTER_TO_TAG_POS = 0.0075_m; // Reef每一侧中间到Tag的距离
}

namespace RobotInfoConstants
{
    static constexpr units::meter_t CENTER_TO_BUMPER = 0.33_m; // 底盘中心到Bumper最外侧的长度
}
namespace LocalizationConstants
{
    static constexpr int MAX_ANGULAR_VELOCITY = 360;        // MT2的最大角速度，单位为度每秒，超出该值则拒绝MT2结果
    static constexpr double MIN_AVG_TAG_AREA = 0.1;         // MT2的最小平均Tag面积，单位为平方像素，低于该值则拒绝MT2结果
    static constexpr double MAX_AMBIGUITY = 0.7;            // MT1的最大模糊度，超出该值则拒绝MT1结果
    static constexpr double MAX_DISTANCE_TO_CAMERA = 3;     // MT1的最大距离，单位为米，超出该值则拒绝MT1结果
    static constexpr double DISTANCE_SWITCH_TO_MT1_YAW = 1; // 距离小于该值时，单位为米，使用MT2 Pose + MT1 Yaw，不然使用MT2 Pose + Yaw
    static constexpr double XY_DEV = 0.01;                  // X和Y的权重
    static constexpr double THETA_DEV = 0.03;               // Yaw的权重
}

namespace RobotConstants
{
    static constexpr std::array<int, 12> ALL_VALID_APRILTAGS = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
    static constexpr std::array<int, 6> RED_VALID_APRILTAGS = {6, 7, 8, 9, 10, 11};
    static constexpr std::array<int, 6> BLUE_VALID_APRILTAGS = {17, 18, 19, 20, 21, 22};
}
