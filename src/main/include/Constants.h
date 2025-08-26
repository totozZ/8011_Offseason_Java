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

    static constexpr units::meter_t LEFT_TO_TAG_POS = 0.5_m;   // Reef每一侧左边到Tag的距离
    static constexpr units::meter_t RIGHT_TO_TAG_POS = -0.5_m; // Reef每一侧右边到Tag的距离
    static constexpr units::meter_t CENTER_TO_TAG_POS = 0.3_m; // Reef每一侧中间到Tag的距离
}

namespace RobotInfoConstants
{
    static constexpr units::meter_t CENTER_TO_BUMPER = 0.33_m; // 底盘中心到Bumper最外侧的长度
}