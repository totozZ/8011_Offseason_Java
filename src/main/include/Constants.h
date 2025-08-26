#pragma once

#include <frc/geometry/Pose2d.h>
#include <units/length.h>
#include <iostream>

namespace AutoAlignConstants
{
    static constexpr std::array<frc::Pose2d, 6> RED_TAG_POS{
        frc::Pose2d{530.49_in, 129.97_in, 120_deg},
        frc::Pose2d{546.87_in, 158.30_in, 180_deg},
        frc::Pose2d{530.49_in, 186.63_in, -120_deg},
        frc::Pose2d{497.77_in, 186.63_in, -60_deg},
        frc::Pose2d{481.39_in, 158.30_in, 0_deg},
        frc::Pose2d{497.77_in, 129.97_in, 60_deg}};

    static constexpr std::array<frc::Pose2d, 6> BLUE_TAG_POS{
        frc::Pose2d{160.39_in, 129.97_in, 60_deg},
        frc::Pose2d{144.00_in, 158.30_in, 0_deg},
        frc::Pose2d{160.39_in, 186.63_in, -60_deg},
        frc::Pose2d{193.10_in, 186.63_in, -120_deg},
        frc::Pose2d{209.49_in, 158.30_in, 180_deg},
        frc::Pose2d{193.10_in, 129.97_in, 120_deg}};

    static constexpr units::meter_t LEFT_TO_TAG_POS = 0.5_m;
    static constexpr units::meter_t RIGHT_TO_TAG_POS = -0.5_m;
    static constexpr units::meter_t CENTER_TO_TAG_POS = 0.3_m;
}

namespace RobotInfoConstants
{
    static constexpr units::meter_t CENTER_TO_BUMPER = 0.33_m;
}