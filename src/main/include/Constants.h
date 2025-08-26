#pragma once

#include <frc/geometry/Pose2d.h>
#include <units/length.h>
#include <iostream>

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
