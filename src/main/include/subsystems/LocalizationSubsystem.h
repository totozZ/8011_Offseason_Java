#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/GenericEntry.h>
#include <frc/geometry/Pose2d.h>
#include <frc/Timer.h>
#include <iostream>
#include "ctre/phoenix6/Utils.hpp"
#include "LimelightHelpers.h"
#include "subsystems/CommandSwerveDrivetrain.h"
#include "Constants.h"

namespace subsystems
{
  class LocalizationSubsystem : public frc2::SubsystemBase
  {
  public:
    LocalizationSubsystem(CommandSwerveDrivetrain *drivetrain);
    void Periodic() override;
    void setRobotOrientation();
    void setIMUMode(int mode);
    void switchVisionMode();
    void locationInit();
    void updatePoseEstimator();
    void updateAngularVelocity();
    bool shouldRejectMT2(const LimelightHelpers::PoseEstimate &poseEstimate);
    bool shouldRejectMT1(const LimelightHelpers::PoseEstimate &poseEstimate);

  private:
    CommandSwerveDrivetrain *m_drivetrain;

    bool location_init_flag = false; // 为True时已经完成初始位置，为False时未完成初始位置
    bool is_using_mt1_yaw = false;   // 为True时使用MT2 Pose + MT1 Yaw，为False时使用MT2 Pose + Yaw

    double current_angular_velocity;            // 当前角速度
    double current_yaw = 0.0;                   // 当前Yaw
    double last_yaw = 0.0;                      // 上一次Yaw
    double delta_yaw = 0.0;                     // 当前Yaw - 上一次Yaw
    units::second_t current_time = 0_s;         // 当前时间
    units::second_t last_yaw_update_time = 0_s; // 上一次Yaw更新时间
    units::second_t deltaTime = 0_s;            // 当前时间 - 上一次Yaw更新时间
    double avg_distance;                        // 所有Tag到机器人的平均距离

    frc::Pose2d current_pose; // 当前机器人位置

    double xy_dev;     // X和Y的权重
    double theta_dev;  // Yaw的权重
    int LED_state = 1; // LED状态，为0时没有更新定位，为1时更新定位

    LimelightHelpers::PoseEstimate mt1_left; // MT1的位置
    LimelightHelpers::PoseEstimate mt2_left; // MT2的位置
    frc::Pose2d mt_mix_left;                 // MT2 Pose + MT1 Yaw的混合位置
  };
}