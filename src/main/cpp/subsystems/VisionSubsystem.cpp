// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/VisionSubsystem.h"

#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructTopic.h>

#include <array>
#include <cmath>
#include <iostream>

using namespace subsystems;
namespace {
bool g_publish_vision_debug = true;
}  // namespace

VisionSubsystem::VisionSubsystem(CommandSwerveDrivetrain* drivetrain,
                                 LEDSubsystem* ledsub)
    : drivetrain_(drivetrain), ledsub_(ledsub) {
  if (!drivetrain_) {
    throw std::runtime_error(
        "VisionSubsystem: drivetrain pointer cannot be null!");
  }

  vision_table_ = nt::NetworkTableInstance::GetDefault().GetTable("Vision");
}

void VisionSubsystem::Periodic() {
  static double last_debug_publish_s = -1.0;
  static constexpr double kDebugPublishPeriodS = 0.1;
  const double now_s = frc::Timer::GetFPGATimestamp().value();
  g_publish_vision_debug =
      (last_debug_publish_s < 0.0) ||
      ((now_s - last_debug_publish_s) >= kDebugPublishPeriodS);
  if (g_publish_vision_debug) {
    last_debug_publish_s = now_s;
  }

  static constexpr double kPeriodicOverrunMs = 5.0;
  static int periodic_overrun_count = 0;
  static double periodic_ms_max = 0.0;
  const double t_start_s = frc::Timer::GetFPGATimestamp().value();

  try {
    LimelightMeasurement();
    LED_control();
    if (g_publish_vision_debug) {
      for (size_t i = 0; i < VisionConstants::limelightNames.size(); ++i) {
        frc::SmartDashboard::PutNumber(
            "vision_mode_" + VisionConstants::limelightNames[i],
            vision_modes_[i]);
      }
    }

  } catch (const std::exception& e) {
    std::cout << "VisionSubsystem Periodic Failed: " << e.what() << std::endl;
  }

  const double periodic_ms =
      (frc::Timer::GetFPGATimestamp().value() - t_start_s) * 1000.0;
  if (periodic_ms > periodic_ms_max) {
    periodic_ms_max = periodic_ms;
  }
  if (periodic_ms > kPeriodicOverrunMs) {
    ++periodic_overrun_count;
  }
  frc::SmartDashboard::PutNumber("Perf/VisionPeriodicMs", periodic_ms);
  frc::SmartDashboard::PutNumber("Perf/VisionPeriodicMsMax", periodic_ms_max);
  frc::SmartDashboard::PutNumber("Perf/VisionPeriodicOverrunCount",
                                 periodic_overrun_count);
}

void VisionSubsystem::UpdateAngularVelocity() {
  // 不使用机器人角度进行计算，而直接调用陀螺仪
  currentAngularVelocity_ = drivetrain_->GetPigeon2()
                                .GetAngularVelocityZDevice()
                                .GetValueAsDouble();  // 获取当前角速度

  if (g_publish_vision_debug) {
    frc::SmartDashboard::PutNumber("Vision_AngularVelocity",
                                   currentAngularVelocity_);
  }
}

void VisionSubsystem::SetLimelightIMUMode(std::string limelightname_,
                                          LimelightIMUMode mode) {
  currentIMUMode = mode;
  switch (currentIMUMode) {
      /**
       * @brief 以下function来自Limelight，别动！
       */
    case LimelightIMUMode::ExternalIMU:
      LimelightHelpers::setLimelightNTDouble(limelightname_, "imumode_set", 0);
      break;
    case LimelightIMUMode::SeedingMode:
      LimelightHelpers::setLimelightNTDouble(limelightname_, "imumode_set", 1);
      break;
    case LimelightIMUMode::InternalIMU:
      LimelightHelpers::setLimelightNTDouble(limelightname_, "imumode_set", 2);
      break;
    case LimelightIMUMode::FusedIMU:
      LimelightHelpers::setLimelightNTDouble(limelightname_, "imumode_set", 4);
      break;
  }

  LimelightHelpers::SetRobotOrientation(
      limelightname_,
      drivetrain_->GetcurrentPose().Rotation().Degrees().value(),0, 0, 0,
      0,0);
}

bool VisionSubsystem::ShouldRejectMetatagPose(
    const LimelightHelpers::PoseEstimate& pose_estimate) {
  bool reject = false;  // 默认不拒绝
  std::string reason;   // 记录拒绝原因

  // 检查 tag 数量
  if (pose_estimate.tagCount == 0) {
    reject = true;
    reason = "No Tags";
  }
  // 检查角速度是否过高
  if (std::abs(currentAngularVelocity_) > kMaxAngularVelocity_) {
    reject = true;
    reason +=
        reason.empty() ? "High Angular Velocity" : " & High Angular Velocity";
  }
  // 检查 ambiguity（模糊度），仅在 tagCount > 0 时有效
  if (pose_estimate.tagCount > 0 &&
      pose_estimate.rawFiducials[0].ambiguity > 0.7) {
    reject = true;
    reason += reason.empty() ? "High Ambiguity" : " & High Ambiguity";
  }
  // 检查距离，确保在合理范围内，仅在 tagCount > 0 时有效
  if (pose_estimate.tagCount > 0 &&
      (pose_estimate.rawFiducials[0].distToCamera > 3.3 ||
       pose_estimate.rawFiducials[0].distToCamera < 0.26)) {
    reject = true;
    reason += reason.empty() ? "DistToCamera" : " & DistToCamera";
  }
  // 如果拒绝，记录原因到 SmartDashboard
  if (reject) {
    frc::SmartDashboard::PutString("Vision_Reject_Reason", reason);
  }
  return reject;
}

void VisionSubsystem::UpdateVisionMode(size_t index) {
  if (vision_table_ && g_publish_vision_debug) {
    std::string name = VisionConstants::limelightNames[index];
    // Keep publishers alive for the whole robot runtime.
    // auto mt1_timestamp_pub =
    //     vision_table_->GetDoubleTopic("MT1TimestampSec_" + name).Publish();
    // auto mt2_timestamp_pub =
    //     vision_table_->GetDoubleTopic("MT2TimestampSec_" + name).Publish();
    // auto mt12_delta_pub =
    //     vision_table_->GetDoubleTopic("MT12TimestampDeltaSec_" + name)
    //         .Publish();
    // auto mt1_pose_pub =
    //     vision_table_->GetStructTopic<frc::Pose2d>("MT1Pose_" + name).Publish();
    // auto mt2_pose_pub =
    //     vision_table_->GetStructTopic<frc::Pose2d>("MT2Pose_" + name).Publish();

  //   const double mt1_timestamp_s = mt1_poses_[index].timestampSeconds.value();
  //   const double mt2_timestamp_s = mt2_poses_[index].timestampSeconds.value();
  //   mt1_timestamp_pub.Set(mt1_timestamp_s);
  //   mt2_timestamp_pub.Set(mt2_timestamp_s);
  //   mt12_delta_pub.Set(mt2_timestamp_s - mt1_timestamp_s);
  //   mt1_pose_pub.Set(mt1_poses_[index].pose);
  //   mt2_pose_pub.Set(mt2_poses_[index].pose);
  }

  if (ShouldRejectMetatagPose(mt2_poses_[index])) {
    vision_modes_[index] = 0;
    return;
  }

  if (mt2_poses_[index].rawFiducials.size() > 0 &&
      mt2_poses_[index].rawFiducials[0].distToCamera < switch_distance_ &&
      !ShouldRejectMetatagPose(mt1_poses_[index])) {
    vision_modes_[index] = 2;
  } else {
    vision_modes_[index] = 1;
  }
}

void VisionSubsystem::LimelightMeasurement() {
  UpdateAngularVelocity();

  // 设置当前 IMU 模式
  // currentIMUMode = frc::DriverStation::IsDisabled()
  //                      ? LimelightIMUMode::SeedingMode
  //                      : LimelightIMUMode::FusedIMU;
  //currentIMUMode=LimelightIMUMode::FusedIMU;
  currentIMUMode=LimelightIMUMode::ExternalIMU;
  if (g_publish_vision_debug) {
    frc::SmartDashboard::PutNumber("limelight_imu_mode",
                                   static_cast<int>(currentIMUMode));
  }

  for (size_t i = 0; i < VisionConstants::limelightNames.size(); ++i) {
    std::string camera_name = VisionConstants::limelightNames[i];
    SetLimelightIMUMode(camera_name, currentIMUMode);

    // 获取 Limelight 数据
    auto mt1_optional =
        LimelightHelpers::getBotPoseEstimate_wpiBlue(camera_name);
    auto mt2_optional =
        LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2(camera_name);
    mt1_poses_[i] = mt1_optional.value_or(LimelightHelpers::PoseEstimate{});
    mt2_poses_[i] = mt2_optional.value_or(LimelightHelpers::PoseEstimate{});

    //   if(mt1_optional.has_value()) {
    //   Test(mt1_poses_[i], mt2_poses_[i].pose);
    // }

    UpdateVisionMode(i);

    if (vision_modes_[i]) {  // 视觉通过不为0

      /**
       * @brief 以下代码来自581，别动！
       */
      double kVisionStdScale = 1.0;  // 先 1.5~2.5 试
      double avgDistance = mt2_poses_[i].avgTagDist;
      double xyDev = 0.01 * std::pow(avgDistance, 1.2) * kVisionStdScale;
      double thetaDev = 0.03 * std::pow(avgDistance, 1.2) * kVisionStdScale;
      std::array<double, 3> estStdDevs = {xyDev, xyDev, thetaDev};

      switch (vision_modes_[i]) {
        case 1:                      // 使用 mt2 更新
          estStdDevs[2] = 10000000;  // 使用外部 IMU 时不信任 Limelight 的 yaw
          drivetrain_->AddVisionMeasurement(
              mt2_poses_[i].pose, mt2_poses_[i].timestampSeconds,
              std::array{estStdDevs[0], estStdDevs[1], estStdDevs[2]});
          break;

        case 2: {  // 混合模式，在近处disable下使用mt1的yaw进行校准
          //if (disable_mix) {
            auto mt_mix = frc::Pose2d{mt2_poses_[i].pose.Translation(),
                                      mt1_poses_[i].pose.Rotation()};//rotation was mt1
            drivetrain_->AddVisionMeasurement(
                mt_mix, mt2_poses_[i].timestampSeconds,
                std::array{estStdDevs[0], estStdDevs[1], estStdDevs[2]});
          // } else {
          //   estStdDevs[2] = 10000000;  // 使用外部 IMU 时不信任 Limelight 的 yaw
          //   drivetrain_->AddVisionMeasurement(
          //       mt2_poses_[i].pose, mt2_poses_[i].timestampSeconds,
          //       std::array{estStdDevs[0], estStdDevs[1], estStdDevs[2]});
          // }
        } break;

        default:
          break;
      }
    }
  }
}

void VisionSubsystem::LED_control() {
  // if (vision_mode_ == 0) {
  // ledsub_->SetLEDState(LEDSubsystem::AnimationType::Blue);
  // }
  // else if (vision_mode_ == 1) {
  //   if (gpdetection_ && gpdetection_->GetValid()) {
  //   ledsub_->SetLEDState(LEDSubsystem::AnimationType::Green);
  //   } else {
  //   ledsub_->SetLEDState(LEDSubsystem::AnimationType::Red);
  //   }
  //   }
  // else if (vision_mode_ == 2) {
  // ledsub_->SetLEDState(LEDSubsystem::AnimationType::Purple);
  // }
}

void VisionSubsystem::Test(LimelightHelpers::PoseEstimate mt1,
                           frc::Pose2d mt2) {
  static constexpr size_t kYawWindow = 100;
  static std::array<double, kYawWindow> mt1_samples{};
  static std::array<double, kYawWindow> mt2_samples{};
  static size_t mt1_index = 0;
  static size_t mt2_index = 0;
  static size_t mt1_count = 0;
  static size_t mt2_count = 0;

  auto push_sample = [](std::array<double, kYawWindow>& samples, size_t& index,
                        size_t& count, double value) {
    const size_t window = samples.size();
    samples[index] = value;
    index = (index + 1) % window;
    if (count < window) {
      ++count;
    }
  };

  auto stddev = [](const std::array<double, kYawWindow>& samples,
                   size_t count) -> double {
    if (count == 0) {
      return 0.0;
    }
    double sum = 0.0;
    for (size_t i = 0; i < count; ++i) {
      sum += samples[i];
    }
    double mean = sum / static_cast<double>(count);
    double acc = 0.0;
    for (size_t i = 0; i < count; ++i) {
      double d = samples[i] - mean;
      acc += d * d;
    }
    return std::sqrt(acc / static_cast<double>(count));
  };

  (void)mt1;
  double mt1_yaw = mt1_poses_[0].pose.Rotation().Degrees().value();
  double mt2_yaw = drivetrain_->GetcurrentPose().Rotation().Degrees().value();

  push_sample(mt1_samples, mt1_index, mt1_count, mt1_yaw);
  push_sample(mt2_samples, mt2_index, mt2_count, mt2_yaw);

  frc::SmartDashboard::PutNumber("mt1_yaw", mt1_yaw);
  frc::SmartDashboard::PutNumber("mt1_yaw_stddev",
                                 stddev(mt1_samples, mt1_count));
  frc::SmartDashboard::PutNumber("mt2_yaw_stddev",
                                 stddev(mt2_samples, mt2_count));

  frc::SmartDashboard::PutNumber(
      "pigeon_yaw", drivetrain_->GetcurrentPose().Rotation().Degrees().value());

  (void)mt2;
}