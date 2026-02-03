// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/VisionSubsystem.h"

#include "frc8011/GPDetection.h"

#include <frc/DriverStation.h>
#include <iostream>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructTopic.h>

using namespace subsystems;

VisionSubsystem::VisionSubsystem(CommandSwerveDrivetrain* drivetrain,
                                 LEDSubsystem* ledsub,
                                 GPDetection* gpdetection)
    : drivetrain_(drivetrain),
    ledsub_(ledsub),
      gpdetection_(gpdetection){  // 修改：m_drivetrain -> drivetrain_
    if (!drivetrain_) {
        throw std::runtime_error("VisionSubsystem: drivetrain pointer cannot be null!");
    }
    if (!ledsub_) {
        throw std::runtime_error("VisionSubsystem: visionSub pointer cannot be null!");
    }
    if (!gpdetection_) {
        throw std::runtime_error("VisionSubsystem: gpdetection pointer cannot be null!");
    }

    vision_table_ = nt::NetworkTableInstance::GetDefault().GetTable("Vision");

}

void VisionSubsystem::Periodic() {
  try {

  LimelightMeasurement();
  LED_control();
  frc::SmartDashboard::PutNumber("vision_mode", vision_mode_);

  } catch (const std::exception& e) {
    std::cout << "VisionSubsystem Periodic Failed: " << e.what() << std::endl;
}

}

  void VisionSubsystem::UpdateAngularVelocity() {
    // 不使用机器人角度进行计算，而直接调用陀螺仪
    currentAngularVelocity_ = drivetrain_->GetPigeon2()
                                  .GetAngularVelocityZDevice()
                                  .GetValueAsDouble();  // 获取当前角速度
}

void VisionSubsystem::SetLimelightIMUMode(std::string limelightname_, LimelightIMUMode mode)
{
  currentIMUMode = mode;
  switch (currentIMUMode)
  {
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
      drivetrain_->GetcurrentPose().Rotation().Degrees().value(),
      0, 0, 0, 0, 0);
}


bool VisionSubsystem::ShouldRejectMetatagPose(const LimelightHelpers::PoseEstimate& pose_estimate) {
  bool reject = false;  // 默认不拒�?
  std::string reason;   // 记录拒绝原因

  // 检�?tag 数量
  if (pose_estimate.tagCount == 0) {
    reject = true;
    reason = "No Tags";
  }
  // 检查角速度是否过高
  if (std::abs(currentAngularVelocity_) > kMaxAngularVelocity_)
  {
    reject = true;
    reason += reason.empty() ? "High Angular Velocity" : " & High Angular Velocity";
  }
  // 检�?ambiguity（模糊度），仅在 tagCount > 0 时有�?
  if (pose_estimate.tagCount > 0 && pose_estimate.rawFiducials[0].ambiguity > 0.7) {
    reject = true;
    reason += reason.empty() ? "High Ambiguity" : " & High Ambiguity";
  }
  // 检查距离，确保在合理范围内，仅�?tagCount > 0 时有�?
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

void VisionSubsystem::UpdateVisionMode() {
    
  // 检�?mt2 数据有效�?
  if (ShouldRejectMetatagPose(mt2_left_pose_)) {
    vision_mode_ = 0;  // 视觉不通过
    return;
  }

  // 检查混合模式条�?
  if (mt2_left_pose_.rawFiducials[0].distToCamera < switch_distance_ &&
      !ShouldRejectMetatagPose(mt1_left_pose_)) {
    vision_mode_ = 2;  // 混合模式
  } else {
    vision_mode_ = 1;  // 仅使�?mt2
  }
      

}


  void VisionSubsystem::LimelightMeasurement() {
    UpdateAngularVelocity();
    
    // 设置当前 IMU 模式
    currentIMUMode = frc::DriverStation::IsDisabled()
                         ? LimelightIMUMode::SeedingMode
                         : LimelightIMUMode::FusedIMU;
    SetLimelightIMUMode(limelight_left_name_, currentIMUMode);
    frc::SmartDashboard::PutNumber("limelight_imu_mode",
                                   static_cast<int>(currentIMUMode));

    // 获取 Limelight 数据
    // 左边limelight
    auto mt1_left_optional =
        LimelightHelpers::getBotPoseEstimate_wpiBlue(limelight_left_name_);
    auto mt2_left_optional =
        LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2(
            limelight_left_name_);
    mt1_left_pose_ = mt1_left_optional.value_or(LimelightHelpers::PoseEstimate{});
    mt2_left_pose_ = mt2_left_optional.value_or(LimelightHelpers::PoseEstimate{});


    if(mt1_left_optional.has_value()) {
    Test(mt1_left_pose_, mt2_left_pose_.pose);
  }

    UpdateVisionMode();

    if(vision_mode_) { // 视觉通过不为0

      /**
       * @brief 以下代码来自581，别动！
       */
      double avgDistance = mt2_left_pose_.avgTagDist;
      double xyDev = 0.01 * std::pow(avgDistance, 1.2);
      double thetaDev = 0.03 * std::pow(avgDistance, 1.2);
      std::array<double, 3> estStdDevs = {xyDev, xyDev, thetaDev};

      switch (vision_mode_) {
        case 1: // 使用 mt2 更新
          estStdDevs[2] = 10000000; // 使用外部imu时不信任limelight的yaw�?
          drivetrain_->AddVisionMeasurement(
              mt2_left_pose_.pose,
              mt2_left_pose_.timestampSeconds,
              std::array{estStdDevs[0], estStdDevs[1], estStdDevs[2]});
          break;

        case 2: {// 混合模式，在近处disable下使用mt1的yaw进行校准
          if (disable_mix) {
          auto mt_left_mix = frc::Pose2d{mt2_left_pose_.pose.Translation(), mt1_left_pose_.pose.Rotation()};
          drivetrain_->AddVisionMeasurement(
              mt_left_mix,
              mt2_left_pose_.timestampSeconds,
              std::array{estStdDevs[0], estStdDevs[1], estStdDevs[2]});
            }
            else {
                  estStdDevs[2] = 10000000; // 使用外部imu时不信任limelight的yaw�?
              drivetrain_->AddVisionMeasurement(
                  mt2_left_pose_.pose,
                  mt2_left_pose_.timestampSeconds,
                  std::array{estStdDevs[0], estStdDevs[1], estStdDevs[2]});
            }
      }      
          break;
        
        default:
          break;
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




void VisionSubsystem::Test(LimelightHelpers::PoseEstimate mt1, frc::Pose2d mt2) {
  static constexpr size_t kYawWindow = 100;
  static std::array<double, kYawWindow> mt1_samples{};
  static std::array<double, kYawWindow> mt2_samples{};
  static size_t mt1_index = 0;
  static size_t mt2_index = 0;
  static size_t mt1_count = 0;
  static size_t mt2_count = 0;

  auto push_sample = [](std::array<double, kYawWindow> &samples,
                        size_t &index, size_t &count, double value) {
    const size_t window = samples.size();
    samples[index] = value;
    index = (index + 1) % window;
    if (count < window) {
      ++count;
    }
  };

  auto stddev = [](const std::array<double, kYawWindow> &samples,
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
  double mt1_yaw = mt1_left_pose_.pose.Rotation().Degrees().value();
  double mt2_yaw = drivetrain_->GetcurrentPose().Rotation().Degrees().value();

  push_sample(mt1_samples, mt1_index, mt1_count, mt1_yaw);
  push_sample(mt2_samples, mt2_index, mt2_count, mt2_yaw);

  frc::SmartDashboard::PutNumber("mt1_yaw", mt1_yaw);
  frc::SmartDashboard::PutNumber("mt1_yaw_stddev",
                                 stddev(mt1_samples, mt1_count));
  frc::SmartDashboard::PutNumber("mt2_yaw_stddev",
                                 stddev(mt2_samples, mt2_count));

  frc::SmartDashboard::PutNumber(
      "pigeon_yaw",
      drivetrain_->GetcurrentPose().Rotation().Degrees().value());

  if (vision_table_) {
    vision_table_->GetStructTopic<frc::Pose2d>("MT2Pose").Publish().Set(mt2);
    vision_table_->GetStructTopic<frc::Pose2d>("MT1Pose").Publish().Set(mt1_left_pose_.pose);

  }
}

