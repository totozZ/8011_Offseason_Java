// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/VisionSubsystem.h"

#include "subsystems/GPDetection.h"

#include <iostream>

using namespace subsystems;
using namespace posConstants;

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

  apriltags.April_Init();

}

frc2::CommandPtr VisionSubsystem::VisionMethodCommand() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return RunOnce([/* this */] { /* one-time action goes here */ });
}

bool VisionSubsystem::VisionCondition() {
  // Query some boolean state, such as a digital sensor.
  return false;
}

void VisionSubsystem::Periodic() {
  try {

  apriltags.GetVisionInfo();
  // Cal_Vision_pos();
  Get_tagpos();
  CalTarget_posleft();
  CalTarget_posright();
  LimelightMeasurement();
  LED_control();
  frc::SmartDashboard::PutNumber("vision_mode", vision_mode_);
  } catch (const std::exception& e) {
    std::cout << "VisionSubsystem Periodic Failed: " << e.what() << std::endl;
}

}

void VisionSubsystem::SimulationPeriodic() {

  // Implementation of subsystem simulation periodic method goes here.

}




void VisionSubsystem::Get_tagpos() {
// right�?2 17;left: 20 19
  // 根据视觉识别结果来确定tag的位�?
  switch(static_cast<int>(apriltags.Getapriltag_id())) {
    case 17:
      tag_pos = frc::Pose2d(pos_x2, pos_y2, frc::Rotation2d(60_deg));
      // tag_pos = frc::Pose2d(pos_x1, pos_y1, frc::Rotation2d(0_deg));
      break;
      case 8:
      tag_pos = frc::Pose2d(17.55_m - pos_x2, 8.05_m - pos_y2, frc::Rotation2d(-120_deg));
      // tag_pos = frc::Pose2d(pos_x1, pos_y1, frc::Rotation2d(0_deg));
      break;
    case 18:
      tag_pos = frc::Pose2d(pos_x1, pos_y1, frc::Rotation2d(0_deg));
      break;
    case 7:
      // tag_pos = frc::Pose2d(pos_x2, pos_y2, frc::Rotation2d(60_deg));
      tag_pos = frc::Pose2d(17.55_m - pos_x1, 8.05_m - pos_y1, frc::Rotation2d(180_deg));
      break;
    case 19:
      tag_pos = frc::Pose2d(pos_x2, pos_y3, frc::Rotation2d(-60_deg));
      break;
      case 6:
      tag_pos = frc::Pose2d(17.55_m - pos_x2, 8.05_m - pos_y3, frc::Rotation2d(120_deg)); // -60
      break;
    case 20:
      tag_pos = frc::Pose2d(pos_x3, pos_y3, frc::Rotation2d(-120_deg));
      break;
    case 11:
      tag_pos = frc::Pose2d(17.55_m - pos_x3, 8.05_m - pos_y3, frc::Rotation2d(60_deg));
      break;
    case 21:
      tag_pos = frc::Pose2d(pos_x4, pos_y1, frc::Rotation2d(180_deg));
      break;
    case 10:
    tag_pos = frc::Pose2d(17.55_m - pos_x4, 8.05_m - pos_y1, frc::Rotation2d(0_deg));
    break;
    case 22:
      tag_pos = frc::Pose2d(pos_x3, pos_y2, frc::Rotation2d(120_deg));
    break;
    case 9:
      tag_pos = frc::Pose2d(17.55_m - pos_x3, 8.05_m - pos_y2, frc::Rotation2d(-60_deg));
    break;
      default:
      tag_pos = tag_pos;
      break;
  }; 
}

frc::Pose2d VisionSubsystem::CalTarget_posleft() {


  Target_pos_left = frc::Pose2d{
    units::meter_t{tag_pos.Translation().X().value() - 
                  std::cos(tag_pos.Rotation().Radians().value()) * distance.value() - std::sin(tag_pos.Rotation().Radians().value()) * balldistance.value()},
                  
    units::meter_t{tag_pos.Translation().Y().value() - 
                  std::sin(tag_pos.Rotation().Radians().value()) * distance.value() + std::cos(tag_pos.Rotation().Radians().value()) * balldistance.value()},
    
    frc::Rotation2d{units::degree_t{tag_pos.Rotation().Degrees().value()}}

  };

  return Target_pos_left;
}

frc::Pose2d VisionSubsystem::CalTarget_posright() {
         
// 右边，左边球则第四个项相�?底盘中心最终的目标位姿，要减去底盘中心的距离以及左右珊瑚礁的偏�?
Target_pos_right = frc::Pose2d{
    units::meter_t{tag_pos.Translation().X().value() - 
                  std::cos(tag_pos.Rotation().Radians().value()) * distance.value() + std::sin(tag_pos.Rotation().Radians().value()) * balldistance_right.value()},
                  
    units::meter_t{tag_pos.Translation().Y().value() - 
                  std::sin(tag_pos.Rotation().Radians().value()) * distance.value() - std::cos(tag_pos.Rotation().Radians().value()) * balldistance_right.value()},
                  
    tag_pos.Rotation()
};

  return Target_pos_right;
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

  if (currentIMUMode == LimelightIMUMode::ExternalIMU) {
    LimelightHelpers::SetRobotOrientation(limelightname_, drivetrain_->GetcurrentPose().Rotation().Degrees().value(), currentAngularVelocity_, 0, 0, 0, 0);
  }
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
  if (mt2_left_pose_.rawFiducials[0].distToCamera < swiitch_distance_ &&
      !ShouldRejectMetatagPose(mt1_left_pose_)) {
    vision_mode_ = 2;  // 混合模式
  } else {
    vision_mode_ = 1;  // 仅使�?mt2
  }


}


  void VisionSubsystem::LimelightMeasurement() {
    UpdateAngularVelocity();
    
    // 设置当前 IMU 模式
    SetLimelightIMUMode(limelight_left_name_, currentIMUMode);

    // 获取 Limelight 数据
    // 左边limelight
    auto mt1_left_optional =
        LimelightHelpers::getBotPoseEstimate_wpiBlue(limelight_left_name_);
    auto mt2_left_optional =
        LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2(
            limelight_left_name_);
    mt1_left_pose_ = mt1_left_optional.value_or(LimelightHelpers::PoseEstimate{});
    mt2_left_pose_ = mt2_left_optional.value_or(LimelightHelpers::PoseEstimate{});

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
              ctre::phoenix6::utils::FPGAToCurrentTime(mt2_left_pose_.timestampSeconds),
              std::array{estStdDevs[0], estStdDevs[1], estStdDevs[2]});
          break;

        case 2: {// 混合模式
          auto mt_left_mix = frc::Pose2d{mt2_left_pose_.pose.Translation(), mt1_left_pose_.pose.Rotation()};
          drivetrain_->AddVisionMeasurement(
              mt_left_mix,
              ctre::phoenix6::utils::FPGAToCurrentTime(mt2_left_pose_.timestampSeconds),
              std::array{estStdDevs[0], estStdDevs[1], estStdDevs[2]});
          break;
        }
        default:
          break;
      }

    }
}

void VisionSubsystem::LED_control() {
      if (vision_mode_ == 0) {
      ledsub_->SetLEDState(LEDSubsystem::AnimationType::Blue);
      }
      else if (vision_mode_ == 1) {
        if (gpdetection_ && gpdetection_->GetValid()) {
        ledsub_->SetLEDState(LEDSubsystem::AnimationType::Green);
        } else {
        ledsub_->SetLEDState(LEDSubsystem::AnimationType::Red);
        }
        }
      else if (vision_mode_ == 2) {
      ledsub_->SetLEDState(LEDSubsystem::AnimationType::Purple);
      }
}



