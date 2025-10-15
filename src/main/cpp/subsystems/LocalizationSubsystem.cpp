#include "subsystems/LocalizationSubsystem.h"
using namespace subsystems;

LocalizationSubsystem::LocalizationSubsystem(CommandSwerveDrivetrain *drivetrain) : m_drivetrain(drivetrain)
{
  locationInit();
  setIMUMode(0);
}

void LocalizationSubsystem::Periodic()
{
  updateAngularVelocity();
  switchVisionMode();
  setRobotOrientation();
  updatePoseEstimator();
}

// 对 LL 设置当前 Yaw
void LocalizationSubsystem::setRobotOrientation()
{
  for (const auto &ll_name : LLConstants::LOCALIZATION_LL_NAMES)
  {
    LimelightHelpers::SetRobotOrientation(std::string(ll_name), m_drivetrain->GetState().Pose.Rotation().Degrees().value(), current_angular_velocity, 0, 0, 0, 0);
  }
}

// 设置 IMU 模式
void LocalizationSubsystem::setIMUMode(int mode)
{
  // Mode 0：使用外部 IMU
  // Mode 1：使用外部 IMU 播种内部 IMU
  // Mode 2：使用内部 IMU
  // Mode 3: 使用内部 IMU，并使用 MT1 辅助收敛
  // Mode 4：使用内部IMU，并使用外部IMU辅助收敛
  for (const auto &ll_name : LLConstants::LOCALIZATION_LL_NAMES)
  {
    LimelightHelpers::setLimelightNTDouble(std::string(ll_name), "imumode_set", mode);
  }
}

// 判断是否拒绝 MT2 的结果
bool LocalizationSubsystem::shouldRejectMT2(const LimelightHelpers::PoseEstimate &poseEstimate)
{
  // 检查角速度是否过高
  if (current_angular_velocity > LocalizationConstants::MAX_ANGULAR_VELOCITY)
  {
    frc::SmartDashboard::PutString("MT2_Reject_Reason", "High Angular Velocity");
    return true;
  }

  // 检查是否有 Tag
  if (poseEstimate.tagCount == 0)
  {
    frc::SmartDashboard::PutString("MT2_Reject_Reason", "No Tags");
    return true;
  }

  // 检查 Tag 是否过远（通过 Tag 面积判断）
  if (poseEstimate.avgTagArea < LocalizationConstants::MIN_AVG_TAG_AREA)
  {
    frc::SmartDashboard::PutString("MT2_Reject_Reason", "Tags Too Far");
    return true;
  }

  frc::SmartDashboard::PutString("MT2_Reject_Reason", "None");
  return false;
}

// 判断是否拒绝 MT1 的结果
bool LocalizationSubsystem::shouldRejectMT1(const LimelightHelpers::PoseEstimate &poseEstimate)
{
  // 检查是否有 Tag
  if (poseEstimate.tagCount == 0)
  {
    frc::SmartDashboard::PutString("MT1_Reject_Reason", "No Tags");
    return true;
  }

  // 检查模糊度是否过高（模糊度越高，越不相信视 LL 结果）
  if (poseEstimate.rawFiducials.size() > 0 &&
      poseEstimate.rawFiducials[0].ambiguity > LocalizationConstants::MAX_AMBIGUITY)
  {
    frc::SmartDashboard::PutString("MT1_Reject_Reason", "High Ambiguity");
    return true;
  }

  // 检查距离是否过远（距离越远，越不相信视 LL 结果）
  if (poseEstimate.rawFiducials.size() > 0 &&
      poseEstimate.rawFiducials[0].distToCamera > LocalizationConstants::MAX_DISTANCE_TO_CAMERA)
  {
    frc::SmartDashboard::PutString("MT1_Reject_Reason", "DistToCamera");
    return true;
  }

  frc::SmartDashboard::PutString("MT1_Reject_Reason", "None");
  return false;
}

// 更新角速度
void LocalizationSubsystem::updateAngularVelocity()
{
  current_pose = m_drivetrain->GetState().Pose;
  current_yaw = current_pose.Rotation().Degrees().value();
  units::second_t current_time = frc::Timer::GetFPGATimestamp();

  if (last_yaw_update_time > 0_s)
  {
    delta_yaw = current_yaw - last_yaw;
    deltaTime = current_time - last_yaw_update_time;

    if (deltaTime.value() > 180.0)
    {
      delta_yaw -= 360.0;
    }

    if (deltaTime.value() < -180.0)
    {
      delta_yaw += 360.0;
    }

    if (deltaTime.value() > 0.0)
    {
      current_angular_velocity = std::abs(delta_yaw / deltaTime.value());
    }
  }

  last_yaw = current_yaw;
  last_yaw_update_time = current_time;
}

// 更新当前位置（使用 Pose Estimator）
// https://github.com/team581/2025-beta/blob/main/src/main/java/frc/robot/vision/limelight/Limelight.java
void LocalizationSubsystem::updatePoseEstimator()
{
  // 储存 MT1 和 MT2 的结果
  for (const auto &ll_name : LLConstants::LOCALIZATION_LL_NAMES)
  {
    mt1_left = LimelightHelpers::getBotPoseEstimate_wpiBlue(std::string(ll_name));
    mt2_left = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2(std::string(ll_name));
  }

  // 判断是否已经初始化位置
  if (!location_init_flag)
  {
    locationInit();
  }
  else if (!shouldRejectMT2(mt2_left))
  {
    // 使用 LL 更新时 LED State 为 1
    LED_state = 1;

    // 使用 Distance 计算权重，Distance 越大，越不相信视 LL 结果
    avg_distance = mt2_left.avgTagDist;
    xy_dev = LocalizationConstants::XY_DEV * std::pow(avg_distance, 1.2);
    theta_dev = LocalizationConstants::THETA_DEV * std::pow(avg_distance, 1.2);

    std::array<double, 3> estStdDevs = {xy_dev, xy_dev, theta_dev};
    if (is_using_mt1_yaw)
    {
      // 使用 MT2 Pose + MT1 Yaw
      mt_mix_left = frc::Pose2d{mt2_left.pose.Translation(), mt1_left.pose.Rotation()};
      m_drivetrain->AddVisionMeasurement(
          mt_mix_left,
          ctre::phoenix6::utils::FPGAToCurrentTime(mt2_left.timestampSeconds),
          estStdDevs);
    }
    else
    {
      // 使用 MT2 Pose + Yaw
      // MT2 依赖来自 Robot Pose 的 Yaw，不需要通过视觉更新，所以把 Yaw 的权重调到很大
      estStdDevs[2] = 1000000;
      m_drivetrain->AddVisionMeasurement(
          mt2_left.pose,
          ctre::phoenix6::utils::FPGAToCurrentTime(mt2_left.timestampSeconds),
          estStdDevs);
    }
  }
  else
  {
    // 不使用 LL 更新时 LED State 为 0
    LED_state = 0;
  }
}

// 选择更新方法
void LocalizationSubsystem::switchVisionMode()
{
  if (mt2_left.rawFiducials.size() > 0 &&
      (mt2_left.rawFiducials[0].distToCamera < LocalizationConstants::DISTANCE_SWITCH_TO_MT1_YAW) &&
      !shouldRejectMT1(mt1_left))
  {
    // Tag 到底盘时小于 1 米，同时不拒绝 MT1 时，使用 MT2 Pose + MT1 Yaw
    is_using_mt1_yaw = true;
  }
  else
  {
    // 不然使用MT2 Pose + Yaw
    is_using_mt1_yaw = false;
  }
  frc::SmartDashboard::PutNumber("is_using_mt1_yaw", is_using_mt1_yaw);
}

// 初始位置
void LocalizationSubsystem::locationInit()
{
  if (location_init_flag == 0)
  {
    // 使用首个有效的 MT1 值作为初始值
    if (!shouldRejectMT1(mt1_left))
    {
      m_drivetrain->ResetPose(mt1_left.pose);
      for (const auto &ll_name : LLConstants::LOCALIZATION_LL_NAMES)
      {
        LimelightHelpers::SetRobotOrientation(std::string(ll_name), mt1_left.pose.Rotation().Degrees().value(), 0, 0, 0, 0, 0);
      }
      location_init_flag = 1;
    }
  }
}