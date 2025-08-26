#include "subsystems/LocalizationSubsystem.h"
using namespace subsystems;

LocalizationSubsystem::LocalizationSubsystem(CommandSwerveDrivetrain *drivetrain) : m_drivetrain(drivetrain)
{
  locationInit();
  setIMUMode(0);
}

void LocalizationSubsystem::Periodic()
{
  switchVisionMode();
  setRobotOrientation();
  updatePoseEstimator();
}

// 对Limelight设置当前Yaw
void LocalizationSubsystem::setRobotOrientation()
{
  LimelightHelpers::SetRobotOrientation("limelight-left", m_drivetrain->GetState().Pose.Rotation().Degrees().value(), current_angular_velocity, 0, 0, 0, 0);
}

// 设置IMU模式
void LocalizationSubsystem::setIMUMode(int mode)
{
  // Mode 0：使用外部IMU
  // Mode 1：使用外部IMU播种内部IMU
  // Mode 2：使用内部IMU
  // Mode 3: 使用内部IMU，并使用MT1辅助收敛
  // Mode 4：使用内部IMU，并使用外部IMU辅助收敛
  LimelightHelpers::setLimelightNTDouble("limelight-left", "imumode_set", mode);
}

// 判断是否拒绝MT2的结果
bool LocalizationSubsystem::shouldRejectMT2(const LimelightHelpers::PoseEstimate &poseEstimate)
{
  // 检查角速度是否过高
  if (current_angular_velocity > 360)
  {
    frc::SmartDashboard::PutString("MT2_Reject_Reason", "High Angular Velocity");
    return true;
  }

  // 检查是否有Tag
  if (poseEstimate.tagCount == 0)
  {
    frc::SmartDashboard::PutString("MT2_Reject_Reason", "No Tags");
    return true;
  }

  // 检查Tag是否过远（通过Tag面积判断）
  if (poseEstimate.avgTagArea < 0.1)
  {
    frc::SmartDashboard::PutString("MT2_Reject_Reason", "Tags Too Far");
    return true;
  }

  frc::SmartDashboard::PutString("MT2_Reject_Reason", "None");
  return false;
}

// 判断是否拒绝MT1的结果
bool LocalizationSubsystem::shouldRejectMT1(const LimelightHelpers::PoseEstimate &poseEstimate)
{
  // 检查是否有Tag
  if (poseEstimate.tagCount == 0)
  {
    frc::SmartDashboard::PutString("MT1_Reject_Reason", "No Tags");
    return true;
  }

  // 检查模糊度是否过高
  if (poseEstimate.rawFiducials[0].ambiguity > 0.7)
  {
    frc::SmartDashboard::PutString("MT1_Reject_Reason", "High Ambiguity");
    return true;
  }

  // 检查距离是否过远
  if (poseEstimate.rawFiducials[0].distToCamera > 3)
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

// 更新当前位置（使用Pose Estimator）
void LocalizationSubsystem::updatePoseEstimator()
{
  // 储存MT1和MT2的结果
  mt1_left = LimelightHelpers::getBotPoseEstimate_wpiBlue("limelight-left");
  mt2_left = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");

  // 判断是否已经初始位置
  if (!location_init_flag)
  {
    locationInit();
  }
  else if (!shouldRejectMT2(mt2_left))
  {
    // 使用Limelight更新时LED State为1
    LED_state = 1;

    // 使用Distance计算权重，Distance越大，越不相信视Limelight结果
    avg_distance = mt2_left.avgTagDist;
    xy_dev = 0.01 * std::pow(avg_distance, 1.2);
    theta_dev = 0.03 * std::pow(avg_distance, 1.2);

    std::array<double, 3> estStdDevs = {xy_dev, xy_dev, theta_dev};
    if (is_using_mt1_yaw)
    {
      // 使用MT2 Pose + MT1 Yaw
      mt_mix_left = frc::Pose2d{mt2_left.pose.Translation(), mt1_left.pose.Rotation()};
      m_drivetrain->AddVisionMeasurement(
          mt_mix_left,
          ctre::phoenix6::utils::FPGAToCurrentTime(mt2_left.timestampSeconds),
          std::array{estStdDevs[0], estStdDevs[1], estStdDevs[2]});
    }
    else
    {
      // 使用MT2 Pose + Yaw
      // MT2依赖来自Robot Pose的Yaw，不需要通过视觉更新，所以把Yaw的权重调到很大
      estStdDevs[2] = 1000000;
      m_drivetrain->AddVisionMeasurement(
          mt2_left.pose,
          ctre::phoenix6::utils::FPGAToCurrentTime(mt2_left.timestampSeconds),
          std::array{estStdDevs[0], estStdDevs[1], estStdDevs[2]});
    }
  }
  else
  {
    // 不使用Limelight更新时LED State为0
    LED_state = 0;
  }
}

// 选择更新方法
void LocalizationSubsystem::switchVisionMode()
{
  if ((mt2_left.rawFiducials[0].distToCamera < 1) && !shouldRejectMT1(mt1_left))
  {
    // Tag到底盘时小于1米，同时不拒绝MT1时，使用MT2 Pose + MT1 Yaw
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
    // 使用首个有效的MT1值作为初始值
    if (!shouldRejectMT1(mt1_left))
    {
      m_drivetrain->ResetPose(mt1_left.pose);
      LimelightHelpers::SetRobotOrientation("limelight-left", mt1_left.pose.Rotation().Degrees().value(), 0, 0, 0, 0, 0);
      location_init_flag = 1;
    }
  }
}