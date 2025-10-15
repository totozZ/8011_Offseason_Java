#include "subsystems/GPDetection.h"
using namespace subsystems;

GPDetection::GPDetection(CommandSwerveDrivetrain *drivetrain) : m_drivetrain(drivetrain)
{
  for (const auto &ll_name : LLConstants::DETECTION_LL_NAMES)
  {
    LL_NT_table = nt::NetworkTableInstance::GetDefault().GetTable(std::string(ll_name));
  }
  GP_detection_NT_table = nt::NetworkTableInstance::GetDefault().GetTable("GPDetectionSubsystem");
}

void GPDetection::Periodic()
{
  try
  {
    // https://github.com/ArchdukeTim/2025-Firefly/blob/main/GPDetection/GPDetection.cpp
    object_seen = LL_NT_table->GetString("tdclass", "nothing");
    if (object_seen == "algae")
    {
      std::vector<double> t2dValues = LL_NT_table->GetNumberArray("t2d", std::vector<double>(17));
      ty = LL_NT_table->GetNumber("ty", ty);    // 垂直偏角
      tx = -(LL_NT_table->GetNumber("tx", tx)); // 水平偏角
      distance = caculateDistance(ty);
      GP_detection_NT_table->PutNumber("distance2", distance);
      result = caculateTargetPose(distance, tx);
      frc::SmartDashboard::PutNumberArray("Pose", std::array{result.X().value(), result.Y().value(), result.Rotation().Degrees().value()});
      // GP_detection_NT_table->GetStructTopic<frc::Pose2d>("Pose")
      //     .Publish()
      //     .Set(caculateTargetPose(caculateDistance2(ty), tx));
    }
  }
  catch (const std::exception &e)
  {
    GP_detection_NT_table->PutString("Periodic Error", e.what());
  }
}

// https://github.com/zzhangje/ChronosChain/blob/master/src/main/java/frc/robot/subsystem/vision/ObjectDetectionVision.java
double GPDetection::caculateDistance(double ty)
{
  double height_difference = GPDetectionConstants::HEIGHT_OF_LOLLIPOP - GPDetectionConstants::HEIGHT_OF_CAMERA;
  double angle = ty + GPDetectionConstants::YAW_OF_CAMERA;
  return height_difference / tan(angleToRadius(angle));
}

// https://github.com/zzhangje/ChronosChain/blob/master/src/main/java/frc/robot/subsystem/vision/ObjectDetectionVision.java
frc::Pose2d GPDetection::caculateTargetPose(double distance, double tx)
{
  double yaw = 180 + tx;                                                                 // LL 在机器后方
  frc::Transform2d cam_to_target{{distance * 1_m, frc::Rotation2d{yaw * 1_deg}}, 0_deg}; // LL 到物体的偏移
  frc::Transform2d robot_to_target = cam_to_target + LLConstants::ROBOT_TO_DETECTION_LL; // 机器到物体的偏移
  frc::Pose2d current_pose = m_drivetrain->GetState().Pose;
  return current_pose.TransformBy(robot_to_target); // 物体在场地的坐标
}

double GPDetection::angleToRadius(double angle)
{
  return angle * std::numbers::pi / 180.0;
}