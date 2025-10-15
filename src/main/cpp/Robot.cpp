#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <frc/DriverStation.h>

Robot::Robot() {}

void Robot::RobotPeriodic()
{
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit()
{
  for (const auto &ll_name : LLConstants::LOCALIZATION_LL_NAMES)
  {
    nt::NetworkTableInstance::GetDefault().GetTable(std::string(ll_name))->PutNumber("throttle_set", 200);
  }
  for (const auto &ll_name : LLConstants::DETECTION_LL_NAMES)
  {
    nt::NetworkTableInstance::GetDefault().GetTable(std::string(ll_name))->PutNumber("throttle_set", 200);
  }
}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit()
{
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand)
  {
    m_autonomousCommand->Schedule();
  }

  for (const auto &ll_name : LLConstants::LOCALIZATION_LL_NAMES)
  {
    nt::NetworkTableInstance::GetDefault().GetTable(std::string(ll_name))->PutNumber("throttle_set", 0);
  }
  for (const auto &ll_name : LLConstants::DETECTION_LL_NAMES)
  {
    nt::NetworkTableInstance::GetDefault().GetTable(std::string(ll_name))->PutNumber("throttle_set", 0);
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit()
{
  for (const auto &ll_name : LLConstants::LOCALIZATION_LL_NAMES)
  {
    nt::NetworkTableInstance::GetDefault().GetTable(std::string(ll_name))->PutNumber("throttle_set", 0);
  }
  for (const auto &ll_name : LLConstants::DETECTION_LL_NAMES)
  {
    nt::NetworkTableInstance::GetDefault().GetTable(std::string(ll_name))->PutNumber("throttle_set", 0);
  }
  if (m_autonomousCommand)
  {
    m_autonomousCommand->Cancel();
  }
}

void Robot::TeleopPeriodic()
{
  nt::NetworkTableInstance::GetDefault().GetTable("Info")->PutNumber("Time", frc::DriverStation::GetMatchTime().value());
}

void Robot::TeleopExit() {}

void Robot::TestInit()
{
  frc2::CommandScheduler::GetInstance().CancelAll();
  for (const auto &ll_name : LLConstants::LOCALIZATION_LL_NAMES)
  {
    nt::NetworkTableInstance::GetDefault().GetTable(std::string(ll_name))->PutNumber("throttle_set", 0);
  }
  for (const auto &ll_name : LLConstants::DETECTION_LL_NAMES)
  {
    nt::NetworkTableInstance::GetDefault().GetTable(std::string(ll_name))->PutNumber("throttle_set", 0);
  }
}

void Robot::SimulationInit()
{
  // Initialize simulation for the drivetrain
  m_container.drivetrain.SimulationInit();

  // 设置初始的零位组件位置
  std::vector<frc::Pose3d> zeroedPoses = {
      frc::Pose3d{-0.3006_m, 0_m, 0.25265_m, frc::Rotation3d{}},
      frc::Pose3d{-0.3006_m, 0_m, 0.25265_m, frc::Rotation3d{}}};
  zeroed_component_pose.Set(zeroedPoses);
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif