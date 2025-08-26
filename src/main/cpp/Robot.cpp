// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "LimelightHelpers.h"

#include <frc2/command/CommandScheduler.h>

Robot::Robot() {}

void Robot::RobotPeriodic()
{
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit()
{
  nt::NetworkTableInstance::GetDefault().GetTable("limelight-left")->PutNumber("throttle_set", 200);
  nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->PutNumber("throttle_set", 200);
}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit()
{
  if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed)
  {
    LimelightHelpers::SetFiducialIDFiltersOverride("limelight-left", std::vector<int>(RobotConstants::RED_VALID_APRILTAGS.begin(), RobotConstants::RED_VALID_APRILTAGS.end()));
  }
  else if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue)
  {
    LimelightHelpers::SetFiducialIDFiltersOverride("limelight-left", std::vector<int>(RobotConstants::BLUE_VALID_APRILTAGS.begin(), RobotConstants::BLUE_VALID_APRILTAGS.end()));
  }
  else
  {
    LimelightHelpers::SetFiducialIDFiltersOverride("limelight-left", std::vector<int>(RobotConstants::ALL_VALID_APRILTAGS.begin(), RobotConstants::ALL_VALID_APRILTAGS.end()));
  }
  nt::NetworkTableInstance::GetDefault().GetTable("limelight-left")->PutNumber("throttle_set", 0);
  nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->PutNumber("throttle_set", 0);

  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand)
  {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit()
{
  LimelightHelpers::SetFiducialIDFiltersOverride("limelight-left", std::vector<int>(RobotConstants::ALL_VALID_APRILTAGS.begin(), RobotConstants::ALL_VALID_APRILTAGS.end()));
  nt::NetworkTableInstance::GetDefault().GetTable("limelight-left")->PutNumber("throttle_set", 0);
  nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->PutNumber("throttle_set", 200);

  if (m_autonomousCommand)
  {
    m_autonomousCommand->Cancel();
  }
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::TestInit()
{
  LimelightHelpers::SetFiducialIDFiltersOverride("limelight-left", std::vector<int>(RobotConstants::ALL_VALID_APRILTAGS.begin(), RobotConstants::ALL_VALID_APRILTAGS.end()));
  nt::NetworkTableInstance::GetDefault().GetTable("limelight-left")->PutNumber("throttle_set", 0);
  nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->PutNumber("throttle_set", 0);
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
