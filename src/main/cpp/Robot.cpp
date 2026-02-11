// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "LimelightHelpers.h"

#include <frc2/command/CommandScheduler.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/math.h>

Robot::Robot() {}

void Robot::RobotPeriodic() {
  m_timeAndJoystickReplay.Update();
  frc2::CommandScheduler::GetInstance().Run();

}

void Robot::DisabledInit() {
  // Seed IMU
  m_container.clientSub.PubRobotInit(0);
  m_container.visionSub.disable_mix = 1;
}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
  nt::NetworkTableInstance::GetDefault()
      .GetTable("limelight-front")
      ->PutNumber("throttle_set", 0);
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand) {
    frc2::CommandScheduler::GetInstance().Schedule(m_autonomousCommand);
  }
}

void Robot::AutonomousPeriodic() {
  try {
  } catch (const std::exception &e) {
    std::cout << "auto Periodic Failed: " << e.what() << std::endl;
  }
}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
  nt::NetworkTableInstance::GetDefault()
      .GetTable("limelight-front")
      ->PutNumber("throttle_set", 0);

  m_container.clientSub.PubRobotInit(1);
  if (m_autonomousCommand) {
    frc2::CommandScheduler::GetInstance().Cancel(m_autonomousCommand);
  }
  m_container.visionSub.disable_mix = 0;

}

void Robot::TeleopPeriodic() {
  try {
  } catch (const std::exception &e) {
    std::cout << "pathplanner Periodic Failed: " << e.what() << std::endl;
  }
}

void Robot::TeleopExit() {}

void Robot::TestInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

void Robot::SimulationInit() {
  frc::SmartDashboard::PutData("Field", &m_simField);
}

void Robot::SimulationPeriodic() {
  m_simField.SetRobotPose(m_container.drivetrain.GetState().Pose);
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
