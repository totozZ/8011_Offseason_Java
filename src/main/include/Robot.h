#pragma once

#include <optional>

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>
#include "LimelightHelpers.h"
#include "RobotContainer.h"
#include "Constants.h"
#include <frc/DriverStation.h>

class Robot : public frc::TimedRobot
{
public:
  Robot();
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void DisabledExit() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void AutonomousExit() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TeleopExit() override;
  void TestInit() override;
  void TestPeriodic() override;
  void TestExit() override;

private:
  frc2::Command *m_autonomousCommand;

  RobotContainer m_container;
};
