#pragma once

#include <optional>

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>
#include "LimelightHelpers.h"
#include "RobotContainer.h"
#include "Constants.h"
#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/Timer.h>
#include <frc2/command/CommandScheduler.h>
#include <networktables/StructTopic.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/StringTopic.h>
#include <networktables/NetworkTableInstance.h>

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
  void SimulationInit() override;

private:
  frc2::Command *m_autonomousCommand;

  RobotContainer m_container;

  std::shared_ptr<nt::NetworkTable> PIDTable = nt::NetworkTableInstance::GetDefault().GetTable("Simulation");

  nt::StructArrayPublisher<frc::Pose3d> zeroed_component_pose = PIDTable->GetStructArrayTopic<frc::Pose3d>("ZeroedComponentPose").Publish();
  nt::StructPublisher<frc::Pose3d> final_component_pose = PIDTable->GetStructTopic<frc::Pose3d>("FinalComponentPose").Publish();
};
