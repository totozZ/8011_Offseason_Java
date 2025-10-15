#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/GenericEntry.h>
#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/Timer.h>
#include <vector>
#include "Constants.h"
#include <networktables/NetworkTableInstance.h>
#include "subsystems/CommandSwerveDrivetrain.h"
#include "ctre/phoenix6/swerve/SwerveDrivetrain.hpp"
#include <networktables/StructTopic.h>
#include <iostream>

namespace subsystems
{
  class GPDetection : public frc2::SubsystemBase
  {
  private:
    CommandSwerveDrivetrain *m_drivetrain;

    std::shared_ptr<nt::NetworkTable> LL_NT_table;
    std::shared_ptr<nt::NetworkTable> GP_detection_NT_table;

    std::string object_seen;

    double ty = 0;
    double tx = 0;
    double angle = 0;
    double distance = 0;
    frc::Pose2d result;

  public:
    void Periodic() override;
    GPDetection(CommandSwerveDrivetrain *drivetrain);
    double angleToRadius(double angle);
    double caculateDistance(double ty);
    frc::Pose2d caculateTargetPose(double distance, double tx);
  };
}