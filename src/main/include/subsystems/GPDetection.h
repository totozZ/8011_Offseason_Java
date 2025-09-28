#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/GenericEntry.h>
#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>
#include <frc/Timer.h>
#include <vector>
#include "Constants.h"
#include <networktables/NetworkTableInstance.h>
#include "subsystems/CommandSwerveDrivetrain.h"

namespace subsystems
{
  class GPDetection : public frc2::SubsystemBase
  {
  private:
    CommandSwerveDrivetrain *m_drivetrain;

    std::shared_ptr<nt::NetworkTable> NT_table;
    static std::map<std::string, GPDetection *> limelight_controls;

    std::string object_seen;

    double GP_width_pixels = 0;
    double ty = 0;
    double angle = 0;
    double distance1 = 0;
    double distance2 = 0;

  public:
    void Periodic() override;
    GPDetection(std::string name, CommandSwerveDrivetrain *drivetrain);
    double angleToRadius(double angle);
  };
}