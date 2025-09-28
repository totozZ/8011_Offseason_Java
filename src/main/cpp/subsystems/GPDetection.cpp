#include "subsystems/GPDetection.h"
#include "subsystems/CommandSwerveDrivetrain.h"
#include "ctre/phoenix6/swerve/SwerveDrivetrain.hpp"
#include <iostream>

using namespace subsystems;

std::map<std::string, GPDetection *> GPDetection::limelight_controls;

GPDetection::GPDetection(std::string in_name, CommandSwerveDrivetrain *drivetrain)
{
  limelight_controls[in_name] = this;
  NT_table = nt::NetworkTableInstance::GetDefault().GetTable("limelight-back");
}

void GPDetection::Periodic()
{
  try
  {
    object_seen = NT_table->GetString("tdclass", "nothing");
    if (object_seen == "algae")
    {
      std::vector<double> t2dValues = NT_table->GetNumberArray("t2d", std::vector<double>(17));
      GP_width_pixels = t2dValues[14];
      ty = NT_table->GetNumber("ty", ty);
      angle = GP_width_pixels / GPDetectionConstants::PIXELS_OF_CAMERA * GPDetectionConstants::HORIZONTAL_FOV;
      distance1 = GPDetectionConstants::ALGAE_RADIUS / (tan(angleToRadius(angle / 2)));
      distance2 = (GPDetectionConstants::HEIGHT_OF_LOLLIPOP - GPDetectionConstants::HEIGHT_OF_CAMERA) / tan(angleToRadius(ty + GPDetectionConstants::HORIZONTAL_YAW_OF_CAMERA));
      frc::SmartDashboard::PutNumber("distance1", distance1);
      frc::SmartDashboard::PutNumber("distance2", distance2);
    }
  }
  catch (const std::exception &e)
  {
    frc::SmartDashboard::PutString("GPDetection Periodic Failed: ", e.what());
  }
}

double GPDetection::angleToRadius(double angle)
{
  return angle * M_PI / 180.0;
}