#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/filter/LinearFilter.h>
#include "subsystems/CommandSwerveDrivetrain.h" 
#include "subsystems/ShooterSubsystem.h" 
#include "subsystems/FeederSubsystem.h"
#include <ctre/phoenix6/swerve/SwerveRequest.hpp>
using namespace subsystems;
class AutoRealTimeAimDrive : public frc2::CommandHelper<frc2::Command, AutoRealTimeAimDrive> {
 public:
  AutoRealTimeAimDrive(CommandSwerveDrivetrain* drive, ShooterSubsystem* sh, FeederSubsystem* fed, double AOSDeg);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

 private:
  FeederSubsystem* feeder;
  CommandSwerveDrivetrain* m_drive;
  ShooterSubsystem*  m_shooter;

  bool invertA=false;
  bool invertD=false;
  frc::PIDController m_xPID{3, 0, 0.1}; 
  frc::PIDController m_yPID{3, 0, 0.1}; 
  double maxSpeedCoeff=0.12;
  units::meters_per_second_t MaxSpeed = TunerConstants::kSpeedAt12Volts;
  units::radians_per_second_t MaxAngularRate = 0.75_tps;

      frc::Pose2d m_targetWaypoint;
    const double kTranslationTolerance = 0.1;
  double targetX=0;
  double targetY=0;
  double angleOfShooter;
  swerve::requests::FieldCentricFacingAngle driveClosed;
                    

  bool useClosedLoop = true;
};