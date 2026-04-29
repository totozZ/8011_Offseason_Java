#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/filter/LinearFilter.h>
#include "subsystems/CommandSwerveDrivetrain.h" 
#include "subsystems/ShooterSubsystem.h" 
#include <ctre/phoenix6/swerve/SwerveRequest.hpp>
using namespace subsystems;
class RealTimeAimDrive : public frc2::CommandHelper<frc2::Command, RealTimeAimDrive> {
 public:
  RealTimeAimDrive(CommandSwerveDrivetrain* drive, ShooterSubsystem* sh, std::function<double()> vx, std::function<double()> vy, double AOSDeg);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

 private:
  CommandSwerveDrivetrain* m_drive;
  ShooterSubsystem*  m_shooter;
  // frc::PIDController m_aimPID{8, 0.2, 0.3}; 
  //frc::PIDController m_aimPID{2, 0.2, 0.3}; 
  units::meters_per_second_t MaxSpeed = TunerConstants::kSpeedAt12Volts;
  units::radians_per_second_t MaxAngularRate = 0.75_tps;

  std::function<double()> m_vXSupplier;//传入手柄数值函数控制移动
  std::function<double()> m_vYSupplier;

  swerve::requests::FieldCentricFacingAngle driveClosed;
  swerve::requests::SwerveDriveBrake driveBrake;
                    
  double angleOfShooter;
  bool useClosedLoop = true;
};