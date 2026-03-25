#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/filter/LinearFilter.h>
#include "subsystems/CommandSwerveDrivetrain.h" 
#include "subsystems/ShooterSubsystem.h" 
#include <ctre/phoenix6/swerve/SwerveRequest.hpp>
#include "subsystems/GroundIntakeSubsystem.h"
using namespace subsystems;
class intakeNextToHub : public frc2::CommandHelper<frc2::Command, intakeNextToHub> {
 public:
  intakeNextToHub(CommandSwerveDrivetrain* drive, ShooterSubsystem* sh, GroundIntakeSubsystem* g, bool Oppo);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

 private:
  bool opposite;
  CommandSwerveDrivetrain* m_drive;
  ShooterSubsystem*  m_shooter;
  GroundIntakeSubsystem* m_ground;
  frc::PIDController m_movePIDX{3, 0,0.1}; 
  frc::PIDController m_movePIDY{3,0,0.1};
  bool arrived=false;
  bool isRed=false;
  //frc::PIDController m_aimPID{2, 0.2, 0.3}; 
  units::meters_per_second_t MaxSpeed = TunerConstants::kSpeedAt12Volts;
  units::radians_per_second_t MaxAngularRate = 0.75_tps;
  double startY=0;
  double targetX=0;
  double rawTargetRotation=0;
  double targetY=0;
  double targetDire=0;
  double speedYDef=0;
  bool stopRightAway=false;
  swerve::requests::FieldCentricFacingAngle driveClosed;
};