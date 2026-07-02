#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/controller/PIDController.h>
#include "subsystems/CommandSwerveDrivetrain.h"
#include "subsystems/GroundIntakeSubsystem.h"
#include "frc2/command/button/CommandXboxController.h"
using namespace subsystems;

class intakeNextToWall : public frc2::CommandHelper<frc2::Command, intakeNextToWall> {
public:
  intakeNextToWall(CommandSwerveDrivetrain* drive, GroundIntakeSubsystem* g,
                   frc2::CommandXboxController* j, bool oppo);
  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End(bool interrupted) override;

private:
  frc2::CommandXboxController* joy;
  CommandSwerveDrivetrain* m_drive;
  GroundIntakeSubsystem* m_ground;
  bool opposite;
  bool isRed;

  // 状态标记
  bool arrivedAtFirstPoint = false; 
  bool arrivedAt0=false;
  bool finishedAll = false;
  bool stopRightAway=false;
  double targetSpeed=0;
  // 目标点 1 (途径点)
  double targetX_0 = 0;
  double targetY_0 = 0;
  double targetR_0=0;

  double targetX_1 = 0;
  double targetY_1 = 0;
  double targetR_1=0;
  // 目标点 2 (最终吸球点/墙边点)
  double targetX_2 = 0;
  double targetY_2 = 0;
  double targetR_2=0;

  double rawTargetRotation = 0;
  double targetDire = 0;
  units::meters_per_second_t MaxSpeed = TunerConstants::kSpeedAt12Volts; // 确保你有这个常量

  frc::PIDController m_movePIDX{3.0, 0.0, 0.1}; // 请根据你的实际情况调整 PID
  frc::PIDController m_movePIDY{3.0, 0.0, 0.1};

  ctre::phoenix6::swerve::requests::FieldCentricFacingAngle driveClosed;
};
