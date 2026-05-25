#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/geometry/Translation2d.h>
#include <frc/Timer.h>
#include <functional>

// 引入你的所有子系统头文件 (请确保路径和文件名与你的工程一致)
#include "subsystems/CommandSwerveDrivetrain.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/FeederSubsystem.h"
#include "subsystems/GroundIntakeSubsystem.h"
using namespace subsystems;

class PassBallCommand: public frc2::CommandHelper<frc2::Command, PassBallCommand> {
 public:
  // 构造函数现在接收 4 个子系统的指针，以及 2 个手柄输入的 Supplier
  PassBallCommand(CommandSwerveDrivetrain* drive, 
                  ShooterSubsystem* shooter,
                  FeederSubsystem* feeder,
                  
                  std::function<double()> vx, 
                  std::function<double()> vy,
                double AOS);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

 private:
    double angleOfShooter;
  //frc::Timer m_timer;
  bool upOrDown=false;
  // 存放传入的 4 个子系统指针
  CommandSwerveDrivetrain* m_drive;
  ShooterSubsystem* m_shooter;
  FeederSubsystem* m_feeder;
  

  // 存放手柄输入
  std::function<double()> m_vXSupplier;
  std::function<double()> m_vYSupplier;
  units::meters_per_second_t MaxSpeed = TunerConstants::kSpeedAt12Volts;
  units::radians_per_second_t MaxAngularRate = 0.75_tps;
  // 底盘控制请求
  swerve::requests::FieldCentricFacingAngle driveClosed{};
  bool rightPos=false;
  bool rightRot=false;
  bool rightSpeed=false;
  // 🌟 蓝方的两个传球落点 (实战请根据场地测出准确坐标！)
  const frc::Translation2d kBlueLeftTarget{units::meter_t{3.0}, units::meter_t{5.5}};
  const frc::Translation2d kBlueRightTarget{units::meter_t{3.0}, units::meter_t{2.5}};
};