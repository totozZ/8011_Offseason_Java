// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"
#include "Telemetry.h"
#include "commands/ComplexCommand.h"
#include "commands/intakeNextToHub.h"
#include "commands/AutoMoveOpen.h"
#include "commands/AutoMoveClosed.h"
#include "commands/RealTimeAimDrive.h"
#include "commands/intakeNextToWall.h"
#include "frc8011/ClientSubsystem.h"
#include "subsystems/CommandSwerveDrivetrain.h"
#include "subsystems/FeederSubsystem.h"
#include "subsystems/GroundIntakeSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/VisionSubsystem.h"
#include "subsystems/WeidaiSub.h"
#include "commands/Autos.h"
#include "commands/intakeNextToSide.h"
#include "commands/PassBallCommand.h"
#include "commands/AutoMoveCircle.h"

class RobotContainer
{
private:
  units::meters_per_second_t MaxSpeed = TunerConstants::kSpeedAt12Volts;
  units::radians_per_second_t MaxAngularRate = 0.75_tps;
  /* Setting up bindings for necessary control of the swerve drive platform */
  swerve::requests::FieldCentric drive = swerve::requests::FieldCentric{}
                                             .WithDeadband(MaxSpeed * 0.07)
                                             .WithRotationalDeadband(MaxAngularRate * 0.05)
                                             .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage);

  swerve::requests::FieldCentric driveClosed = swerve::requests::FieldCentric{}
                                                   .WithDeadband(MaxSpeed * 0.07)
                                                   .WithRotationalDeadband(MaxAngularRate * 0.05)
                                                   .WithDriveRequestType(swerve::DriveRequestType::Velocity)
                                                   .WithSteerRequestType(swerve::SteerRequestType::Position);

  bool useClosedLoop = false;
  bool ground_intake_prepared_ = false;
  bool shooterEnabled=false;
  Telemetry logger{ MaxSpeed };
  bool driveLikeTank=false;
  frc::PIDController rot{9,0,0.1};

  enum class AutoMode {
    kDoNothing,
    OutDepot,
    DoubleOutLeft,
    DoubleOutRight,
    OutAndHP,
    RedOutDepot,
    RedDoubleOutLeft,
    RedDoubleOutRight,
    RedOutAndHP,
    LowLeft,
    LowRight
  };
  
  
public:
  subsystems::CommandSwerveDrivetrain drivetrain{ TunerConstants::CreateDrivetrain() };
  subsystems::VisionSubsystem visionSub;
  subsystems::ShooterSubsystem shooterSub;
  subsystems::FeederSubsystem feederSub;
  subsystems::WeidaiSub weidaiSub;
  //subsystems::ClientSubsystem clientSub;
  subsystems::GroundIntakeSubsystem groundIntakeSub;
  frc2::CommandXboxController joystick{ 0 };
  ComplexCommand complexcommand;
  subsystems::ClientSubsystem client{&drivetrain};
  void UpdateDriverPerspective() {
    // 1. 获取当前 FMS 或 Driver Station 下发的联盟颜色
    auto alliance = frc::DriverStation::GetAlliance();

    // 2. 只有在成功获取到颜色的情况下才进行配置
    if (alliance.has_value()) {
        if (alliance.value() == frc::DriverStation::Alliance::kRed) {
            drivetrain.SetOperatorPerspectiveForward(frc::Rotation2d{180_deg});
        } else {
            // 如果是蓝方，告诉 CTRE 底盘：
            // “场地的 0 度方向（也就是 +X 方向），就是车手的正前方。”
            drivetrain.SetOperatorPerspectiveForward(frc::Rotation2d{0_deg});
        }
    }
}


// 2. 声明 SendableChooser 变量
  frc::SendableChooser<AutoMode> m_autoChooser;
// 记录上一次选的是哪条路线
  AutoMode m_lastSelectedAuto = AutoMode::kDoNothing;
  
  //用来装提前实例化的自动指令
  std::optional<frc2::CommandPtr> m_preloadedAuto;

  // 3. 把你原本写在 GetAutonomousCommand 里的 switch/if 逻辑抽离出来
  frc2::CommandPtr GenerateAutoCommand();
  void refreshAutoMode();
private:
  /* Path follower */
  frc::SendableChooser<frc2::Command*> autoChooser;

  std::optional<frc2::CommandPtr> pid_align_command;  // 当前正在执行的PID对齐Command

public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

private:
  void ConfigureBindings();
};
