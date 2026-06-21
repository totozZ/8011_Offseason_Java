#pragma once

#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"
#include "Telemetry.h"
#include "commands/ComplexCommand.h"
#include "frc8011/ClientSubsystem.h"
#include "shooting/ShotTable.h"
#include "subsystems/CommandSwerveDrivetrain.h"
#include "subsystems/FeederSubsystem.h"
#include "subsystems/GroundIntakeSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/VisionSubsystem.h"

class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();
  void UpdateDriverPerspective();

  subsystems::CommandSwerveDrivetrain drivetrain{
      TunerConstants::CreateDrivetrain()};
  subsystems::VisionSubsystem visionSub;
  subsystems::ShooterSubsystem shooterSub;
  subsystems::FeederSubsystem feederSub;
  subsystems::GroundIntakeSubsystem groundIntakeSub;
  frc2::CommandXboxController joystick{0};
  ComplexCommand complexcommand;
  subsystems::ClientSubsystem client{&drivetrain};

 private:
  void ConfigureBindings();
  void ConfigureNamedCommands();
  frc2::CommandPtr MakeHubShootCommand();
  frc2::CommandPtr MakePassShootCommand();
  frc2::CommandPtr MakeFallbackShootCommand(
      shooting::ShotSetpoint setpoint);
  bool IsHubShootingRegion();
  bool UseOpponentRoute();

  units::meters_per_second_t max_speed_ = TunerConstants::kSpeedAt12Volts;
  units::radians_per_second_t max_angular_rate_ = 0.95_tps;
  swerve::requests::FieldCentric drive_closed_ =
      swerve::requests::FieldCentric{}
          .WithDeadband(max_speed_ * 0.07)
          .WithRotationalDeadband(max_angular_rate_ * 0.05)
          .WithDriveRequestType(swerve::DriveRequestType::Velocity)
          .WithSteerRequestType(swerve::SteerRequestType::Position);

  bool ground_intake_prepared_ = false;
  bool drive_like_tank_ = false;
  Telemetry logger_{max_speed_};
  frc::PIDController rotation_controller_{9, 0, 0.1};
  frc2::CommandPtr do_nothing_command_{frc2::cmd::None()};
  frc::SendableChooser<frc2::Command*> auto_chooser_;
};
