#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/RobotModeTriggers.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <frc/RobotBase.h>
#include "simulation/SimulationManager.h"

RobotContainer::RobotContainer() : autoAlign(&drivetrain, &joystick), localization(&drivetrain), gpDetection(&drivetrain)
{
    autoChooser = pathplanner::AutoBuilder::buildAutoChooser("Tests");
    frc::SmartDashboard::PutData("Auto Mode", &autoChooser);

    ConfigureBindings();
}

void RobotContainer::ConfigureBindings()
{
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.SetDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.ApplyRequest([this]() -> auto &&
                                {
                                    return drive.WithVelocityX(-joystick.GetLeftY() * MaxSpeed)      // Drive forward with negative Y (forward)
                                        .WithVelocityY(-joystick.GetLeftX() * MaxSpeed)              // Drive left with negative X (left)
                                        .WithRotationalRate(-joystick.GetRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
                                }));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    frc2::RobotModeTriggers::Disabled().WhileTrue(
        drivetrain.ApplyRequest([]
                                { return swerve::requests::Idle{}; })
            .IgnoringDisable(true));

    // reset the field-centric heading on left bumper press
    joystick.Start().OnTrue(drivetrain.RunOnce([this]
                                               { drivetrain.SeedFieldCentric(); }));

    joystick.LeftBumper()
        .OnTrue(
            frc2::cmd::RunOnce([this]
                               {
                                   pid_align_command = autoAlign.PIDAlignCommand(subsystems::AutoAlignSubsystem::Position::LEFT);
                                   pid_align_command->Schedule(); }))
        .OnFalse(
            frc2::cmd::RunOnce([this]
                               {
                                   if (pid_align_command && pid_align_command->IsScheduled())
                                   {
                                       pid_align_command->Cancel();
                                   }
                                   pid_align_command.reset();
                                   autoAlign.reset(); }));

    joystick.RightBumper()
        .OnTrue(
            frc2::cmd::RunOnce([this]
                               {
                                   pid_align_command = autoAlign.PIDAlignCommand(subsystems::AutoAlignSubsystem::Position::RIGHT);
                                   pid_align_command->Schedule(); }))
        .OnFalse(
            frc2::cmd::RunOnce([this]
                               {
                                   if (pid_align_command && pid_align_command->IsScheduled())
                                   {
                                       pid_align_command->Cancel();
                                   }
                                   pid_align_command.reset();
                                   autoAlign.reset(); }));

    joystick.POVDown()
        .OnTrue(
            frc2::cmd::RunOnce([this]
                               {
                                   pid_align_command = autoAlign.PIDAlignCommand(subsystems::AutoAlignSubsystem::Position::CENTER);
                                   pid_align_command->Schedule(); }))
        .OnFalse(
            frc2::cmd::RunOnce([this]
                               {
                                   if (pid_align_command && pid_align_command->IsScheduled())
                                   {
                                       pid_align_command->Cancel();
                                   }
                                   pid_align_command.reset();
                                   autoAlign.reset(); }));
}

frc2::Command *RobotContainer::GetAutonomousCommand()
{
    return autoChooser.GetSelected();
}
