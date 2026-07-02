// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
    private final double maxSpeedMetersPerSecond =
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double maxAngularRateRadiansPerSecond =
            RotationsPerSecond.of(0.95).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric driveClosed = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeedMetersPerSecond * 0.07)
            .withRotationalDeadband(maxAngularRateRadiansPerSecond * 0.05)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.Position);
    private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

    private final Telemetry logger = new Telemetry(maxSpeedMetersPerSecond);
    private final CommandXboxController joystick =
            new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final Command doNothingCommand = Commands.none().withName("Do Nothing");
    private final SendableChooser<Command> autoChooser;

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureNamedCommands();
        autoChooser = buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();
    }

    private void configureNamedCommands() {
        NamedCommands.registerCommand("Intake", Commands.none().withName("Intake placeholder"));
        NamedCommands.registerCommand(
                "ShootTower",
                Commands.none().withName("ShootTower placeholder").withTimeout(3.0));
        NamedCommands.registerCommand(
                "StopAll",
                Commands.runOnce(this::safeStopMechanisms, shooter, drivetrain));
    }

    private SendableChooser<Command> buildAutoChooser() {
        SendableChooser<Command> chooser = AutoBuilder.isConfigured()
                ? AutoBuilder.buildAutoChooser("Do Nothing")
                : new SendableChooser<>();
        chooser.setDefaultOption("Do Nothing (Safe)", doNothingCommand);
        return chooser;
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                driveClosed
                    .withVelocityX(
                        -joystick.getLeftY()
                            * maxSpeedMetersPerSecond
                            * Constants.OperatorConstants.speedRate)
                    .withVelocityY(
                        -joystick.getLeftX()
                            * maxSpeedMetersPerSecond
                            * Constants.OperatorConstants.speedRate)
                    .withRotationalRate(
                        -joystick.getRightX()
                            * maxAngularRateRadiansPerSecond
                            * Constants.OperatorConstants.angularSpeedRate)
            )
        );

        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idleRequest).ignoringDisable(true)
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        Command selected = autoChooser.getSelected();
        return selected != null ? selected : doNothingCommand;
    }

    public void updateDriverPerspective() {
        drivetrain.updateDriverPerspective();
    }

    public void onAutonomousExit() {
        safeStopMechanisms();
    }

    public void onTeleopInit() {
        drivetrain.setControl(idleRequest);
    }

    public void safeStopMechanisms() {
        shooter.stop();
        drivetrain.setControl(idleRequest);
    }
}
