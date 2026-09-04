// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.auto.Auto;
import frc.robot.babyauto.BabyAuto;
import frc.robot.config.DrivetrainProfile;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.KitBotFuelSubsystem;

/** Constructs the robot subsystems, operator bindings, and autonomous chooser. */
public class RobotContainer implements AutoCloseable {
    private static final DrivetrainProfile DRIVETRAIN_PROFILE =
            Constants.DriveConstants.ACTIVE_PROFILE;
    private final double maxSpeedMetersPerSecond =
            DRIVETRAIN_PROFILE.speedAt12Volts().in(MetersPerSecond);
    private final double maxAngularRateRadiansPerSecond =
            Constants.DriveConstants.MAX_ANGULAR_RATE_RADIANS_PER_SECOND;

    private final SwerveRequest.FieldCentric driveRequest =
            new SwerveRequest.FieldCentric()
                    .withDeadband(
                            maxSpeedMetersPerSecond
                                    * Constants.OperatorConstants.TRANSLATION_DEADBAND)
                    .withRotationalDeadband(
                            maxAngularRateRadiansPerSecond
                                    * Constants.OperatorConstants.ROTATION_DEADBAND)
                    .withDriveRequestType(DriveRequestType.Velocity)
                    .withSteerRequestType(SteerRequestType.Position);
    private final SwerveRequest.SwerveDriveBrake brakeRequest =
            new SwerveRequest.SwerveDriveBrake();

    private final CommandXboxController driverController = new CommandXboxController(
            Constants.OperatorConstants.DRIVER_CONTROLLER_PORT);
    private final Command doNothingCommand = Commands.none().withName("Do Nothing");

    public final CommandSwerveDrivetrain drivetrain = DRIVETRAIN_PROFILE.createDrivetrain();
    private final Telemetry telemetry = new Telemetry();
    private final KitBotFuelSubsystem fuel = new KitBotFuelSubsystem();
    private final BabyAuto babyAuto = new BabyAuto(drivetrain, fuel);
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        drivetrain.registerTelemetry(telemetry::publish);
        configureBindings();
        autoChooser = buildAutoChooser();

        // This SendableChooser is the one approved /SmartDashboard namespace exception.
        SmartDashboard.putData(Constants.TelemetryConstants.AUTO_CHOOSER_KEY, autoChooser);

        // SysId is intentionally not bound by default. Lift the wheels and review the docs first.
        // configureSysIdBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> driveRequest
                .withVelocityX(
                        -driverController.getLeftY()
                                * maxSpeedMetersPerSecond
                                * Constants.OperatorConstants.DRIVE_SPEED_SCALE)
                .withVelocityY(
                        -driverController.getLeftX()
                                * maxSpeedMetersPerSecond
                                * Constants.OperatorConstants.DRIVE_SPEED_SCALE)
                .withRotationalRate(
                        -driverController.getRightX()
                                * maxAngularRateRadiansPerSecond
                                * Constants.OperatorConstants.TURN_SPEED_SCALE)));

        driverController.start().onTrue(
                Commands.runOnce(drivetrain::seedFieldCentric, drivetrain)
                        .withName("ResetDriverHeading"));
        driverController.x().whileTrue(
                drivetrain.applyRequest(() -> brakeRequest).withName("SwerveXLock"));
    }

    /**
     * Example SysId bindings. Never call this until the robot is secured with all wheels off the
     * ground and the selected routine is reviewed.
     */
    @SuppressWarnings("unused")
    private void configureSysIdBindings() {
        driverController.povUp().whileTrue(
                drivetrain.translationSysIdQuasistatic(SysIdRoutine.Direction.kForward));
        driverController.povDown().whileTrue(
                drivetrain.translationSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        driverController.povRight().whileTrue(
                drivetrain.translationSysIdDynamic(SysIdRoutine.Direction.kForward));
        driverController.povLeft().whileTrue(
                drivetrain.translationSysIdDynamic(SysIdRoutine.Direction.kReverse));

        // The steer and rotation routines remain available on CommandSwerveDrivetrain. Bind only
        // one routine at a time so an operator cannot select the wrong characterization by accident.
    }

    private SendableChooser<Command> buildAutoChooser() {
        SendableChooser<Command> chooser = new SendableChooser<>();
        chooser.setDefaultOption("Do Nothing", doNothingCommand);
        try {
            Command studentAuto = babyAuto.protect(Auto.build(babyAuto)).withName("Baby Auto");
            chooser.addOption("Baby Auto", studentAuto);
        } catch (RuntimeException exception) {
            DriverStation.reportError(
                    "Baby Auto build failed; using Do Nothing: "
                            + exception.getMessage(),
                    exception.getStackTrace());
        }
        return chooser;
    }

    public Command getAutonomousCommand() {
        Command selected = autoChooser.getSelected();
        return selected == null ? doNothingCommand : selected;
    }

    /** Immediately removes all activity-controlled outputs. */
    public void stopAll() {
        babyAuto.stopAll();
    }

    @Override
    public void close() {
        stopAll();
        fuel.close();
        telemetry.close();
        drivetrain.close();
    }
}
