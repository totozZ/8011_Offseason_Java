// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import frc.robot.config.DriverControls;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.KitBotFuelSubsystem;

/** Constructs the robot subsystems, operator bindings, and autonomous chooser. */
public class RobotContainer implements AutoCloseable {
    private static final DrivetrainProfile DRIVETRAIN_PROFILE =
            Constants.DriveConstants.ACTIVE_PROFILE;
    private final SwerveRequest.RobotCentric driveRequest =
            CommandSwerveDrivetrain.createRobotCentricRequest();
    private final SwerveRequest.SwerveDriveBrake brakeRequest =
            new SwerveRequest.SwerveDriveBrake();

    private final CommandXboxController driverController = new CommandXboxController(
            Constants.OperatorConstants.DRIVER_CONTROLLER_PORT);
    private final Command doNothingCommand = Commands.none().withName("Do Nothing");

    public final CommandSwerveDrivetrain drivetrain = DRIVETRAIN_PROFILE.createDrivetrain();
    private final Telemetry telemetry = new Telemetry();
    final KitBotFuelSubsystem fuel = new KitBotFuelSubsystem();
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
        drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> {
            // A default command can also run during autonomous waits or Do Nothing.
            if (!DriverStation.isTeleopEnabled()) {
                return driveRequest.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(0.0);
            }
            var speeds = DriverControls.toRobotSpeeds(
                    -driverController.getLeftY(),
                    -driverController.getLeftX(),
                    -driverController.getRightX());
            return driveRequest.withVelocityX(speeds.vxMetersPerSecond)
                    .withVelocityY(speeds.vyMetersPerSecond)
                    .withRotationalRate(speeds.omegaRadiansPerSecond);
        }));

        driverController.x().and(DriverStation::isTeleopEnabled).whileTrue(
                drivetrain.applyRequest(() -> brakeRequest).withName("SwerveXLock"));

        // Right trigger has priority if both triggers are held. Releasing it while the left
        // trigger remains held restarts intake automatically.
        var shootTrigger = driverController.rightTrigger()
                .and(DriverStation::isTeleopEnabled);
        driverController.leftTrigger()
                .and(driverController.rightTrigger().negate())
                .and(DriverStation::isTeleopEnabled)
                .whileTrue(fuel.holdIntakeCommand().withName("ManualIntake"));
        shootTrigger.whileTrue(fuel.holdShootCommand().withName("ManualShoot"));
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
