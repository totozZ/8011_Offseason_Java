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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.commands.ComplexCommands;
import frc.robot.commands.PassBallCommand;
import frc.robot.commands.RealTimeAimDrive;
import frc.robot.commands.ShootWithTableCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.shooting.AllianceSide;
import frc.robot.shooting.ShotSetpoint;
import frc.robot.shooting.ShotTable;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;
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
    private final FeederSubsystem feeder = new FeederSubsystem();
    private final GroundIntakeSubsystem groundIntake = new GroundIntakeSubsystem();
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final ComplexCommands complexCommand =
            new ComplexCommands(drivetrain, feeder, groundIntake);
    private final Command doNothingCommand = Commands.none().withName("Do Nothing");
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        configureNamedCommands();
        autoChooser = buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();
    }

    private void configureNamedCommands() {
        NamedCommands.registerCommand("Intake", complexCommand.groundIntakePrepareCommand());
        NamedCommands.registerCommand(
                "ShootTower",
                new ShootWithTableCommand(
                        shooter,
                        feeder,
                        ShotTable::tower,
                        () -> true)
                    .withTimeout(3.0));
        NamedCommands.registerCommand(
                "StopAll",
                Commands.runOnce(this::safeStopMechanisms, shooter, feeder, groundIntake, drivetrain));
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

        joystick.start().onTrue(
            Commands.sequence(
                complexCommand.groundIntakeResetCommand(),
                groundIntake.setPitchNormPositionCommand(0.07)
            )
        );

        Command mainShot = Commands
            .either(
                makeHubShootCommand(),
                makePassShootCommand(),
                this::isHubShootingRegion
            )
            .onlyIf(() -> {
                boolean known = DriverStation.getAlliance().isPresent();
                if (!known) {
                    SmartDashboard.putString("Shooting/BlockedReason", "Alliance unknown");
                }
                return known;
            });
        joystick.rightTrigger()
            .whileTrue(mainShot)
            .onFalse(complexCommand.groundIntakeResetCommand());

        joystick.rightBumper().whileTrue(
            Commands.sequence(
                complexCommand.groundIntakeResetCommand(),
                Commands.either(
                    complexCommand.passBump(false),
                    complexCommand.passBump(true),
                    this::useOpponentRoute
                )
            )
        );

        joystick.leftTrigger()
            .whileTrue(complexCommand.groundIntakePrepareCommand())
            .onFalse(complexCommand.groundIntakeResetCommand());

        joystick.povUp()
            .whileTrue(makeFallbackShootCommand(ShotTable.zeroPitchFallback()))
            .onFalse(complexCommand.groundIntakeResetCommand());

        joystick.povDown()
            .whileTrue(makeFallbackShootCommand(ShotTable.tower()))
            .onFalse(complexCommand.groundIntakeResetCommand());

        joystick.povRight()
            .onTrue(complexCommand.groundIntakeAntiCommand())
            .onFalse(complexCommand.groundIntakeResetCommand());

        joystick.y().whileTrue(
            Commands.sequence(
                Commands.runOnce(
                    () -> {
                        shooter.setIdle();
                        feeder.stop();
                    },
                    shooter,
                    feeder
                ),
                Commands.either(
                    complexCommand.passTrench(false),
                    complexCommand.passTrench(true),
                    this::useOpponentRoute
                )
            )
        );
    }

    private Command makeHubShootCommand() {
        return Commands.sequence(
            complexCommand.groundIntakeResetCommand(),
            Commands.parallel(
                new RealTimeAimDrive(drivetrain),
                new ShootWithTableCommand(
                    shooter,
                    feeder,
                    () -> ShotTable.hub(drivetrain.getDistanceToHub()),
                    () -> drivetrain.getSomAngleDiff() <= 3.0
                ),
                complexCommand.groundIntakeAssistCommand().repeatedly()
            )
        );
    }

    private Command makePassShootCommand() {
        return Commands.sequence(
            complexCommand.groundIntakeResetCommand(),
            Commands.parallel(
                new PassBallCommand(drivetrain, shooter, feeder),
                complexCommand.groundIntakeAssistCommand().repeatedly()
            )
        );
    }

    private Command makeFallbackShootCommand(ShotSetpoint setpoint) {
        return Commands.sequence(
            complexCommand.groundIntakeResetCommand(),
            Commands.parallel(
                Commands.run(drivetrain::setBrakeRequest, drivetrain),
                new ShootWithTableCommand(
                    shooter,
                    feeder,
                    () -> setpoint,
                    () -> true,
                    true
                )
            )
        );
    }

    private boolean isHubShootingRegion() {
        return DriverStation.getAlliance()
            .map(alliance -> {
                AllianceSide side = alliance == Alliance.Red ? AllianceSide.RED : AllianceSide.BLUE;
                return ShotTable.isHubRegion(side, drivetrain.getState().Pose.getX());
            })
            .orElse(false);
    }

    private boolean useOpponentRoute() {
        return DriverStation.getAlliance()
            .map(alliance -> {
                double xMeters = drivetrain.getState().Pose.getX();
                double midfieldMeters = Constants.FieldConstants.fieldLengthMeters / 2.0;
                return alliance == Alliance.Red
                    ? xMeters >= midfieldMeters
                    : xMeters <= midfieldMeters;
            })
            .orElse(false);
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
        groundIntake.setTeleopRollerCurrentLimit();
        drivetrain.setControl(idleRequest);
    }

    public void safeStopMechanisms() {
        feeder.stop();
        shooter.setIdle();
        groundIntake.stop();
        drivetrain.setControl(idleRequest);
    }
}
