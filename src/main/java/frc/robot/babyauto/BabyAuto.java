// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.babyauto;

import java.util.Objects;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.KitBotFuelSubsystem;

/** Safe, small command vocabulary exposed to students during the BabyAuto activity. */
public final class BabyAuto {
    private final DriveController driveController;
    private final FuelController fuelController;
    private final Motor intakeMotor;
    private final Motor shooterMotor;

    public BabyAuto(
            CommandSwerveDrivetrain drivetrain,
            KitBotFuelSubsystem fuelSubsystem) {
        this(new SwerveDriveController(drivetrain), new KitBotFuelController(fuelSubsystem));
    }

    BabyAuto(DriveController driveController, FuelController fuelController) {
        this.driveController = Objects.requireNonNull(driveController);
        this.fuelController = Objects.requireNonNull(fuelController);
        intakeMotor = new Motor(MotorRole.INTAKE);
        shooterMotor = new Motor(MotorRole.SHOOTER);
    }

    /** Runs commands one after another. */
    public static Command sequence(Command... commands) {
        validateCommands(commands);
        return Commands.sequence(commands);
    }

    /** Runs commands at the same time. WPILib rejects conflicting subsystem requirements. */
    public static Command parallel(Command... commands) {
        validateCommands(commands);
        return Commands.parallel(commands);
    }

    /** Waits for a positive, finite number of seconds. */
    public static Command waitSeconds(double seconds) {
        return Commands.waitSeconds(requireDuration(seconds, "Wait time"));
    }

    /** Starts a new, independent robot-relative drive action. */
    public DriveBuilder drive() {
        return new DriveBuilder(0.0, 0.0, 0.0);
    }

    /** Intakes for the requested number of seconds and then stops both rollers. */
    public Command intakeFor(double seconds) {
        double duration = requireDuration(seconds, "Intake time");
        return fuelController
                .holdIntake()
                .withTimeout(duration)
                .finallyDo(fuelController::stop);
    }

    /** Spins up for one second, launches for the requested time, and then stops both rollers. */
    public Command shootFor(double seconds) {
        double launchDuration = requireDuration(seconds, "Shoot time");
        return sequence(
                        fuelController
                                .holdSpinUp()
                                .withTimeout(Constants.BabyAutoConstants.SHOOTER_SPIN_UP_SECONDS),
                        fuelController.holdLaunch().withTimeout(launchDuration))
                .finallyDo(fuelController::stop);
    }

    public Motor intake() {
        return intakeMotor;
    }

    public Motor shooter() {
        return shooterMotor;
    }

    /** Adds the final safety net used around the command returned by student code. */
    public Command protect(Command command) {
        return Objects.requireNonNull(command, "Auto.build() returned null")
                .withTimeout(Constants.BabyAutoConstants.MAX_AUTO_SECONDS)
                .finallyDo(this::stopAll);
    }

    /** Immediately removes drivetrain and fuel outputs, reporting but containing stop failures. */
    public void stopAll() {
        try {
            driveController.stop();
        } catch (RuntimeException exception) {
            DriverStation.reportError(
                    "BabyAuto drivetrain stop failed: " + exception.getMessage(),
                    exception.getStackTrace());
        }
        try {
            fuelController.stop();
        } catch (RuntimeException exception) {
            DriverStation.reportError(
                    "BabyAuto fuel stop failed: " + exception.getMessage(),
                    exception.getStackTrace());
        }
    }

    /** Builder for one timed, robot-relative Swerve movement. */
    public final class DriveBuilder {
        private final double vxMetersPerSecond;
        private final double vyMetersPerSecond;
        private final double rotationDegreesPerSecond;

        private DriveBuilder(
                double vxMetersPerSecond,
                double vyMetersPerSecond,
                double rotationDegreesPerSecond) {
            this.vxMetersPerSecond = vxMetersPerSecond;
            this.vyMetersPerSecond = vyMetersPerSecond;
            this.rotationDegreesPerSecond = rotationDegreesPerSecond;
        }

        /** Sets forward (+) / backward (-) speed in meters per second. */
        public DriveBuilder setVx(double metersPerSecond) {
            return new DriveBuilder(
                    requireFinite(metersPerSecond, "Vx"),
                    vyMetersPerSecond,
                    rotationDegreesPerSecond);
        }

        /** Sets left (+) / right (-) speed in meters per second. */
        public DriveBuilder setVy(double metersPerSecond) {
            return new DriveBuilder(
                    vxMetersPerSecond,
                    requireFinite(metersPerSecond, "Vy"),
                    rotationDegreesPerSecond);
        }

        /** Sets counter-clockwise (+) / clockwise (-) speed in degrees per second. */
        public DriveBuilder setRotation(double degreesPerSecond) {
            return new DriveBuilder(
                    vxMetersPerSecond,
                    vyMetersPerSecond,
                    requireFinite(degreesPerSecond, "Rotation"));
        }

        /** Builds a command that drives for the requested positive, finite time. */
        public Command forSeconds(double seconds) {
            double duration = requireDuration(seconds, "Drive time");
            DriveValues limited = limitDrive(
                    vxMetersPerSecond,
                    vyMetersPerSecond,
                    rotationDegreesPerSecond);
            return driveController
                    .hold(
                            limited.vxMetersPerSecond(),
                            limited.vyMetersPerSecond(),
                            Math.toRadians(limited.rotationDegreesPerSecond()))
                    .withTimeout(duration)
                    .finallyDo(driveController::stop);
        }
    }

    /** One named KitBot motor for low-level student experiments. */
    public final class Motor {
        private final MotorRole role;

        private Motor(MotorRole role) {
            this.role = role;
        }

        /** Immediately sets and retains normalized output, clamped to -1 through +1. */
        public Command speed(double speed) {
            double limited = limitMotorSpeed(speed, role.displayName);
            return role == MotorRole.INTAKE
                    ? fuelController.setIntakeSpeed(limited)
                    : fuelController.setShooterSpeed(limited);
        }

        /** Immediately stops this motor. */
        public Command stop() {
            return role == MotorRole.INTAKE
                    ? fuelController.stopIntake()
                    : fuelController.stopShooter();
        }
    }

    static DriveValues limitDrive(
            double vxMetersPerSecond,
            double vyMetersPerSecond,
            double rotationDegreesPerSecond) {
        requireFinite(vxMetersPerSecond, "Vx");
        requireFinite(vyMetersPerSecond, "Vy");
        requireFinite(rotationDegreesPerSecond, "Rotation");

        double limitedVx = vxMetersPerSecond;
        double limitedVy = vyMetersPerSecond;
        double translationMagnitude = Math.hypot(limitedVx, limitedVy);
        double maxTranslation = Constants.BabyAutoConstants.MAX_TRANSLATION_METERS_PER_SECOND;
        if (translationMagnitude > maxTranslation) {
            double scale = maxTranslation / translationMagnitude;
            limitedVx *= scale;
            limitedVy *= scale;
            DriverStation.reportWarning(
                    "BabyAuto translation was limited to " + maxTranslation + " m/s",
                    false);
        }

        double maxRotation = Constants.BabyAutoConstants.MAX_ROTATION_DEGREES_PER_SECOND;
        double limitedRotation = Math.max(
                -maxRotation,
                Math.min(maxRotation, rotationDegreesPerSecond));
        if (limitedRotation != rotationDegreesPerSecond) {
            DriverStation.reportWarning(
                    "BabyAuto rotation was limited to +/-" + maxRotation + " deg/s",
                    false);
        }
        return new DriveValues(limitedVx, limitedVy, limitedRotation);
    }

    static double limitMotorSpeed(double speed, String name) {
        requireFinite(speed, name + " speed");
        double limited = Math.max(-1.0, Math.min(1.0, speed));
        if (limited != speed) {
            DriverStation.reportWarning(
                    "BabyAuto " + name + " speed was limited to +/-1.0",
                    false);
        }
        return limited;
    }

    private static double requireFinite(double value, String name) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(name + " must be finite");
        }
        return value;
    }

    private static double requireDuration(double seconds, String name) {
        requireFinite(seconds, name);
        if (seconds <= 0.0) {
            throw new IllegalArgumentException(name + " must be greater than zero");
        }
        return seconds;
    }

    private static void validateCommands(Command[] commands) {
        Objects.requireNonNull(commands, "Command list must not be null");
        for (Command command : commands) {
            Objects.requireNonNull(command, "Commands must not contain null");
        }
    }

    record DriveValues(
            double vxMetersPerSecond,
            double vyMetersPerSecond,
            double rotationDegreesPerSecond) {}

    private enum MotorRole {
        INTAKE("intake"),
        SHOOTER("shooter");

        private final String displayName;

        MotorRole(String displayName) {
            this.displayName = displayName;
        }
    }

    private static final class SwerveDriveController implements DriveController {
        private final CommandSwerveDrivetrain drivetrain;
        private final SwerveRequest.RobotCentric stopRequest = createRequest();

        private SwerveDriveController(CommandSwerveDrivetrain drivetrain) {
            this.drivetrain = Objects.requireNonNull(drivetrain);
        }

        @Override
        public Command hold(
                double vxMetersPerSecond,
                double vyMetersPerSecond,
                double rotationRadiansPerSecond) {
            SwerveRequest.RobotCentric request = createRequest()
                    .withVelocityX(vxMetersPerSecond)
                    .withVelocityY(vyMetersPerSecond)
                    .withRotationalRate(rotationRadiansPerSecond);
            return drivetrain.applyRequest(() -> request);
        }

        @Override
        public void stop() {
            drivetrain.setControl(stopRequest
                    .withVelocityX(0.0)
                    .withVelocityY(0.0)
                    .withRotationalRate(0.0));
        }

        private static SwerveRequest.RobotCentric createRequest() {
            return CommandSwerveDrivetrain.createRobotCentricRequest();
        }
    }

    private static final class KitBotFuelController implements FuelController {
        private final KitBotFuelSubsystem subsystem;

        private KitBotFuelController(KitBotFuelSubsystem subsystem) {
            this.subsystem = Objects.requireNonNull(subsystem);
        }

        @Override
        public Command holdIntake() {
            return subsystem.holdIntakeCommand();
        }

        @Override
        public Command holdSpinUp() {
            return subsystem.holdShooterSpinUpCommand();
        }

        @Override
        public Command holdLaunch() {
            return subsystem.holdShootCommand();
        }

        @Override
        public Command setIntakeSpeed(double speed) {
            return subsystem.setIntakeSpeedCommand(speed);
        }

        @Override
        public Command setShooterSpeed(double speed) {
            return subsystem.setShooterSpeedCommand(speed);
        }

        @Override
        public Command stopIntake() {
            return subsystem.stopIntakeCommand();
        }

        @Override
        public Command stopShooter() {
            return subsystem.stopShooterCommand();
        }

        @Override
        public void stop() {
            subsystem.stop();
        }
    }
}

interface DriveController {
    Command hold(
            double vxMetersPerSecond,
            double vyMetersPerSecond,
            double rotationRadiansPerSecond);

    void stop();
}

interface FuelController {
    Command holdIntake();

    Command holdSpinUp();

    Command holdLaunch();

    Command setIntakeSpeed(double speed);

    Command setShooterSpeed(double speed);

    Command stopIntake();

    Command stopShooter();

    void stop();
}
