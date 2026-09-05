package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Thin command-based wrapper around the CTRE-generated drivetrain.
 *
 * <p>Keep mechanism logic out of this class. Tuner-generated hardware values live in
 * {@code TunerConstants}; this class owns only runtime drive integration.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double SIM_LOOP_PERIOD_SECONDS = 0.004;
    private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

    private final SwerveRequest.SysIdSwerveTranslation translationCharacterization =
            new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains steerCharacterization =
            new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation rotationCharacterization =
            new SwerveRequest.SysIdSwerveRotation();

    private final SysIdRoutine translationSysId = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(4.0),
                    null,
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(translationCharacterization.withVolts(output)),
                    null,
                    this));
    private final SysIdRoutine steerSysId = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(7.0),
                    null,
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(steerCharacterization.withVolts(output)),
                    null,
                    this));
    private final SysIdRoutine rotationSysId = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.of(Math.PI / 6.0).per(Second),
                    Volts.of(Math.PI),
                    null,
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        double radiansPerSecond = output.in(Volts);
                        setControl(rotationCharacterization.withRotationalRate(radiansPerSecond));
                        SignalLogger.writeDouble("Rotational_Rate", radiansPerSecond);
                    },
                    null,
                    this));

    private Notifier simulationNotifier;
    private double lastSimulationTimeSeconds;

    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        finishConstruction();
    }

    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        finishConstruction();
    }

    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(
                drivetrainConstants,
                odometryUpdateFrequency,
                odometryStandardDeviation,
                visionStandardDeviation,
                modules);
        finishConstruction();
    }

    /** Continuously applies the supplied request and removes motor output when the command ends. */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> setControl(requestSupplier.get())).finallyDo(() -> setControl(idleRequest));
    }

    public Command translationSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return translationSysId.quasistatic(direction);
    }

    public Command translationSysIdDynamic(SysIdRoutine.Direction direction) {
        return translationSysId.dynamic(direction);
    }

    public Command steerSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return steerSysId.quasistatic(direction);
    }

    public Command steerSysIdDynamic(SysIdRoutine.Direction direction) {
        return steerSysId.dynamic(direction);
    }

    public Command rotationSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return rotationSysId.quasistatic(direction);
    }

    public Command rotationSysIdDynamic(SysIdRoutine.Direction direction) {
        return rotationSysId.dynamic(direction);
    }

    /** Shared control mode for both hardware profiles, teleop, and timed BabyAuto actions. */
    public static SwerveRequest.RobotCentric createRobotCentricRequest() {
        return new SwerveRequest.RobotCentric()
                .withDriveRequestType(DriveRequestType.Velocity)
                .withSteerRequestType(SteerRequestType.Position);
    }

    private void finishConstruction() {
        // No heading reset, alliance perspective, or pose-based AutoBuilder in this classroom branch.
        // RobotCentric and X-lock use module feedback, not Pigeon heading or valid odometry.
        if (Utils.isSimulation()) {
            startSimulationThread();
        }
    }

    private void startSimulationThread() {
        lastSimulationTimeSeconds = Utils.getCurrentTimeSeconds();
        simulationNotifier = new Notifier(() -> {
            double currentTimeSeconds = Utils.getCurrentTimeSeconds();
            double deltaTimeSeconds = currentTimeSeconds - lastSimulationTimeSeconds;
            lastSimulationTimeSeconds = currentTimeSeconds;
            updateSimState(deltaTimeSeconds, RobotController.getBatteryVoltage());
        });
        simulationNotifier.startPeriodic(SIM_LOOP_PERIOD_SECONDS);
    }

    @Override
    public void addVisionMeasurement(Pose2d pose, double timestampSeconds) {
        super.addVisionMeasurement(pose, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    @Override
    public void addVisionMeasurement(
            Pose2d pose,
            double timestampSeconds,
            Matrix<N3, N1> standardDeviations) {
        super.addVisionMeasurement(
                pose,
                Utils.fpgaToCurrentTime(timestampSeconds),
                standardDeviations);
    }

    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }

    @Override
    public void close() {
        if (simulationNotifier != null) {
            simulationNotifier.close();
            simulationNotifier = null;
        }
        super.close();
    }
}
