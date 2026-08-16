package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;

/** Publishes typed swerve telemetry for Elastic and AdvantageScope at approximately 50 Hz. */
public final class Telemetry implements AutoCloseable {
    private static final double PUBLISH_PERIOD_SECONDS = 0.02;

    private final StructPublisher<Pose2d> posePublisher;
    private final StructPublisher<ChassisSpeeds> speedsPublisher;
    private final StructArrayPublisher<SwerveModuleState> moduleStatesPublisher;
    private final StructArrayPublisher<SwerveModuleState> moduleTargetsPublisher;
    private final StructArrayPublisher<SwerveModulePosition> modulePositionsPublisher;
    private final DoublePublisher timestampPublisher;
    private final DoublePublisher odometryFrequencyPublisher;
    private double lastPublishTimestampSeconds = Double.NEGATIVE_INFINITY;

    public Telemetry() {
        NetworkTable table = NetworkTableInstance.getDefault()
                .getTable("FRC8011")
                .getSubTable("Drive");
        posePublisher = table.getStructTopic("Pose", Pose2d.struct).publish();
        speedsPublisher = table.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
        moduleStatesPublisher = table
                .getStructArrayTopic("ModuleStates", SwerveModuleState.struct)
                .publish();
        moduleTargetsPublisher = table
                .getStructArrayTopic("ModuleTargets", SwerveModuleState.struct)
                .publish();
        modulePositionsPublisher = table
                .getStructArrayTopic("ModulePositions", SwerveModulePosition.struct)
                .publish();
        timestampPublisher = table.getDoubleTopic("TimestampS").publish();
        odometryFrequencyPublisher = table.getDoubleTopic("OdometryFrequencyHz").publish();
    }

    public void publish(SwerveDriveState state) {
        if (state.Timestamp - lastPublishTimestampSeconds < PUBLISH_PERIOD_SECONDS) {
            return;
        }
        lastPublishTimestampSeconds = state.Timestamp;

        posePublisher.set(state.Pose);
        speedsPublisher.set(state.Speeds);
        moduleStatesPublisher.set(state.ModuleStates);
        moduleTargetsPublisher.set(state.ModuleTargets);
        modulePositionsPublisher.set(state.ModulePositions);
        timestampPublisher.set(state.Timestamp);
        odometryFrequencyPublisher.set(state.OdometryPeriod > 0.0
                ? 1.0 / state.OdometryPeriod
                : 0.0);

        SignalLogger.writeStruct("FRC8011/Drive/Pose", Pose2d.struct, state.Pose);
    }

    @Override
    public void close() {
        posePublisher.close();
        speedsPublisher.close();
        moduleStatesPublisher.close();
        moduleTargetsPublisher.close();
        modulePositionsPublisher.close();
        timestampPublisher.close();
        odometryFrequencyPublisher.close();
    }
}
