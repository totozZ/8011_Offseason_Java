// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.vision.LimelightIO;
import frc.robot.vision.LimelightIO.PoseEstimate;

/** Fuses translation-only MegaTag2 measurements from the three robot Limelights. */
public class VisionSubsystem extends SubsystemBase implements AutoCloseable {
    private final CommandSwerveDrivetrain drivetrain;
    private final CameraState[] cameras;

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        if (drivetrain == null) {
            throw new IllegalArgumentException("VisionSubsystem drivetrain cannot be null");
        }

        this.drivetrain = drivetrain;
        cameras = new CameraState[Constants.VisionConstants.LIMELIGHT_NAMES.length];
        for (int index = 0; index < cameras.length; index++) {
            String cameraName = Constants.VisionConstants.LIMELIGHT_NAMES[index];
            cameras[index] = new CameraState(cameraName, new LimelightIO(cameraName));
        }
    }

    @Override
    public void periodic() {
        double pigeonYawDegrees = drivetrain.getPigeon2().getYaw().getValueAsDouble();
        double angularVelocityDegreesPerSecond = drivetrain
                .getPigeon2()
                .getAngularVelocityZDevice()
                .getValueAsDouble();

        for (CameraState camera : cameras) {
            updateCamera(camera, pigeonYawDegrees, angularVelocityDegreesPerSecond);
        }
    }

    private void updateCamera(
            CameraState camera,
            double pigeonYawDegrees,
            double angularVelocityDegreesPerSecond) {
        try {
            // MegaTag2 requires the robot orientation every loop. Limelight owns its vendor table.
            camera.io().setExternalImuMode();
            camera.io().setRobotOrientation(pigeonYawDegrees);

            Optional<PoseEstimate> optionalEstimate = camera.io().getMegaTag2Estimate();
            PoseEstimate estimate = optionalEstimate.orElse(null);
            String rejectReason = rejectionReason(estimate, angularVelocityDegreesPerSecond);
            boolean accepted = rejectReason.isEmpty();

            camera.publish(estimate, accepted, accepted ? "None" : rejectReason);
            if (!accepted) {
                return;
            }

            double[] standardDeviations = calculateStandardDeviations(
                    estimate.averageTagDistanceMeters());
            drivetrain.addVisionMeasurement(
                    estimate.pose(),
                    estimate.timestampSeconds(),
                    VecBuilder.fill(
                            standardDeviations[0],
                            standardDeviations[1],
                            standardDeviations[2]));
        } catch (RuntimeException exception) {
            camera.reportFailure(exception);
        }
    }

    static String rejectionReason(
            PoseEstimate estimate,
            double angularVelocityDegreesPerSecond) {
        StringBuilder reason = new StringBuilder();
        if (estimate == null || estimate.tagCount() <= 0) {
            appendReason(reason, "NoTags");
        }

        if (!Double.isFinite(angularVelocityDegreesPerSecond)
                || Math.abs(angularVelocityDegreesPerSecond)
                        > Constants.VisionConstants.MAX_ANGULAR_VELOCITY_DEGREES_PER_SECOND) {
            appendReason(reason, "HighAngularVelocity");
        }

        if (estimate != null && estimate.tagCount() > 0) {
            double averageDistanceMeters = estimate.averageTagDistanceMeters();
            if (!Double.isFinite(averageDistanceMeters)
                    || averageDistanceMeters
                            < Constants.VisionConstants.MIN_AVERAGE_DISTANCE_METERS
                    || averageDistanceMeters
                            > Constants.VisionConstants.MAX_AVERAGE_DISTANCE_METERS) {
                appendReason(reason, "DistanceOutOfRange");
            }
        }
        return reason.toString();
    }

    static double[] calculateStandardDeviations(double averageDistanceMeters) {
        double xyMeters = Constants.VisionConstants.XY_STANDARD_DEVIATION_COEFFICIENT
                * Math.pow(
                        averageDistanceMeters,
                        Constants.VisionConstants.DISTANCE_STANDARD_DEVIATION_EXPONENT);
        return new double[] {
            xyMeters,
            xyMeters,
            Constants.VisionConstants.HEADING_STANDARD_DEVIATION_RADIANS
        };
    }

    @Override
    public void close() {
        for (CameraState camera : cameras) {
            camera.close();
        }
    }

    private static void appendReason(StringBuilder reason, String addition) {
        if (!reason.isEmpty()) {
            reason.append('|');
        }
        reason.append(addition);
    }

    private static final class CameraState {
        private final String cameraName;
        private final LimelightIO io;
        private final BooleanPublisher acceptedPublisher;
        private final StringPublisher rejectReasonPublisher;
        private final IntegerPublisher tagCountPublisher;
        private final DoublePublisher averageDistancePublisher;
        private final DoublePublisher latencyPublisher;
        private final StructPublisher<Pose2d> posePublisher;
        private final IntegerPublisher failureCountPublisher;
        private int failureCount;

        CameraState(String cameraName, LimelightIO io) {
            this.cameraName = cameraName;
            this.io = io;

            NetworkTable table = NetworkTableInstance.getDefault()
                    .getTable("FRC8011")
                    .getSubTable("Vision")
                    .getSubTable(cameraName);
            acceptedPublisher = table.getBooleanTopic("Accepted").publish();
            rejectReasonPublisher = table.getStringTopic("RejectReason").publish();
            tagCountPublisher = table.getIntegerTopic("TagCount").publish();
            averageDistancePublisher = table.getDoubleTopic("AverageDistanceM").publish();
            latencyPublisher = table.getDoubleTopic("LatencyMs").publish();
            posePublisher = table.getStructTopic("Pose", Pose2d.struct).publish();
            failureCountPublisher = table.getIntegerTopic("FailureCount").publish();
        }

        LimelightIO io() {
            return io;
        }

        void publish(PoseEstimate estimate, boolean accepted, String rejectReason) {
            acceptedPublisher.set(accepted);
            rejectReasonPublisher.set(rejectReason);
            tagCountPublisher.set(estimate == null ? 0 : estimate.tagCount());
            averageDistancePublisher.set(
                    estimate == null ? Double.NaN : estimate.averageTagDistanceMeters());
            latencyPublisher.set(estimate == null ? Double.NaN : estimate.latencyMilliseconds());
            posePublisher.set(estimate == null ? Pose2d.kZero : estimate.pose());
            failureCountPublisher.set(failureCount);
        }

        void reportFailure(RuntimeException exception) {
            failureCount++;
            acceptedPublisher.set(false);
            rejectReasonPublisher.set("Exception:" + exception.getClass().getSimpleName());
            failureCountPublisher.set(failureCount);

            // Report the first error and then once per 50 repeats to avoid flooding the DS.
            if (failureCount == 1 || failureCount % 50 == 0) {
                String message = exception.getMessage() == null
                        ? exception.getClass().getSimpleName()
                        : exception.getMessage();
                DriverStation.reportWarning(
                        "Vision camera " + cameraName + " failed: " + message,
                        false);
            }
        }

        void close() {
            acceptedPublisher.close();
            rejectReasonPublisher.close();
            tagCountPublisher.close();
            averageDistancePublisher.close();
            latencyPublisher.close();
            posePublisher.close();
            failureCountPublisher.close();
        }
    }
}
