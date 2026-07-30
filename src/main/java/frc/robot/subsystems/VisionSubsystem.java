// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.logging.RobotHealthLogger;
import frc.robot.vision.LimelightIO;
import frc.robot.vision.LimelightIO.PoseEstimate;
import frc.robot.vision.LimelightIO.RawFiducial;

public class VisionSubsystem extends SubsystemBase {
    static final int MODE_REJECTED = 0;
    static final int MODE_MEGATAG2 = 1;
    static final int MODE_MIXED = 2;

    private static final double MAX_ANGULAR_VELOCITY_DEG_PER_SEC = 100.0;
    private static final double MAX_AMBIGUITY = 0.5;
    private static final double MIN_CAMERA_DISTANCE_METERS = 0.26;
    private static final double MAX_CAMERA_DISTANCE_METERS = 4.50;
    private static final double MIX_DISTANCE_METERS = 1.4;

    private final CommandSwerveDrivetrain drivetrain;
    private final LimelightIO[] cameras;
    private final int[] visionModes;
    private int periodicFailureCount = 0;

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        if (drivetrain == null) {
            throw new IllegalArgumentException("VisionSubsystem drivetrain cannot be null");
        }
        this.drivetrain = drivetrain;
        cameras = new LimelightIO[Constants.VisionConstants.limelightNames.length];
        visionModes = new int[cameras.length];
        for (int index = 0; index < cameras.length; index++) {
            cameras[index] = new LimelightIO(
                    Constants.VisionConstants.limelightNames[index]);
        }
    }

    @Override
    public void periodic() {
        try {
            updateMeasurements();
        } catch (RuntimeException ex) {
            periodicFailureCount++;
            SmartDashboard.putNumber("Vision/PeriodicFailureCount", periodicFailureCount);
            SmartDashboard.putString(
                    "Vision/LastFailure",
                    ex.getMessage() == null ? ex.getClass().getSimpleName() : ex.getMessage());
            if (periodicFailureCount == 1 || periodicFailureCount % 50 == 0) {
                DriverStation.reportWarning(
                        "Vision periodic fallback: "
                                + SmartDashboard.getString("Vision/LastFailure", "unknown"),
                        false);
            }
        }
    }

    private void updateMeasurements() {
        double angularVelocityDegPerSecond = drivetrain.getPigeon2()
                .getAngularVelocityZDevice()
                .getValueAsDouble();
        double robotYawDegrees = drivetrain.getState().Pose.getRotation().getDegrees();

        for (int index = 0; index < cameras.length; index++) {
            LimelightIO camera = cameras[index];
            camera.setExternalImuMode();
            camera.setRobotOrientation(robotYawDegrees);

            Optional<PoseEstimate> mt1 = camera.getMegaTag1Estimate();
            Optional<PoseEstimate> mt2 = camera.getMegaTag2Estimate();
            int mode = selectVisionMode(
                    mt1.orElse(null),
                    mt2.orElse(null),
                    angularVelocityDegPerSecond);
            visionModes[index] = mode;

            if (mode == MODE_REJECTED) {
                continue;
            }

            PoseEstimate mt2Estimate = mt2.orElseThrow();
            double[] standardDeviations = calculateStandardDeviations(
                    mt2Estimate.averageTagDistanceMeters(),
                    mode);
            Pose2d measurement = mode == MODE_MIXED
                    ? new Pose2d(
                            mt2Estimate.pose().getTranslation(),
                            mt1.orElseThrow().pose().getRotation())
                    : mt2Estimate.pose();
            drivetrain.addVisionMeasurement(
                    measurement,
                    mt2Estimate.timestampSeconds(),
                    VecBuilder.fill(
                            standardDeviations[0],
                            standardDeviations[1],
                            standardDeviations[2]));
        }
    }

    static int selectVisionMode(
            PoseEstimate mt1,
            PoseEstimate mt2,
            double angularVelocityDegPerSecond) {
        if (!rejectionReason(mt2, angularVelocityDegPerSecond).isEmpty()) {
            return MODE_REJECTED;
        }
        RawFiducial mt2Fiducial = mt2.rawFiducials().get(0);
        if (mt2Fiducial.distanceToCameraMeters() < MIX_DISTANCE_METERS
                && rejectionReason(mt1, angularVelocityDegPerSecond).isEmpty()) {
            return MODE_MIXED;
        }
        return MODE_MEGATAG2;
    }

    static String rejectionReason(
            PoseEstimate estimate,
            double angularVelocityDegPerSecond) {
        StringBuilder reason = new StringBuilder();
        if (estimate == null || estimate.tagCount() == 0) {
            appendReason(reason, "No Tags");
        }
        if (!Double.isFinite(angularVelocityDegPerSecond)
                || Math.abs(angularVelocityDegPerSecond)
                        > MAX_ANGULAR_VELOCITY_DEG_PER_SEC) {
            appendReason(reason, "High Angular Velocity");
        }
        if (estimate != null && estimate.tagCount() > 0) {
            if (estimate.rawFiducials().isEmpty()) {
                appendReason(reason, "Missing Fiducial Data");
            } else {
                RawFiducial fiducial = estimate.rawFiducials().get(0);
                if (fiducial.ambiguity() > MAX_AMBIGUITY) {
                    appendReason(reason, "High Ambiguity");
                }
                if (fiducial.distanceToCameraMeters() > MAX_CAMERA_DISTANCE_METERS
                        || fiducial.distanceToCameraMeters()
                                < MIN_CAMERA_DISTANCE_METERS) {
                    appendReason(reason, "DistToCamera");
                }
            }
        }
        SmartDashboard.putString(
                "Vision_Reject_Reason",
                reason.length() == 0 ? "None" : reason.toString());
        return reason.toString();
    }

    static double[] calculateStandardDeviations(
            double averageTagDistanceMeters,
            int mode) {
        double distanceFactor = Math.pow(averageTagDistanceMeters, 1.2);
        double xyDeviation = 0.01 * distanceFactor;
        double thetaDeviation = mode == MODE_MEGATAG2
                ? 10_000_000.0
                : 0.03 * distanceFactor;
        return new double[] {xyDeviation, xyDeviation, thetaDeviation};
    }

    /** Registers vision acceptance state; cameras remain NetworkTables inputs. */
    public void registerHealthLogging(RobotHealthLogger logger) {
        if (logger == null) {
            return;
        }
        logger.registerSubsystem(
                "Vision",
                this,
                this::hasAcceptedMeasurement,
                this::getHealthState);
    }

    private boolean hasAcceptedMeasurement() {
        for (int mode : visionModes) {
            if (mode != MODE_REJECTED) {
                return true;
            }
        }
        return false;
    }

    private String getHealthState() {
        int rejected = 0;
        int megaTag2 = 0;
        int mixed = 0;
        for (int mode : visionModes) {
            switch (mode) {
                case MODE_MEGATAG2 -> megaTag2++;
                case MODE_MIXED -> mixed++;
                default -> rejected++;
            }
        }
        return "Accepted=" + (megaTag2 + mixed)
                + ",MegaTag2=" + megaTag2
                + ",Mixed=" + mixed
                + ",Rejected=" + rejected
                + ",Errors=" + periodicFailureCount;
    }

    private static void appendReason(StringBuilder reason, String addition) {
        if (reason.length() > 0) {
            reason.append(" & ");
        }
        reason.append(addition);
    }
}
