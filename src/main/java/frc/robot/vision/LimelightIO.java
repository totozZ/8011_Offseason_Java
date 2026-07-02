// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public final class LimelightIO {
    public record RawFiducial(
            int id,
            double txnc,
            double tync,
            double targetArea,
            double distanceToCameraMeters,
            double distanceToRobotMeters,
            double ambiguity) {}

    public record PoseEstimate(
            Pose2d pose,
            double timestampSeconds,
            double latencyMilliseconds,
            int tagCount,
            double tagSpan,
            double averageTagDistanceMeters,
            double averageTagArea,
            List<RawFiducial> rawFiducials) {}

    private static final int POSE_VALUE_COUNT = 11;
    private static final int VALUES_PER_FIDUCIAL = 7;

    private final NetworkTable table;

    public LimelightIO(String cameraName) {
        table = NetworkTableInstance.getDefault().getTable(cameraName);
    }

    public void setExternalImuMode() {
        table.getEntry("imumode_set").setDouble(0.0);
    }

    public void setRobotOrientation(double yawDegrees) {
        table.getEntry("robot_orientation_set").setDoubleArray(
                new double[] {yawDegrees, 0.0, 0.0, 0.0, 0.0, 0.0});
    }

    public Optional<PoseEstimate> getMegaTag1Estimate() {
        return readPoseEstimate("botpose_wpiblue");
    }

    public Optional<PoseEstimate> getMegaTag2Estimate() {
        return readPoseEstimate("botpose_orb_wpiblue");
    }

    private Optional<PoseEstimate> readPoseEstimate(String entryName) {
        NetworkTableEntry entry = table.getEntry(entryName);
        return parsePoseEstimate(
                entry.getDoubleArray(new double[0]),
                entry.getLastChange());
    }

    static Optional<PoseEstimate> parsePoseEstimate(
            double[] poseValues,
            long lastChangeMicroseconds) {
        if (poseValues == null || poseValues.length < POSE_VALUE_COUNT) {
            return Optional.empty();
        }
        for (double value : poseValues) {
            if (!Double.isFinite(value)) {
                return Optional.empty();
            }
        }

        int tagCount = (int) poseValues[7];
        if (tagCount < 0) {
            return Optional.empty();
        }

        double latencyMilliseconds = poseValues[6];
        double timestampSeconds = lastChangeMicroseconds / 1_000_000.0
                - latencyMilliseconds / 1_000.0;
        Pose2d pose = new Pose2d(
                poseValues[0],
                poseValues[1],
                Rotation2d.fromDegrees(poseValues[5]));

        List<RawFiducial> rawFiducials = new ArrayList<>();
        int expectedValueCount = POSE_VALUE_COUNT + VALUES_PER_FIDUCIAL * tagCount;
        if (poseValues.length == expectedValueCount) {
            for (int index = 0; index < tagCount; index++) {
                int base = POSE_VALUE_COUNT + index * VALUES_PER_FIDUCIAL;
                rawFiducials.add(new RawFiducial(
                        (int) poseValues[base],
                        poseValues[base + 1],
                        poseValues[base + 2],
                        poseValues[base + 3],
                        poseValues[base + 4],
                        poseValues[base + 5],
                        poseValues[base + 6]));
            }
        }

        return Optional.of(new PoseEstimate(
                pose,
                timestampSeconds,
                latencyMilliseconds,
                tagCount,
                poseValues[8],
                poseValues[9],
                poseValues[10],
                List.copyOf(rawFiducials)));
    }
}
