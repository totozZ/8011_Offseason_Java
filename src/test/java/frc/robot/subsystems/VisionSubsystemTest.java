// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;

import frc.robot.vision.LimelightIO.PoseEstimate;
import frc.robot.vision.LimelightIO.RawFiducial;

import org.junit.jupiter.api.Test;

class VisionSubsystemTest {
    private static final double EPSILON = 1e-9;

    @Test
    void rejectsCppThresholdViolations() {
        PoseEstimate valid = estimate(1.0, 0.1);

        assertEquals("", VisionSubsystem.rejectionReason(valid, 100.0));
        assertTrue(VisionSubsystem.rejectionReason(valid, 100.1)
                .contains("High Angular Velocity"));
        assertTrue(VisionSubsystem.rejectionReason(estimate(1.0, 0.51), 0.0)
                .contains("High Ambiguity"));
        assertTrue(VisionSubsystem.rejectionReason(estimate(4.51, 0.1), 0.0)
                .contains("DistToCamera"));
        assertTrue(VisionSubsystem.rejectionReason(estimate(0.25, 0.1), 0.0)
                .contains("DistToCamera"));
    }

    @Test
    void selectsMixedModeOnlyForCloseAcceptedMt1AndMt2() {
        assertEquals(
                VisionSubsystem.MODE_MIXED,
                VisionSubsystem.selectVisionMode(
                        estimate(1.0, 0.1),
                        estimate(1.0, 0.1),
                        0.0));
        assertEquals(
                VisionSubsystem.MODE_MEGATAG2,
                VisionSubsystem.selectVisionMode(
                        estimate(1.0, 0.6),
                        estimate(1.0, 0.1),
                        0.0));
        assertEquals(
                VisionSubsystem.MODE_MEGATAG2,
                VisionSubsystem.selectVisionMode(
                        estimate(2.0, 0.1),
                        estimate(2.0, 0.1),
                        0.0));
    }

    @Test
    void preservesCppDistanceScaledStandardDeviations() {
        double[] mt2 = VisionSubsystem.calculateStandardDeviations(
                2.0,
                VisionSubsystem.MODE_MEGATAG2);
        double[] mixed = VisionSubsystem.calculateStandardDeviations(
                2.0,
                VisionSubsystem.MODE_MIXED);
        double factor = Math.pow(2.0, 1.2);

        assertEquals(0.01 * factor, mt2[0], EPSILON);
        assertEquals(10_000_000.0, mt2[2], EPSILON);
        assertEquals(0.03 * factor, mixed[2], EPSILON);
    }

    private static PoseEstimate estimate(
            double distanceToCameraMeters,
            double ambiguity) {
        return new PoseEstimate(
                Pose2d.kZero,
                1.0,
                0.0,
                1,
                0.0,
                distanceToCameraMeters,
                0.0,
                List.of(new RawFiducial(
                        1,
                        0.0,
                        0.0,
                        0.0,
                        distanceToCameraMeters,
                        distanceToCameraMeters,
                        ambiguity)));
    }
}
