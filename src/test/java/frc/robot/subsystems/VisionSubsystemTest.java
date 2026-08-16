// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;

import frc.robot.vision.LimelightIO.PoseEstimate;

import org.junit.jupiter.api.Test;

class VisionSubsystemTest {
    private static final double EPSILON = 1e-9;

    @Test
    void acceptsOnlyFiniteMegaTag2MeasurementsInsideSpinAndDistanceLimits() {
        PoseEstimate valid = estimate(1, 1.0);

        assertEquals("", VisionSubsystem.rejectionReason(valid, 100.0));
        assertTrue(VisionSubsystem.rejectionReason(null, 0.0).contains("NoTags"));
        assertTrue(VisionSubsystem.rejectionReason(estimate(0, 1.0), 0.0)
                .contains("NoTags"));
        assertTrue(VisionSubsystem.rejectionReason(valid, 100.1)
                .contains("HighAngularVelocity"));
        assertTrue(VisionSubsystem.rejectionReason(valid, Double.NaN)
                .contains("HighAngularVelocity"));
        assertTrue(VisionSubsystem.rejectionReason(estimate(1, 4.51), 0.0)
                .contains("DistanceOutOfRange"));
        assertTrue(VisionSubsystem.rejectionReason(estimate(1, 0.25), 0.0)
                .contains("DistanceOutOfRange"));
    }

    @Test
    void scalesTranslationNoiseByDistanceAndNeverTrustsVisionHeading() {
        double[] deviations = VisionSubsystem.calculateStandardDeviations(2.0);
        double expectedTranslationMeters = 0.01 * Math.pow(2.0, 1.2);

        assertEquals(expectedTranslationMeters, deviations[0], EPSILON);
        assertEquals(expectedTranslationMeters, deviations[1], EPSILON);
        assertEquals(10_000_000.0, deviations[2], EPSILON);
    }

    private static PoseEstimate estimate(int tagCount, double averageDistanceMeters) {
        return new PoseEstimate(
                Pose2d.kZero,
                1.0,
                12.0,
                tagCount,
                0.0,
                averageDistanceMeters,
                0.0,
                List.of());
    }
}
