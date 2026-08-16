// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import static org.junit.jupiter.api.Assertions.*;

import java.util.Optional;

import frc.robot.vision.LimelightIO.PoseEstimate;

import org.junit.jupiter.api.Test;

class LimelightIOTest {
    private static final double EPSILON = 1e-9;

    @Test
    void parsesCppPoseAndEmbeddedFiducialLayout() {
        double[] values = {
            2.0, 3.0, 0.1, 1.0, 2.0, 45.0,
            20.0, 1.0, 0.4, 2.5, 0.8,
            7.0, 0.1, -0.2, 0.8, 2.4, 2.5, 0.15
        };

        PoseEstimate estimate = LimelightIO.parsePoseEstimate(
                values,
                5_000_000L).orElseThrow();

        assertEquals(2.0, estimate.pose().getX(), EPSILON);
        assertEquals(3.0, estimate.pose().getY(), EPSILON);
        assertEquals(45.0, estimate.pose().getRotation().getDegrees(), EPSILON);
        assertEquals(4.98, estimate.timestampSeconds(), EPSILON);
        assertEquals(1, estimate.tagCount());
        assertEquals(2.5, estimate.averageTagDistanceMeters(), EPSILON);
        assertEquals(7, estimate.rawFiducials().get(0).id());
        assertEquals(0.15, estimate.rawFiducials().get(0).ambiguity(), EPSILON);
    }

    @Test
    void rejectsMissingOrShortPoseArrays() {
        Optional<PoseEstimate> missing = LimelightIO.parsePoseEstimate(
                new double[0],
                0L);
        Optional<PoseEstimate> shortPose = LimelightIO.parsePoseEstimate(
                new double[] {1.0, 2.0, 3.0},
                0L);

        assertTrue(missing.isEmpty());
        assertTrue(shortPose.isEmpty());
    }

    @Test
    void rejectsNonFiniteValuesAndNegativeTagCounts() {
        double[] nonFinite = {
            0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN,
            0.0, 0.0, 0.0, 0.0, 0.0
        };
        double[] negativeTags = {
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, -1.0, 0.0, 0.0, 0.0
        };

        assertTrue(LimelightIO.parsePoseEstimate(nonFinite, 0L).isEmpty());
        assertTrue(LimelightIO.parsePoseEstimate(negativeTags, 0L).isEmpty());
    }
}
