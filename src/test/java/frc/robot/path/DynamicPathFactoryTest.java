// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.path;

import static org.junit.jupiter.api.Assertions.*;

import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import org.junit.jupiter.api.Test;

class DynamicPathFactoryTest {
    private static final double EPSILON = 1e-9;

    @Test
    void straightPathKeepsCppConstraintsAndPreventsFlipping() {
        Optional<PathPlannerPath> result = DynamicPathFactory.createStraightPath(
                Pose2d.kZero,
                new Pose2d(2.0, 1.0, Rotation2d.fromDegrees(90.0)));

        assertTrue(result.isPresent());
        PathPlannerPath path = result.orElseThrow();
        assertTrue(path.preventFlipping);
        assertEquals(3, path.getWaypoints().size());
        assertEquals(1.25, path.getGlobalConstraints().maxVelocityMPS(), EPSILON);
        assertEquals(1.0, path.getGlobalConstraints().maxAccelerationMPSSq(), EPSILON);
        assertEquals(90.0, path.getGoalEndState().rotation().getDegrees(), EPSILON);
    }

    @Test
    void straightPathRejectsTargetInsideCppMinimumDistance() {
        Optional<PathPlannerPath> result = DynamicPathFactory.createStraightPath(
                Pose2d.kZero,
                new Pose2d(0.049, 0.0, Rotation2d.kZero));

        assertTrue(result.isEmpty());
    }

    @Test
    void multiPointPathRejectsEmptyTargets() {
        assertTrue(DynamicPathFactory.createMultiPointPath(
                Pose2d.kZero,
                List.of()).isEmpty());
    }

    @Test
    void shootOnMoveRequiresAtLeastTwoTargets() {
        assertTrue(DynamicPathFactory.createShootOnMovePath(
                Pose2d.kZero,
                new ChassisSpeeds(),
                List.of(new Pose2d(1.0, 0.0, Rotation2d.kZero))).isEmpty());
    }

    @Test
    void shootOnMovePreservesStartSpeedAndRotationTargets() {
        List<Pose2d> targets = List.of(
                new Pose2d(1.0, 0.0, Rotation2d.fromDegrees(20.0)),
                new Pose2d(2.0, 0.5, Rotation2d.fromDegrees(30.0)),
                new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(40.0)));

        PathPlannerPath path = DynamicPathFactory.createShootOnMovePath(
                Pose2d.kZero,
                new ChassisSpeeds(0.3, 0.4, 0.0),
                targets).orElseThrow();

        assertTrue(path.preventFlipping);
        assertEquals(0.5, path.getIdealStartingState().velocityMPS(), EPSILON);
        assertEquals(2, path.getRotationTargets().size());
        assertEquals(40.0, path.getGoalEndState().rotation().getDegrees(), EPSILON);
    }

    @Test
    void shootOnMoveTargetsUseCppSpacingAndFaceHub() {
        Pose2d current = new Pose2d(2.0, 3.0, Rotation2d.kZero);
        Translation2d hub = new Translation2d(4.0, 3.0);

        List<Pose2d> targets = DynamicPathFactory.createShootOnMoveTargets(
                current,
                hub,
                0.0);

        assertEquals(11, targets.size());
        assertEquals(2.7, targets.get(0).getX(), EPSILON);
        assertEquals(3.0, targets.get(0).getY(), EPSILON);
        assertEquals(0.0, targets.get(0).getRotation().getDegrees(), EPSILON);
    }
}
