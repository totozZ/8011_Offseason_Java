// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.path;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public final class DynamicPathFactory {
    private static final double MIN_PATH_DISTANCE_METERS = 0.05;
    private static final double SHOOT_PATH_POINT_SPACING_METERS = 0.7;
    private static final double SHOOT_PATH_MAX_DISTANCE_METERS = 8.0;

    private DynamicPathFactory() {}

    public static Optional<PathPlannerPath> createStraightPath(
            Pose2d currentPose,
            Pose2d targetPose) {
        return createStraightPath(
                currentPose,
                targetPose,
                1.25,
                1.0,
                270.0,
                240.0);
    }

    public static Optional<PathPlannerPath> createAutoStraightPath(
            Pose2d currentPose,
            Pose2d targetPose,
            double maxSpeedMetersPerSecond,
            double maxAccelerationMetersPerSecondSquared) {
        return createStraightPath(
                currentPose,
                targetPose,
                maxSpeedMetersPerSecond,
                maxAccelerationMetersPerSecondSquared,
                540.0,
                980.0);
    }

    public static Optional<PathPlannerPath> createMultiPointPath(
            Pose2d currentPose,
            List<Pose2d> targetPoses) {
        if (!isFinite(currentPose)
                || targetPoses == null
                || targetPoses.isEmpty()
                || targetPoses.stream().anyMatch(pose -> !isFinite(pose))
                || currentPose.getTranslation().getDistance(
                        targetPoses.get(0).getTranslation()) < MIN_PATH_DISTANCE_METERS) {
            return Optional.empty();
        }

        List<Pose2d> pathPoses = new ArrayList<>();
        Rotation2d initialHeading = targetPoses.get(0).getTranslation()
                .minus(currentPose.getTranslation())
                .getAngle();
        pathPoses.add(new Pose2d(currentPose.getTranslation(), initialHeading));
        pathPoses.addAll(targetPoses);

        return createPath(
                pathPoses,
                List.of(),
                new PathConstraints(
                        1.8,
                        1.8,
                        Math.toRadians(640.0),
                        Math.toRadians(980.0)),
                null,
                new GoalEndState(0.0, targetPoses.get(targetPoses.size() - 1).getRotation()));
    }

    public static Optional<PathPlannerPath> createShootOnMovePath(
            Pose2d currentPose,
            ChassisSpeeds currentSpeeds,
            List<Pose2d> targetPoses) {
        if (!isFinite(currentPose)
                || currentSpeeds == null
                || !areFinite(currentSpeeds)
                || targetPoses == null
                || targetPoses.size() <= 1
                || targetPoses.stream().anyMatch(pose -> !isFinite(pose))) {
            return Optional.empty();
        }

        double startSpeed = Math.hypot(
                currentSpeeds.vxMetersPerSecond,
                currentSpeeds.vyMetersPerSecond);
        Rotation2d initialHeading = startSpeed > 0.1
                ? new Rotation2d(
                        currentSpeeds.vxMetersPerSecond,
                        currentSpeeds.vyMetersPerSecond)
                : targetPoses.get(0).getTranslation()
                        .minus(currentPose.getTranslation())
                        .getAngle();

        List<Pose2d> pathPoses = new ArrayList<>();
        pathPoses.add(new Pose2d(currentPose.getTranslation(), initialHeading));
        for (int i = 0; i < targetPoses.size() - 1; i++) {
            Rotation2d travelHeading = targetPoses.get(i + 1).getTranslation()
                    .minus(targetPoses.get(i).getTranslation())
                    .getAngle();
            pathPoses.add(new Pose2d(targetPoses.get(i).getTranslation(), travelHeading));
        }
        int lastIndex = targetPoses.size() - 1;
        Rotation2d finalTravelHeading = targetPoses.get(lastIndex).getTranslation()
                .minus(targetPoses.get(lastIndex - 1).getTranslation())
                .getAngle();
        pathPoses.add(new Pose2d(
                targetPoses.get(lastIndex).getTranslation(),
                finalTravelHeading));

        List<RotationTarget> rotationTargets = new ArrayList<>();
        for (int i = 0; i < targetPoses.size() - 1; i++) {
            rotationTargets.add(new RotationTarget(
                    i + 1.0,
                    targetPoses.get(i).getRotation()));
        }

        return createPath(
                pathPoses,
                rotationTargets,
                new PathConstraints(
                        0.7,
                        1.8,
                        Math.toRadians(640.0),
                        Math.toRadians(980.0)),
                new IdealStartingState(startSpeed, currentPose.getRotation()),
                new GoalEndState(
                        0.0,
                        targetPoses.get(lastIndex).getRotation()));
    }

    public static List<Pose2d> createShootOnMoveTargets(
            Pose2d currentPose,
            Translation2d hubPosition,
            double directionDegrees) {
        if (!isFinite(currentPose)
                || hubPosition == null
                || !Double.isFinite(hubPosition.getX())
                || !Double.isFinite(hubPosition.getY())
                || !Double.isFinite(directionDegrees)) {
            return List.of();
        }

        Rotation2d travelDirection = Rotation2d.fromDegrees(
                -normalizeDegrees(directionDegrees));
        int pointCount = (int) (SHOOT_PATH_MAX_DISTANCE_METERS
                / SHOOT_PATH_POINT_SPACING_METERS);
        List<Pose2d> targetPoses = new ArrayList<>(pointCount);
        for (int count = 1; count <= pointCount; count++) {
            Translation2d point = currentPose.getTranslation().plus(
                    new Translation2d(
                            SHOOT_PATH_POINT_SPACING_METERS * count,
                            travelDirection));
            Rotation2d targetRotation = hubPosition.minus(point).getAngle();
            targetPoses.add(new Pose2d(point, targetRotation));
        }
        return targetPoses;
    }

    private static Optional<PathPlannerPath> createStraightPath(
            Pose2d currentPose,
            Pose2d targetPose,
            double maxSpeedMetersPerSecond,
            double maxAccelerationMetersPerSecondSquared,
            double maxAngularSpeedDegreesPerSecond,
            double maxAngularAccelerationDegreesPerSecondSquared) {
        if (!isFinite(currentPose)
                || !isFinite(targetPose)
                || !isPositiveFinite(maxSpeedMetersPerSecond)
                || !isPositiveFinite(maxAccelerationMetersPerSecondSquared)
                || currentPose.getTranslation().getDistance(
                        targetPose.getTranslation()) < MIN_PATH_DISTANCE_METERS) {
            return Optional.empty();
        }

        Rotation2d travelHeading = targetPose.getTranslation()
                .minus(currentPose.getTranslation())
                .getAngle();
        Translation2d midpoint = currentPose.getTranslation()
                .plus(targetPose.getTranslation())
                .div(2.0);
        List<Pose2d> pathPoses = List.of(
                new Pose2d(currentPose.getTranslation(), travelHeading),
                new Pose2d(midpoint, travelHeading),
                new Pose2d(targetPose.getTranslation(), travelHeading));

        return createPath(
                pathPoses,
                List.of(),
                new PathConstraints(
                        maxSpeedMetersPerSecond,
                        maxAccelerationMetersPerSecondSquared,
                        Math.toRadians(maxAngularSpeedDegreesPerSecond),
                        Math.toRadians(maxAngularAccelerationDegreesPerSecondSquared)),
                null,
                new GoalEndState(0.0, targetPose.getRotation()));
    }

    private static Optional<PathPlannerPath> createPath(
            List<Pose2d> pathPoses,
            List<RotationTarget> rotationTargets,
            PathConstraints constraints,
            IdealStartingState startingState,
            GoalEndState goalEndState) {
        try {
            PathPlannerPath path = new PathPlannerPath(
                    PathPlannerPath.waypointsFromPoses(pathPoses),
                    rotationTargets,
                    List.of(),
                    List.of(),
                    List.of(),
                    constraints,
                    startingState,
                    goalEndState,
                    false);
            path.preventFlipping = true;
            return Optional.of(path);
        } catch (RuntimeException ex) {
            return Optional.empty();
        }
    }

    private static boolean isFinite(Pose2d pose) {
        return pose != null
                && Double.isFinite(pose.getX())
                && Double.isFinite(pose.getY())
                && Double.isFinite(pose.getRotation().getRadians());
    }

    private static boolean areFinite(ChassisSpeeds speeds) {
        return Double.isFinite(speeds.vxMetersPerSecond)
                && Double.isFinite(speeds.vyMetersPerSecond)
                && Double.isFinite(speeds.omegaRadiansPerSecond);
    }

    private static boolean isPositiveFinite(double value) {
        return Double.isFinite(value) && value > 0.0;
    }

    private static double normalizeDegrees(double angleDegrees) {
        double result = angleDegrees;
        while (result > 180.0) {
            result -= 360.0;
        }
        while (result < -180.0) {
            result += 360.0;
        }
        return result;
    }
}
