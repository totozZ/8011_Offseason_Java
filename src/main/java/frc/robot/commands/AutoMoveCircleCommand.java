// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoMoveCircleCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final double angleOffsetRad;
    private final double baseCenterX;
    private final double baseCenterY;
    private final double radiusMeters;
    private final double baseTargetAngleDeg;
    private final double targetVelocityMetersPerSecond;
    private final double baseTargetHeadingDeg;
    private final boolean baseIsCcw;
    private final boolean stopAtEnd;
    private final boolean faceTravelDirection;
    private final boolean invertAlliance;
    private final boolean invertDirection;

    private final PIDController radialPidX = new PIDController(2.0, 0.1, 0.1);
    private final PIDController radialPidY = new PIDController(2.0, 0.1, 0.1);
    private final PIDController distPid = new PIDController(3.0, 0.1, 0.1);
    private final SwerveRequest.FieldCentricFacingAngle driveClosed =
            new SwerveRequest.FieldCentricFacingAngle();

    private double centerX = 0.0;
    private double centerY = 0.0;
    private double targetAngleDeg = 0.0;
    private double targetHeadingDeg = 0.0;
    private boolean isCcw = false;
    private int direction = 1;
    private Rotation2d lastAngleFromCenter = Rotation2d.kZero;
    private double accumulatedDegrees = 0.0;
    private double totalDegreesNeeded = 0.0;
    private Translation2d targetPoint = new Translation2d();

    public AutoMoveCircleCommand(
            CommandSwerveDrivetrain drivetrain,
            double centerX,
            double centerY,
            double radiusMeters,
            double targetAngleDeg,
            boolean isCcw,
            double targetVelocityMetersPerSecond,
            boolean stopAtEnd,
            double targetHeadingDeg,
            boolean faceTravelDirection,
            boolean invertAlliance,
            boolean invertDirection,
            double angleOffsetRad) {
        this.drivetrain = drivetrain;
        this.angleOffsetRad = angleOffsetRad;
        this.baseCenterX = centerX;
        this.baseCenterY = centerY;
        this.radiusMeters = radiusMeters;
        this.baseTargetAngleDeg = targetAngleDeg;
        this.targetVelocityMetersPerSecond = targetVelocityMetersPerSecond;
        this.baseTargetHeadingDeg = targetHeadingDeg;
        this.baseIsCcw = isCcw;
        this.stopAtEnd = stopAtEnd;
        this.faceTravelDirection = faceTravelDirection;
        this.invertAlliance = invertAlliance;
        this.invertDirection = invertDirection;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        driveClosed
                .withHeadingPID(8.0, 0.0, 0.1)
                .withDeadband(0.05)
                .withRotationalDeadband(0.1)
                .withDriveRequestType(DriveRequestType.Velocity)
                .withSteerRequestType(SteerRequestType.Position)
                .withMaxAbsRotationalRate(3.14 * 1.5);

        centerX = baseCenterX;
        centerY = baseCenterY;
        targetAngleDeg = baseTargetAngleDeg;
        targetHeadingDeg = baseTargetHeadingDeg;
        isCcw = baseIsCcw;

        if (invertDirection) {
            centerY = Constants.FieldConstants.fieldWidthMeters - centerY;
            targetAngleDeg = -targetAngleDeg;
            targetHeadingDeg = -targetHeadingDeg;
            isCcw = !isCcw;
        }
        if (invertAlliance) {
            centerX = Constants.FieldConstants.fieldLengthMeters - centerX;
            targetAngleDeg = 180.0 - targetAngleDeg;
            targetHeadingDeg = 180.0 - targetHeadingDeg;
            isCcw = !isCcw;
        }

        direction = isCcw ? 1 : -1;
        Translation2d center = new Translation2d(centerX, centerY);
        Translation2d current = drivetrain.getState().Pose.getTranslation();
        lastAngleFromCenter = current.minus(center).getAngle();
        accumulatedDegrees = 0.0;

        double startDeg = lastAngleFromCenter.getDegrees();
        double diff = targetAngleDeg - startDeg;
        if (isCcw) {
            while (diff < 0.0) {
                diff += 360.0;
            }
            while (diff >= 360.0) {
                diff -= 360.0;
            }
        } else {
            diff = -diff;
            while (diff < 0.0) {
                diff += 360.0;
            }
            while (diff >= 360.0) {
                diff -= 360.0;
            }
        }
        totalDegreesNeeded = diff;

        double targetRad = Math.toRadians(targetAngleDeg);
        targetPoint = new Translation2d(
                centerX + radiusMeters * Math.cos(targetRad),
                centerY + radiusMeters * Math.sin(targetRad));

        radialPidX.reset();
        radialPidY.reset();
        distPid.reset();
    }

    @Override
    public void execute() {
        Translation2d center = new Translation2d(centerX, centerY);
        Translation2d current = drivetrain.getState().Pose.getTranslation();
        Rotation2d currentAngleFromCenter = current.minus(center).getAngle();

        double deltaDeg = currentAngleFromCenter.minus(lastAngleFromCenter).getDegrees();
        accumulatedDegrees += deltaDeg * direction;
        lastAngleFromCenter = currentAngleFromCenter;

        double thetaRad = currentAngleFromCenter.getRadians();
        Translation2d idealPoint = new Translation2d(
                centerX + radiusMeters * Math.cos(thetaRad),
                centerY + radiusMeters * Math.sin(thetaRad));

        double radialX = radialPidX.calculate(current.getX(), idealPoint.getX());
        double radialY = radialPidY.calculate(current.getY(), idealPoint.getY());

        double totalArcLength = totalDegreesNeeded * Math.PI / 180.0 * radiusMeters;
        double currentArcLength = accumulatedDegrees * Math.PI / 180.0 * radiusMeters;
        double baseTangent = targetVelocityMetersPerSecond;

        if (stopAtEnd) {
            baseTangent = distPid.calculate(currentArcLength, totalArcLength);
        }
        baseTangent = clamp(
                baseTangent,
                -targetVelocityMetersPerSecond,
                targetVelocityMetersPerSecond);

        double tangentX = -Math.sin(thetaRad);
        double tangentY = Math.cos(thetaRad);
        double tangentVelX = tangentX * direction * baseTangent;
        double tangentVelY = tangentY * direction * baseTangent;

        Rotation2d heading;
        if (faceTravelDirection) {
            double angleRad = Math.atan2(tangentY * direction, tangentX * direction);
            heading = Rotation2d.fromRadians(angleRad + angleOffsetRad * direction);
        } else {
            heading = Rotation2d.fromDegrees(targetHeadingDeg);
        }

        double speedX = tangentVelX + radialX;
        double speedY = tangentVelY + radialY;
        double currentMagnitude = Math.hypot(speedX, speedY);
        if (currentMagnitude > 1e-6) {
            speedX = (speedX / currentMagnitude) * baseTangent;
            speedY = (speedY / currentMagnitude) * baseTangent;
        } else {
            speedX = 0.0;
            speedY = 0.0;
        }

        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            speedX = -speedX;
            speedY = -speedY;
            heading = heading.plus(Rotation2d.fromDegrees(180.0));
        }
        if (drivetrain.isAutoShooting()) {
            heading = Rotation2d.fromDegrees(drivetrain.getAutoRotDeg());
        }

        drivetrain.setControl(
                driveClosed
                        .withVelocityX(speedX)
                        .withVelocityY(speedY)
                        .withTargetDirection(heading));
    }

    @Override
    public boolean isFinished() {
        Translation2d current = drivetrain.getState().Pose.getTranslation();
        double distanceToTarget = current.getDistance(targetPoint);
        boolean positionReached =
                distanceToTarget < 0.15 && accumulatedDegrees >= totalDegreesNeeded * 0.5;
        boolean angleReached = accumulatedDegrees >= totalDegreesNeeded;
        return positionReached || angleReached;
    }

    @Override
    public void end(boolean interrupted) {
        if (stopAtEnd || interrupted) {
            drivetrain.setIdleRequest();
        } else {
            drivetrain.setSafeCoastWithCurrentSpeeds();
        }
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
