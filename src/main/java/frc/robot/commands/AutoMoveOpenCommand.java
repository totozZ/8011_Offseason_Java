// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoMoveOpenCommand extends Command {
    private static final double TRANSLATION_TOLERANCE_METERS = 0.1;

    private final CommandSwerveDrivetrain drivetrain;
    private final double baseTargetX;
    private final double baseTargetY;
    private final double baseTargetHeadingDeg;
    private final double targetVelocityMetersPerSecond;
    private final double endSpeedMetersPerSecond;
    private final boolean invertAlliance;
    private final boolean invertDirection;
    private final double maxSpeedMetersPerSecond =
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final SwerveRequest.FieldCentricFacingAngle driveClosed =
            new SwerveRequest.FieldCentricFacingAngle();

    private Pose2d targetWaypoint = Pose2d.kZero;
    private double angleToTargetDeg = 0.0;
    private double initialDistanceMeters = 0.0;
    private double targetRotDeg = 0.0;

    public AutoMoveOpenCommand(
            CommandSwerveDrivetrain drivetrain,
            double targetX,
            double targetY,
            double targetHeadingDeg,
            double targetVelocityMetersPerSecond,
            boolean invertAlliance,
            boolean invertDirection) {
        this(
                drivetrain,
                targetX,
                targetY,
                targetHeadingDeg,
                targetVelocityMetersPerSecond,
                invertAlliance,
                invertDirection,
                targetVelocityMetersPerSecond);
    }

    public AutoMoveOpenCommand(
            CommandSwerveDrivetrain drivetrain,
            double targetX,
            double targetY,
            double targetHeadingDeg,
            double targetVelocityMetersPerSecond,
            boolean invertAlliance,
            boolean invertDirection,
            double endSpeedMetersPerSecond) {
        this.drivetrain = drivetrain;
        this.baseTargetX = targetX;
        this.baseTargetY = targetY;
        this.baseTargetHeadingDeg = targetHeadingDeg;
        this.targetVelocityMetersPerSecond = targetVelocityMetersPerSecond;
        this.invertAlliance = invertAlliance;
        this.invertDirection = invertDirection;
        this.endSpeedMetersPerSecond = endSpeedMetersPerSecond;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        driveClosed
                .withHeadingPID(8.0, 0.0, 0.1)
                .withDeadband(maxSpeedMetersPerSecond * 0.05)
                .withRotationalDeadband(0.1)
                .withMaxAbsRotationalRate(3.14 * 1.5)
                .withDriveRequestType(DriveRequestType.Velocity)
                .withSteerRequestType(SteerRequestType.Position);

        double x = baseTargetX;
        double y = baseTargetY;
        double heading = baseTargetHeadingDeg;

        if (invertDirection) {
            heading = -heading;
            y = Constants.FieldConstants.fieldWidthMeters - y;
        }
        if (invertAlliance) {
            x = Constants.FieldConstants.fieldLengthMeters - x;
            heading = normalizeDeg(180.0 - heading);
        }

        double realHeading = heading;
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            realHeading -= 180.0;
        }

        targetWaypoint = new Pose2d(
                new Translation2d(x, y),
                Rotation2d.fromDegrees(realHeading));
        Translation2d currentTranslation = drivetrain.getState().Pose.getTranslation();
        angleToTargetDeg = targetWaypoint.getTranslation().minus(currentTranslation).getAngle().getDegrees();
        if (angleToTargetDeg < 0.0) {
            angleToTargetDeg += 360.0;
        }
        initialDistanceMeters = currentTranslation.getDistance(targetWaypoint.getTranslation());
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getState().Pose;
        Translation2d difference = targetWaypoint.getTranslation().minus(currentPose.getTranslation());
        Rotation2d angleTo = difference.getAngle();
        double distanceMeters = currentPose.getTranslation().getDistance(targetWaypoint.getTranslation());
        double ratio = initialDistanceMeters > 1e-9 ? distanceMeters / initialDistanceMeters : 0.0;
        double realTargetVelocity = ratio * targetVelocityMetersPerSecond
                + (1.0 - ratio) * endSpeedMetersPerSecond;

        double suppX = realTargetVelocity * angleTo.getCos();
        double suppY = realTargetVelocity * angleTo.getSin();

        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            suppX = -suppX;
            suppY = -suppY;
        }

        targetRotDeg = drivetrain.isAutoShooting()
                ? drivetrain.getAutoRotDeg()
                : targetWaypoint.getRotation().getDegrees();
        drivetrain.setControl(
                driveClosed
                        .withVelocityX(suppX)
                        .withVelocityY(suppY)
                        .withTargetDirection(Rotation2d.fromDegrees(targetRotDeg)));
    }

    @Override
    public boolean isFinished() {
        Pose2d currentPose = drivetrain.getState().Pose;
        double distanceMeters = currentPose.getTranslation().getDistance(targetWaypoint.getTranslation());
        Rotation2d angleNow = targetWaypoint.getTranslation()
                .minus(currentPose.getTranslation())
                .getAngle();
        Rotation2d angleTarget = Rotation2d.fromDegrees(angleToTargetDeg);
        return distanceMeters <= TRANSLATION_TOLERANCE_METERS
                || Math.abs(angleNow.minus(angleTarget).getDegrees()) >= 80.0;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            drivetrain.setIdleRequest();
        } else {
            drivetrain.setSafeCoastWithCurrentSpeeds();
        }
    }

    private static double normalizeDeg(double angleDeg) {
        double result = angleDeg;
        while (result > 180.0) {
            result -= 360.0;
        }
        while (result < -180.0) {
            result += 360.0;
        }
        return result;
    }
}
