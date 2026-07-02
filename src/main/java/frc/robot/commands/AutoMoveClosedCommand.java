// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoMoveClosedCommand extends Command {
    private static final double TRANSLATION_TOLERANCE_METERS = 0.10;

    private final CommandSwerveDrivetrain drivetrain;
    private final double baseTargetX;
    private final double baseTargetY;
    private final double baseTargetHeadingDeg;
    private final double targetVelocityMetersPerSecond;
    private final boolean invertAlliance;
    private final boolean invertDirection;
    private final PIDController movePidX = new PIDController(4.5, 0.1, 0.1);
    private final PIDController movePidY = new PIDController(4.5, 0.1, 0.1);
    private final double maxSpeedMetersPerSecond =
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final SwerveRequest.FieldCentricFacingAngle driveClosed =
            new SwerveRequest.FieldCentricFacingAngle();

    private Pose2d targetWaypoint = Pose2d.kZero;
    private double targetX = 0.0;
    private double targetY = 0.0;
    private double realHeadDeg = 0.0;
    private double targetRotDeg = 0.0;

    public AutoMoveClosedCommand(
            CommandSwerveDrivetrain drivetrain,
            double targetX,
            double targetY,
            double targetHeadingDeg,
            double targetVelocityMetersPerSecond,
            boolean invertAlliance,
            boolean invertDirection) {
        this.drivetrain = drivetrain;
        this.baseTargetX = targetX;
        this.baseTargetY = targetY;
        this.baseTargetHeadingDeg = targetHeadingDeg;
        this.targetVelocityMetersPerSecond = targetVelocityMetersPerSecond;
        this.invertAlliance = invertAlliance;
        this.invertDirection = invertDirection;
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

        targetX = baseTargetX;
        targetY = baseTargetY;
        double heading = baseTargetHeadingDeg;

        if (invertDirection) {
            heading = -heading;
            targetY = Constants.FieldConstants.fieldWidthMeters - targetY;
        }
        if (invertAlliance) {
            targetX = Constants.FieldConstants.fieldLengthMeters - targetX;
            heading = normalizeDeg(180.0 - heading);
        }

        realHeadDeg = heading;
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            realHeadDeg -= 180.0;
        }
        targetWaypoint = new Pose2d(
                new Translation2d(targetX, targetY),
                Rotation2d.fromDegrees(realHeadDeg));
        movePidX.reset();
        movePidY.reset();
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getState().Pose;
        double speedX = movePidX.calculate(currentPose.getX(), targetX);
        double speedY = movePidY.calculate(currentPose.getY(), targetY);
        double speedMagnitude = Math.hypot(speedX, speedY);
        if (speedMagnitude >= targetVelocityMetersPerSecond) {
            double coeff = speedMagnitude / targetVelocityMetersPerSecond;
            speedX /= coeff;
            speedY /= coeff;
        }

        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            speedX = -speedX;
            speedY = -speedY;
        }

        targetRotDeg = drivetrain.isAutoShooting() ? drivetrain.getAutoRotDeg() : realHeadDeg;
        drivetrain.setControl(
                driveClosed
                        .withVelocityX(speedX)
                        .withVelocityY(speedY)
                        .withTargetDirection(Rotation2d.fromDegrees(targetRotDeg)));
    }

    @Override
    public boolean isFinished() {
        Pose2d currentPose = drivetrain.getState().Pose;
        double distanceMeters = currentPose.getTranslation().getDistance(targetWaypoint.getTranslation());
        double angleDiffDeg = Math.abs(
                currentPose.getRotation().minus(Rotation2d.fromDegrees(targetRotDeg)).getDegrees());
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            angleDiffDeg = 180.0 - angleDiffDeg;
        }
        SmartDashboard.putNumber("Adiff", angleDiffDeg);
        return distanceMeters <= TRANSLATION_TOLERANCE_METERS && angleDiffDeg < 5.0;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setIdleRequest();
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
