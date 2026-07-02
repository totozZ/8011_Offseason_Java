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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RealTimeAimDrive extends Command {
    private final CommandSwerveDrivetrain drive;
    private final double shooterFacingOffsetDeg;
    private final double maxSpeedMetersPerSecond =
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    private final SwerveRequest.FieldCentricFacingAngle facingRequest =
            new SwerveRequest.FieldCentricFacingAngle();

    public RealTimeAimDrive(CommandSwerveDrivetrain drive) {
        this(drive, 180.0);
    }

    public RealTimeAimDrive(CommandSwerveDrivetrain drive, double shooterFacingOffsetDeg) {
        this.drive = drive;
        this.shooterFacingOffsetDeg = shooterFacingOffsetDeg;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.setSomAngleDiff(180.0);
        facingRequest
                .withHeadingPID(8.0, 0.0, 0.1)
                .withDeadband(maxSpeedMetersPerSecond * 0.05)
                .withRotationalDeadband(0.1)
                .withMaxAbsRotationalRate(4.71)
                .withDriveRequestType(DriveRequestType.Velocity)
                .withSteerRequestType(SteerRequestType.Position);
    }

    @Override
    public void execute() {
        Pose2d pose = drive.getState().Pose;
        Translation2d hub = drive.getHubPosition();
        Rotation2d targetDirection = Rotation2d
                .fromRadians(Math.atan2(hub.getY() - pose.getY(), hub.getX() - pose.getX()))
                .minus(Rotation2d.fromDegrees(shooterFacingOffsetDeg));
        double angleErrorDeg = Math.abs(targetDirection.minus(pose.getRotation()).getDegrees());

        drive.setSomAngleDiff(angleErrorDeg);
        if (angleErrorDeg < 2.0) {
            drive.setBrakeRequest();
        } else {
            drive.setControl(
                    facingRequest
                            .withVelocityX(0.0)
                            .withVelocityY(0.0)
                            .withTargetDirection(targetDirection));
        }

        SmartDashboard.putNumber("Shooting/AimErrorDeg", angleErrorDeg);
        SmartDashboard.putBoolean("Shooting/DrivetrainLocked", angleErrorDeg < 2.0);
    }

    @Override
    public void end(boolean interrupted) {
        drive.setSomAngleDiff(180.0);
        drive.setSafeCoastWithCurrentSpeeds();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
