// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.GroundIntakeSubsystem;

public class IntakeNextToHubCommand extends Command {
    private final CommandSwerveDrivetrain drive;
    private final GroundIntakeSubsystem groundIntake;
    private final DoubleSupplier leftYSupplier;
    private final boolean opposite;
    private final PIDController movePidX = new PIDController(3.0, 0.0, 0.1);
    private final PIDController movePidY = new PIDController(3.0, 0.0, 0.1);
    private final double maxSpeedMetersPerSecond =
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final SwerveRequest.FieldCentricFacingAngle driveClosed =
            new SwerveRequest.FieldCentricFacingAngle();

    private boolean arrived = false;
    private boolean isRed = false;
    private double startY = 0.0;
    private double targetX = 0.0;
    private double rawTargetRotation = 0.0;
    private double targetY = 0.0;
    private double targetDirectionDeg = 0.0;
    private double speedYDefault = 0.0;
    private boolean stopRightAway = false;

    public IntakeNextToHubCommand(
            CommandSwerveDrivetrain drive,
            GroundIntakeSubsystem groundIntake,
            DoubleSupplier leftYSupplier,
            boolean opposite) {
        this.drive = drive;
        this.groundIntake = groundIntake;
        this.leftYSupplier = leftYSupplier;
        this.opposite = opposite;
        addRequirements(drive, groundIntake);
    }

    @Override
    public void initialize() {
        movePidX.reset();
        movePidY.reset();
        isRed = false;
        arrived = false;

        driveClosed
                .withHeadingPID(8.0, 0.0, 0.1)
                .withDeadband(maxSpeedMetersPerSecond * 0.05)
                .withRotationalDeadband(0.1)
                .withMaxAbsRotationalRate(3.14)
                .withDriveRequestType(DriveRequestType.Velocity)
                .withSteerRequestType(SteerRequestType.Position);

        startY = drive.getState().Pose.getY();
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            targetX = 10.55;
            isRed = true;
        } else {
            targetX = 5.6;
        }
        if (opposite) {
            targetX = Constants.FieldConstants.fieldLengthMeters - targetX;
        }

        if (startY <= 4.035) {
            targetY = 2.0;
            rawTargetRotation = 90.0;
            speedYDefault = 2.0;
        } else {
            targetY = 6.0;
            rawTargetRotation = -90.0;
            speedYDefault = -2.0;
        }
        if (targetX <= 8.27) {
            rawTargetRotation = 180.0 - rawTargetRotation;
        }

        stopRightAway = false;
        groundIntake.setPitchNormPosition(Constants.GroundIntakeConstants.pitchNormPosition);
        groundIntake.setRollerVelocity(100.0);
    }

    @Override
    public void execute() {
        Pose2d currentPose = drive.getState().Pose;
        double currentX = currentPose.getX();
        double currentY = currentPose.getY();
        stopRightAway = currentX <= 4.0 || currentX >= 12.5;
        double speedX = 0.0;
        double speedY = 0.0;
        double distance = Math.hypot(currentX - targetX, currentY - targetY);

        if (!arrived) {
            speedX = movePidX.calculate(currentX, targetX);
            speedY = movePidY.calculate(currentY, targetY);
            double speedMagnitude = Math.hypot(speedX, speedY);
            double maxSpeed = 2.4;
            if (speedMagnitude >= maxSpeed) {
                double coeff = speedMagnitude / maxSpeed;
                speedX /= coeff;
                speedY /= coeff;
            }
        } else {
            speedX = -leftYSupplier.getAsDouble()
                    * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)
                    * 0.2;
            if (isRed) {
                speedX = -speedX;
            }
            speedY = speedYDefault;
        }

        if (isRed) {
            speedX = -speedX;
            speedY = -speedY;
            targetDirectionDeg = rawTargetRotation - 180.0;
        } else {
            targetDirectionDeg = rawTargetRotation;
        }

        drive.setControl(
                driveClosed
                        .withVelocityX(speedX)
                        .withVelocityY(speedY)
                        .withTargetDirection(Rotation2d.fromDegrees(targetDirectionDeg)));

        double angleDiff = targetDirectionDeg - drive.getState().Pose.getRotation().getDegrees();
        while (angleDiff > 180.0) {
            angleDiff -= 360.0;
        }
        while (angleDiff < -180.0) {
            angleDiff += 360.0;
        }
        angleDiff = Math.abs(angleDiff);
        boolean rotationReady = isRed ? angleDiff >= 160.0 : angleDiff <= 20.0;
        if (distance < 0.1 && rotationReady) {
            arrived = true;
        }

        SmartDashboard.putBoolean("arrived", arrived);
        SmartDashboard.putNumber("aDis", distance);
        SmartDashboard.putNumber("aAngleDiff", angleDiff);
    }

    @Override
    public void end(boolean interrupted) {
        groundIntake.stop();
        drive.setSafeCoastWithCurrentSpeeds();
    }

    @Override
    public boolean isFinished() {
        double currentY = drive.getState().Pose.getY();
        return stopRightAway || (arrived && (currentY <= 1.0 || currentY >= 7.0));
    }
}
