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

public class IntakeNextToWallCommand extends Command {
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

    private boolean arrivedAtFirstPoint = false;
    private boolean arrivedAt0 = false;
    private boolean finishedAll = false;
    private boolean stopRightAway = false;
    private boolean isRed = false;
    private double targetSpeed = 0.0;
    private double targetX0 = 0.0;
    private double targetY0 = 0.0;
    private double targetR0 = 0.0;
    private double targetX1 = 0.0;
    private double targetY1 = 0.0;
    private double targetR1 = 0.0;
    private double targetX2 = 0.0;
    private double targetY2 = 0.0;
    private double targetR2 = 0.0;
    private double targetDirectionDeg = 0.0;

    public IntakeNextToWallCommand(
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
        arrivedAtFirstPoint = false;
        arrivedAt0 = false;
        finishedAll = false;

        driveClosed
                .withHeadingPID(8.0, 0.0, 0.1)
                .withDeadband(maxSpeedMetersPerSecond * 0.05)
                .withRotationalDeadband(0.1)
                .withMaxAbsRotationalRate(3.14)
                .withDriveRequestType(DriveRequestType.Velocity)
                .withSteerRequestType(SteerRequestType.Position);

        isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        boolean symmetric = isRed;
        if (opposite) {
            symmetric = !isRed;
        }

        double currentY = drive.getState().Pose.getY();
        if (symmetric) {
            currentY = Constants.FieldConstants.fieldWidthMeters - currentY;
        }
        if (currentY < 3.7) {
            targetX0 = 0.6;
            targetY0 = 0.6;
            targetR0 = -135.0;
            targetX1 = 0.6;
            targetY1 = 0.5;
            targetR1 = 110.0;
            targetX2 = 0.45;
            targetY2 = 2.7;
            targetR2 = 110.0;
        } else {
            targetX0 = 3.0;
            targetY0 = 7.25;
            targetR0 = 135.0;
            targetX1 = 0.75;
            targetY1 = 7.5;
            targetR1 = -110.0;
            targetX2 = 0.45;
            targetY2 = 5.0;
            targetR2 = -110.0;
        }

        if (symmetric) {
            targetX0 = Constants.FieldConstants.fieldLengthMeters - targetX0;
            targetY0 = Constants.FieldConstants.fieldWidthMeters - targetY0;
            if (!isRed) {
                targetR0 -= 180.0;
            }
            targetX1 = Constants.FieldConstants.fieldLengthMeters - targetX1;
            targetY1 = Constants.FieldConstants.fieldWidthMeters - targetY1;
            if (!isRed) {
                targetR1 -= 180.0;
            }
            targetX2 = Constants.FieldConstants.fieldLengthMeters - targetX2;
            targetY2 = Constants.FieldConstants.fieldWidthMeters - targetY2;
            if (!isRed) {
                targetR2 -= 180.0;
            }
        }
        if (opposite && isRed) {
            targetR0 -= 180.0;
            targetR1 -= 180.0;
            targetR2 -= 180.0;
        }

        double diffX = drive.getState().Pose.getX() - targetX1;
        double diffY = drive.getState().Pose.getY() - targetY1;
        stopRightAway = Math.hypot(diffX, diffY) >= 10.0;
        groundIntake.setPitchNormPosition(Constants.GroundIntakeConstants.pitchNormPosition);
        groundIntake.setRollerVelocity(100.0);
    }

    @Override
    public void execute() {
        Pose2d currentPose = drive.getState().Pose;
        double currentX = currentPose.getX();
        double currentY = currentPose.getY();
        double speedX = 0.0;
        double speedY = 0.0;

        if (!arrivedAtFirstPoint) {
            double distanceToFirst = Math.hypot(currentX - targetX1, currentY - targetY1);
            targetSpeed = 2.2;
            speedX = movePidX.calculate(currentX, targetX1);
            speedY = movePidY.calculate(currentY, targetY1);
            if (distanceToFirst < 0.1) {
                arrivedAtFirstPoint = true;
                movePidX.reset();
                movePidY.reset();
            }
            targetDirectionDeg = targetR1;

            double speedMagnitude = Math.hypot(speedX, speedY);
            if (speedMagnitude >= targetSpeed) {
                double coeff = speedMagnitude / targetSpeed;
                speedX /= coeff;
                speedY /= coeff;
            }
        } else {
            targetSpeed = 1.5;
            speedX = -leftYSupplier.getAsDouble()
                    * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)
                    * 0.2;
            speedY = clamp(movePidY.calculate(currentY, targetY2), -targetSpeed, targetSpeed);
            if (isRed) {
                speedX = -speedX;
            }
            if (targetY1 < targetY2) {
                finishedAll = currentY >= targetY2;
            } else {
                finishedAll = currentY <= targetY2;
            }
            targetDirectionDeg = targetR2;
        }

        if (isRed) {
            speedX = -speedX;
            speedY = -speedY;
        }

        drive.setControl(
                driveClosed
                        .withVelocityX(speedX)
                        .withVelocityY(speedY)
                        .withTargetDirection(Rotation2d.fromDegrees(targetDirectionDeg)));

        SmartDashboard.putBoolean("arrivedAtFirstPoint", arrivedAtFirstPoint);
        SmartDashboard.putBoolean("finishedAll", finishedAll);
    }

    @Override
    public void end(boolean interrupted) {
        groundIntake.stop();
        drive.setSafeCoastWithCurrentSpeeds();
    }

    @Override
    public boolean isFinished() {
        return finishedAll || stopRightAway;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
