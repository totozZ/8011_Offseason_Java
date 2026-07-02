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

public class IntakeNextToSideCommand extends Command {
    private final CommandSwerveDrivetrain drive;
    private final GroundIntakeSubsystem groundIntake;
    private final DoubleSupplier leftXSupplier;
    @SuppressWarnings("unused")
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
    private double speedLowerLimit = 0.2;
    private double targetDirectionDeg = 0.0;
    private double speedMagnitude = 0.0;

    public IntakeNextToSideCommand(
            CommandSwerveDrivetrain drive,
            GroundIntakeSubsystem groundIntake,
            DoubleSupplier leftXSupplier,
            boolean opposite) {
        this.drive = drive;
        this.groundIntake = groundIntake;
        this.leftXSupplier = leftXSupplier;
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
        double currentY = drive.getState().Pose.getY();
        double currentX = drive.getState().Pose.getX();
        if (currentX < 11.54 && currentX > 5.0) {
            targetX1 = 10.54;
            targetX2 = 6.0;
            targetR1 = 180.0;
            targetR2 = 180.0;
            if (currentX < 8.27) {
                targetX1 = Constants.FieldConstants.fieldLengthMeters - targetX1;
                targetY1 = Constants.FieldConstants.fieldWidthMeters - targetY1;
                targetX2 = Constants.FieldConstants.fieldLengthMeters - targetX2;
                targetY2 = Constants.FieldConstants.fieldWidthMeters - targetY2;
                targetR1 = 0.0;
                targetR2 = 0.0;
            }
        } else if (currentX < 5.0) {
            targetX1 = 3.5;
            targetX2 = 0.55;
            targetR1 = 180.0;
            targetR2 = 180.0;
            if (currentX < 2.0) {
                double temp = targetX1;
                targetX1 = targetX2;
                targetX2 = temp;
                targetR1 = 0.0;
                targetR2 = 0.0;
            }
        } else {
            targetX1 = Constants.FieldConstants.fieldLengthMeters - 3.5;
            targetX2 = Constants.FieldConstants.fieldLengthMeters - 0.55;
            targetR1 = 0.0;
            targetR2 = 0.0;
            if (currentX > 14.56) {
                double temp = targetX1;
                targetX1 = targetX2;
                targetX2 = temp;
                targetR1 = 180.0;
                targetR2 = 180.0;
            }
        }
        if (currentY < 4.035) {
            targetY1 = 0.60;
            targetY2 = 0.60;
            targetR1 = -targetR1;
            targetR2 = -targetR2;
        } else {
            targetY1 = Constants.FieldConstants.fieldWidthMeters - 0.60;
            targetY2 = Constants.FieldConstants.fieldWidthMeters - 0.60;
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
            speedMagnitude = Math.hypot(speedX, speedY);
            if (speedMagnitude > 1e-6 && speedMagnitude < speedLowerLimit) {
                double coeff = speedMagnitude / speedLowerLimit;
                speedX /= coeff;
                speedY /= coeff;
            }
            if (speedMagnitude >= targetSpeed) {
                double coeff = speedMagnitude / targetSpeed;
                speedX /= coeff;
                speedY /= coeff;
            }
        } else {
            targetSpeed = 2.0;
            speedX = clamp(movePidX.calculate(currentX, targetX2), -targetSpeed, targetSpeed);
            speedY = -leftXSupplier.getAsDouble()
                    * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)
                    * 0.2;
            if (isRed) {
                speedY = -speedY;
            }
            boolean goForward = targetR2 <= 90.0 && targetR2 >= -90.0;
            finishedAll = (goForward && currentX > targetX2) || (!goForward && currentX < targetX2);
            targetDirectionDeg = targetR2;
        }

        if (isRed) {
            speedX = -speedX;
            speedY = -speedY;
            targetDirectionDeg -= 180.0;
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
        return finishedAll;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
