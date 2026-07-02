// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.shooting.ShotSetpoint;
import frc.robot.shooting.ShotTable;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class PassBallCommand extends Command {
    private final CommandSwerveDrivetrain drive;
    private final ShooterSubsystem shooter;
    private final FeederSubsystem feeder;
    private final double shooterFacingOffsetDeg;
    private boolean timerStarted = false;
    private boolean feeding = false;
    private final Timer shotTimer = new Timer();

    private final double maxSpeedMetersPerSecond =
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final SwerveRequest.FieldCentricFacingAngle facingRequest =
            new SwerveRequest.FieldCentricFacingAngle();

    private final Translation2d blueLeftTarget = new Translation2d(3.0, 5.5);
    private final Translation2d blueRightTarget = new Translation2d(3.0, 2.5);

    public PassBallCommand(
            CommandSwerveDrivetrain drive,
            ShooterSubsystem shooter,
            FeederSubsystem feeder) {
        this(drive, shooter, feeder, 180.0);
    }

    public PassBallCommand(
            CommandSwerveDrivetrain drive,
            ShooterSubsystem shooter,
            FeederSubsystem feeder,
            double shooterFacingOffsetDeg) {
        this.drive = drive;
        this.shooter = shooter;
        this.feeder = feeder;
        this.shooterFacingOffsetDeg = shooterFacingOffsetDeg;
        addRequirements(drive, shooter, feeder);
    }

    @Override
    public void initialize() {
        timerStarted = false;
        feeding = false;
        shotTimer.stop();
        shotTimer.reset();
        feeder.stop();

        if (shooter.getPitchHomeState() == ShooterSubsystem.PitchHomeState.UNHOMED
                || shooter.getPitchHomeState() == ShooterSubsystem.PitchHomeState.FAULT) {
            shooter.beginPitchHoming();
        }

        drive.setSomAngleDiff(180.0);
        facingRequest
                .withHeadingPID(9.0, 0.0, 0.1)
                .withDeadband(maxSpeedMetersPerSecond * 0.05)
                .withRotationalDeadband(0.1)
                .withMaxAbsRotationalRate(3.14)
                .withDriveRequestType(DriveRequestType.Velocity)
                .withSteerRequestType(SteerRequestType.Position);
    }

    @Override
    public void execute() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            feeder.stop();
            shooter.setIdle();
            return;
        }

        boolean isRed = alliance.get() == Alliance.Red;
        Pose2d pose = drive.getState().Pose;
        Translation2d target = pose.getY() > 4.0 ? blueLeftTarget : blueRightTarget;
        if (isRed) {
            target = new Translation2d(
                    Constants.FieldConstants.fieldLengthMeters - target.getX(),
                    target.getY());
        }

        Rotation2d targetDirection = Rotation2d
                .fromRadians(Math.atan2(target.getY() - pose.getY(), target.getX() - pose.getX()))
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

        if (!shooter.isPitchHomed()) {
            feeder.setBackwardFeederDuty(0.0);
            feeder.setUpwardDuty(0.0);
            return;
        }

        double distance = pose.getTranslation().getDistance(target);
        ShotSetpoint setpoint = ShotTable.pass(distance);
        shooter.applyShotSetpoint(setpoint);
        feeder.setUpwardFeederVelocity(setpoint.feederRps());

        if (!timerStarted) {
            shotTimer.restart();
            timerStarted = true;
        }

        boolean clearOfHub = pose.getY() <= 3.5 || pose.getY() >= 4.5;
        boolean ready = clearOfHub
                && angleErrorDeg <= 3.0
                && shooter.isFlywheelReady(0.7)
                && shooter.isPitchReady(0.75)
                && Math.abs(feeder.getUpwardFeederVelocity() - setpoint.feederRps()) <= 2.0;

        if (ShotTable.feedAllowed(shooter.isPitchHomed(), ready, shotTimer.hasElapsed(1.5))) {
            feeding = true;
        }
        feeder.setBackwardFeederDuty(feeding ? 1.0 : 0.0);

        SmartDashboard.putNumber("Shooting/PassDistanceM", distance);
        SmartDashboard.putBoolean("Shooting/PassReady", ready);
        SmartDashboard.putBoolean("Shooting/PassForcedFeed", feeding && !ready);
    }

    @Override
    public void end(boolean interrupted) {
        shotTimer.stop();
        feeder.stop();
        shooter.setIdle();
        drive.setSomAngleDiff(180.0);
        drive.setSafeCoastWithCurrentSpeeds();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
