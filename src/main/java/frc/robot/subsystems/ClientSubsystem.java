// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.logging.RobotHealthLogger;

public class ClientSubsystem extends SubsystemBase {
    public record MatchPhaseState(
            String phase,
            double countdownSeconds,
            boolean canShootNow) {}

    private final CommandSwerveDrivetrain drivetrain;
    private final SendableChooser<Boolean> firstAttackerChooser = new SendableChooser<>();
    private final Field2d field = new Field2d();

    public ClientSubsystem(CommandSwerveDrivetrain drivetrain) {
        if (drivetrain == null) {
            throw new IllegalArgumentException("ClientSubsystem drivetrain cannot be null");
        }
        this.drivetrain = drivetrain;

        firstAttackerChooser.setDefaultOption("OUR Alliance First", true);
        firstAttackerChooser.addOption("OPPONENT Alliance First", false);
        SmartDashboard.putData("First Shoot Setting", firstAttackerChooser);
        SmartDashboard.putData("Field", field);

        PathPlannerLogging.setLogActivePathCallback(
                poses -> field.getObject("path").setPoses(poses));
        WebServer.start(
                5800,
                Filesystem.getDeployDirectory().getAbsolutePath());
    }

    @Override
    public void periodic() {
        try {
            field.setRobotPose(drivetrain.getState().Pose);
            double matchTimeSeconds = DriverStation.getMatchTime();
            SmartDashboard.putNumber("MatchTime", matchTimeSeconds);
            SmartDashboard.putNumber(
                    "BatteryVoltage",
                    RobotController.getBatteryVoltage());

            MatchPhaseState phase = calculateMatchPhase(
                    matchTimeSeconds,
                    DriverStation.isAutonomousEnabled(),
                    DriverStation.isTeleopEnabled(),
                    !Boolean.FALSE.equals(firstAttackerChooser.getSelected()));
            SmartDashboard.putString("Match_Phase", phase.phase());
            SmartDashboard.putNumber("Phase_Countdown", phase.countdownSeconds());
            SmartDashboard.putBoolean("Can_Shoot_Now", phase.canShootNow());
        } catch (RuntimeException ex) {
            SmartDashboard.putString(
                    "ClientSubsystem Periodic Failed:",
                    ex.getMessage() == null ? ex.getClass().getSimpleName() : ex.getMessage());
        }
    }

    static MatchPhaseState calculateMatchPhase(
            double matchTimeSeconds,
            boolean autonomousEnabled,
            boolean teleopEnabled,
            boolean ourAllianceFirst) {
        if (matchTimeSeconds < 0.0 || (!autonomousEnabled && !teleopEnabled)) {
            return new MatchPhaseState("Waiting/Disabled", 0.0, false);
        }
        if (autonomousEnabled) {
            return new MatchPhaseState(
                    "AUTO (Both Active)",
                    matchTimeSeconds,
                    true);
        }
        if (matchTimeSeconds > 130.0) {
            return new MatchPhaseState(
                    "Transition (Both)",
                    matchTimeSeconds - 130.0,
                    true);
        }
        if (matchTimeSeconds >= 105.0) {
            return switchState(1, matchTimeSeconds - 105.0, ourAllianceFirst);
        }
        if (matchTimeSeconds >= 80.0) {
            return switchState(2, matchTimeSeconds - 80.0, !ourAllianceFirst);
        }
        if (matchTimeSeconds >= 55.0) {
            return switchState(3, matchTimeSeconds - 55.0, ourAllianceFirst);
        }
        if (matchTimeSeconds >= 30.0) {
            return switchState(4, matchTimeSeconds - 30.0, !ourAllianceFirst);
        }
        return new MatchPhaseState(
                "ENDGAME (Both)",
                matchTimeSeconds,
                true);
    }

    /** Registers the dashboard/client connection context. */
    public void registerHealthLogging(RobotHealthLogger logger) {
        if (logger == null) {
            return;
        }
        logger.registerSubsystem(
                "Client",
                this,
                DriverStation::isDSAttached,
                () -> DriverStation.isDSAttached() ? "DriverStationConnected" : "Disconnected");
    }

    private static MatchPhaseState switchState(
            int switchNumber,
            double countdownSeconds,
            boolean ourTurn) {
        return new MatchPhaseState(
                "Switch " + switchNumber + (ourTurn ? " (OUR Turn)" : " (OPPONENT)"),
                countdownSeconds,
                ourTurn);
    }
}
