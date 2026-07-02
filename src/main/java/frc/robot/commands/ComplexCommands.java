// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import java.util.Set;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;

public class ComplexCommands {
    private final CommandSwerveDrivetrain drive;
    private final FeederSubsystem feeder;
    private final GroundIntakeSubsystem groundIntake;

    public ComplexCommands(
            CommandSwerveDrivetrain drive,
            FeederSubsystem feeder,
            GroundIntakeSubsystem groundIntake) {
        this.drive = drive;
        this.feeder = feeder;
        this.groundIntake = groundIntake;
    }

    public Command groundIntakePrepareCommand() {
        return Commands.sequence(
                groundIntake.setRollerDutyCycleCommand(0.8),
                groundIntake.setPitchNormPositionCommand(
                        Constants.GroundIntakeConstants.pitchNormPosition));
    }

    public Command groundIntakeAssistCommand() {
        return Commands.sequence(
                groundIntake.setRollerVelocityCommand(30.0),
                groundIntake.setPitchNormPositionCommand(0.6),
                Commands.waitSeconds(0.4),
                groundIntake.setPitchNormPositionCommand(0.90),
                Commands.waitSeconds(0.4));
    }

    public Command groundIntakeAntiCommand() {
        return groundIntake.setRollerVelocityCommand(-20.0);
    }

    public Command groundIntakeResetCommand() {
        return Commands.parallel(
                groundIntake.stopCommand(),
                feeder.setBackwardFeederDutyCommand(0.0));
    }

    public Command passBump(boolean atOpponentSide) {
        return Commands.defer(
                () -> {
                    double innerX = 3.43;
                    final double innerY = 5.5;
                    double innerRotation = 45.0;
                    double outerX = 6.0;
                    final double outerY = 5.5;
                    double outerRotation = 45.0;

                    double x = drive.getState().Pose.getX();
                    double y = drive.getState().Pose.getY();
                    double targetSpeed = 0.7 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
                    boolean invertY = y <= Constants.FieldConstants.fieldWidthMeters / 2.0;
                    boolean invertAlliance =
                            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
                    if (atOpponentSide) {
                        invertAlliance = !invertAlliance;
                    }

                    if (x > Constants.FieldConstants.hubPassBlueBoundaryXMeters
                            && x < Constants.FieldConstants.hubPassRedBoundaryXMeters) {
                        innerX -= 0.5;
                        innerRotation = 135.0;
                        outerRotation = 135.0;
                        return Commands.sequence(
                                new AutoMoveOpenCommand(
                                        drive,
                                        outerX,
                                        outerY,
                                        outerRotation,
                                        targetSpeed,
                                        invertAlliance,
                                        invertY),
                                new AutoMoveClosedCommand(
                                        drive,
                                        innerX,
                                        innerY,
                                        innerRotation,
                                        targetSpeed,
                                        invertAlliance,
                                        invertY));
                    }

                    outerX += 0.5;
                    return Commands.sequence(
                            new AutoMoveOpenCommand(
                                    drive,
                                    innerX,
                                    innerY,
                                    innerRotation,
                                    targetSpeed,
                                    invertAlliance,
                                    invertY),
                            new AutoMoveClosedCommand(
                                    drive,
                                    outerX,
                                    outerY,
                                    outerRotation,
                                    targetSpeed,
                                    invertAlliance,
                                    invertY));
                },
                Set.of(drive));
    }

    public Command passTrench(boolean atOpponentSide) {
        return Commands.defer(
                () -> {
                    double innerX = 3.1;
                    double innerY = 7.4;
                    double innerRotation = 0.0;
                    double outerX = 6.0;
                    double outerY = 7.4;
                    double outerRotation = 0.0;

                    double currentRotation = drive.getState().Pose.getRotation().getDegrees();
                    if (currentRotation > 90.0 || currentRotation < -90.0) {
                        innerRotation = 180.0;
                        outerRotation = 180.0;
                    }

                    double x = drive.getState().Pose.getX();
                    double y = drive.getState().Pose.getY();
                    double targetSpeed = 0.7 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
                    boolean invertY = y <= Constants.FieldConstants.fieldWidthMeters / 2.0;
                    boolean invertAlliance =
                            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
                    if (atOpponentSide) {
                        invertAlliance = !invertAlliance;
                    }
                    if (invertAlliance) {
                        innerRotation -= 180.0;
                        outerRotation -= 180.0;
                    }

                    double virtualX = invertAlliance
                            ? Constants.FieldConstants.fieldLengthMeters - x
                            : x;
                    double virtualY = invertY
                            ? Constants.FieldConstants.fieldWidthMeters - y
                            : y;
                    final double offsetDistance = 0.5;

                    if (x > Constants.FieldConstants.hubPassBlueBoundaryXMeters
                            && x < Constants.FieldConstants.hubPassRedBoundaryXMeters) {
                        outerX += 0.2;
                        Rotation2d reverse = Rotation2d.fromDegrees(180.0);
                        Rotation2d approach = Rotation2d.fromRadians(
                                Math.atan2(outerY - virtualY, outerX - virtualX));
                        if (Math.abs(reverse.minus(approach).getDegrees()) >= 35.0) {
                            outerX -= offsetDistance * approach.getCos();
                            outerY -= offsetDistance * approach.getSin();
                            return Commands.sequence(
                                    new AutoMoveOpenCommand(
                                            drive,
                                            outerX,
                                            outerY,
                                            outerRotation,
                                            targetSpeed,
                                            invertAlliance,
                                            invertY),
                                    new AutoMoveCircleCommand(
                                            drive,
                                            outerX - 0.6,
                                            innerY - 0.6,
                                            0.3,
                                            90.0,
                                            true,
                                            targetSpeed,
                                            false,
                                            outerRotation,
                                            false,
                                            invertAlliance,
                                            invertY,
                                            0.0),
                                    new AutoMoveClosedCommand(
                                            drive,
                                            innerX - 1.0,
                                            innerY,
                                            innerRotation,
                                            targetSpeed,
                                            invertAlliance,
                                            invertY));
                        }
                        return Commands.sequence(
                                new AutoMoveOpenCommand(
                                        drive,
                                        outerX,
                                        outerY,
                                        outerRotation,
                                        targetSpeed,
                                        invertAlliance,
                                        invertY),
                                new AutoMoveClosedCommand(
                                        drive,
                                        innerX - 1.0,
                                        innerY,
                                        innerRotation,
                                        targetSpeed,
                                        invertAlliance,
                                        invertY));
                    }

                    innerX -= 0.2;
                    Rotation2d forward = Rotation2d.kZero;
                    Rotation2d approach = Rotation2d.fromRadians(
                            Math.atan2(innerY - virtualY, innerX - virtualX));
                    if (Math.abs(forward.minus(approach).getDegrees()) >= 35.0) {
                        innerX -= offsetDistance * approach.getCos();
                        innerY -= offsetDistance * approach.getSin();
                        return Commands.sequence(
                                new AutoMoveOpenCommand(
                                        drive,
                                        innerX,
                                        innerY,
                                        innerRotation,
                                        targetSpeed,
                                        invertAlliance,
                                        invertY),
                                new AutoMoveCircleCommand(
                                        drive,
                                        innerX + 0.6,
                                        outerY - 0.6,
                                        0.2,
                                        90.0,
                                        false,
                                        targetSpeed,
                                        false,
                                        outerRotation,
                                        false,
                                        invertAlliance,
                                        invertY,
                                        0.0),
                                new AutoMoveClosedCommand(
                                        drive,
                                        outerX + 1.0,
                                        outerY,
                                        outerRotation,
                                        targetSpeed,
                                        invertAlliance,
                                        invertY));
                    }

                    return Commands.sequence(
                            new AutoMoveOpenCommand(
                                    drive,
                                    innerX,
                                    innerY,
                                    innerRotation,
                                    targetSpeed,
                                    invertAlliance,
                                    invertY),
                            new AutoMoveClosedCommand(
                                    drive,
                                    outerX + 1.0,
                                    outerY,
                                    outerRotation,
                                    targetSpeed,
                                    invertAlliance,
                                    invertY));
                },
                Set.of(drive));
    }
}
