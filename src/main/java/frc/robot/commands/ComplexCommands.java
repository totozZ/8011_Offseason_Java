// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;

public class ComplexCommands {
    @SuppressWarnings("unused")
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
}
