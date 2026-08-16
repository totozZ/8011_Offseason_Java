// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Objects;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ExampleSubsystem;

/** Runs the example TalonFX in VelocityVoltage using a mechanism rotations/second supplier. */
public class ExampleCommand extends Command {
    private final ExampleSubsystem subsystem;
    private final DoubleSupplier targetVelocityRotationsPerSecond;

    public ExampleCommand(
            ExampleSubsystem subsystem,
            DoubleSupplier targetVelocityRotationsPerSecond) {
        this.subsystem = Objects.requireNonNull(subsystem);
        this.targetVelocityRotationsPerSecond = Objects.requireNonNull(
                targetVelocityRotationsPerSecond);
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        subsystem.setVelocityRotationsPerSecond(
                targetVelocityRotationsPerSecond.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
