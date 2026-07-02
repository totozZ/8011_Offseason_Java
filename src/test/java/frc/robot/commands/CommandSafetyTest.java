// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.*;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.shooting.ShotSetpoint;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import org.junit.jupiter.api.Test;

class CommandSafetyTest {
    private static final double EPSILON = 1e-9;

    @Test
    void shootCommandOwnsActuatorsAndInterruptionReturnsShooterToIdle() {
        ShooterSubsystem shooter = new ShooterSubsystem();
        FeederSubsystem feeder = new FeederSubsystem();
        ShootWithTableCommand command = new ShootWithTableCommand(
                shooter,
                feeder,
                () -> new ShotSetpoint(30.0, 20.0, 15.0),
                () -> true);

        assertEquals(Set.of(shooter, feeder), command.getRequirements());

        shooter.applyShotSetpoint(new ShotSetpoint(30.0, 20.0, 15.0));
        command.end(true);

        assertEquals(0.0, shooter.getTargetSetpoint().flywheelRps(), EPSILON);
        assertEquals(0.2, shooter.getTargetSetpoint().pitchDeg(), EPSILON);
    }

    @Test
    void subsystemStopCommandsOwnTheSubsystemTheyStop() {
        ShooterSubsystem shooter = new ShooterSubsystem();
        FeederSubsystem feeder = new FeederSubsystem();
        GroundIntakeSubsystem groundIntake = new GroundIntakeSubsystem();

        assertOwnsOnly(shooter.stopCommand(), shooter);
        assertOwnsOnly(feeder.stopCommand(), feeder);
        assertOwnsOnly(groundIntake.stopCommand(), groundIntake);
    }

    private static void assertOwnsOnly(
            Command command,
            edu.wpi.first.wpilibj2.command.Subsystem subsystem) {
        assertEquals(Set.of(subsystem), command.getRequirements());
    }
}
