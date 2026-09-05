// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

import org.junit.jupiter.api.Test;

class KitBotFuelSubsystemTest {
    private static final double EPSILON = 1e-9;

    @Test
    void talonFxConfigurationMatchesReferenceDirectionAndSafetyLimit() {
        var configuration = KitBotFuelSubsystem.createMotorConfiguration();

        assertEquals(
                InvertedValue.CounterClockwise_Positive,
                configuration.MotorOutput.Inverted);
        assertEquals(NeutralModeValue.Coast, configuration.MotorOutput.NeutralMode);
        assertEquals(
                Constants.BabyAutoConstants.MOTOR_CURRENT_LIMIT_AMPS,
                configuration.CurrentLimits.SupplyCurrentLimit,
                EPSILON);
        assertTrue(configuration.CurrentLimits.SupplyCurrentLimitEnable);
    }

    @Test
    void intakeAndShootDutyCyclesMatchSoccerBotReference() {
        assertEquals(0.5,
                Constants.BabyAutoConstants.INTAKE_MOTOR_INTAKE_DUTY_CYCLE, EPSILON);
        assertEquals(0.7,
                Constants.BabyAutoConstants.SHOOTER_MOTOR_INTAKE_DUTY_CYCLE, EPSILON);
        assertEquals(0.5,
                Constants.BabyAutoConstants.INTAKE_MOTOR_SHOOT_DUTY_CYCLE, EPSILON);
        assertEquals(-0.7,
                Constants.BabyAutoConstants.SHOOTER_MOTOR_SHOOT_DUTY_CYCLE, EPSILON);
    }
}
