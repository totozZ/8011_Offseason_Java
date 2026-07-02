// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.frc8011;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class WayiMotorConversionTest {
    private static final double EPSILON = 1e-9;

    @Test
    void positionRoundTripPreservesCppGearInvertAndOffsetFormula() {
        double mechanismPosition = 0.42;
        double motorPosition = WayiMotor.mechanismToMotorPosition(
                mechanismPosition,
                18.67,
                -1.0,
                7.5);

        assertEquals(
                mechanismPosition,
                WayiMotor.motorToMechanismPosition(
                        motorPosition,
                        18.67,
                        -1.0,
                        7.5),
                EPSILON);
    }

    @Test
    void velocityRoundTripPreservesCppGearAndInvertFormula() {
        double mechanismVelocity = 31.25;
        double motorVelocity = WayiMotor.mechanismToMotorVelocity(
                mechanismVelocity,
                18.67,
                -1.0);

        assertEquals(
                mechanismVelocity,
                WayiMotor.motorToMechanismVelocity(
                        motorVelocity,
                        18.67,
                        -1.0),
                EPSILON);
    }

    @Test
    void normalizedPositionUsesCppClampAndRangeFormula() {
        assertEquals(
                0.5,
                WayiMotor.normalizedToMechanismPosition(0.5, 0.0, 1.0),
                EPSILON);
        assertEquals(
                1.0,
                WayiMotor.normalizedToMechanismPosition(2.0, 0.0, 1.0),
                EPSILON);
        assertEquals(
                -1.0,
                WayiMotor.normalizedToMechanismPosition(-2.0, 0.0, 1.0),
                EPSILON);
    }
}
