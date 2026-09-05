// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import frc.robot.generated.TunerConstants;

import org.junit.jupiter.api.Test;

class HardwareConstantsTest {
    @Test
    void generatedSwerveHasFourUsableAndDistinctModules() {
        assertNotNull(TunerConstants.DrivetrainConstants);
        assertTrue(TunerConstants.kSpeedAt12Volts.baseUnitMagnitude() > 0.0);

        List<SwerveModuleConstants<?, ?, ?>> modules = List.of(
                TunerConstants.FrontLeft,
                TunerConstants.FrontRight,
                TunerConstants.BackLeft,
                TunerConstants.BackRight);
        Set<Integer> driveIds = new HashSet<>();
        Set<Integer> steerIds = new HashSet<>();
        Set<Integer> encoderIds = new HashSet<>();
        Set<String> locations = new HashSet<>();

        for (SwerveModuleConstants<?, ?, ?> module : modules) {
            assertNotNull(module);
            assertTrue(driveIds.add(module.DriveMotorId));
            assertTrue(steerIds.add(module.SteerMotorId));
            assertTrue(encoderIds.add(module.EncoderId));
            assertTrue(Double.isFinite(module.LocationX));
            assertTrue(Double.isFinite(module.LocationY));
            assertTrue(locations.add(module.LocationX + "," + module.LocationY));
            assertTrue(module.WheelRadius > 0.0);
            assertTrue(module.DriveMotorGearRatio > 0.0);
            assertTrue(module.SteerMotorGearRatio > 0.0);
            assertTrue(module.SpeedAt12Volts > 0.0);
        }
    }

    @Test
    void classroomHardwareAndSafetyLimitsAreLocked() {
        assertEquals("rio", Constants.CanConstants.RIO_CAN_BUS.getName());
        assertEquals(1, Constants.CanConstants.REV_PDH_ID);
        assertEquals(20, Constants.BabyAutoConstants.INTAKE_MOTOR_CAN_ID);
        assertEquals(21, Constants.BabyAutoConstants.SHOOTER_MOTOR_CAN_ID);
        assertEquals(60, Constants.BabyAutoConstants.MOTOR_CURRENT_LIMIT_AMPS);
        assertEquals(1.0, Constants.BabyAutoConstants.MAX_TRANSLATION_METERS_PER_SECOND);
        assertEquals(90.0, Constants.BabyAutoConstants.MAX_ROTATION_DEGREES_PER_SECOND);
        assertEquals(15.0, Constants.BabyAutoConstants.MAX_AUTO_SECONDS);
        assertFalse(Constants.ExampleConstants.ENABLE_EXAMPLE_SUBSYSTEM);
    }
}
