// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.util.Units;

import frc.robot.generated.TunerConstants;

import org.junit.jupiter.api.Test;

class HardwareConstantsTest {
    private static final double EPSILON = 1e-9;

    @Test
    void swerveMatchesCppCanBusPigeonAndModuleIds() {
        assertEquals("CANivore", TunerConstants.DrivetrainConstants.CANBusName);
        assertEquals(33, TunerConstants.DrivetrainConstants.Pigeon2Id);

        List<SwerveModuleConstants<
                TalonFXConfiguration,
                TalonFXConfiguration,
                CANcoderConfiguration>> modules = List.of(
                TunerConstants.FrontLeft,
                TunerConstants.FrontRight,
                TunerConstants.BackLeft,
                TunerConstants.BackRight);
        Set<Integer> ids = new HashSet<>();
        for (SwerveModuleConstants<?, ?, ?> module : modules) {
            assertTrue(ids.add(module.SteerMotorId));
            assertTrue(ids.add(module.DriveMotorId));
            assertTrue(ids.add(module.EncoderId));
        }
        assertEquals(Set.of(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12), ids);
    }

    @Test
    void swerveMatchesCppGeometryRatiosOffsetsAndLimits() {
        assertModule(
                TunerConstants.FrontLeft,
                2,
                1,
                3,
                -0.376952,
                10.875,
                10.875,
                false);
        assertModule(
                TunerConstants.FrontRight,
                5,
                4,
                6,
                0.197021484375,
                10.875,
                -10.875,
                true);
        assertModule(
                TunerConstants.BackLeft,
                8,
                7,
                9,
                -0.379150,
                -10.875,
                10.875,
                false);
        assertModule(
                TunerConstants.BackRight,
                11,
                10,
                12,
                -0.35205078125,
                -10.875,
                -10.875,
                true);
    }

    @Test
    void foundationRioDevicesAndExampleSafetyDefaultsAreLocked() {
        assertEquals("rio", Constants.CanConstants.RIO_CAN_BUS.getName());
        assertEquals(1, Constants.CanConstants.REV_PDH_ID);
        assertEquals(26, Constants.CanConstants.CANDLE_ID);
        assertFalse(Constants.ExampleConstants.ENABLE_EXAMPLE_SUBSYSTEM);
    }

    private static void assertModule(
            SwerveModuleConstants<
                    TalonFXConfiguration,
                    TalonFXConfiguration,
                    CANcoderConfiguration> module,
            int driveId,
            int steerId,
            int encoderId,
            double encoderOffsetRotations,
            double xInches,
            double yInches,
            boolean driveInverted) {
        assertEquals(driveId, module.DriveMotorId);
        assertEquals(steerId, module.SteerMotorId);
        assertEquals(encoderId, module.EncoderId);
        assertEquals(encoderOffsetRotations, module.EncoderOffset, EPSILON);
        assertEquals(Units.inchesToMeters(xInches), module.LocationX, EPSILON);
        assertEquals(Units.inchesToMeters(yInches), module.LocationY, EPSILON);
        assertEquals(driveInverted, module.DriveMotorInverted);
        assertTrue(module.SteerMotorInverted);
        assertFalse(module.EncoderInverted);

        assertEquals(6.746031746031747, module.DriveMotorGearRatio, EPSILON);
        assertEquals(21.428571428571427, module.SteerMotorGearRatio, EPSILON);
        assertEquals(3.5714285714285716, module.CouplingGearRatio, EPSILON);
        assertEquals(Units.inchesToMeters(2.008), module.WheelRadius, EPSILON);
        assertEquals(4.59, module.SpeedAt12Volts, EPSILON);

        assertEquals(
                40.0,
                module.DriveMotorInitialConfigs.CurrentLimits.SupplyCurrentLimit,
                EPSILON);
        assertEquals(
                90.0,
                module.DriveMotorInitialConfigs.CurrentLimits.StatorCurrentLimit,
                EPSILON);
        assertTrue(module.DriveMotorInitialConfigs.CurrentLimits.SupplyCurrentLimitEnable);
        assertTrue(module.DriveMotorInitialConfigs.CurrentLimits.StatorCurrentLimitEnable);

        assertEquals(
                20.0,
                module.SteerMotorInitialConfigs.CurrentLimits.SupplyCurrentLimit,
                EPSILON);
        assertEquals(
                60.0,
                module.SteerMotorInitialConfigs.CurrentLimits.StatorCurrentLimit,
                EPSILON);
        assertTrue(module.SteerMotorInitialConfigs.CurrentLimits.SupplyCurrentLimitEnable);
        assertTrue(module.SteerMotorInitialConfigs.CurrentLimits.StatorCurrentLimitEnable);
    }
}
