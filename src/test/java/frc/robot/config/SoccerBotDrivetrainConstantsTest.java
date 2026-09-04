// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import frc.robot.generated.TunerConstants;

import org.junit.jupiter.api.Test;

class SoccerBotDrivetrainConstantsTest {
    private static final double EPSILON = 1e-9;

    @Test
    void profilesExposeTheirOwnMeasuredSpeed() {
        assertEquals(
                TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
                DrivetrainProfile.NORMAL.speedAt12Volts().in(MetersPerSecond),
                EPSILON);
        assertEquals(
                4.72,
                DrivetrainProfile.SOCCER_BOT.speedAt12Volts().in(MetersPerSecond),
                EPSILON);
    }

    @Test
    void soccerBotCanMapAndPigeonMatchTheSumProject() {
        assertEquals("rio", SoccerBotDrivetrainConstants.DRIVETRAIN_CONSTANTS.CANBusName);
        assertEquals(13, SoccerBotDrivetrainConstants.DRIVETRAIN_CONSTANTS.Pigeon2Id);

        assertModule(SoccerBotDrivetrainConstants.FRONT_LEFT, 1, 2, 3);
        assertModule(SoccerBotDrivetrainConstants.FRONT_RIGHT, 4, 5, 6);
        assertModule(SoccerBotDrivetrainConstants.BACK_LEFT, 11, 10, 12);
        assertModule(SoccerBotDrivetrainConstants.BACK_RIGHT, 8, 7, 9);

        List<SwerveModuleConstants<?, ?, ?>> modules = List.of(
                SoccerBotDrivetrainConstants.FRONT_LEFT,
                SoccerBotDrivetrainConstants.FRONT_RIGHT,
                SoccerBotDrivetrainConstants.BACK_LEFT,
                SoccerBotDrivetrainConstants.BACK_RIGHT);
        Set<Integer> allCanIds = new HashSet<>();
        for (SwerveModuleConstants<?, ?, ?> module : modules) {
            assertTrue(allCanIds.add(module.DriveMotorId));
            assertTrue(allCanIds.add(module.SteerMotorId));
            assertTrue(allCanIds.add(module.EncoderId));
        }
    }

    @Test
    void soccerBotKeepsTheFrontLeftOnlyDriveDirectionCorrection() {
        // Right-side drive inversion and the normal back-left direction are unchanged. The
        // front-left drive is the single mechanical exception called out in the sum project.
        assertTrue(SoccerBotDrivetrainConstants.FRONT_LEFT.DriveMotorInverted);
        assertTrue(SoccerBotDrivetrainConstants.FRONT_RIGHT.DriveMotorInverted);
        assertFalse(SoccerBotDrivetrainConstants.BACK_LEFT.DriveMotorInverted);
        assertTrue(SoccerBotDrivetrainConstants.BACK_RIGHT.DriveMotorInverted);

        for (SwerveModuleConstants<?, ?, ?> module : List.of(
                SoccerBotDrivetrainConstants.FRONT_LEFT,
                SoccerBotDrivetrainConstants.FRONT_RIGHT,
                SoccerBotDrivetrainConstants.BACK_LEFT,
                SoccerBotDrivetrainConstants.BACK_RIGHT)) {
            assertTrue(module.SteerMotorInverted);
            assertFalse(module.EncoderInverted);
        }
    }

    @Test
    void soccerBotGeometryOffsetsRatiosAndCurrentLimitsMatchTheSumProject() {
        assertLocation(SoccerBotDrivetrainConstants.FRONT_LEFT, 10.25, 16.5);
        assertLocation(SoccerBotDrivetrainConstants.FRONT_RIGHT, 10.25, -16.5);
        assertLocation(SoccerBotDrivetrainConstants.BACK_LEFT, -10.25, 16.5);
        assertLocation(SoccerBotDrivetrainConstants.BACK_RIGHT, -10.25, -16.5);

        assertEquals(0.037353515625,
                SoccerBotDrivetrainConstants.FRONT_LEFT.EncoderOffset, EPSILON);
        assertEquals(0.204345703125,
                SoccerBotDrivetrainConstants.FRONT_RIGHT.EncoderOffset, EPSILON);
        assertEquals(0.25830078125,
                SoccerBotDrivetrainConstants.BACK_LEFT.EncoderOffset, EPSILON);
        assertEquals(0.331787109375,
                SoccerBotDrivetrainConstants.BACK_RIGHT.EncoderOffset, EPSILON);

        SwerveModuleConstants<
                TalonFXConfiguration,
                TalonFXConfiguration,
                CANcoderConfiguration> module = SoccerBotDrivetrainConstants.FRONT_LEFT;
        assertEquals(6.538461538461539, module.DriveMotorGearRatio, EPSILON);
        assertEquals(15.42857142857143, module.SteerMotorGearRatio, EPSILON);
        assertEquals(2.8333333333333335, module.CouplingGearRatio, EPSILON);
        assertEquals(Inches.of(2.0).in(Meters), module.WheelRadius, EPSILON);
        assertEquals(1.91, module.SteerMotorGains.kV, EPSILON);
        assertEquals(70.0,
                module.DriveMotorInitialConfigs.CurrentLimits.SupplyCurrentLimit, EPSILON);
        assertTrue(module.DriveMotorInitialConfigs.CurrentLimits.SupplyCurrentLimitEnable);
        assertEquals(60.0,
                module.SteerMotorInitialConfigs.CurrentLimits.StatorCurrentLimit, EPSILON);
        assertTrue(module.SteerMotorInitialConfigs.CurrentLimits.StatorCurrentLimitEnable);
    }

    private static void assertModule(
            SwerveModuleConstants<?, ?, ?> module,
            int driveMotorId,
            int steerMotorId,
            int encoderId) {
        assertEquals(driveMotorId, module.DriveMotorId);
        assertEquals(steerMotorId, module.SteerMotorId);
        assertEquals(encoderId, module.EncoderId);
    }

    private static void assertLocation(
            SwerveModuleConstants<?, ?, ?> module,
            double xInches,
            double yInches) {
        assertEquals(Inches.of(xInches).in(Meters), module.LocationX, EPSILON);
        assertEquals(Inches.of(yInches).in(Meters), module.LocationY, EPSILON);
    }
}
