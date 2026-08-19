// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.lang.reflect.Field;
import java.util.Map;

import com.revrobotics.config.BaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

import org.junit.jupiter.api.Test;

class KitBotFuelSubsystemTest {
    @Test
    void officialCurrentLimitAndLauncherDirectionAreConfigured() throws Exception {
        Map<Integer, Object> feederParameters = parameters(
                KitBotFuelSubsystem.createFeederConfiguration());
        Map<Integer, Object> launcherParameters = parameters(
                KitBotFuelSubsystem.createLauncherConfiguration());

        assertTrue(feederParameters.containsValue(
                Constants.BabyAutoConstants.MOTOR_CURRENT_LIMIT_AMPS));
        assertFalse(feederParameters.containsValue(Boolean.TRUE));
        assertTrue(launcherParameters.containsValue(
                Constants.BabyAutoConstants.MOTOR_CURRENT_LIMIT_AMPS));
        assertTrue(launcherParameters.containsValue(Boolean.TRUE));
    }

    @SuppressWarnings("unchecked")
    private static Map<Integer, Object> parameters(SparkMaxConfig configuration) throws Exception {
        Field parameters = BaseConfig.class.getDeclaredField("parameters");
        parameters.setAccessible(true);
        return (Map<Integer, Object>) parameters.get(configuration);
    }
}
