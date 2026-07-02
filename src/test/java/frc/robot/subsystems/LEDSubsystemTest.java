// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.signals.RGBWColor;

import org.junit.jupiter.api.Test;

class LEDSubsystemTest {
    @Test
    void mapsOnlyCppImplementedSolidStates() {
        assertEquals(
                new RGBWColor(255, 0, 0),
                LEDSubsystem.colorForState(LEDSubsystem.AnimationType.RED).orElseThrow());
        assertEquals(
                new RGBWColor(0, 255, 0),
                LEDSubsystem.colorForState(LEDSubsystem.AnimationType.GREEN).orElseThrow());
        assertEquals(
                new RGBWColor(0, 0, 255),
                LEDSubsystem.colorForState(LEDSubsystem.AnimationType.BLUE).orElseThrow());
        assertEquals(
                new RGBWColor(255, 0, 255),
                LEDSubsystem.colorForState(LEDSubsystem.AnimationType.PURPLE).orElseThrow());
        assertTrue(LEDSubsystem.colorForState(LEDSubsystem.AnimationType.NONE).isEmpty());
        assertTrue(LEDSubsystem.colorForState(LEDSubsystem.AnimationType.RAINBOW).isEmpty());
    }
}
