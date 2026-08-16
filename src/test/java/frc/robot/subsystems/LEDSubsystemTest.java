// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.ctre.phoenix6.signals.RGBWColor;

import org.junit.jupiter.api.Test;

class LEDSubsystemTest {
    @Test
    void mapsFoundationStatesToSpecifiedSolidColors() {
        assertEquals(new RGBWColor(0, 0, 0),
                LEDSubsystem.colorForState(LEDSubsystem.State.OFF));
        assertEquals(new RGBWColor(0, 0, 255),
                LEDSubsystem.colorForState(LEDSubsystem.State.DISABLED));
        assertEquals(new RGBWColor(0, 255, 0),
                LEDSubsystem.colorForState(LEDSubsystem.State.TELEOP));
        assertEquals(new RGBWColor(255, 0, 255),
                LEDSubsystem.colorForState(LEDSubsystem.State.AUTONOMOUS));
        assertEquals(new RGBWColor(255, 255, 0),
                LEDSubsystem.colorForState(LEDSubsystem.State.TEST));
        assertEquals(new RGBWColor(255, 0, 0),
                LEDSubsystem.colorForState(LEDSubsystem.State.FAULT));
    }

    @Test
    void faultAlwaysOverridesRequestedMode() {
        for (LEDSubsystem.State state : LEDSubsystem.State.values()) {
            assertEquals(
                    LEDSubsystem.State.FAULT,
                    LEDSubsystem.resolveState(state, true));
        }
        assertEquals(
                LEDSubsystem.State.TELEOP,
                LEDSubsystem.resolveState(LEDSubsystem.State.TELEOP, false));
    }
}
