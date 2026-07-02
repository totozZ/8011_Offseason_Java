// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooting;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class ShotTableTest {
    private static final double EPSILON = 1e-9;

    @Test
    void hubTableClampsBelowFirstEntry() {
        ShotSetpoint setpoint = ShotTable.hub(0.5);

        assertEquals(1855.0 / 60.0, setpoint.flywheelRps(), EPSILON);
        assertEquals(1425.0 / 60.0, setpoint.feederRps(), EPSILON);
        assertEquals(17.5, setpoint.pitchDeg(), EPSILON);
    }

    @Test
    void hubTableInterpolatesBetweenEntries() {
        ShotSetpoint setpoint = ShotTable.hub(1.85);

        assertEquals(1865.0 / 60.0, setpoint.flywheelRps(), EPSILON);
        assertEquals(1432.0 / 60.0, setpoint.feederRps(), EPSILON);
        assertEquals(18.0, setpoint.pitchDeg(), EPSILON);
    }

    @Test
    void feedRequiresPitchHomeAndReadyOrTimeout() {
        assertFalse(ShotTable.feedAllowed(false, true, true));
        assertFalse(ShotTable.feedAllowed(true, false, false));
        assertTrue(ShotTable.feedAllowed(true, true, false));
        assertTrue(ShotTable.feedAllowed(true, false, true));
    }

    @Test
    void hubRegionRequiresKnownAlliance() {
        assertFalse(ShotTable.isHubRegion(AllianceSide.UNKNOWN, 1.0));
        assertTrue(ShotTable.isHubRegion(AllianceSide.BLUE, 4.0));
        assertFalse(ShotTable.isHubRegion(AllianceSide.BLUE, 6.0));
        assertTrue(ShotTable.isHubRegion(AllianceSide.RED, 12.0));
        assertFalse(ShotTable.isHubRegion(AllianceSide.RED, 10.0));
    }
}
