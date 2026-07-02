// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import frc.robot.subsystems.ClientSubsystem.MatchPhaseState;

import org.junit.jupiter.api.Test;

class ClientSubsystemTest {
    private static final double EPSILON = 1e-9;

    @Test
    void disabledAndAutoMatchCppBehavior() {
        MatchPhaseState disabled = ClientSubsystem.calculateMatchPhase(
                -1.0,
                false,
                false,
                true);
        MatchPhaseState auto = ClientSubsystem.calculateMatchPhase(
                12.5,
                true,
                false,
                true);

        assertEquals("Waiting/Disabled", disabled.phase());
        assertEquals(0.0, disabled.countdownSeconds(), EPSILON);
        assertFalse(disabled.canShootNow());
        assertEquals("AUTO (Both Active)", auto.phase());
        assertEquals(12.5, auto.countdownSeconds(), EPSILON);
        assertTrue(auto.canShootNow());
    }

    @Test
    void teleopAlternatesShootPermissionAtCppBoundaries() {
        MatchPhaseState transition = teleop(135.0, true);
        MatchPhaseState switch1 = teleop(130.0, true);
        MatchPhaseState switch2 = teleop(104.9, true);
        MatchPhaseState switch3 = teleop(79.9, true);
        MatchPhaseState switch4 = teleop(54.9, true);
        MatchPhaseState endgame = teleop(29.9, true);

        assertEquals("Transition (Both)", transition.phase());
        assertTrue(transition.canShootNow());
        assertEquals("Switch 1 (OUR Turn)", switch1.phase());
        assertTrue(switch1.canShootNow());
        assertEquals("Switch 2 (OPPONENT)", switch2.phase());
        assertFalse(switch2.canShootNow());
        assertEquals("Switch 3 (OUR Turn)", switch3.phase());
        assertTrue(switch3.canShootNow());
        assertEquals("Switch 4 (OPPONENT)", switch4.phase());
        assertFalse(switch4.canShootNow());
        assertEquals("ENDGAME (Both)", endgame.phase());
        assertTrue(endgame.canShootNow());
    }

    @Test
    void opponentFirstInvertsOnlySwitchPeriods() {
        assertFalse(teleop(120.0, false).canShootNow());
        assertTrue(teleop(90.0, false).canShootNow());
        assertFalse(teleop(70.0, false).canShootNow());
        assertTrue(teleop(40.0, false).canShootNow());
    }

    private static MatchPhaseState teleop(
            double matchTimeSeconds,
            boolean ourAllianceFirst) {
        return ClientSubsystem.calculateMatchPhase(
                matchTimeSeconds,
                false,
                true,
                ourAllianceFirst);
    }
}
