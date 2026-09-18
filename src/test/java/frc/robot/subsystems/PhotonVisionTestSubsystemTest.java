// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.subsystems.PhotonVisionTestSubsystem.PhotonFrame;
import frc.robot.subsystems.PhotonVisionTestSubsystem.ResultTracker;
import frc.robot.subsystems.PhotonVisionTestSubsystem.State;

import org.junit.jupiter.api.Test;

class PhotonVisionTestSubsystemTest {
    private static final double EPSILON = 1e-9;
    private static final double TIMEOUT_SECONDS = 0.50;

    @Test
    void disabledIgnoresCameraResults() {
        ResultTracker tracker = new ResultTracker(TIMEOUT_SECONDS);

        State disabled = tracker.update(false, true, new PhotonFrame(7, 1), 10.0);
        State reenabledWithoutNewFrame = tracker.update(true, true, null, 10.01);

        assertFalse(disabled.hasTarget());
        assertEquals(-1, disabled.tagId());
        assertEquals(0, disabled.targetCount());
        assertEquals("Disabled", disabled.status());
        assertEquals("NoTarget", reenabledWithoutNewFrame.status());
    }

    @Test
    void disconnectedCameraOverridesQueuedResults() {
        ResultTracker tracker = new ResultTracker(TIMEOUT_SECONDS);

        State disconnected = tracker.update(true, false, new PhotonFrame(7, 1), 20.0);

        assertFalse(disconnected.cameraConnected());
        assertFalse(disconnected.hasTarget());
        assertEquals(-1, disconnected.tagId());
        assertEquals(0, disconnected.targetCount());
        assertEquals("CameraDisconnected", disconnected.status());
    }

    @Test
    void connectedFrameWithoutAprilTagsReportsNoTarget() {
        ResultTracker tracker = new ResultTracker(TIMEOUT_SECONDS);

        State state = tracker.update(true, true, new PhotonFrame(-1, 0), 30.0);

        assertTrue(state.cameraConnected());
        assertFalse(state.hasTarget());
        assertEquals(-1, state.tagId());
        assertEquals(0, state.targetCount());
        assertEquals(0.0, state.resultAgeMilliseconds(), EPSILON);
        assertEquals("NoTarget", state.status());
    }

    @Test
    void aprilTagFrameReportsYesBestTagAndCount() {
        ResultTracker tracker = new ResultTracker(TIMEOUT_SECONDS);

        State state = tracker.update(true, true, new PhotonFrame(11, 2), 40.0);

        assertTrue(state.hasTarget());
        assertEquals(11, state.tagId());
        assertEquals(2, state.targetCount());
        assertEquals("YES", state.status());
    }

    @Test
    void staleAprilTagResultTimesOut() {
        ResultTracker tracker = new ResultTracker(TIMEOUT_SECONDS);
        tracker.update(true, true, new PhotonFrame(3, 1), 50.0);

        State stale = tracker.update(true, true, null, 50.0 + TIMEOUT_SECONDS + 0.001);

        assertFalse(stale.hasTarget());
        assertEquals(-1, stale.tagId());
        assertEquals(0, stale.targetCount());
        assertEquals(501.0, stale.resultAgeMilliseconds(), EPSILON);
        assertEquals("NoTarget", stale.status());
    }
}
