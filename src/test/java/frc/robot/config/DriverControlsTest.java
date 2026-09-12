package frc.robot.config;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.Constants;

import org.junit.jupiter.api.Test;

class DriverControlsTest {
    @Test
    void diagonalInputPreservesDirectionAndSharesStraightLineSpeedLimit() {
        var straight = DriverControls.toRobotSpeeds(1.0, 0.0, 0.0);
        var diagonal = DriverControls.toRobotSpeeds(1.0, -1.0, 0.0);
        assertEquals(straight.vxMetersPerSecond,
                Math.hypot(diagonal.vxMetersPerSecond, diagonal.vyMetersPerSecond), 1e-9);
        assertEquals(diagonal.vxMetersPerSecond, -diagonal.vyMetersPerSecond, 1e-9);
        // The hardware free speed itself is locked in SoccerBotDrivetrainConstantsTest; here we
        // only check that full stick travel maps to the common limit times the drive scale.
        assertEquals(Constants.DriveConstants.COMMON_SPEED_METERS_PER_SECOND
                * Constants.OperatorConstants.DRIVE_SPEED_SCALE,
                straight.vxMetersPerSecond, 1e-9);
        for (var profile : DrivetrainProfile.values()) {
            assertTrue(Constants.DriveConstants.COMMON_SPEED_METERS_PER_SECOND
                    <= profile.speedAt12Volts().baseUnitMagnitude());
        }
    }

    @Test
    void sticksHaveIndependentDeadbandsAndSymmetricResponse() {
        var idle = DriverControls.toRobotSpeeds(0.02, -0.02, 0.03);
        assertEquals(0.0, idle.vxMetersPerSecond, 1e-9);
        assertEquals(0.0, idle.vyMetersPerSecond, 1e-9);
        assertEquals(0.0, idle.omegaRadiansPerSecond, 1e-9);
        var positive = DriverControls.toRobotSpeeds(0.3, 0.4, 0.5);
        var negative = DriverControls.toRobotSpeeds(-0.3, -0.4, -0.5);
        assertEquals(positive.vxMetersPerSecond, -negative.vxMetersPerSecond, 1e-9);
        assertEquals(positive.vyMetersPerSecond, -negative.vyMetersPerSecond, 1e-9);
        assertEquals(positive.omegaRadiansPerSecond, -negative.omegaRadiansPerSecond, 1e-9);
        assertEquals(0.75, positive.vxMetersPerSecond / positive.vyMetersPerSecond, 1e-9);
    }
}
