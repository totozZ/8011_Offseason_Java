package frc.robot.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import edu.wpi.first.math.geometry.Translation3d;
import org.junit.jupiter.api.Test;

class PhotonCameraConfigTest {
    @Test
    void mountingUsesMetersAndConvertsDegreesWithWpilibAxes() {
        var transform = new PhotonCameraConfig(0.3, -0.2, 0.5, 0, 0, 90).robotToCamera();
        assertEquals(new Translation3d(0.3, -0.2, 0.5), transform.getTranslation());
        var cameraForward = new Translation3d(1, 0, 0).rotateBy(transform.getRotation());
        assertEquals(0, cameraForward.getX(), 1e-9);
        assertEquals(1, cameraForward.getY(), 1e-9);
    }

    @Test
    void rejectsInvalidMountParameters() {
        assertThrows(IllegalArgumentException.class,
                () -> new PhotonCameraConfig(0, 0, Double.NaN, 0, 0, 0));
        assertThrows(IllegalArgumentException.class,
                () -> new PhotonCameraConfig(0, 0, 0, 0, Double.POSITIVE_INFINITY, 0));
    }
}
