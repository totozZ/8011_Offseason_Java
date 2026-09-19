package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.photonvision.PhotonPoseEstimator;

/** Camera mounting transform relative to the same robot origin used by drivetrain odometry. */
public record PhotonCameraConfig(
        double xMeters,
        double yMeters,
        double zMeters,
        double rollDegrees,
        double pitchDegrees,
        double yawDegrees) {
    public PhotonCameraConfig {
        if (!Double.isFinite(xMeters) || !Double.isFinite(yMeters) || !Double.isFinite(zMeters)
                || !Double.isFinite(rollDegrees) || !Double.isFinite(pitchDegrees)
                || !Double.isFinite(yawDegrees)) {
            throw new IllegalArgumentException("Camera mounting parameters must be finite");
        }
    }

    /** WPILib axes: X forward, Y left, Z up; rotations follow the right-hand rule. */
    public Transform3d robotToCamera() {
        return new Transform3d(
                new Translation3d(xMeters, yMeters, zMeters),
                new Rotation3d(Math.toRadians(rollDegrees), Math.toRadians(pitchDegrees),
                        Math.toRadians(yawDegrees)));
    }

    /** Creates an estimator with this mount and the caller's actual field layout. */
    public PhotonPoseEstimator createEstimator(AprilTagFieldLayout fieldLayout) {
        return new PhotonPoseEstimator(java.util.Objects.requireNonNull(fieldLayout), robotToCamera());
    }

    /** Applies a changed mount to an existing estimator. Call from the robot loop thread. */
    public void applyTo(PhotonPoseEstimator estimator) {
        java.util.Objects.requireNonNull(estimator).setRobotToCameraTransform(robotToCamera());
    }
}
