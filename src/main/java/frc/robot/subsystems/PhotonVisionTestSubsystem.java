// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Minimal, telemetry-only PhotonVision hardware-link test for camera
 * {@code 9281cam1}.
 *
 * <p>
 * This subsystem deliberately does not estimate robot pose, feed drivetrain
 * odometry, or
 * control hardware. It remains independent of the production Limelight vision
 * subsystem.
 */
public final class PhotonVisionTestSubsystem extends SubsystemBase implements AutoCloseable {
    static final double RESULT_TIMEOUT_SECONDS = 0.50;

    private final PhotonCamera camera = new PhotonCamera("9281cam2");
    private final ResultTracker resultTracker = new ResultTracker(RESULT_TIMEOUT_SECONDS);

    private final BooleanEntry enabledEntry;
    private final BooleanPublisher cameraConnectedPublisher;
    private final BooleanPublisher hasTargetPublisher;
    private final IntegerPublisher tagIdPublisher;
    private final IntegerPublisher targetCountPublisher;
    private final DoublePublisher resultAgePublisher;
    private final DoublePublisher distancePublisher;
    private final StringPublisher statusPublisher;

    public PhotonVisionTestSubsystem() {
        NetworkTable table = NetworkTableInstance.getDefault()
                .getTable("FRC8011")
                .getSubTable("Vision")
                .getSubTable("PhotonTest");
        enabledEntry = table.getBooleanTopic("Enabled").getEntry(false);
        cameraConnectedPublisher = table.getBooleanTopic("CameraConnected").publish();
        hasTargetPublisher = table.getBooleanTopic("HasTarget").publish();
        tagIdPublisher = table.getIntegerTopic("TagId").publish();
        targetCountPublisher = table.getIntegerTopic("TargetCount").publish();
        resultAgePublisher = table.getDoubleTopic("ResultAgeMs").publish();
        distancePublisher = table.getDoubleTopic("DistanceMeters").publish();
        statusPublisher = table.getStringTopic("Status").publish();

        enabledEntry.setDefault(false);
        publish(new State(false, false, -1, 0, -1.0, "Disabled", -1.0));
    }

    @Override
    public void periodic() {
        // PhotonLib drains a FIFO here; this must remain the only call in each robot
        // cycle.
        List<PhotonPipelineResult> unreadResults = camera.getAllUnreadResults();
        PhotonFrame newestFrame = unreadResults.isEmpty()
                ? null
                : summarize(unreadResults.get(unreadResults.size() - 1));

        State state = resultTracker.update(
                enabledEntry.get(false),
                camera.isConnected(),
                newestFrame,
                Timer.getFPGATimestamp());
        publish(state);
    }

    private void publish(State state) {
        cameraConnectedPublisher.set(state.cameraConnected());
        hasTargetPublisher.set(state.hasTarget());
        tagIdPublisher.set(state.tagId());
        targetCountPublisher.set(state.targetCount());
        resultAgePublisher.set(state.resultAgeMilliseconds());
        distancePublisher.set(state.distanceMeters());
        statusPublisher.set(state.status());
    }

    private static PhotonFrame summarize(PhotonPipelineResult result) {
        int bestTagId = result.hasTargets()
                ? result.getBestTarget().getFiducialId()
                : -1;
        int aprilTagCount = 0;
        for (PhotonTrackedTarget target : result.getTargets()) {
            int fiducialId = target.getFiducialId();
            if (fiducialId >= 0) {
                aprilTagCount++;
            }
        }
        double distanceMeters = bestTagId >= 0
                ? result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm()
                : -1.0;
        return new PhotonFrame(bestTagId, aprilTagCount, distanceMeters);
    }

    @Override
    public void close() {
        camera.close();
        enabledEntry.close();
        cameraConnectedPublisher.close();
        hasTargetPublisher.close();
        tagIdPublisher.close();
        targetCountPublisher.close();
        resultAgePublisher.close();
        distancePublisher.close();
        statusPublisher.close();
    }

    record PhotonFrame(int bestTagId, int aprilTagCount, double distanceMeters) {
        PhotonFrame(int bestTagId, int aprilTagCount) {
            this(bestTagId, aprilTagCount, -1.0);
        }
    }

    record State(
            boolean cameraConnected,
            boolean hasTarget,
            int tagId,
            int targetCount,
            double resultAgeMilliseconds,
            String status,
            double distanceMeters) {
    }

    static final class ResultTracker {
        private final double timeoutSeconds;
        private PhotonFrame lastFrame;
        private double lastResultTimeSeconds = Double.NaN;

        ResultTracker(double timeoutSeconds) {
            if (!Double.isFinite(timeoutSeconds) || timeoutSeconds <= 0.0) {
                throw new IllegalArgumentException("Result timeout must be positive and finite");
            }
            this.timeoutSeconds = timeoutSeconds;
        }

        State update(
                boolean enabled,
                boolean cameraConnected,
                PhotonFrame newestFrame,
                double nowSeconds) {
            if (!enabled) {
                clearLastResult();
                return new State(cameraConnected, false, -1, 0, -1.0, "Disabled", -1.0);
            }
            if (!cameraConnected) {
                clearLastResult();
                return new State(false, false, -1, 0, -1.0, "CameraDisconnected", -1.0);
            }

            if (newestFrame != null) {
                lastFrame = newestFrame;
                lastResultTimeSeconds = nowSeconds;
            }

            double resultAgeSeconds = Double.isFinite(lastResultTimeSeconds)
                    ? Math.max(0.0, nowSeconds - lastResultTimeSeconds)
                    : Double.NaN;
            double resultAgeMilliseconds = Double.isFinite(resultAgeSeconds)
                    ? resultAgeSeconds * 1000.0
                    : -1.0;

            if (lastFrame == null || resultAgeSeconds > timeoutSeconds) {
                return new State(true, false, -1, 0, resultAgeMilliseconds, "NoTarget", -1.0);
            }
            if (lastFrame.aprilTagCount() <= 0 || lastFrame.bestTagId() < 0) {
                return new State(true, false, -1, 0, resultAgeMilliseconds, "NoTarget", -1.0);
            }
            return new State(
                    true,
                    true,
                    lastFrame.bestTagId(),
                    lastFrame.aprilTagCount(),
                    resultAgeMilliseconds,
                    "YES",
                    Double.isFinite(lastFrame.distanceMeters()) && lastFrame.distanceMeters() >= 0.0
                            ? lastFrame.distanceMeters()
                            : -1.0);
        }

        private void clearLastResult() {
            lastFrame = null;
            lastResultTimeSeconds = Double.NaN;
        }
    }
}
