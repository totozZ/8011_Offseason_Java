package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.junit.jupiter.api.Test;

class FoundationSimulationSmokeTest {
    @Test
    void robotStartsFoundationServicesAndPublishesExpectedTopics() throws InterruptedException {
        assertTrue(HAL.initialize(500, 0));

        Robot robot = new Robot();
        try {
            robot.robotInit();
            robot.disabledInit();

            for (int loop = 0; loop < 10; loop++) {
                robot.robotPeriodic();
                Thread.sleep(20);
            }
            NetworkTableInstance.getDefault().flushLocal();

            assertTrue(topicExists("/FRC8011/Robot/BatteryVoltageV"));
            assertTrue(topicExists("/FRC8011/Drive/Pose"));
            assertTrue(topicExists("/FRC8011/Vision/limelight-left/Accepted"));
            assertTrue(topicExists("/FRC8011/Vision/limelight-back/Accepted"));
            assertTrue(topicExists("/FRC8011/Vision/limelight-right/Accepted"));
            assertTrue(topicExists("/FRC8011/Vision/PhotonTest/Enabled"));
            assertTrue(topicExists("/FRC8011/Vision/PhotonTest/CameraConnected"));
            assertTrue(topicExists("/FRC8011/Vision/PhotonTest/HasTarget"));
            assertTrue(topicExists("/FRC8011/Vision/PhotonTest/TagId"));
            assertTrue(topicExists("/FRC8011/Vision/PhotonTest/TargetCount"));
            assertTrue(topicExists("/FRC8011/Vision/PhotonTest/ResultAgeMs"));
            assertTrue(topicExists("/FRC8011/Vision/PhotonTest/Status"));
            assertTrue(topicExists("/FRC8011/LED/State"));
            assertTrue(topicExists("/SmartDashboard/FRC8011/Auto/Chooser/.type"));

            assertFalse(topicExists("/FRC8011/Example/PositionRot"));
        } finally {
            CommandScheduler.getInstance().cancelAll();
            robot.close();
            DataLogManager.stop();
        }
    }

    private static boolean topicExists(String path) {
        return NetworkTableInstance.getDefault().getTopic(path).exists();
    }
}
