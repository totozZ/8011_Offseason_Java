package frc.robot;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
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
            assertTrue(topicExists("/SmartDashboard/FRC8011/Auto/Chooser/.type"));
            assertArrayEquals(
                    new String[] {"Do Nothing", "Baby Auto"},
                    NetworkTableInstance.getDefault()
                            .getStringArrayTopic(
                                    "/SmartDashboard/FRC8011/Auto/Chooser/options")
                            .subscribe(new String[0])
                            .get());

            assertFalse(topicExists("/FRC8011/Vision/limelight-left/Accepted"));
            assertFalse(topicExists("/FRC8011/LED/State"));
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
