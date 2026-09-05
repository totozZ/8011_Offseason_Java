package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.junit.jupiter.api.Test;

class RobotControlsSimulationTest {
    @Test
    void autonomousIgnoresSticksAndXWhileTeleopStillDrives() throws Exception {
        assertTrue(HAL.initialize(500, 0));
        var scheduler = CommandScheduler.getInstance();
        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.setAutonomous(true);
        var controller = new XboxControllerSim(0);
        controller.setLeftY(-1.0);
        controller.setXButton(true);
        DriverStationSim.notifyNewData();
        try (var container = new RobotContainer()) {
            try {
                scheduler.schedule(container.getAutonomousCommand());
                runScheduler(scheduler);
                for (var target : container.drivetrain.getStateCopy().ModuleTargets) {
                    assertEquals(0.0, target.speedMetersPerSecond, 1e-6);
                }
                controller.setXButton(false);
                DriverStationSim.setAutonomous(false);
                DriverStationSim.notifyNewData();
                runScheduler(scheduler);
                for (var target : container.drivetrain.getStateCopy().ModuleTargets) {
                    assertEquals(Constants.DriveConstants.COMMON_SPEED_METERS_PER_SECOND
                            * Constants.OperatorConstants.DRIVE_SPEED_SCALE,
                            target.speedMetersPerSecond, 1e-6);
                    assertEquals(0.0, target.angle.getDegrees(), 1e-6);
                }

                controller.setLeftTriggerAxis(1.0);
                DriverStationSim.notifyNewData();
                runScheduler(scheduler);
                assertTrue(isFuelCommandScheduled(scheduler, container, "ManualIntake"));

                controller.setRightTriggerAxis(1.0);
                DriverStationSim.notifyNewData();
                runScheduler(scheduler);
                assertTrue(isFuelCommandScheduled(scheduler, container, "ManualShoot"));
                assertFalse(isFuelCommandScheduled(scheduler, container, "ManualIntake"));

                controller.setRightTriggerAxis(0.0);
                DriverStationSim.notifyNewData();
                runScheduler(scheduler);
                assertTrue(isFuelCommandScheduled(scheduler, container, "ManualIntake"));

                controller.setLeftTriggerAxis(0.0);
                DriverStationSim.notifyNewData();
                runScheduler(scheduler);
                assertFalse(isFuelCommandScheduled(scheduler, container, "ManualIntake"));
                assertFalse(isFuelCommandScheduled(scheduler, container, "ManualShoot"));

                // Disable cancels the default drive command; its end action must remove output.
                DriverStationSim.setEnabled(false);
                DriverStationSim.notifyNewData();
                runScheduler(scheduler);
                assertFalse(container.drivetrain.getDefaultCommand().isScheduled());
            } finally {
                scheduler.cancelAll();
                scheduler.getDefaultButtonLoop().clear();
                scheduler.unregisterAllSubsystems();
            }
        } finally {
            DriverStationSim.resetData();
            DriverStationSim.notifyNewData();
        }
    }

    private static void runScheduler(CommandScheduler scheduler) throws InterruptedException {
        for (int i = 0; i < 10; i++) {
            scheduler.run();
            Thread.sleep(20);
        }
    }

    private static boolean isFuelCommandScheduled(
            CommandScheduler scheduler, RobotContainer container, String commandName) {
        var command = scheduler.requiring(container.fuel);
        return command != null && commandName.equals(command.getName());
    }

}
