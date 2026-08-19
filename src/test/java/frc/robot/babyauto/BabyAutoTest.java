// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.babyauto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class BabyAutoTest {
    private static final double EPSILON = 1e-9;

    private final CommandScheduler scheduler = CommandScheduler.getInstance();

    @BeforeAll
    static void initializeHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @BeforeEach
    void enableAutonomousSimulation() {
        scheduler.cancelAll();
        scheduler.enable();
        SimHooks.restartTiming();
        SimHooks.pauseTiming();
        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setAutonomous(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
    }

    @AfterEach
    void cleanUpSchedulerAndTiming() {
        scheduler.cancelAll();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        SimHooks.resumeTiming();
    }

    @Test
    void translationVectorAndRotationAreLimitedWithoutChangingDirection() {
        BabyAuto.DriveValues limited = BabyAuto.limitDrive(3.0, 4.0, 180.0);

        assertEquals(0.6, limited.vxMetersPerSecond(), EPSILON);
        assertEquals(0.8, limited.vyMetersPerSecond(), EPSILON);
        assertEquals(90.0, limited.rotationDegreesPerSecond(), EPSILON);

        BabyAuto.DriveValues negative = BabyAuto.limitDrive(-0.3, -0.4, -120.0);
        assertEquals(-0.3, negative.vxMetersPerSecond(), EPSILON);
        assertEquals(-0.4, negative.vyMetersPerSecond(), EPSILON);
        assertEquals(-90.0, negative.rotationDegreesPerSecond(), EPSILON);
    }

    @Test
    void invalidNumbersAndTimesAreRejected() {
        FakeDriveController drive = new FakeDriveController();
        FakeFuelController fuel = new FakeFuelController();
        BabyAuto robot = new BabyAuto(drive, fuel);

        assertThrows(
                IllegalArgumentException.class,
                () -> robot.drive().setVx(Double.NaN));
        assertThrows(
                IllegalArgumentException.class,
                () -> robot.drive().setVy(Double.POSITIVE_INFINITY));
        assertThrows(
                IllegalArgumentException.class,
                () -> robot.drive().setRotation(Double.NEGATIVE_INFINITY));
        assertThrows(
                IllegalArgumentException.class,
                () -> robot.drive().forSeconds(0.0));
        assertThrows(IllegalArgumentException.class, () -> BabyAuto.waitSeconds(-1.0));
        assertThrows(
                IllegalArgumentException.class,
                () -> robot.feeder().speed(Double.NaN));
    }

    @Test
    void rawMotorSpeedIsClampedAndStopIsExplicit() {
        FakeDriveController drive = new FakeDriveController();
        FakeFuelController fuel = new FakeFuelController();
        BabyAuto robot = new BabyAuto(drive, fuel);

        Command command = BabyAuto.sequence(
                robot.feeder().speed(5.0),
                BabyAuto.waitSeconds(5.0));
        scheduler.schedule(command);
        scheduler.run();

        assertEquals(1.0, fuel.feederSpeed, EPSILON);
        assertTrue(fuel.feederActive);

        command.cancel();
        scheduler.schedule(robot.feeder().stop());
        scheduler.run();
        assertFalse(fuel.feederActive);
    }

    @Test
    void intakeAndDriveRunInParallelAndStopNormally() {
        FakeDriveController drive = new FakeDriveController();
        FakeFuelController fuel = new FakeFuelController();
        BabyAuto robot = new BabyAuto(drive, fuel);

        Command command = robot.protect(BabyAuto.parallel(
                robot.intakeFor(0.06),
                robot.drive()
                        .setVx(0.6)
                        .setVy(0.2)
                        .setRotation(30.0)
                        .forSeconds(0.06)));
        scheduler.schedule(command);
        scheduler.run();

        assertEquals(FuelState.INTAKE, fuel.state);
        assertTrue(drive.active);
        assertEquals(0.6, drive.vxMetersPerSecond, EPSILON);
        assertEquals(0.2, drive.vyMetersPerSecond, EPSILON);
        assertEquals(Math.toRadians(30.0), drive.rotationRadiansPerSecond, EPSILON);

        stepAndRun(0.08);
        assertFalse(command.isScheduled());
        assertFalse(drive.active);
        assertEquals(FuelState.STOPPED, fuel.state);
    }

    @Test
    void shootSpinsUpThenLaunchesAndStops() {
        FakeDriveController drive = new FakeDriveController();
        FakeFuelController fuel = new FakeFuelController();
        BabyAuto robot = new BabyAuto(drive, fuel);

        Command command = robot.shootFor(0.06);
        scheduler.schedule(command);
        scheduler.run();
        assertEquals(FuelState.SPIN_UP, fuel.state);

        stepAndRun(1.01);
        stepAndRun(0.02);
        assertEquals(FuelState.LAUNCH, fuel.state);

        stepAndRun(0.07);
        assertFalse(command.isScheduled());
        assertEquals(FuelState.STOPPED, fuel.state);
    }

    @Test
    void cancellingProtectedStudentAutoStopsEveryOutput() {
        FakeDriveController drive = new FakeDriveController();
        FakeFuelController fuel = new FakeFuelController();
        BabyAuto robot = new BabyAuto(drive, fuel);

        Command command = robot.protect(BabyAuto.parallel(
                BabyAuto.sequence(
                        robot.feeder().speed(0.7),
                        BabyAuto.waitSeconds(5.0)),
                robot.drive().setVx(0.5).forSeconds(5.0)));
        scheduler.schedule(command);
        scheduler.run();

        assertTrue(drive.active);
        assertTrue(fuel.feederActive);

        command.cancel();
        assertFalse(drive.active);
        assertEquals(FuelState.STOPPED, fuel.state);
        assertFalse(fuel.feederActive);
        assertFalse(fuel.launcherActive);
    }

    @Test
    void protectedStudentAutoHasAbsoluteFifteenSecondTimeout() {
        FakeDriveController drive = new FakeDriveController();
        FakeFuelController fuel = new FakeFuelController();
        BabyAuto robot = new BabyAuto(drive, fuel);

        Command command = robot.protect(BabyAuto.parallel(
                robot.intakeFor(100.0),
                robot.drive().setVx(0.5).forSeconds(100.0)));
        scheduler.schedule(command);
        scheduler.run();
        assertTrue(command.isScheduled());

        stepAndRun(15.01);
        assertFalse(command.isScheduled());
        assertFalse(drive.active);
        assertEquals(FuelState.STOPPED, fuel.state);
    }

    @Test
    void independentDriveBuildersDoNotShareValues() {
        FakeDriveController drive = new FakeDriveController();
        FakeFuelController fuel = new FakeFuelController();
        BabyAuto robot = new BabyAuto(drive, fuel);

        Command xOnly = robot.drive().setVx(0.4).forSeconds(5.0);
        Command yOnly = robot.drive().setVy(0.7).forSeconds(5.0);

        scheduler.schedule(xOnly);
        scheduler.run();
        assertEquals(0.4, drive.vxMetersPerSecond, EPSILON);
        assertEquals(0.0, drive.vyMetersPerSecond, EPSILON);
        xOnly.cancel();

        scheduler.schedule(yOnly);
        scheduler.run();
        assertEquals(0.0, drive.vxMetersPerSecond, EPSILON);
        assertEquals(0.7, drive.vyMetersPerSecond, EPSILON);
        yOnly.cancel();
    }

    private void stepAndRun(double seconds) {
        SimHooks.stepTiming(seconds);
        scheduler.run();
    }

    private static final class FakeDriveController implements DriveController {
        private final Subsystem subsystem = new Subsystem() {};
        private boolean active;
        private double vxMetersPerSecond;
        private double vyMetersPerSecond;
        private double rotationRadiansPerSecond;

        @Override
        public Command hold(
                double vxMetersPerSecond,
                double vyMetersPerSecond,
                double rotationRadiansPerSecond) {
            return Commands.run(() -> {
                active = true;
                this.vxMetersPerSecond = vxMetersPerSecond;
                this.vyMetersPerSecond = vyMetersPerSecond;
                this.rotationRadiansPerSecond = rotationRadiansPerSecond;
            }, subsystem);
        }

        @Override
        public void stop() {
            active = false;
            vxMetersPerSecond = 0.0;
            vyMetersPerSecond = 0.0;
            rotationRadiansPerSecond = 0.0;
        }
    }

    private static final class FakeFuelController implements FuelController {
        private final Subsystem subsystem = new Subsystem() {};
        private FuelState state = FuelState.STOPPED;
        private boolean feederActive;
        private boolean launcherActive;
        private double feederSpeed;

        @Override
        public Command holdIntake() {
            return Commands.run(() -> setState(FuelState.INTAKE), subsystem);
        }

        @Override
        public Command holdSpinUp() {
            return Commands.run(() -> setState(FuelState.SPIN_UP), subsystem);
        }

        @Override
        public Command holdLaunch() {
            return Commands.run(() -> setState(FuelState.LAUNCH), subsystem);
        }

        @Override
        public Command setFeederSpeed(double speed) {
            return Commands.runOnce(() -> {
                feederSpeed = speed;
                feederActive = speed != 0.0;
            }, subsystem);
        }

        @Override
        public Command setLauncherSpeed(double speed) {
            return Commands.runOnce(() -> launcherActive = speed != 0.0, subsystem);
        }

        @Override
        public Command stopFeeder() {
            return Commands.runOnce(() -> feederActive = false, subsystem);
        }

        @Override
        public Command stopLauncher() {
            return Commands.runOnce(() -> launcherActive = false, subsystem);
        }

        @Override
        public void stop() {
            state = FuelState.STOPPED;
            feederActive = false;
            launcherActive = false;
        }

        private void setState(FuelState state) {
            this.state = state;
            feederActive = true;
            launcherActive = true;
        }
    }

    private enum FuelState {
        STOPPED,
        INTAKE,
        SPIN_UP,
        LAUNCH
    }
}
