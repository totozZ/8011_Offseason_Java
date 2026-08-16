// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.subsystems.LEDSubsystem;

/** Owns WPILib lifecycle, logging, system power telemetry, and autonomous scheduling. */
public class Robot extends TimedRobot {
    private static final double BROWNOUT_VOLTAGE_VOLTS = 7.0;
    private static final int SYSTEM_TELEMETRY_PERIOD_LOOPS = 5; // 20 ms loop -> 10 Hz.

    private RobotContainer robotContainer;
    private Command autonomousCommand;
    private PowerDistribution powerDistribution;

    private DoublePublisher batteryVoltagePublisher;
    private DoublePublisher totalCurrentPublisher;
    private DoublePublisher brownoutVoltagePublisher;
    private BooleanPublisher brownedOutPublisher;
    private StringPublisher modePublisher;
    private int telemetryLoopCounter;

    @Override
    public void robotInit() {
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog(), true);
        RobotController.setBrownoutVoltage(BROWNOUT_VOLTAGE_VOLTS);

        powerDistribution = new PowerDistribution(
                Constants.CanConstants.REV_PDH_ID,
                PowerDistribution.ModuleType.kRev);
        initializeSystemTelemetry();
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        if (telemetryLoopCounter++ % SYSTEM_TELEMETRY_PERIOD_LOOPS == 0) {
            publishSystemTelemetry();
        }
    }

    @Override
    public void disabledInit() {
        robotContainer.setLedState(LEDSubsystem.State.DISABLED);
    }

    @Override
    public void autonomousInit() {
        robotContainer.setLedState(LEDSubsystem.State.AUTONOMOUS);
        autonomousCommand = robotContainer.getAutonomousCommand();
        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }

    @Override
    public void teleopInit() {
        robotContainer.setLedState(LEDSubsystem.State.TELEOP);
        if (autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(autonomousCommand);
            autonomousCommand = null;
        }
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        robotContainer.setLedState(LEDSubsystem.State.TEST);
    }

    @Override
    public void close() {
        if (robotContainer != null) {
            robotContainer.close();
        }
        if (powerDistribution != null) {
            powerDistribution.close();
        }
        closePublishers();
        super.close();
    }

    private void initializeSystemTelemetry() {
        NetworkTable table = NetworkTableInstance.getDefault()
                .getTable("FRC8011")
                .getSubTable("Robot");
        batteryVoltagePublisher = table.getDoubleTopic("BatteryVoltageV").publish();
        totalCurrentPublisher = table.getDoubleTopic("PDHTotalCurrentA").publish();
        brownoutVoltagePublisher = table.getDoubleTopic("BrownoutVoltageV").publish();
        brownedOutPublisher = table.getBooleanTopic("BrownedOut").publish();
        modePublisher = table.getStringTopic("Mode").publish();
    }

    private void publishSystemTelemetry() {
        boolean brownedOut = RobotController.isBrownedOut();
        batteryVoltagePublisher.set(RobotController.getBatteryVoltage());
        totalCurrentPublisher.set(powerDistribution.getTotalCurrent());
        brownoutVoltagePublisher.set(BROWNOUT_VOLTAGE_VOLTS);
        brownedOutPublisher.set(brownedOut);
        modePublisher.set(currentMode());
        robotContainer.setLedFault(brownedOut);
    }

    private static String currentMode() {
        if (DriverStation.isDisabled()) {
            return "Disabled";
        }
        if (DriverStation.isAutonomous()) {
            return "Autonomous";
        }
        if (DriverStation.isTeleop()) {
            return "Teleop";
        }
        if (DriverStation.isTest()) {
            return "Test";
        }
        return "Unknown";
    }

    private void closePublishers() {
        if (batteryVoltagePublisher != null) {
            batteryVoltagePublisher.close();
            totalCurrentPublisher.close();
            brownoutVoltagePublisher.close();
            brownedOutPublisher.close();
            modePublisher.close();
        }
    }
}
