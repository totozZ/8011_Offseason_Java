// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private static final double kSchedulerOverrunMs = 20.0;

    private Command autonomousCommand;
    private final RobotContainer robotContainer = new RobotContainer();
    private final Field2d simField = new Field2d();
    private int schedulerOverrunCount = 0;
    private double schedulerLoopMsMax = 0.0;
    private double lastEpochPrintSeconds = -1.0;

    @Override
    public void robotPeriodic() {
        CommandScheduler scheduler = CommandScheduler.getInstance();
        double loopStartSeconds = Timer.getFPGATimestamp();

        robotContainer.updateDriverPerspective();
        scheduler.run();

        double loopEndSeconds = Timer.getFPGATimestamp();
        double schedulerLoopMs = (loopEndSeconds - loopStartSeconds) * 1000.0;
        schedulerLoopMsMax = Math.max(schedulerLoopMsMax, schedulerLoopMs);

        SmartDashboard.putNumber("SchedulerLoopMs", schedulerLoopMs);
        SmartDashboard.putNumber("SchedulerLoopMsMax", schedulerLoopMsMax);
        SmartDashboard.putNumber("SchedulerLoopOverrunThresholdMs", kSchedulerOverrunMs);
        SmartDashboard.putBoolean("SchedulerLoopOverrun", schedulerLoopMs > kSchedulerOverrunMs);

        if (schedulerLoopMs > kSchedulerOverrunMs) {
            schedulerOverrunCount++;
            if (lastEpochPrintSeconds < 0.0 || (loopEndSeconds - lastEpochPrintSeconds) > 0.5) {
                scheduler.printWatchdogEpochs();
                lastEpochPrintSeconds = loopEndSeconds;
            }
        }
        SmartDashboard.putNumber("SchedulerLoopOverrunCount", schedulerOverrunCount);
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {
        robotContainer.onAutonomousExit();
    }

    @Override
    public void teleopInit() {
        robotContainer.onTeleopInit();
        if (autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationInit() {
        SmartDashboard.putData("Simulation Field", simField);
    }

    @Override
    public void simulationPeriodic() {
        simField.setRobotPose(robotContainer.drivetrain.getState().Pose);
    }
}
