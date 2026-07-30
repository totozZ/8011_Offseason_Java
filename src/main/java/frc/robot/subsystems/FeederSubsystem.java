// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.frc8011.WayiMotor;
import frc.robot.logging.RobotHealthLogger;

public class FeederSubsystem extends SubsystemBase {
    private final WayiMotor backwardFeeder = new WayiMotor(
            Constants.FeederConstants.backwardFeederMotorId,
            Constants.CanConstants.rioCanBus);
    private final WayiMotor upwardFeeder = new WayiMotor(
            Constants.FeederConstants.upwardFeederMotorId,
            Constants.CanConstants.rioCanBus);
    private boolean healthActive = false;
    private String healthState = "Idle";
    private double backwardHealthDemand;
    private double upwardHealthDemand;
    private double healthUpwardVelocity;

    public FeederSubsystem() {
        initialize();
    }

    @Override
    public void periodic() {
        upwardFeeder.receiveVelocity();
        healthUpwardVelocity = upwardFeeder.getCachedVelocity();
    }

    private void initialize() {
        TalonFXConfiguration backwardConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive))
                .withSlot0(new Slot0Configs()
                        .withKS(0.12)
                        .withKV(0.12)
                        .withKP(0.03))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(80)
                        .withSupplyCurrentLowerLimit(80)
                        .withStatorCurrentLimit(120)
                        .withSupplyCurrentLimitEnable(true));
        applyWithRetry(backwardFeeder, backwardConfig);
        backwardFeeder.setInvert(-1);
        backwardFeeder.setCurrentSpeed(0.2);

        TalonFXConfiguration upwardConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.CounterClockwise_Positive))
                .withSlot0(new Slot0Configs()
                        .withKS(10)
                        .withKP(9))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(120)
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(40)
                        .withSupplyCurrentLowerLimit(40)
                        .withSupplyCurrentLimitEnable(true));
        applyWithRetry(upwardFeeder, upwardConfig);
        upwardFeeder.setInvert(-1);
        upwardFeeder.setStatusSignalUpdateFrequency(50);
    }

    public void setUpwardFeederVelocity(double velocityRps) {
        upwardHealthDemand = velocityRps;
        updateHealthState();
        upwardFeeder.setVelocityTorqueCurrent(velocityRps);
        upwardFeeder.control();
    }

    public void setBackwardFeederDuty(double duty) {
        backwardHealthDemand = duty;
        updateHealthState();
        backwardFeeder.setNormalizedDutyCycle(duty);
        backwardFeeder.control();
    }

    public Command setBackwardFeederDutyCommand(double duty) {
        return runOnce(() -> setBackwardFeederDuty(duty));
    }

    public void setUpwardDuty(double duty) {
        upwardHealthDemand = duty;
        updateHealthState();
        upwardFeeder.setNormalizedDutyCycle(duty);
        upwardFeeder.control();
    }

    public void stop() {
        backwardHealthDemand = 0.0;
        upwardHealthDemand = 0.0;
        healthActive = false;
        healthState = "Idle";
        backwardFeeder.setCoast();
        upwardFeeder.setCoast();
        backwardFeeder.control();
        upwardFeeder.control();
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }

    public double getBackwardFeederVelocity() {
        return backwardFeeder.getVelocity();
    }

    public double getUpwardFeederVelocity() {
        return upwardFeeder.getVelocity();
    }

    public double getUpwardFeederCurrent() {
        return upwardFeeder.getCurrent();
    }

    /** Registers feeder motors and their sustained run state with the central logger. */
    public void registerHealthLogging(RobotHealthLogger logger) {
        if (logger == null) {
            return;
        }
        logger.registerTalonFX("Feeder", "Backward", backwardFeeder.getMotor());
        logger.registerTalonFX("Feeder", "Upward", upwardFeeder.getMotor());
        logger.registerSubsystem(
                "Feeder",
                this,
                () -> healthActive,
                () -> healthState,
                () -> upwardFeeder.getData().targetVelocity,
                () -> healthUpwardVelocity);
    }

    private void updateHealthState() {
        healthActive = Math.abs(backwardHealthDemand) > 1e-6
                || Math.abs(upwardHealthDemand) > 1e-6;
        healthState = healthActive ? "Feeding" : "Idle";
    }

    private static void applyWithRetry(WayiMotor motor, TalonFXConfiguration config) {
        for (int i = 0; i < 5; i++) {
            StatusCode status = motor.applyConfig(config);
            if (status.isOK()) {
                return;
            }
        }
    }
}
