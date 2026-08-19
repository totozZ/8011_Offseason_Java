// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

/** Owns the two brushed SPARK MAX motors on the official 2026 KitBot fuel mechanism. */
public class KitBotFuelSubsystem extends SubsystemBase implements AutoCloseable {
    private final SparkMax feederMotor = new SparkMax(
            Constants.BabyAutoConstants.FEEDER_MOTOR_CAN_ID,
            MotorType.kBrushed);
    private final SparkMax launcherMotor = new SparkMax(
            Constants.BabyAutoConstants.INTAKE_LAUNCHER_MOTOR_CAN_ID,
            MotorType.kBrushed);

    public KitBotFuelSubsystem() {
        reportConfiguration(
                "feeder",
                feederMotor.configure(
                        createFeederConfiguration(),
                        ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters));
        reportConfiguration(
                "intake/launcher",
                launcherMotor.configure(
                        createLauncherConfiguration(),
                        ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters));
    }

    /** Immediately sets and retains normalized feeder output in the range -1 to +1. */
    public void setFeederSpeed(double speed) {
        feederMotor.set(clampNormalizedSpeed(speed));
    }

    /** Immediately sets and retains normalized launcher output in the range -1 to +1. */
    public void setLauncherSpeed(double speed) {
        launcherMotor.set(clampNormalizedSpeed(speed));
    }

    /** Continuously applies the official KitBot intake voltage pair. */
    public Command holdIntakeCommand() {
        return run(() -> {
            feederMotor.setVoltage(Constants.BabyAutoConstants.INTAKE_FEEDER_VOLTS);
            launcherMotor.setVoltage(Constants.BabyAutoConstants.INTAKE_LAUNCHER_VOLTS);
        });
    }

    /** Continuously spins up the launcher while holding fuel away from it. */
    public Command holdSpinUpCommand() {
        return run(() -> {
            feederMotor.setVoltage(Constants.BabyAutoConstants.SPIN_UP_FEEDER_VOLTS);
            launcherMotor.setVoltage(Constants.BabyAutoConstants.LAUNCH_LAUNCHER_VOLTS);
        });
    }

    /** Continuously feeds fuel into the running launcher. */
    public Command holdLaunchCommand() {
        return run(() -> {
            feederMotor.setVoltage(Constants.BabyAutoConstants.LAUNCH_FEEDER_VOLTS);
            launcherMotor.setVoltage(Constants.BabyAutoConstants.LAUNCH_LAUNCHER_VOLTS);
        });
    }

    public Command setFeederSpeedCommand(double speed) {
        return runOnce(() -> setFeederSpeed(speed));
    }

    public Command setLauncherSpeedCommand(double speed) {
        return runOnce(() -> setLauncherSpeed(speed));
    }

    public Command stopFeederCommand() {
        return runOnce(feederMotor::stopMotor);
    }

    public Command stopLauncherCommand() {
        return runOnce(launcherMotor::stopMotor);
    }

    /** Removes output from both fuel motors. */
    public void stop() {
        feederMotor.stopMotor();
        launcherMotor.stopMotor();
    }

    @Override
    public void close() {
        stop();
        feederMotor.close();
        launcherMotor.close();
    }

    static SparkMaxConfig createFeederConfiguration() {
        SparkMaxConfig configuration = new SparkMaxConfig();
        configuration.smartCurrentLimit(Constants.BabyAutoConstants.MOTOR_CURRENT_LIMIT_AMPS);
        return configuration;
    }

    static SparkMaxConfig createLauncherConfiguration() {
        SparkMaxConfig configuration = new SparkMaxConfig();
        configuration
                .inverted(true)
                .smartCurrentLimit(Constants.BabyAutoConstants.MOTOR_CURRENT_LIMIT_AMPS);
        return configuration;
    }

    private static double clampNormalizedSpeed(double speed) {
        if (!Double.isFinite(speed)) {
            throw new IllegalArgumentException("Motor speed must be finite");
        }
        return MathUtil.clamp(speed, -1.0, 1.0);
    }

    private static void reportConfiguration(String motorName, REVLibError status) {
        if (status != REVLibError.kOk) {
            DriverStation.reportError(
                    "KitBot " + motorName + " SPARK MAX configuration failed: " + status,
                    false);
        }
    }
}
