// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

/** Owns the CAN 20 intake and CAN 21 shooter TalonFX motors. */
public class KitBotFuelSubsystem extends SubsystemBase implements AutoCloseable {
    private final TalonFX intakeMotor = new TalonFX(
            Constants.BabyAutoConstants.INTAKE_MOTOR_CAN_ID,
            Constants.CanConstants.RIO_CAN_BUS);
    private final TalonFX shooterMotor = new TalonFX(
            Constants.BabyAutoConstants.SHOOTER_MOTOR_CAN_ID,
            Constants.CanConstants.RIO_CAN_BUS);
    private final DutyCycleOut intakeRequest = new DutyCycleOut(0.0).withEnableFOC(false);
    private final DutyCycleOut shooterRequest = new DutyCycleOut(0.0).withEnableFOC(false);
    private final CoastOut coastRequest = new CoastOut();

    public KitBotFuelSubsystem() {
        TalonFXConfiguration configuration = createMotorConfiguration();
        reportConfiguration("intake", intakeMotor.getConfigurator().apply(configuration));
        reportConfiguration("shooter", shooterMotor.getConfigurator().apply(configuration));
    }

    /** Runs both motors in the intake direction used by the reference SoccerBot project. */
    public Command holdIntakeCommand() {
        return run(() -> setOutputs(
                Constants.BabyAutoConstants.INTAKE_MOTOR_INTAKE_DUTY_CYCLE,
                Constants.BabyAutoConstants.SHOOTER_MOTOR_INTAKE_DUTY_CYCLE))
                .finallyDo(this::stop);
    }

    /** Spins up only the shooter motor before an automatic shot. */
    public Command holdShooterSpinUpCommand() {
        return run(() -> {
            intakeMotor.setControl(coastRequest);
            setShooterSpeed(Constants.BabyAutoConstants.SHOOTER_MOTOR_SHOOT_DUTY_CYCLE);
        }).finallyDo(this::stop);
    }

    /** Runs both motors in the shoot direction used by the reference SoccerBot project. */
    public Command holdShootCommand() {
        return run(() -> setOutputs(
                Constants.BabyAutoConstants.INTAKE_MOTOR_SHOOT_DUTY_CYCLE,
                Constants.BabyAutoConstants.SHOOTER_MOTOR_SHOOT_DUTY_CYCLE))
                .finallyDo(this::stop);
    }

    public void setIntakeSpeed(double dutyCycle) {
        intakeMotor.setControl(intakeRequest.withOutput(clampNormalizedSpeed(dutyCycle)));
    }

    public void setShooterSpeed(double dutyCycle) {
        shooterMotor.setControl(shooterRequest.withOutput(clampNormalizedSpeed(dutyCycle)));
    }

    public Command setIntakeSpeedCommand(double dutyCycle) {
        return runOnce(() -> setIntakeSpeed(dutyCycle));
    }

    public Command setShooterSpeedCommand(double dutyCycle) {
        return runOnce(() -> setShooterSpeed(dutyCycle));
    }

    public Command stopIntakeCommand() {
        return runOnce(() -> intakeMotor.setControl(coastRequest));
    }

    public Command stopShooterCommand() {
        return runOnce(() -> shooterMotor.setControl(coastRequest));
    }

    /** Coasts both mechanism motors. */
    public void stop() {
        intakeMotor.setControl(coastRequest);
        shooterMotor.setControl(coastRequest);
    }

    @Override
    public void close() {
        stop();
        intakeMotor.close();
        shooterMotor.close();
    }

    static TalonFXConfiguration createMotorConfiguration() {
        return new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.CounterClockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Coast))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(
                                Constants.BabyAutoConstants.MOTOR_CURRENT_LIMIT_AMPS)
                        .withSupplyCurrentLimitEnable(true));
    }

    private void setOutputs(double intakeDutyCycle, double shooterDutyCycle) {
        setIntakeSpeed(intakeDutyCycle);
        setShooterSpeed(shooterDutyCycle);
    }

    private static double clampNormalizedSpeed(double dutyCycle) {
        if (!Double.isFinite(dutyCycle)) {
            throw new IllegalArgumentException("Motor duty cycle must be finite");
        }
        return MathUtil.clamp(dutyCycle, -1.0, 1.0);
    }

    private static void reportConfiguration(String motorName, StatusCode status) {
        if (!status.isOK()) {
            DriverStation.reportError(
                    "KitBot " + motorName + " TalonFX configuration failed: " + status,
                    false);
        }
    }
}
