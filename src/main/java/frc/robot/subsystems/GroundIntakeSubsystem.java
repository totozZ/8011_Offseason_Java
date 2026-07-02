// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.frc8011.WayiMotor;

public class GroundIntakeSubsystem extends SubsystemBase {
    private final WayiMotor intakeRollerLeft = new WayiMotor(
            Constants.GroundIntakeConstants.intakeRollerLeftMotorId,
            Constants.CanConstants.rioCanBus);
    private final WayiMotor intakeRollerRight = new WayiMotor(
            Constants.GroundIntakeConstants.intakeRollerRightMotorId,
            Constants.CanConstants.rioCanBus);
    private final WayiMotor intakePitch = new WayiMotor(
            Constants.GroundIntakeConstants.intakePivotMotorId,
            Constants.CanConstants.rioCanBus);

    private double previousStatus = 0.0;
    private double normTargetStatus = 0.0;
    private boolean pitchResetFlag = false;
    private int pitchResetCounter = 0;

    public GroundIntakeSubsystem() {
        initialize();
    }

    @Override
    public void periodic() {
        if (!pitchResetFlag) {
            groundIntakeReset();
            intakePitch.control();
        } else if (normTargetStatus != previousStatus) {
            intakePitch.setNormalizedMotionPosition(normTargetStatus);
            intakePitch.control();
            previousStatus = normTargetStatus;
        }

        SmartDashboard.putBoolean("GI_pitch_reset", getPitchResetFlag());
    }

    private void initialize() {
        TalonFXConfiguration rollerRightConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.CounterClockwise_Positive))
                .withSlot0(new Slot0Configs()
                        .withKG(0.0)
                        .withKS(9.0)
                        .withKV(0.0)
                        .withKA(0.0)
                        .withKP(1.2)
                        .withKI(0.0)
                        .withKD(0.0))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(40)
                        .withSupplyCurrentLimitEnable(true)
                        .withSupplyCurrentLowerLimit(40));
        applyWithRetry(intakeRollerRight, rollerRightConfig);
        intakeRollerRight.setInvert(-1);

        intakeRollerLeft.setFollower(intakeRollerRight.getData().deviceId, true);
        intakeRollerLeft.control();

        TalonFXConfiguration pitchConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.CounterClockwise_Positive))
                .withSlot0(new Slot0Configs()
                        .withKG(0.0)
                        .withKS(0.0)
                        .withKV(0.0)
                        .withKA(0.0)
                        .withKP(2.0)
                        .withKI(0.0)
                        .withKD(0.0))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(0.0)
                        .withMotionMagicExpo_kV(0.1)
                        .withMotionMagicExpo_kA(0.20))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(20)
                        .withSupplyCurrentLimitEnable(true));
        applyWithRetry(intakePitch, pitchConfig);
        intakePitch.setGearRatio(18.67);
        intakePitch.setInvert(1);
        intakePitch.setPhysicalLimits(
                0.0,
                Constants.GroundIntakeConstants.intakePitchMotorMaxPositionRot / 18.67,
                120.0 / 18.67,
                40.0);
        intakePitch.setCurrentSpeed(0.07);
    }

    public void setRollerVelocity(double velocityRps) {
        intakeRollerRight.setVelocityTorqueCurrent(velocityRps);
        intakeRollerRight.control();
    }

    public Command setRollerVelocityCommand(double velocityRps) {
        return runOnce(() -> setRollerVelocity(velocityRps));
    }

    public void setRollerDutyCycle(double dutyCycle) {
        intakeRollerRight.setNormalizedDutyCycle(dutyCycle);
        intakeRollerRight.control();
    }

    public Command setRollerDutyCycleCommand(double dutyCycle) {
        return runOnce(() -> setRollerDutyCycle(dutyCycle));
    }

    public void stop() {
        intakeRollerRight.setCoast();
        intakeRollerRight.control();
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }

    public void setTeleopRollerCurrentLimit() {
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(40)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLowerLimit(40);
        for (int i = 0; i < 5; i++) {
            StatusCode status = intakeRollerRight.getMotor().getConfigurator().apply(currentLimits);
            if (status.isOK()) {
                return;
            }
        }
    }

    public void setPitchNormPosition(double norm) {
        normTargetStatus = norm;
    }

    public Command setPitchNormPositionCommand(double norm) {
        return runOnce(() -> setPitchNormPosition(norm));
    }

    public double getPitchCurrent() {
        return intakePitch.getCurrent();
    }

    public double getPitchNormPosition() {
        return intakePitch.getNormalizedPosition();
    }

    public boolean getPitchResetFlag() {
        return pitchResetFlag;
    }

    private void groundIntakeReset() {
        if (!DriverStation.isEnabled()) {
            pitchResetCounter = 0;
            return;
        }

        intakePitch.setCurrent(-60.0);

        if (intakePitch.getCurrent() < -20.0) {
            pitchResetCounter++;
        }

        if (pitchResetCounter >= 1) {
            intakePitch.reset(intakePitch.getAbsPosition());
            pitchResetFlag = true;
        }
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
