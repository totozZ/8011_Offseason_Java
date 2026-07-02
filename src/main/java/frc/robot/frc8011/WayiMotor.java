// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.frc8011;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotBase;

public class WayiMotor {
    public static final class ConfigData {
        public final int deviceId;
        public final CANBus canBus;

        public double targetPosition;
        public double targetVelocity;
        public double targetCurrent;
        public double normalizedPosition;
        public double normalizedVelocity;
        public double normalizedCurrent;
        public double maxPosition = 1.0;
        public double minPosition = 0.0;
        public double maxVelocity = 50.0;
        public double maxCurrent = 40.0;
        public double currentPosition;
        public double currentVelocity;
        public double currentCurrent;
        public double currentNormalizedPosition;
        public double currentNormalizedVelocity;
        public double currentNormalizedCurrent;
        public double offset = 0.0;
        public double gearRatio = 1.0;
        public int mode = 0;
        public double invert = 1.0;
        public double currentSpeed = 0.5;
        public int followerId = -1;
        public boolean followInvert = false;

        private ConfigData(int deviceId, CANBus canBus) {
            this.deviceId = deviceId;
            this.canBus = canBus;
        }
    }

    private static final int MODE_BRAKE = 0;
    private static final int MODE_VELOCITY = 1;
    private static final int MODE_POSITION = 2;
    private static final int MODE_CURRENT = 3;
    private static final int MODE_MOTION_MAGIC = 4;
    private static final int MODE_MOTION_MAGIC_VELOCITY = 5;
    private static final int MODE_DUTY = 6;
    private static final int MODE_MOTION_MAGIC_POSITION = 7;
    private static final int MODE_FOLLOWER = 8;
    private static final int MODE_VELOCITY_TORQUE_CURRENT = 9;
    private static final int MODE_COAST = 13;
    private static final int MODE_POSITION_TORQUE_CURRENT = 14;
    private static final int MODE_MOTION_MAGIC_TORQUE_CURRENT = 15;

    private final ConfigData data;
    private final TalonFX motor;
    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Current> torqueCurrentSignal;
    private boolean refreshStatusSignalsOnReceive = false;

    private final StaticBrake brake = new StaticBrake();
    private final CoastOut coast = new CoastOut();
    private final VelocityVoltage velocity = new VelocityVoltage(0).withSlot(0);
    private final MotionMagicVelocityTorqueCurrentFOC motionMagicVelocity =
            new MotionMagicVelocityTorqueCurrentFOC(0).withUpdateFreqHz(500).withSlot(0);
    private final VelocityTorqueCurrentFOC velocityTorqueCurrent =
            new VelocityTorqueCurrentFOC(0).withUpdateFreqHz(500).withSlot(0);
    private final PositionVoltage position = new PositionVoltage(0).withSlot(0);
    private final PositionTorqueCurrentFOC positionTorqueCurrent =
            new PositionTorqueCurrentFOC(0).withUpdateFreqHz(500).withSlot(0);
    private final MotionMagicExpoVoltage motionMagicPosition =
            new MotionMagicExpoVoltage(0).withUpdateFreqHz(50).withSlot(0).withEnableFOC(true);
    private final TorqueCurrentFOC torque = new TorqueCurrentFOC(0).withUpdateFreqHz(500);
    private final MotionMagicExpoTorqueCurrentFOC motionMagic =
            new MotionMagicExpoTorqueCurrentFOC(0).withUpdateFreqHz(500).withSlot(0);
    private final MotionMagicTorqueCurrentFOC motionMagicTorqueCurrent =
            new MotionMagicTorqueCurrentFOC(0).withUpdateFreqHz(500).withSlot(0);
    private final DutyCycleOut dutyCycle = new DutyCycleOut(0).withUpdateFreqHz(500);
    private final PositionDutyCycle positionDutyCycle = new PositionDutyCycle(0).withSlot(0);
    private final VoltageOut voltageOut = new VoltageOut(0).withUpdateFreqHz(500);

    public WayiMotor(int deviceId, CANBus canBus) {
        data = new ConfigData(deviceId, canBus);
        motor = new TalonFX(deviceId, canBus);
        positionSignal = motor.getPosition(false);
        velocitySignal = motor.getVelocity(false);
        torqueCurrentSignal = motor.getTorqueCurrent(false);
        BaseStatusSignal.setUpdateFrequencyForAll(20, positionSignal, velocitySignal, torqueCurrentSignal);
    }

    public void control() {
        if (RobotBase.isSimulation()) {
            return;
        }

        switch (data.mode) {
            case MODE_BRAKE -> motor.setControl(brake);
            case MODE_COAST -> motor.setControl(coast);
            case MODE_VELOCITY -> motor.setControl(velocity.withVelocity(toMotorVelocity(data.targetVelocity)));
            case MODE_POSITION -> motor.setControl(position.withPosition(toMotorPosition(data.targetPosition)));
            case MODE_CURRENT -> motor.setControl(
                    torque.withOutput(data.targetCurrent * data.invert)
                            .withMaxAbsDutyCycle(data.currentSpeed));
            case MODE_MOTION_MAGIC -> motor.setControl(
                    motionMagic.withPosition(toMotorPosition(data.targetPosition)));
            case MODE_MOTION_MAGIC_VELOCITY -> motor.setControl(
                    motionMagicVelocity.withVelocity(toMotorVelocity(data.targetVelocity)));
            case MODE_DUTY -> motor.setControl(
                    dutyCycle.withOutput((data.targetVelocity / data.maxVelocity) * data.invert));
            case MODE_MOTION_MAGIC_POSITION -> motor.setControl(
                    motionMagicPosition.withPosition(toMotorPosition(data.targetPosition)));
            case MODE_FOLLOWER -> motor.setControl(new Follower(
                    data.followerId,
                    data.followInvert ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
            case MODE_VELOCITY_TORQUE_CURRENT -> motor.setControl(
                    velocityTorqueCurrent.withVelocity(toMotorVelocity(data.targetVelocity)));
            case MODE_POSITION_TORQUE_CURRENT -> motor.setControl(
                    positionTorqueCurrent.withPosition(toMotorPosition(data.targetPosition)));
            case MODE_MOTION_MAGIC_TORQUE_CURRENT -> motor.setControl(
                    motionMagicTorqueCurrent.withPosition(toMotorPosition(data.targetPosition)));
            default -> {
            }
        }
    }

    public void receive() {
        if (RobotBase.isSimulation()) {
            return;
        }

        receivePosition();
        receiveVelocity();
        receiveCurrent();

        double positionRange = data.maxPosition - data.minPosition;
        data.currentNormalizedPosition =
                Math.abs(positionRange) > 1e-9
                        ? (data.currentPosition - data.minPosition) / positionRange
                        : 0.0;
        data.currentNormalizedVelocity =
                Math.abs(data.maxVelocity) > 1e-9 ? data.currentVelocity / data.maxVelocity : 0.0;
        data.currentNormalizedCurrent =
                Math.abs(data.maxCurrent) > 1e-9 ? data.currentCurrent / data.maxCurrent : 0.0;
    }

    public void receivePosition() {
        if (RobotBase.isSimulation()) {
            return;
        }
        if (refreshStatusSignalsOnReceive) {
            BaseStatusSignal.refreshAll(positionSignal);
        }
        data.currentPosition = fromMotorPosition(positionSignal.getValueAsDouble());
    }

    public void receiveVelocity() {
        if (RobotBase.isSimulation()) {
            return;
        }
        if (refreshStatusSignalsOnReceive) {
            BaseStatusSignal.refreshAll(velocitySignal);
        }
        data.currentVelocity = fromMotorVelocity(velocitySignal.getValueAsDouble());
    }

    public void receiveCurrent() {
        if (RobotBase.isSimulation()) {
            return;
        }
        if (refreshStatusSignalsOnReceive) {
            BaseStatusSignal.refreshAll(torqueCurrentSignal);
        }
        data.currentCurrent = torqueCurrentSignal.getValueAsDouble() * data.invert;
    }

    public void refreshNow() {
        BaseStatusSignal.refreshAll(positionSignal, velocitySignal, torqueCurrentSignal);
    }

    public StatusCode applyConfig(TalonFXConfiguration config) {
        return motor.getConfigurator().apply(config);
    }

    public void reset(double offset) {
        data.offset = offset;
        setBrake();
    }

    public void setGearRatio(double gearRatio) {
        data.gearRatio = gearRatio;
    }

    public void setInvert(double invert) {
        data.invert = invert;
    }

    public void setBrake() {
        data.mode = MODE_BRAKE;
    }

    public void setCoast() {
        data.mode = MODE_COAST;
    }

    public void setPhysicalLimits(double minPosition, double maxPosition, double maxVelocity, double maxCurrent) {
        data.minPosition = minPosition;
        data.maxPosition = maxPosition;
        data.maxVelocity = maxVelocity;
        data.maxCurrent = maxCurrent;
    }

    public void setCurrentSpeed(double speed) {
        data.currentSpeed = speed;
    }

    public void setStatusSignalUpdateFrequency(double frequencyHz) {
        BaseStatusSignal.setUpdateFrequencyForAll(
                frequencyHz,
                positionSignal,
                velocitySignal,
                torqueCurrentSignal);
    }

    public void setVelocityTorqueCurrent(double velocityRps) {
        data.mode = MODE_VELOCITY_TORQUE_CURRENT;
        data.targetVelocity = velocityRps;
    }

    public void setNormalizedDutyCycle(double duty) {
        data.normalizedVelocity = clamp(duty, -1.0, 1.0);
        data.mode = MODE_DUTY;
        data.targetVelocity = data.normalizedVelocity * data.maxVelocity;
    }

    public void setNormalizedMotionPosition(double normalizedPosition) {
        data.normalizedPosition = clamp(normalizedPosition, -1.0, 1.0);
        data.mode = MODE_MOTION_MAGIC_POSITION;
        data.targetPosition = data.normalizedPosition * (data.maxPosition - data.minPosition)
                + data.minPosition;
    }

    public void setCurrent(double currentAmps) {
        data.mode = MODE_CURRENT;
        data.targetCurrent = currentAmps;
    }

    public void setVoltage(double volts) {
        if (!RobotBase.isSimulation()) {
            motor.setControl(voltageOut.withOutput(volts));
        }
    }

    public void setFollower(int leaderId, boolean invert) {
        data.followerId = leaderId;
        data.followInvert = invert;
        data.mode = MODE_FOLLOWER;
    }

    public ConfigData getData() {
        return data;
    }

    public TalonFX getMotor() {
        return motor;
    }

    public double getPosition() {
        return fromMotorPosition(motor.getPosition().getValueAsDouble());
    }

    public double getVelocity() {
        return fromMotorVelocity(motor.getVelocity().getValueAsDouble());
    }

    public double getCurrent() {
        return motor.getTorqueCurrent().getValueAsDouble() * data.invert;
    }

    public double getNormalizedPosition() {
        double positionRange = data.maxPosition - data.minPosition;
        return Math.abs(positionRange) > 1e-9 ? (getPosition() - data.minPosition) / positionRange : 0.0;
    }

    public double getAbsPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    private double toMotorVelocity(double mechanismVelocity) {
        return mechanismVelocity * data.gearRatio * data.invert;
    }

    private double toMotorPosition(double mechanismPosition) {
        return mechanismPosition * data.gearRatio * data.invert + data.offset;
    }

    private double fromMotorVelocity(double motorVelocity) {
        return motorVelocity / data.gearRatio * data.invert;
    }

    private double fromMotorPosition(double motorPosition) {
        return (motorPosition - data.offset) / data.gearRatio * data.invert;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
