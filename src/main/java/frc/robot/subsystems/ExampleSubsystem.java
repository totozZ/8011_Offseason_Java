// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Copyable Phoenix 6 TalonFX teaching example.
 *
 * <p>Every numeric configuration value below is a placeholder. Recalculate it for the real
 * mechanism, units, reduction, motor, breaker, and failure modes before enabling this subsystem.
 */
public class ExampleSubsystem extends SubsystemBase implements AutoCloseable {
    private final TalonFX motor;

    // Reuse control request objects. Do not allocate requests in periodic().
    private final NeutralOut neutralRequest = new NeutralOut();
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0);
    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0).withSlot(0);
    private final PositionVoltage positionRequest = new PositionVoltage(0.0).withSlot(0);
    private final MotionMagicVoltage motionMagicRequest =
            new MotionMagicVoltage(0.0).withSlot(0);
    private final TorqueCurrentFOC torqueCurrentRequest =
            new TorqueCurrentFOC(0.0).withMaxAbsDutyCycle(0.5);
    private final Follower followerRequest =
            new Follower(0, MotorAlignmentValue.Aligned);

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Current> supplyCurrentSignal;
    private final StatusSignal<Current> statorCurrentSignal;

    private final DoublePublisher positionPublisher;
    private final DoublePublisher velocityPublisher;
    private final DoublePublisher supplyCurrentPublisher;
    private final DoublePublisher statorCurrentPublisher;
    private final StringPublisher controlModePublisher;
    private final BooleanEntry liveTuningEnabledEntry;
    private final DoubleEntry tuningVelocityTargetEntry;
    private String controlMode = "Neutral";

    public ExampleSubsystem(int motorCanId, String canBusName) {
        motor = new TalonFX(motorCanId, new CANBus(canBusName));
        applyConfiguration(createMotorConfiguration());

        positionSignal = motor.getPosition(false);
        velocitySignal = motor.getVelocity(false);
        supplyCurrentSignal = motor.getSupplyCurrent(false);
        statorCurrentSignal = motor.getStatorCurrent(false);
        BaseStatusSignal.setUpdateFrequencyForAll(
                20.0,
                positionSignal,
                velocitySignal,
                supplyCurrentSignal,
                statorCurrentSignal);

        NetworkTable exampleTable = NetworkTableInstance.getDefault()
                .getTable("FRC8011")
                .getSubTable("Example");
        positionPublisher = exampleTable.getDoubleTopic("PositionRot").publish();
        velocityPublisher = exampleTable.getDoubleTopic("VelocityRps").publish();
        supplyCurrentPublisher = exampleTable.getDoubleTopic("SupplyCurrentA").publish();
        statorCurrentPublisher = exampleTable.getDoubleTopic("StatorCurrentA").publish();
        controlModePublisher = exampleTable.getStringTopic("ControlMode").publish();

        NetworkTable tuningTable = NetworkTableInstance.getDefault()
                .getTable("FRC8011")
                .getSubTable("Tuning")
                .getSubTable("Example");
        liveTuningEnabledEntry = tuningTable
                .getBooleanTopic("EnableVelocityControl")
                .getEntry(false);
        tuningVelocityTargetEntry = tuningTable
                .getDoubleTopic("VelocityTargetRps")
                .getEntry(0.0);
        liveTuningEnabledEntry.setDefault(false);
        tuningVelocityTargetEntry.setDefault(0.0);
    }

    @Override
    public void periodic() {
        // Deliberately gated. Turning this NT value on can move the mechanism immediately.
        if (liveTuningEnabledEntry.get(false)) {
            setVelocityRotationsPerSecond(tuningVelocityTargetEntry.get(0.0));
        }

        BaseStatusSignal.refreshAll(
                positionSignal,
                velocitySignal,
                supplyCurrentSignal,
                statorCurrentSignal);
        positionPublisher.set(positionSignal.getValueAsDouble());
        velocityPublisher.set(velocitySignal.getValueAsDouble());
        supplyCurrentPublisher.set(supplyCurrentSignal.getValueAsDouble());
        statorCurrentPublisher.set(statorCurrentSignal.getValueAsDouble());
        controlModePublisher.set(controlMode);
    }

    /** Sends a neutral request. Use this in command end() methods. */
    public void stop() {
        motor.setControl(neutralRequest);
        controlMode = "Neutral";
    }

    /** Changes future neutral output to electrically resist motion. */
    public void setBrakeMode() {
        reportStatus("Brake mode", motor.setNeutralMode(NeutralModeValue.Brake));
    }

    /** Changes future neutral output to allow the mechanism to coast. */
    public void setCoastMode() {
        reportStatus("Coast mode", motor.setNeutralMode(NeutralModeValue.Coast));
    }

    /** Open-loop duty cycle in the range -1 to +1. */
    public void setDutyCycle(double dutyCycle) {
        motor.setControl(dutyCycleRequest.withOutput(dutyCycle));
        controlMode = "DutyCycleOut";
    }

    /** Open-loop motor voltage in volts. */
    public void setVoltage(double volts) {
        motor.setControl(voltageRequest.withOutput(volts));
        controlMode = "VoltageOut";
    }

    /** Closed-loop mechanism velocity in rotations per second. */
    public void setVelocityRotationsPerSecond(double rotationsPerSecond) {
        motor.setControl(velocityRequest.withVelocity(rotationsPerSecond));
        controlMode = "VelocityVoltage";
    }

    /** Closed-loop mechanism position in rotations. */
    public void setPositionRotations(double rotations) {
        motor.setControl(positionRequest.withPosition(rotations));
        controlMode = "PositionVoltage";
    }

    /** Motion Magic mechanism target in rotations. */
    public void setMotionMagicPositionRotations(double rotations) {
        motor.setControl(motionMagicRequest.withPosition(rotations));
        controlMode = "MotionMagicVoltage";
    }

    /** Torque-current request in stator amps. FOC requires a supported/licensed device. */
    public void setTorqueCurrentAmps(double amps) {
        motor.setControl(torqueCurrentRequest.withOutput(amps));
        controlMode = "TorqueCurrentFOC";
    }

    /** Follows a leader on the same CAN bus. */
    public void follow(int leaderCanId, boolean opposeLeader) {
        motor.setControl(followerRequest
                .withLeaderID(leaderCanId)
                .withMotorAlignment(opposeLeader
                        ? MotorAlignmentValue.Opposed
                        : MotorAlignmentValue.Aligned));
        controlMode = "Follower";
    }

    @Override
    public void close() {
        stop();
        positionPublisher.close();
        velocityPublisher.close();
        supplyCurrentPublisher.close();
        statorCurrentPublisher.close();
        controlModePublisher.close();
        liveTuningEnabledEntry.close();
        tuningVelocityTargetEntry.close();
        motor.close();
    }

    static TalonFXConfiguration createMotorConfiguration() {
        return new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.CounterClockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake)
                        // Placeholder output caps. Re-evaluate against the mechanism.
                        .withPeakForwardDutyCycle(0.50)
                        .withPeakReverseDutyCycle(-0.50))
                .withSlot0(new Slot0Configs()
                        // Placeholder volts-based PID/feedforward gains for mechanism units.
                        .withKP(1.0)
                        .withKI(0.0)
                        .withKD(0.0)
                        .withKS(0.20)
                        .withKV(0.12)
                        .withKA(0.01)
                        .withKG(0.0))
                .withMotionMagic(new MotionMagicConfigs()
                        // Mechanism rotations/s, rotations/s^2, and rotations/s^3.
                        .withMotionMagicCruiseVelocity(10.0)
                        .withMotionMagicAcceleration(20.0)
                        .withMotionMagicJerk(200.0))
                .withFeedback(new FeedbackConfigs()
                        // Placeholder rotor rotations per mechanism rotation.
                        .withSensorToMechanismRatio(10.0))
                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                        // Mechanism rotations after SensorToMechanismRatio is applied.
                        .withForwardSoftLimitThreshold(5.0)
                        .withReverseSoftLimitThreshold(-5.0)
                        .withForwardSoftLimitEnable(true)
                        .withReverseSoftLimitEnable(true))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        // Placeholder breaker/battery-side and motor-phase limits in amps.
                        .withSupplyCurrentLimit(30.0)
                        .withSupplyCurrentLimitEnable(true)
                        .withStatorCurrentLimit(60.0)
                        .withStatorCurrentLimitEnable(true))
                .withTorqueCurrent(new TorqueCurrentConfigs()
                        // Placeholder TorqueCurrentFOC stator-current peaks in amps.
                        .withPeakForwardTorqueCurrent(40.0)
                        .withPeakReverseTorqueCurrent(-40.0));
    }

    private void applyConfiguration(TalonFXConfiguration configuration) {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int attempt = 1; attempt <= 3; attempt++) {
            status = motor.getConfigurator().apply(configuration);
            if (status.isOK()) {
                return;
            }
        }
        reportStatus("Configuration", status);
    }

    private static void reportStatus(String action, StatusCode status) {
        if (!status.isOK()) {
            DriverStation.reportError("Example TalonFX " + action + " failed: " + status, false);
        }
    }
}
