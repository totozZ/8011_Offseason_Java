// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Objects;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

/** Minimal CANdle status subsystem. A latched fault overrides every normal state. */
public class LEDSubsystem extends SubsystemBase implements AutoCloseable {
    public enum State {
        OFF,
        DISABLED,
        TELEOP,
        AUTONOMOUS,
        TEST,
        FAULT
    }

    private static final int BUILT_IN_START_INDEX = 0;
    private static final int BUILT_IN_END_INDEX = 7;
    private static final int LEFT_STRIP_START_INDEX = 0;
    private static final int LEFT_STRIP_END_INDEX = 23;
    private static final int RIGHT_STRIP_START_INDEX = 38;
    private static final int RIGHT_STRIP_END_INDEX = 53;

    private static final RGBWColor OFF = new RGBWColor(0, 0, 0);
    private static final RGBWColor BLUE = new RGBWColor(0, 0, 255);
    private static final RGBWColor GREEN = new RGBWColor(0, 255, 0);
    private static final RGBWColor PURPLE = new RGBWColor(255, 0, 255);
    private static final RGBWColor YELLOW = new RGBWColor(255, 255, 0);
    private static final RGBWColor RED = new RGBWColor(255, 0, 0);

    private final CANdle candle = new CANdle(
            Constants.CanConstants.CANDLE_ID,
            Constants.CanConstants.RIO_CAN_BUS);
    private final StringPublisher statePublisher = NetworkTableInstance.getDefault()
            .getTable("FRC8011")
            .getSubTable("LED")
            .getStringTopic("State")
            .publish();

    private State requestedState = State.DISABLED;
    private boolean faultActive;
    private RGBWColor solidOverride;
    private RGBWColor lastAppliedColor;
    private String lastPublishedState = "";

    public LEDSubsystem() {
        CANdleConfiguration configuration = new CANdleConfiguration();
        configuration.LED.StripType = StripTypeValue.RGB;
        configuration.LED.BrightnessScalar = 1.0;
        configuration.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
        candle.getConfigurator().apply(configuration);
        candle.clearAllAnimations();
    }

    @Override
    public void periodic() {
        State effectiveState = resolveState(requestedState, faultActive);
        RGBWColor color = effectiveState == State.FAULT
                ? colorForState(State.FAULT)
                : solidOverride != null ? solidOverride : colorForState(effectiveState);
        String publishedState = solidOverride != null && effectiveState != State.FAULT
                ? "SOLID"
                : effectiveState.name();

        if (!Objects.equals(color, lastAppliedColor)) {
            applySolid(color);
            lastAppliedColor = color;
        }
        if (!publishedState.equals(lastPublishedState)) {
            statePublisher.set(publishedState);
            lastPublishedState = publishedState;
        }
    }

    public void setState(State state) {
        State nonNullState = Objects.requireNonNull(state, "LED state cannot be null");
        solidOverride = null;
        if (nonNullState == State.FAULT) {
            faultActive = true;
        } else {
            requestedState = nonNullState;
        }
    }

    /** Sets or clears the fault latch. While true, the LEDs remain red. */
    public void setFault(boolean active) {
        faultActive = active;
    }

    /** Applies a one-off solid color until the next state request. Fault red still wins. */
    public void setSolid(RGBWColor color) {
        solidOverride = Objects.requireNonNull(color, "LED color cannot be null");
    }

    public void off() {
        setState(State.OFF);
    }

    public State getEffectiveState() {
        return resolveState(requestedState, faultActive);
    }

    public Command setStateCommand(State state) {
        return runOnce(() -> setState(state)).withName("LED-" + state.name());
    }

    public Command setSolidCommand(RGBWColor color) {
        return runOnce(() -> setSolid(color)).withName("LED-Solid");
    }

    public Command offCommand() {
        return runOnce(this::off).withName("LED-Off");
    }

    @Override
    public void close() {
        statePublisher.close();
        candle.close();
    }

    static State resolveState(State requestedState, boolean faultActive) {
        return faultActive ? State.FAULT : requestedState;
    }

    static RGBWColor colorForState(State state) {
        return switch (state) {
            case DISABLED -> BLUE;
            case TELEOP -> GREEN;
            case AUTONOMOUS -> PURPLE;
            case TEST -> YELLOW;
            case FAULT -> RED;
            case OFF -> OFF;
        };
    }

    private void applySolid(RGBWColor color) {
        candle.setControl(
                new SolidColor(BUILT_IN_START_INDEX, BUILT_IN_END_INDEX).withColor(color));
        candle.setControl(
                new SolidColor(LEFT_STRIP_START_INDEX, LEFT_STRIP_END_INDEX).withColor(color));
        candle.setControl(
                new SolidColor(RIGHT_STRIP_START_INDEX, RIGHT_STRIP_END_INDEX).withColor(color));
    }
}
