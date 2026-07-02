// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase implements AutoCloseable {
    public enum AnimationType {
        NONE,
        BLUE,
        GREEN,
        RED,
        PURPLE,
        COLOR_FLOW,
        FIRE,
        LARSON,
        RAINBOW,
        RGB_FADE,
        SINGLE_FADE,
        STROBE,
        TWINKLE,
        TWINKLE_OFF
    }

    private static final int CANDLE_ID = 26;
    private static final int SLOT_0_START_INDEX = 0;
    private static final int SLOT_0_END_INDEX = 23;
    private static final int SLOT_1_START_INDEX = 38;
    private static final int SLOT_1_END_INDEX = 53;

    private static final RGBWColor RED = RGBWColor.fromHSV(0.0, 1.0, 1.0);
    private static final RGBWColor GREEN = RGBWColor.fromHSV(120.0, 1.0, 1.0);
    private static final RGBWColor BLUE = RGBWColor.fromHSV(240.0, 1.0, 1.0);
    private static final RGBWColor PURPLE = RGBWColor.fromHSV(300.0, 1.0, 1.0);

    private final CANBus canBus = new CANBus("rio");
    private final CANdle candle = new CANdle(CANDLE_ID, canBus);
    private AnimationType animationState = AnimationType.NONE;

    public LEDSubsystem() {
        initializeHardware();
    }

    @Override
    public void periodic() {
        colorForState(animationState).ifPresent(this::setExternalSegments);
    }

    public void setLEDState(AnimationType state) {
        animationState = state == null ? AnimationType.NONE : state;
    }

    public AnimationType getLEDState() {
        return animationState;
    }

    @Override
    public void close() {
        candle.close();
    }

    static Optional<RGBWColor> colorForState(AnimationType state) {
        if (state == null) {
            return Optional.empty();
        }
        return switch (state) {
            case RED -> Optional.of(RED);
            case BLUE -> Optional.of(BLUE);
            case GREEN -> Optional.of(GREEN);
            case PURPLE -> Optional.of(PURPLE);
            default -> Optional.empty();
        };
    }

    private void initializeHardware() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.LED.StripType = StripTypeValue.RGB;
        config.LED.BrightnessScalar = 1.0;
        config.CANdleFeatures.StatusLedWhenActive =
                StatusLedWhenActiveValue.Disabled;
        candle.getConfigurator().apply(config);

        candle.clearAllAnimations();
        candle.setControl(new SolidColor(0, 3).withColor(RED));
        candle.setControl(new SolidColor(4, 7).withColor(RED));
        setExternalSegments(RED);
    }

    private void setExternalSegments(RGBWColor color) {
        candle.setControl(
                new SolidColor(SLOT_0_START_INDEX, SLOT_0_END_INDEX)
                        .withColor(color));
        candle.setControl(
                new SolidColor(SLOT_1_START_INDEX, SLOT_1_END_INDEX)
                        .withColor(color));
    }
}
