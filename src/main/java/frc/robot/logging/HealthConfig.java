// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.logging;

import edu.wpi.first.wpilibj.PowerDistribution;

/**
 * Immutable runtime settings for {@link RobotHealthLogger}.
 *
 * <p>The defaults deliberately do not construct a PDH/PDP and do not record
 * NetworkTables. A power-distribution module must be explicitly identified
 * before it can be enabled.
 */
public final class HealthConfig {
    private final double robotSampleHz;
    private final double canSampleHz;
    private final double subsystemSampleHz;
    private final double pdhSampleHz;
    private final double motorFastSampleHz;
    private final double motorCurrentSampleHz;
    private final double motorSlowSampleHz;
    private final double flushPeriodSeconds;
    private final double connectionTimeoutSeconds;
    private final boolean logNetworkTables;
    private final boolean logJoysticks;
    private final boolean startSignalLogger;
    private final boolean configurePhoenixSignalFrequencies;
    private final boolean stopDataLogOnClose;
    private final boolean pdhEnabled;
    private final int pdhModule;
    private final PowerDistribution.ModuleType pdhType;
    private final PdhChannelMap pdhChannelMap;

    private HealthConfig(Builder builder) {
        robotSampleHz = positive(builder.robotSampleHz, "robotSampleHz");
        canSampleHz = positive(builder.canSampleHz, "canSampleHz");
        subsystemSampleHz = positive(builder.subsystemSampleHz, "subsystemSampleHz");
        pdhSampleHz = positive(builder.pdhSampleHz, "pdhSampleHz");
        motorFastSampleHz = positive(builder.motorFastSampleHz, "motorFastSampleHz");
        motorCurrentSampleHz = positive(builder.motorCurrentSampleHz, "motorCurrentSampleHz");
        motorSlowSampleHz = positive(builder.motorSlowSampleHz, "motorSlowSampleHz");
        flushPeriodSeconds = positive(builder.flushPeriodSeconds, "flushPeriodSeconds");
        connectionTimeoutSeconds =
                positive(builder.connectionTimeoutSeconds, "connectionTimeoutSeconds");
        logNetworkTables = builder.logNetworkTables;
        logJoysticks = builder.logJoysticks;
        startSignalLogger = builder.startSignalLogger;
        configurePhoenixSignalFrequencies = builder.configurePhoenixSignalFrequencies;
        stopDataLogOnClose = builder.stopDataLogOnClose;
        pdhEnabled = builder.pdhEnabled;
        pdhModule = builder.pdhModule;
        pdhType = builder.pdhType;
        pdhChannelMap = builder.pdhChannelMap == null
                ? PdhChannelMap.unknown(24)
                : builder.pdhChannelMap;

        if (pdhEnabled && (pdhModule < 0 || pdhType == null)) {
            throw new IllegalArgumentException(
                    "PDH logging requires an explicit non-negative module ID and module type");
        }
    }

    public static HealthConfig defaults() {
        return builder().build();
    }

    public static Builder builder() {
        return new Builder();
    }

    public double robotSampleHz() {
        return robotSampleHz;
    }

    public double canSampleHz() {
        return canSampleHz;
    }

    public double subsystemSampleHz() {
        return subsystemSampleHz;
    }

    public double pdhSampleHz() {
        return pdhSampleHz;
    }

    public double motorFastSampleHz() {
        return motorFastSampleHz;
    }

    public double motorCurrentSampleHz() {
        return motorCurrentSampleHz;
    }

    public double motorSlowSampleHz() {
        return motorSlowSampleHz;
    }

    public double flushPeriodSeconds() {
        return flushPeriodSeconds;
    }

    public double connectionTimeoutSeconds() {
        return connectionTimeoutSeconds;
    }

    public boolean logNetworkTables() {
        return logNetworkTables;
    }

    public boolean logJoysticks() {
        return logJoysticks;
    }

    public boolean startSignalLogger() {
        return startSignalLogger;
    }

    public boolean configurePhoenixSignalFrequencies() {
        return configurePhoenixSignalFrequencies;
    }

    public boolean stopDataLogOnClose() {
        return stopDataLogOnClose;
    }

    public boolean pdhEnabled() {
        return pdhEnabled;
    }

    public int pdhModule() {
        return pdhModule;
    }

    public PowerDistribution.ModuleType pdhType() {
        return pdhType;
    }

    public PdhChannelMap pdhChannelMap() {
        return pdhChannelMap;
    }

    private static double positive(double value, String name) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException(name + " must be finite and greater than zero");
        }
        return value;
    }

    public static final class Builder {
        private double robotSampleHz = 50.0;
        private double canSampleHz = 10.0;
        private double subsystemSampleHz = 50.0;
        private double pdhSampleHz = 20.0;
        private double motorFastSampleHz = 50.0;
        private double motorCurrentSampleHz = 20.0;
        private double motorSlowSampleHz = 5.0;
        private double flushPeriodSeconds = 1.0;
        private double connectionTimeoutSeconds = 0.5;
        private boolean logNetworkTables = false;
        private boolean logJoysticks = true;
        private boolean startSignalLogger = true;
        private boolean configurePhoenixSignalFrequencies = false;
        private boolean stopDataLogOnClose = false;
        private boolean pdhEnabled = false;
        private int pdhModule = -1;
        private PowerDistribution.ModuleType pdhType;
        private PdhChannelMap pdhChannelMap = PdhChannelMap.unknown(24);

        private Builder() {}

        public Builder robotSampleHz(double value) {
            robotSampleHz = value;
            return this;
        }

        public Builder canSampleHz(double value) {
            canSampleHz = value;
            return this;
        }

        public Builder subsystemSampleHz(double value) {
            subsystemSampleHz = value;
            return this;
        }

        public Builder pdhSampleHz(double value) {
            pdhSampleHz = value;
            return this;
        }

        public Builder motorFastSampleHz(double value) {
            motorFastSampleHz = value;
            return this;
        }

        public Builder motorCurrentSampleHz(double value) {
            motorCurrentSampleHz = value;
            return this;
        }

        public Builder motorSlowSampleHz(double value) {
            motorSlowSampleHz = value;
            return this;
        }

        public Builder flushPeriodSeconds(double value) {
            flushPeriodSeconds = value;
            return this;
        }

        public Builder connectionTimeoutSeconds(double value) {
            connectionTimeoutSeconds = value;
            return this;
        }

        public Builder logNetworkTables(boolean value) {
            logNetworkTables = value;
            return this;
        }

        public Builder logJoysticks(boolean value) {
            logJoysticks = value;
            return this;
        }

        public Builder startSignalLogger(boolean value) {
            startSignalLogger = value;
            return this;
        }

        /**
         * Allows this logger to set Phoenix status-signal update frequencies.
         * This is false by default so registering health logging cannot silently
         * change an existing drivetrain's CAN configuration.
         */
        public Builder configurePhoenixSignalFrequencies(boolean value) {
            configurePhoenixSignalFrequencies = value;
            return this;
        }

        public Builder stopDataLogOnClose(boolean value) {
            stopDataLogOnClose = value;
            return this;
        }

        public Builder powerDistribution(
                int module,
                PowerDistribution.ModuleType type,
                PdhChannelMap channelMap) {
            pdhEnabled = true;
            pdhModule = module;
            pdhType = type;
            pdhChannelMap = channelMap;
            return this;
        }

        public Builder disablePowerDistribution() {
            pdhEnabled = false;
            pdhModule = -1;
            pdhType = null;
            return this;
        }

        public HealthConfig build() {
            return new HealthConfig(this);
        }
    }
}
