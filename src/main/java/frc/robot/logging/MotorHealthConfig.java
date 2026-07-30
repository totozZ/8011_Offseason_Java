// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.logging;

/** Per-motor selection and relationship metadata for health logging. */
public final class MotorHealthConfig {
    public enum Role {
        INDEPENDENT,
        LEADER,
        FOLLOWER
    }

    private final Role role;
    private final int leaderDeviceId;
    private final boolean followerOpposed;
    private final boolean logMotion;
    private final boolean logElectrical;
    private final boolean logTemperature;
    private final boolean logFaults;
    private final boolean logLimits;

    private MotorHealthConfig(Builder builder) {
        role = builder.role;
        leaderDeviceId = builder.leaderDeviceId;
        followerOpposed = builder.followerOpposed;
        logMotion = builder.logMotion;
        logElectrical = builder.logElectrical;
        logTemperature = builder.logTemperature;
        logFaults = builder.logFaults;
        logLimits = builder.logLimits;
        if (role == Role.FOLLOWER && leaderDeviceId < 0) {
            throw new IllegalArgumentException("A follower requires a non-negative leader device ID");
        }
    }

    public static MotorHealthConfig defaults() {
        return builder().build();
    }

    public static Builder builder() {
        return new Builder();
    }

    public static MotorHealthConfig leader() {
        return builder().role(Role.LEADER).build();
    }

    public static MotorHealthConfig follower(int leaderDeviceId, boolean opposed) {
        return builder().follower(leaderDeviceId, opposed).build();
    }

    public Role role() {
        return role;
    }

    public int leaderDeviceId() {
        return leaderDeviceId;
    }

    public boolean followerOpposed() {
        return followerOpposed;
    }

    public boolean logMotion() {
        return logMotion;
    }

    public boolean logElectrical() {
        return logElectrical;
    }

    public boolean logTemperature() {
        return logTemperature;
    }

    public boolean logFaults() {
        return logFaults;
    }

    public boolean logLimits() {
        return logLimits;
    }

    public static final class Builder {
        private Role role = Role.INDEPENDENT;
        private int leaderDeviceId = -1;
        private boolean followerOpposed = false;
        private boolean logMotion = true;
        private boolean logElectrical = true;
        private boolean logTemperature = true;
        private boolean logFaults = true;
        private boolean logLimits = true;

        private Builder() {}

        public Builder role(Role value) {
            role = value == null ? Role.INDEPENDENT : value;
            if (role != Role.FOLLOWER) {
                leaderDeviceId = -1;
                followerOpposed = false;
            }
            return this;
        }

        public Builder follower(int leaderId, boolean opposed) {
            role = Role.FOLLOWER;
            leaderDeviceId = leaderId;
            followerOpposed = opposed;
            return this;
        }

        public Builder logMotion(boolean value) {
            logMotion = value;
            return this;
        }

        public Builder logElectrical(boolean value) {
            logElectrical = value;
            return this;
        }

        public Builder logTemperature(boolean value) {
            logTemperature = value;
            return this;
        }

        public Builder logFaults(boolean value) {
            logFaults = value;
            return this;
        }

        public Builder logLimits(boolean value) {
            logLimits = value;
            return this;
        }

        public MotorHealthConfig build() {
            return new MotorHealthConfig(this);
        }
    }
}
