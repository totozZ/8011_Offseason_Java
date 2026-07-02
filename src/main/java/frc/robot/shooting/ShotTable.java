// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooting;

import frc.robot.Constants;

public final class ShotTable {
    private static final ShotTableEntry[] HUB_TABLE = {
        entry(1.700, 1855.0, 1425.0, 17.5),
        entry(2.000, 1875.0, 1439.0, 18.5),
        entry(2.215, 1940.0, 1450.0, 19.0),
        entry(2.495, 2015.0, 1500.0, 19.5),
        entry(2.888, 2065.0, 1550.0, 20.5),
        entry(3.222, 2130.0, 1650.0, 21.5),
        entry(3.570, 2200.0, 1750.0, 22.5),
        entry(3.830, 2240.0, 1800.0, 23.0),
        entry(4.150, 2340.0, 1950.0, 24.5),
        entry(4.750, 2365.0, 2150.0, 26.5),
    };

    private static final ShotTableEntry[] PASS_TABLE = {
        entry(4.977, 1800.0, 1650.0, 30.0),
        entry(5.976, 1850.0, 1700.0, 30.0),
        entry(6.902, 1900.0, 1750.0, 30.0),
        entry(7.912, 2050.0, 1850.0, 30.0),
        entry(8.477, 2100.0, 1900.0, 30.0),
        entry(9.029, 2200.0, 2000.0, 30.0),
        entry(9.590, 2400.0, 2200.0, 30.0),
        entry(10.061, 2550.0, 2350.0, 30.0),
        entry(12.460, 3150.0, 1400.0, 30.0),
        entry(13.133, 3500.0, 1300.0, 30.0),
    };

    private ShotTable() {}

    public static ShotSetpoint hub(double distanceMeters) {
        return interpolate(HUB_TABLE, distanceMeters);
    }

    public static ShotSetpoint pass(double distanceMeters) {
        return interpolate(PASS_TABLE, distanceMeters);
    }

    public static ShotSetpoint tower() {
        return hub(3.048);
    }

    public static ShotSetpoint zeroPitchFallback() {
        return new ShotSetpoint(rpm(1815.0), rpm(1400.0), 0.0);
    }

    public static boolean isHubRegion(AllianceSide alliance, double fieldXMeters) {
        return switch (alliance) {
            case BLUE -> fieldXMeters <= Constants.FieldConstants.hubPassBlueBoundaryXMeters;
            case RED -> fieldXMeters >= Constants.FieldConstants.hubPassRedBoundaryXMeters;
            case UNKNOWN -> false;
        };
    }

    public static boolean feedAllowed(boolean pitchHomed, boolean ready, boolean timeoutElapsed) {
        return pitchHomed && (ready || timeoutElapsed);
    }

    public static ShotSetpoint interpolate(ShotTableEntry[] table, double distanceMeters) {
        if (table.length == 0) {
            return new ShotSetpoint(0.0, 0.0, 0.0);
        }
        if (distanceMeters <= table[0].distanceMeters()) {
            return table[0].setpoint();
        }
        if (distanceMeters >= table[table.length - 1].distanceMeters()) {
            return table[table.length - 1].setpoint();
        }

        for (int i = 1; i < table.length; i++) {
            ShotTableEntry upper = table[i];
            if (distanceMeters <= upper.distanceMeters()) {
                ShotTableEntry lower = table[i - 1];
                double ratio = (distanceMeters - lower.distanceMeters())
                        / (upper.distanceMeters() - lower.distanceMeters());
                return lerp(lower.setpoint(), upper.setpoint(), ratio);
            }
        }

        return table[table.length - 1].setpoint();
    }

    private static ShotTableEntry entry(
            double distanceMeters,
            double flywheelRpm,
            double feederRpm,
            double pitchDeg) {
        return new ShotTableEntry(
                distanceMeters,
                new ShotSetpoint(rpm(flywheelRpm), rpm(feederRpm), pitchDeg));
    }

    private static double rpm(double rpm) {
        return rpm / 60.0;
    }

    private static ShotSetpoint lerp(ShotSetpoint lower, ShotSetpoint upper, double ratio) {
        return new ShotSetpoint(
                lerp(lower.flywheelRps(), upper.flywheelRps(), ratio),
                lerp(lower.feederRps(), upper.feederRps(), ratio),
                lerp(lower.pitchDeg(), upper.pitchDeg(), ratio));
    }

    private static double lerp(double lower, double upper, double ratio) {
        return lower + (upper - lower) * ratio;
    }
}
