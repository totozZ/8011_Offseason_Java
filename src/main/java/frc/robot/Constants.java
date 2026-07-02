// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
    public static final class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final double speedRate = 0.75;
        public static final double angularSpeedRate = 0.95;

        private OperatorConstants() {}
    }

    public static final class FieldConstants {
        public static final double fieldLengthMeters = 16.54;
        public static final double fieldWidthMeters = 8.07;
        public static final double hubPassBlueBoundaryXMeters = 5.0;
        public static final double hubPassRedBoundaryXMeters =
                fieldLengthMeters - hubPassBlueBoundaryXMeters;

        private FieldConstants() {}
    }

    private Constants() {}
}
