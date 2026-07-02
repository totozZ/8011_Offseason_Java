// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Constants {
    public static final class CanConstants {
        public static final CANBus rioCanBus = new CANBus("rio");

        private CanConstants() {}
    }

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

    public static final class FeederConstants {
        public static final int backwardFeederMotorId = 17;
        public static final int upwardFeederMotorId = 18;

        private FeederConstants() {}
    }

    public static final class ShooterConstants {
        public static final int shooterLeftDownMotorId = 12;
        public static final int shooterLeftUpMotorId = 13;
        public static final int shooterRightUpMotorId = 14;
        public static final int shooterRightDownMotorId = 15;
        public static final int shooterPitchMotorId = 16;

        public static final double maxPitchAngleDeg = 34.65;
        public static final double minPitchAngleDeg = 0.0;
        public static final double pitchMotorMaxPositionRot = 12.005;

        private ShooterConstants() {}
    }

    public static final class GroundIntakeConstants {
        public static final int intakeRollerLeftMotorId = 19;
        public static final int intakeRollerRightMotorId = 20;
        public static final int intakePivotMotorId = 21;

        public static final double assistPitchCurrentThresholdAmps = 17.0;
        public static final double assistCooldownSeconds = 1.2;
        public static final double intakePitchMotorMaxPositionRot = 8.2;
        public static final double pitchNormPosition = 0.923;

        private GroundIntakeConstants() {}
    }

    private Constants() {}
}
