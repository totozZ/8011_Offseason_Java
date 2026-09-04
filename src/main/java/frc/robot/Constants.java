// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;

import frc.robot.config.DrivetrainProfile;

/** Project-owned constants. Keep generated drivetrain values in {@code TunerConstants}. */
public final class Constants {
    public static final class CanConstants {
        public static final CANBus RIO_CAN_BUS = new CANBus("rio");
        public static final int REV_PDH_ID = 1;
        public static final int CANDLE_ID = 26;

        private CanConstants() {}
    }

    public static final class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final double DRIVE_SPEED_SCALE = 0.75;
        public static final double TURN_SPEED_SCALE = 0.95;
        public static final double TRANSLATION_DEADBAND = 0.07;
        public static final double ROTATION_DEADBAND = 0.05;

        private OperatorConstants() {}
    }

    public static final class DriveConstants {
        /**
         * The one-line drivetrain switch. Use NORMAL or SOCCER_BOT, then rebuild and deploy.
         *
         * <p>NORMAL intentionally remains selected until the operator explicitly chooses the
         * SoccerBot hardware.
         */

        //两套底盘配置，NORMAL和SOCCER_BOT
        // public static final DrivetrainProfile ACTIVE_PROFILE = DrivetrainProfile.SOCCER_BOT;
        public static final DrivetrainProfile ACTIVE_PROFILE = DrivetrainProfile.NORMAL;

        public static final double MAX_ANGULAR_RATE_RADIANS_PER_SECOND = Math.PI * 1.9;

        private DriveConstants() {}
    }

    /** Teacher-owned limits and official 2026 KitBot roller settings. */
    public static final class BabyAutoConstants {
        public static final double MAX_TRANSLATION_METERS_PER_SECOND = 1.0;
        public static final double MAX_ROTATION_DEGREES_PER_SECOND = 90.0;
        public static final double MAX_AUTO_SECONDS = 15.0;
        public static final double SHOOTER_SPIN_UP_SECONDS = 1.0;

        public static final int INTAKE_LAUNCHER_MOTOR_CAN_ID = 5;
        public static final int FEEDER_MOTOR_CAN_ID = 6;
        public static final int MOTOR_CURRENT_LIMIT_AMPS = 60;

        public static final double INTAKE_FEEDER_VOLTS = -12.0;
        public static final double INTAKE_LAUNCHER_VOLTS = 10.0;
        public static final double SPIN_UP_FEEDER_VOLTS = -6.0;
        public static final double LAUNCH_FEEDER_VOLTS = 9.0;
        public static final double LAUNCH_LAUNCHER_VOLTS = 10.6;

        private BabyAutoConstants() {}
    }

    public static final class VisionConstants {
        public static final String[] LIMELIGHT_NAMES = {
            "limelight-left",
            "limelight-back",
            "limelight-right"
        };

        // Tune these from real logs. They are deliberately centralized for pit review.
        public static final double MAX_ANGULAR_VELOCITY_DEGREES_PER_SECOND = 100.0;
        public static final double MIN_AVERAGE_DISTANCE_METERS = 0.26;
        public static final double MAX_AVERAGE_DISTANCE_METERS = 4.50;
        public static final double XY_STANDARD_DEVIATION_COEFFICIENT = 0.01;
        public static final double DISTANCE_STANDARD_DEVIATION_EXPONENT = 1.2;
        public static final double HEADING_STANDARD_DEVIATION_RADIANS = 10_000_000.0;

        private VisionConstants() {}
    }

    public static final class ExampleConstants {
        /** Keep false until the example CAN ID, bus, limits, ratios, and gains are reviewed. */
        public static final boolean ENABLE_EXAMPLE_SUBSYSTEM = false;

        // Placeholder teaching values. They are not safe defaults for a real mechanism.
        public static final int MOTOR_CAN_ID = 40;
        public static final String MOTOR_CAN_BUS = "rio";

        private ExampleConstants() {}
    }

    public static final class TelemetryConstants {
        public static final String ROOT = "/FRC8011";
        public static final String ROBOT = ROOT + "/Robot";
        public static final String DRIVE = ROOT + "/Drive";
        public static final String VISION = ROOT + "/Vision";
        public static final String LED = ROOT + "/LED";
        public static final String EXAMPLE = ROOT + "/Example";
        public static final String EXAMPLE_TUNING = ROOT + "/Tuning/Example";
        public static final String AUTO_CHOOSER_KEY = "FRC8011/Auto/Chooser";

        private TelemetryConstants() {}
    }

    private Constants() {}
}
