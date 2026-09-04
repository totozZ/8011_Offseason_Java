// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

import edu.wpi.first.units.measure.LinearVelocity;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/** Selectable drivetrain hardware profiles. */
public enum DrivetrainProfile {
    /** The ordinary drivetrain represented by the project's generated TunerConstants file. */
    NORMAL {
        @Override
        public LinearVelocity speedAt12Volts() {
            return TunerConstants.kSpeedAt12Volts;
        }

        @Override
        public CommandSwerveDrivetrain createDrivetrain() {
            return TunerConstants.createDrivetrain();
        }
    },

    /** The SoccerBot drivetrain migrated from C:\Users\95833\Desktop\sum. */
    SOCCER_BOT {
        @Override
        public LinearVelocity speedAt12Volts() {
            return SoccerBotDrivetrainConstants.SPEED_AT_12_VOLTS;
        }

        @Override
        public CommandSwerveDrivetrain createDrivetrain() {
            return SoccerBotDrivetrainConstants.createDrivetrain();
        }
    };

    /** Returns this profile's measured free speed at 12 V. */
    public abstract LinearVelocity speedAt12Volts();

    /** Creates the one drivetrain instance used by RobotContainer. */
    public abstract CommandSwerveDrivetrain createDrivetrain();
}
