// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Effective SoccerBot drivetrain configuration migrated from the {@code sum} C++ project.
 *
 * <p>This is deliberately outside the generated package. Replacing the normal drivetrain's
 * TunerConstants file therefore cannot overwrite the SoccerBot profile.
 */
final class SoccerBotDrivetrainConstants {
    private static final Slot0Configs STEER_GAINS = new Slot0Configs()
            .withKP(100.0)
            .withKI(0.0)
            .withKD(0.5)
            .withKS(0.1)
            .withKV(1.91)
            .withKA(0.0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    private static final Slot0Configs DRIVE_GAINS = new Slot0Configs()
            .withKP(0.1)
            .withKI(0.0)
            .withKD(0.0)
            .withKS(0.0)
            .withKV(0.124)
            .withKA(0.0);

    private static final TalonFXConfiguration DRIVE_INITIAL_CONFIGS =
            new TalonFXConfiguration().withCurrentLimits(
                    new CurrentLimitsConfigs()
                            .withSupplyCurrentLimit(Amps.of(70.0))
                            .withSupplyCurrentLimitEnable(true));
    private static final TalonFXConfiguration STEER_INITIAL_CONFIGS =
            new TalonFXConfiguration().withCurrentLimits(
                    new CurrentLimitsConfigs()
                            .withStatorCurrentLimit(Amps.of(60.0))
                            .withStatorCurrentLimitEnable(true));
    private static final CANcoderConfiguration ENCODER_INITIAL_CONFIGS =
            new CANcoderConfiguration();

    static final LinearVelocity SPEED_AT_12_VOLTS = MetersPerSecond.of(4.72);
    static final SwerveDrivetrainConstants DRIVETRAIN_CONSTANTS =
            new SwerveDrivetrainConstants()
                    .withCANBusName("rio")
                    .withPigeon2Id(13);

    private static final SwerveModuleConstantsFactory<
            TalonFXConfiguration,
            TalonFXConfiguration,
            CANcoderConfiguration> CONSTANT_CREATOR =
                    new SwerveModuleConstantsFactory<
                            TalonFXConfiguration,
                            TalonFXConfiguration,
                            CANcoderConfiguration>()
                            .withDriveMotorGearRatio(6.538461538461539)
                            .withSteerMotorGearRatio(15.42857142857143)
                            .withCouplingGearRatio(2.8333333333333335)
                            .withWheelRadius(Inches.of(2.0))
                            .withSteerMotorGains(STEER_GAINS)
                            .withDriveMotorGains(DRIVE_GAINS)
                            .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                            .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                            .withSlipCurrent(Amps.of(120.0))
                            .withSpeedAt12Volts(SPEED_AT_12_VOLTS)
                            .withDriveMotorType(DriveMotorArrangement.TalonFX_Integrated)
                            .withSteerMotorType(SteerMotorArrangement.TalonFX_Integrated)
                            .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
                            .withDriveMotorInitialConfigs(DRIVE_INITIAL_CONFIGS)
                            .withSteerMotorInitialConfigs(STEER_INITIAL_CONFIGS)
                            .withEncoderInitialConfigs(ENCODER_INITIAL_CONFIGS)
                            .withSteerInertia(KilogramSquareMeters.of(0.01))
                            .withDriveInertia(KilogramSquareMeters.of(0.035))
                            .withSteerFrictionVoltage(Volts.of(0.2))
                            .withDriveFrictionVoltage(Volts.of(0.2));

    // The front-left drive motor is the one mechanically installed in the opposite direction.
    static final SwerveModuleConstants<
            TalonFXConfiguration,
            TalonFXConfiguration,
            CANcoderConfiguration> FRONT_LEFT = createModule(
                    1, 2, 3, Rotations.of(0.037353515625),
                    Inches.of(10.25), Inches.of(16.5), true);
    static final SwerveModuleConstants<
            TalonFXConfiguration,
            TalonFXConfiguration,
            CANcoderConfiguration> FRONT_RIGHT = createModule(
                    4, 5, 6, Rotations.of(0.204345703125),
                    Inches.of(10.25), Inches.of(-16.5), true);
    static final SwerveModuleConstants<
            TalonFXConfiguration,
            TalonFXConfiguration,
            CANcoderConfiguration> BACK_LEFT = createModule(
                    11, 10, 12, Rotations.of(0.25830078125),
                    Inches.of(-10.25), Inches.of(16.5), false);
    static final SwerveModuleConstants<
            TalonFXConfiguration,
            TalonFXConfiguration,
            CANcoderConfiguration> BACK_RIGHT = createModule(
                    8, 7, 9, Rotations.of(0.331787109375),
                    Inches.of(-10.25), Inches.of(-16.5), true);

    private SoccerBotDrivetrainConstants() {}

    static CommandSwerveDrivetrain createDrivetrain() {
        return new CommandSwerveDrivetrain(
                DRIVETRAIN_CONSTANTS, FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT);
    }

    private static SwerveModuleConstants<
            TalonFXConfiguration,
            TalonFXConfiguration,
            CANcoderConfiguration> createModule(
                    int driveMotorId,
                    int steerMotorId,
                    int encoderId,
                    Angle encoderOffset,
                    Distance locationX,
                    Distance locationY,
                    boolean driveMotorInverted) {
        return CONSTANT_CREATOR.createModuleConstants(
                steerMotorId,
                driveMotorId,
                encoderId,
                encoderOffset,
                locationX,
                locationY,
                driveMotorInverted,
                true,
                false);
    }
}
