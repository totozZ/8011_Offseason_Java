// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.frc8011.WayiMotor;
import frc.robot.logging.MotorHealthConfig;
import frc.robot.logging.RobotHealthLogger;
import frc.robot.shooting.ShotSetpoint;

public class ShooterSubsystem extends SubsystemBase {
    public enum PitchHomeState {
        UNHOMED,
        HOMING,
        HOMED,
        FAULT
    }

    private static final double IDLE_SPEED_RPS = 0.0;
    private static final double PITCH_HOME_TIMEOUT_SECONDS = 2.0;
    private static final double PITCH_HOME_CURRENT_AMPS = -10.0;
    private static final double PITCH_HOME_CURRENT_THRESHOLD_AMPS = -8.0;
    private static final int PITCH_HOME_CURRENT_COUNT = 3;
    private static final double PITCH_GEAR_RATIO = 18.67;

    private final WayiMotor shooterLeftDown = new WayiMotor(
            Constants.ShooterConstants.shooterLeftDownMotorId,
            Constants.CanConstants.rioCanBus);
    private final WayiMotor shooterLeftUp = new WayiMotor(
            Constants.ShooterConstants.shooterLeftUpMotorId,
            Constants.CanConstants.rioCanBus);
    private final WayiMotor shooterRightUp = new WayiMotor(
            Constants.ShooterConstants.shooterRightUpMotorId,
            Constants.CanConstants.rioCanBus);
    private final WayiMotor shooterRightDown = new WayiMotor(
            Constants.ShooterConstants.shooterRightDownMotorId,
            Constants.CanConstants.rioCanBus);
    private final WayiMotor shooterPitch = new WayiMotor(
            Constants.ShooterConstants.shooterPitchMotorId,
            Constants.CanConstants.rioCanBus);

    private ShotSetpoint targetSetpoint = new ShotSetpoint(15.0, 0.0, 0.2);
    private boolean shotActive = false;
    private boolean flywheelFollowersArmed = false;
    private PitchHomeState pitchHomeState = PitchHomeState.UNHOMED;
    private int pitchHomeCurrentCounter = 0;
    private final Timer pitchHomeTimer = new Timer();
    private double healthFlywheelVelocity;
    private double healthPitchAngle;

    public ShooterSubsystem() {
        initialize();
    }

    @Override
    public void periodic() {
        shooterRightUp.receiveVelocity();
        healthFlywheelVelocity = shooterRightUp.getCachedVelocity();

        if (!DriverStation.isEnabled()) {
            flywheelFollowersArmed = false;
            shooterRightUp.setCoast();
            shooterRightUp.control();
            shooterPitch.setBrake();
            shooterPitch.control();
            pitchHomeTimer.stop();
            pitchHomeCurrentCounter = 0;
            pitchHomeState = PitchHomeState.UNHOMED;
            return;
        }

        if (pitchHomeState == PitchHomeState.UNHOMED) {
            beginPitchHoming();
        }

        if (pitchHomeState == PitchHomeState.HOMING) {
            runPitchHoming();
        } else if (pitchHomeState == PitchHomeState.HOMED) {
            setShootPitchAngle(targetSetpoint.pitchDeg());
            shooterPitch.control();
        } else {
            shooterPitch.setBrake();
            shooterPitch.control();
        }

        shooterRightUp.setVelocityTorqueCurrent(
                shotActive ? targetSetpoint.flywheelRps() : IDLE_SPEED_RPS);
        shooterRightUp.control();
        if (!flywheelFollowersArmed) {
            commandFlywheelFollowers();
            flywheelFollowersArmed = true;
        }
        healthPitchAngle = getPitchAngle();

        SmartDashboard.putNumber(
                "Shooting/FlywheelTargetRps",
                shotActive ? targetSetpoint.flywheelRps() : IDLE_SPEED_RPS);
        SmartDashboard.putNumber("Shooting/FlywheelActualRps", healthFlywheelVelocity);
        SmartDashboard.putNumber("Shooting/PitchTargetDeg", targetSetpoint.pitchDeg());
        SmartDashboard.putNumber("Shooting/PitchActualDeg", healthPitchAngle);
        SmartDashboard.putNumber("Shooting/PitchHomeState", pitchHomeState.ordinal());
    }

    private void initialize() {
        TalonFXConfiguration flywheelConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.CounterClockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Coast))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(120)
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(50)
                        .withSupplyCurrentLimitEnable(true))
                .withSlot0(new Slot0Configs()
                        .withKS(4.875)
                        .withKP(9));
        applyWithRetry(shooterLeftDown, flywheelConfig);
        applyWithRetry(shooterLeftUp, flywheelConfig);
        applyWithRetry(shooterRightUp, flywheelConfig);
        applyWithRetry(shooterRightDown, flywheelConfig);
        shooterRightUp.setInvert(-1);

        shooterLeftDown.setFollower(shooterRightUp.getData().deviceId, true);
        shooterLeftUp.setFollower(shooterRightUp.getData().deviceId, true);
        shooterRightDown.setFollower(shooterRightUp.getData().deviceId, false);
        commandFlywheelFollowers();

        TalonFXConfiguration pitchConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.CounterClockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(60)
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(20)
                        .withSupplyCurrentLimitEnable(true))
                .withSlot0(new Slot0Configs()
                        .withKP(7)
                        .withKI(0.8)
                        .withKD(0.02))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(0)
                        .withMotionMagicExpo_kV(0.1)
                        .withMotionMagicExpo_kA(0.01));
        applyWithRetry(shooterPitch, pitchConfig);
        shooterPitch.setGearRatio(PITCH_GEAR_RATIO);
        shooterPitch.setInvert(-1);
        shooterPitch.setPhysicalLimits(
                0.0,
                Constants.ShooterConstants.pitchMotorMaxPositionRot / PITCH_GEAR_RATIO,
                120.0 / PITCH_GEAR_RATIO,
                40.0);
        shooterPitch.setCurrentSpeed(0.1);
        shooterPitch.setStatusSignalUpdateFrequency(50);
    }

    public void applyShotSetpoint(ShotSetpoint setpoint) {
        targetSetpoint = setpoint;
        shotActive = true;
    }

    public void setIdle() {
        shotActive = false;
        targetSetpoint = new ShotSetpoint(IDLE_SPEED_RPS, 0.0, 0.2);
    }

    public void stop() {
        shotActive = false;
        shooterRightUp.setCoast();
        shooterRightUp.control();
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }

    public void beginPitchHoming() {
        if (!DriverStation.isEnabled()) {
            pitchHomeState = PitchHomeState.UNHOMED;
            return;
        }
        if (RobotBase.isSimulation()) {
            targetSetpoint = new ShotSetpoint(targetSetpoint.flywheelRps(), targetSetpoint.feederRps(), 0.0);
            pitchHomeState = PitchHomeState.HOMED;
            return;
        }
        pitchHomeState = PitchHomeState.HOMING;
        pitchHomeCurrentCounter = 0;
        pitchHomeTimer.restart();
    }

    public PitchHomeState getPitchHomeState() {
        return pitchHomeState;
    }

    public boolean isPitchHomed() {
        return pitchHomeState == PitchHomeState.HOMED;
    }

    public double getShootVelocity() {
        return shooterRightUp.getVelocity();
    }

    public double getPitchAngle() {
        double normalized = shooterPitch.getNormalizedPosition();
        return normalized
                * (Constants.ShooterConstants.maxPitchAngleDeg
                        - Constants.ShooterConstants.minPitchAngleDeg)
                + Constants.ShooterConstants.minPitchAngleDeg;
    }

    public boolean isFlywheelReady(double toleranceRps) {
        return shotActive
                && Math.abs(shooterRightUp.getVelocity() - targetSetpoint.flywheelRps()) <= toleranceRps;
    }

    public boolean isPitchReady(double toleranceDeg) {
        return isPitchHomed()
                && Math.abs(getPitchAngle() - targetSetpoint.pitchDeg()) <= toleranceDeg;
    }

    public ShotSetpoint getTargetSetpoint() {
        return targetSetpoint;
    }

    /** Registers the shooter hardware and control context with the central logger. */
    public void registerHealthLogging(RobotHealthLogger logger) {
        if (logger == null) {
            return;
        }
        int flywheelLeaderId = shooterRightUp.getData().deviceId;
        logger.registerTalonFX(
                "Shooter",
                "LeftDown",
                shooterLeftDown.getMotor(),
                MotorHealthConfig.follower(flywheelLeaderId, true));
        logger.registerTalonFX(
                "Shooter",
                "LeftUp",
                shooterLeftUp.getMotor(),
                MotorHealthConfig.follower(flywheelLeaderId, true));
        logger.registerTalonFX(
                "Shooter",
                "RightUpLeader",
                shooterRightUp.getMotor(),
                MotorHealthConfig.leader());
        logger.registerTalonFX(
                "Shooter",
                "RightDown",
                shooterRightDown.getMotor(),
                MotorHealthConfig.follower(flywheelLeaderId, false));
        logger.registerTalonFX("Shooter", "Pitch", shooterPitch.getMotor());
        logger.registerSubsystem(
                "Shooter",
                this,
                this::isHealthActive,
                this::getHealthState,
                () -> shotActive ? targetSetpoint.flywheelRps() : IDLE_SPEED_RPS,
                () -> healthFlywheelVelocity);
    }

    private boolean isHealthActive() {
        return shotActive || pitchHomeState == PitchHomeState.HOMING;
    }

    private String getHealthState() {
        if (!DriverStation.isEnabled()) {
            return "Disabled";
        }
        return switch (pitchHomeState) {
            case UNHOMED -> "Unhomed";
            case HOMING -> "Homing";
            case FAULT -> "Fault";
            case HOMED -> {
                if (!shotActive) {
                    yield "Idle";
                }
                yield isHealthFlywheelReady(1.0) && isHealthPitchReady(1.0)
                        ? "Ready"
                        : "SpinningUp";
            }
        };
    }

    private boolean isHealthFlywheelReady(double toleranceRps) {
        return shotActive
                && Math.abs(healthFlywheelVelocity - targetSetpoint.flywheelRps())
                        <= toleranceRps;
    }

    private boolean isHealthPitchReady(double toleranceDeg) {
        return isPitchHomed()
                && Math.abs(healthPitchAngle - targetSetpoint.pitchDeg()) <= toleranceDeg;
    }

    private void runPitchHoming() {
        shooterPitch.setCurrent(PITCH_HOME_CURRENT_AMPS);
        shooterPitch.control();

        if (shooterPitch.getCurrent() < PITCH_HOME_CURRENT_THRESHOLD_AMPS) {
            pitchHomeCurrentCounter++;
        } else {
            pitchHomeCurrentCounter = 0;
        }

        if (pitchHomeCurrentCounter >= PITCH_HOME_CURRENT_COUNT) {
            shooterPitch.reset(shooterPitch.getAbsPosition());
            shooterPitch.setBrake();
            shooterPitch.control();
            targetSetpoint = new ShotSetpoint(targetSetpoint.flywheelRps(), targetSetpoint.feederRps(), 0.0);
            pitchHomeState = PitchHomeState.HOMED;
            pitchHomeTimer.stop();
            return;
        }

        if (pitchHomeTimer.hasElapsed(PITCH_HOME_TIMEOUT_SECONDS)) {
            shooterPitch.setBrake();
            shooterPitch.control();
            pitchHomeState = PitchHomeState.FAULT;
            pitchHomeTimer.stop();
        }
    }

    private void setShootPitchAngle(double targetAngleDeg) {
        double clamped = Math.max(
                Constants.ShooterConstants.minPitchAngleDeg,
                Math.min(Constants.ShooterConstants.maxPitchAngleDeg, targetAngleDeg));
        double normalized = (clamped - Constants.ShooterConstants.minPitchAngleDeg)
                / (Constants.ShooterConstants.maxPitchAngleDeg
                        - Constants.ShooterConstants.minPitchAngleDeg);
        shooterPitch.setNormalizedMotionPosition(normalized);
    }

    private void commandFlywheelFollowers() {
        shooterLeftDown.control();
        shooterLeftUp.control();
        shooterRightDown.control();
    }

    private static void applyWithRetry(WayiMotor motor, TalonFXConfiguration config) {
        for (int i = 0; i < 5; i++) {
            StatusCode status = motor.applyConfig(config);
            if (status.isOK()) {
                return;
            }
        }
    }
}
