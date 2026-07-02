// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.shooting.ShotSetpoint;
import frc.robot.shooting.ShotTable;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootWithTableCommand extends Command {
    private final ShooterSubsystem shooter;
    private final FeederSubsystem feeder;
    private final Supplier<ShotSetpoint> setpointSupplier;
    private final BooleanSupplier alignmentReadySupplier;
    private final boolean forcePitchHome;

    private boolean shotTimerStarted = false;
    private boolean feeding = false;
    private final Timer shotTimer = new Timer();

    public ShootWithTableCommand(
            ShooterSubsystem shooter,
            FeederSubsystem feeder,
            Supplier<ShotSetpoint> setpointSupplier,
            BooleanSupplier alignmentReadySupplier) {
        this(shooter, feeder, setpointSupplier, alignmentReadySupplier, false);
    }

    public ShootWithTableCommand(
            ShooterSubsystem shooter,
            FeederSubsystem feeder,
            Supplier<ShotSetpoint> setpointSupplier,
            BooleanSupplier alignmentReadySupplier,
            boolean forcePitchHome) {
        this.shooter = shooter;
        this.feeder = feeder;
        this.setpointSupplier = setpointSupplier;
        this.alignmentReadySupplier = alignmentReadySupplier;
        this.forcePitchHome = forcePitchHome;
        addRequirements(shooter, feeder);
    }

    @Override
    public void initialize() {
        shotTimer.stop();
        shotTimer.reset();
        shotTimerStarted = false;
        feeding = false;
        feeder.stop();

        if (forcePitchHome
                || shooter.getPitchHomeState() == ShooterSubsystem.PitchHomeState.UNHOMED
                || shooter.getPitchHomeState() == ShooterSubsystem.PitchHomeState.FAULT) {
            shooter.beginPitchHoming();
        }
    }

    @Override
    public void execute() {
        if (!shooter.isPitchHomed()) {
            feeder.setBackwardFeederDuty(0.0);
            feeder.setUpwardDuty(0.0);
            return;
        }

        ShotSetpoint setpoint = setpointSupplier.get();
        shooter.applyShotSetpoint(setpoint);
        feeder.setUpwardFeederVelocity(setpoint.feederRps());

        if (!shotTimerStarted) {
            shotTimer.restart();
            shotTimerStarted = true;
        }

        boolean ready = shooter.isFlywheelReady(0.7)
                && shooter.isPitchReady(0.75)
                && Math.abs(feeder.getUpwardFeederVelocity() - setpoint.feederRps()) <= 2.0
                && alignmentReadySupplier.getAsBoolean();

        if (ShotTable.feedAllowed(shooter.isPitchHomed(), ready, shotTimer.hasElapsed(1.5))) {
            feeding = true;
        }
        feeder.setBackwardFeederDuty(feeding ? 1.0 : 0.0);

        SmartDashboard.putBoolean("Shooting/Ready", ready);
        SmartDashboard.putBoolean("Shooting/ForcedFeed", feeding && !ready);
        SmartDashboard.putBoolean("Shooting/Feeding", feeding);
    }

    @Override
    public void end(boolean interrupted) {
        shotTimer.stop();
        feeder.stop();
        shooter.setIdle();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
