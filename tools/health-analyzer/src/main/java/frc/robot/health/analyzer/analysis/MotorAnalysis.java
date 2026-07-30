package frc.robot.health.analyzer.analysis;

public record MotorAnalysis(
        String subsystem,
        String motor,
        CurrentStatistics supplyCurrent,
        CurrentStatistics statorCurrent,
        SignalStatistics temperature,
        double maximumTemperatureRiseCPerSecond,
        double meanAbsoluteReferenceError,
        double maximumAbsoluteReferenceError,
        double highCurrentLowVelocitySeconds) {}
