package frc.robot.health.analyzer.analysis;

import java.util.Map;

public record SubsystemAnalysis(
        String subsystem,
        double activeSeconds,
        String supplyCurrentSource,
        CurrentStatistics supplyCurrent,
        double energyWattHours,
        double meanMotorImbalanceRatio,
        double maximumMotorImbalanceRatio,
        double imbalanceSecondsAboveThreshold,
        double maximumTemperatureC,
        double maximumTemperatureRiseCPerSecond,
        double meanAbsoluteReferenceError,
        double maximumAbsoluteReferenceError,
        double maximumHighCurrentLowVelocitySeconds,
        Map<String, MotorAnalysis> motors) {

    public SubsystemAnalysis {
        motors = motors == null ? Map.of() : Map.copyOf(motors);
    }
}
