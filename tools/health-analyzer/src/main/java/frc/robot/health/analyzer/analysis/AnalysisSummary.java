package frc.robot.health.analyzer.analysis;

/** Top-level cards consumed by reports and the future UI. */
public record AnalysisSummary(
        long analysisStartMicros,
        double testDurationSeconds,
        double enabledSeconds,
        double startingVoltage,
        double minimumVoltage,
        CurrentStatistics pdhTotalCurrent,
        double consumedAmpHours,
        double consumedWattHours,
        Integer brownoutCount,
        int anomalyCount,
        double maximumTemperatureC,
        double maximumTemperatureRiseCPerSecond,
        double maximumAbsoluteReferenceError,
        double maximumHighCurrentLowVelocitySeconds,
        double maximumCanUtilization) {}
