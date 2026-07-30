package frc.robot.health.analyzer.analysis;

import frc.robot.health.analyzer.rules.Anomaly;
import java.util.List;
import java.util.Map;

/** Immutable analysis output shared by CLI/report/UI layers. */
public record AnalysisResult(
        AnalysisSummary summary,
        List<Anomaly> anomalies,
        Map<String, SignalStatistics> signalStatistics,
        Map<String, SubsystemAnalysis> subsystemStatistics,
        Map<String, String> currentDefinitions) {

    public AnalysisResult {
        anomalies = anomalies == null ? List.of() : List.copyOf(anomalies);
        signalStatistics = signalStatistics == null ? Map.of() : Map.copyOf(signalStatistics);
        subsystemStatistics =
                subsystemStatistics == null ? Map.of() : Map.copyOf(subsystemStatistics);
        currentDefinitions = currentDefinitions == null ? Map.of() : Map.copyOf(currentDefinitions);
    }
}
