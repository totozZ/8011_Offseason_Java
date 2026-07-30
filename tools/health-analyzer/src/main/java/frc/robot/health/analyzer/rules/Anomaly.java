package frc.robot.health.analyzer.rules;

import java.util.Map;

/** A time-addressable finding suitable for a timeline marker and report row. */
public record Anomaly(
        RuleType rule,
        Severity severity,
        long startMicros,
        long endMicros,
        String subsystem,
        String motor,
        String reason,
        String recommendation,
        Map<String, Double> observed) {

    public Anomaly {
        observed = observed == null ? Map.of() : Map.copyOf(observed);
        subsystem = subsystem == null ? "" : subsystem;
        motor = motor == null ? "" : motor;
        reason = reason == null ? "" : reason;
        recommendation = recommendation == null ? "" : recommendation;
    }

    public double durationSeconds() {
        return Math.max(0L, endMicros - startMicros) / 1_000_000.0;
    }
}
