package frc.robot.health.analyzer.rules;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * A deliberately data-oriented rule configuration. Threshold names are owned by each
 * {@link RuleType}, which keeps JSON forward-compatible without hard-coding one large DTO.
 */
public final class RuleConfig {
    private Boolean enabled;
    private Severity severity;
    private Double durationSeconds;
    private Map<String, Double> thresholds;

    public RuleConfig() {}

    public RuleConfig(
            Boolean enabled,
            Severity severity,
            Double durationSeconds,
            Map<String, Double> thresholds) {
        this.enabled = enabled;
        this.severity = severity;
        this.durationSeconds = durationSeconds;
        this.thresholds = thresholds == null ? null : new LinkedHashMap<>(thresholds);
    }

    public boolean enabled() {
        return enabled == null || enabled;
    }

    public Severity severity() {
        return severity == null ? Severity.WARNING : severity;
    }

    public double durationSeconds() {
        return durationSeconds == null ? 0.0 : Math.max(0.0, durationSeconds);
    }

    public double threshold(String name, double fallback) {
        if (thresholds == null) {
            return fallback;
        }
        Double value = thresholds.get(name);
        return value == null || !Double.isFinite(value) ? fallback : value;
    }

    public Map<String, Double> thresholds() {
        return thresholds == null ? Map.of() : Map.copyOf(thresholds);
    }

    static RuleConfig merge(RuleConfig base, RuleConfig override) {
        if (base == null && override == null) {
            return new RuleConfig();
        }
        RuleConfig merged = new RuleConfig();
        merged.enabled = value(override == null ? null : override.enabled, base == null ? null : base.enabled);
        merged.severity = value(override == null ? null : override.severity, base == null ? null : base.severity);
        merged.durationSeconds = value(
                override == null ? null : override.durationSeconds,
                base == null ? null : base.durationSeconds);
        merged.thresholds = new LinkedHashMap<>();
        if (base != null && base.thresholds != null) {
            merged.thresholds.putAll(base.thresholds);
        }
        if (override != null && override.thresholds != null) {
            merged.thresholds.putAll(override.thresholds);
        }
        return merged;
    }

    private static <T> T value(T preferred, T fallback) {
        return preferred != null ? preferred : fallback;
    }
}
