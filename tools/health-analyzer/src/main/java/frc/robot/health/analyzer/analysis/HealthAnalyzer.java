package frc.robot.health.analyzer.analysis;

import frc.robot.health.analyzer.analysis.HealthSignalIndex.MotorKey;
import frc.robot.health.analyzer.model.HealthLog;
import frc.robot.health.analyzer.model.Sample;
import frc.robot.health.analyzer.rules.Anomaly;
import frc.robot.health.analyzer.rules.HealthRules;
import frc.robot.health.analyzer.rules.RuleConfig;
import frc.robot.health.analyzer.rules.RuleEngine;
import frc.robot.health.analyzer.rules.RuleType;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;

/** Stable non-UI facade for log-to-result analysis. */
public final class HealthAnalyzer {
    private HealthAnalyzer() {}

    public static AnalysisResult analyze(HealthLog log, HealthRules rules) {
        if (log == null || rules == null) {
            throw new IllegalArgumentException("log and rules are required");
        }

        Map<String, SignalStatistics> perSignal = new TreeMap<>();
        log.doubleSignals().forEach(
                (path, samples) ->
                        perSignal.put(
                                path,
                                signalStatisticsForSelectedRange(
                                        log, path, samples)));

        HealthSignalIndex index = new HealthSignalIndex(log);
        Map<String, SubsystemAnalysis> perSubsystem = analyzeSubsystems(log, rules, index);
        List<Anomaly> anomalies = new RuleEngine().evaluate(log, rules);
        AnalysisSummary summary = summarize(log, rules, anomalies, perSubsystem);

        return new AnalysisResult(
                summary,
                anomalies,
                perSignal,
                perSubsystem,
                Map.of(
                        "Supply Current",
                        "Battery-side input current for a motor controller, PDH branch, or subsystem.",
                        "Stator Current",
                        "Motor winding/torque-producing current; it must not be summed as robot battery current.",
                        "PDH Total Current",
                        "Whole-robot battery-side current measured by the PDH/PDP."));
    }

    private static AnalysisSummary summarize(
            HealthLog log,
            HealthRules rules,
            List<Anomaly> anomalies,
            Map<String, SubsystemAnalysis> subsystems) {
        List<Sample<Double>> voltage = log.doubleSeries("/Health/Power/Voltage");
        List<Sample<Double>> totalCurrent =
                log.doubleSeries("/Health/Power/TotalCurrent");
        RuleConfig currentRule = rules.resolve(RuleType.HIGH_TOTAL_CURRENT, null);
        double threshold = currentRule.threshold("rawCurrent", 250.0);
        CurrentStatistics total =
                CurrentStatistics.of(
                        "PDH Total Current",
                        totalCurrent,
                        threshold,
                        log.startMicros(),
                        log.endMicros());

        List<Sample<Boolean>> enabledSeries =
                log.booleanSeries("/Health/Robot/Enabled");
        double enabled = enabledSeries.isEmpty()
                ? Double.NaN
                : SeriesMath.trueDuration(
                        enabledSeries, log.startMicros(), log.endMicros());
        double maximumTemperature = log.doubleSignals().entrySet().stream()
                .filter(entry -> entry.getKey().endsWith("/Temperature"))
                .mapToDouble(entry -> SeriesMath.max(entry.getValue()))
                .filter(Double::isFinite)
                .max()
                .orElse(Double.NaN);
        double maximumCan =
                log.doubleSignals().entrySet().stream()
                        .filter(
                                entry ->
                                        entry.getKey()
                                                        .equals(
                                                                "/Health/Robot/CAN/Utilization")
                                                || (entry.getKey()
                                                                .startsWith(
                                                                        "/Health/Robot/CAN/Buses/")
                                                        && entry.getKey()
                                                                .endsWith("/Utilization")))
                        .mapToDouble(entry -> SeriesMath.max(entry.getValue()))
                        .filter(Double::isFinite)
                        .max()
                        .orElse(Double.NaN);
        double maximumTemperatureRise =
                subsystems.values().stream()
                        .mapToDouble(SubsystemAnalysis::maximumTemperatureRiseCPerSecond)
                        .filter(Double::isFinite)
                        .max()
                        .orElse(Double.NaN);
        double maximumReferenceError =
                subsystems.values().stream()
                        .mapToDouble(SubsystemAnalysis::maximumAbsoluteReferenceError)
                        .filter(Double::isFinite)
                        .max()
                        .orElse(Double.NaN);
        double maximumHighCurrentLowVelocity =
                subsystems.values().stream()
                        .mapToDouble(
                                SubsystemAnalysis::maximumHighCurrentLowVelocitySeconds)
                        .filter(Double::isFinite)
                        .max()
                        .orElse(Double.NaN);

        return new AnalysisSummary(
                log.startMicros(),
                log.durationMicros() / 1_000_000.0,
                enabled,
                SeriesMath.first(voltage),
                SeriesMath.min(voltage),
                total,
                total.ampHours(),
                coveredWattHours(log, voltage, totalCurrent),
                RuleEngine.brownoutIncidentCount(log),
                anomalies.size(),
                maximumTemperature,
                maximumTemperatureRise,
                maximumReferenceError,
                maximumHighCurrentLowVelocity,
                maximumCan);
    }

    private static Map<String, SubsystemAnalysis> analyzeSubsystems(
            HealthLog log, HealthRules rules, HealthSignalIndex index) {
        Set<String> subsystemNames = new LinkedHashSet<>(index.subsystems().keySet());
        index.motors().keySet().forEach(key -> subsystemNames.add(key.subsystem()));
        for (String path : log.booleanSignals().keySet()) {
            String name = subsystemFromPath(path);
            if (name != null) {
                subsystemNames.add(name);
            }
        }
        for (String path : log.stringSignals().keySet()) {
            String name = subsystemFromPath(path);
            if (name != null) {
                subsystemNames.add(name);
            }
        }

        Map<String, SubsystemAnalysis> result = new LinkedHashMap<>();
        for (String subsystem : subsystemNames) {
            Map<String, MotorAnalysis> motors =
                    analyzeMotors(log, rules, index, subsystem);
            List<MotorKey> registeredMotors =
                    index.motors().keySet().stream()
                            .filter(key -> key.subsystem().equals(subsystem))
                            .toList();
            List<List<Sample<Double>>> allMotorSupply =
                    registeredMotors.stream()
                            .map(key -> index.motor(key, "SupplyCurrent"))
                            .toList();
            int availableMotorSupply =
                    (int) allMotorSupply.stream().filter(series -> !series.isEmpty()).count();
            int fullyCoveredMotorSupply =
                    (int)
                            allMotorSupply.stream()
                                    .filter(
                                            series ->
                                                    TelemetryCoverage.timelyAndContinuous(
                                                            series,
                                                            log.startMicros(),
                                                            log.endMicros()))
                                    .count();
            boolean completeMotorSupply =
                    !registeredMotors.isEmpty()
                            && fullyCoveredMotorSupply == registeredMotors.size();
            List<Sample<Double>> supply = index.subsystem(subsystem, "SupplyCurrent");
            String source = loggedCurrentSource(log, subsystem);
            boolean talonBacked = isPureTalonCurrentSource(source);
            if (talonBacked) {
                if (completeMotorSupply) {
                    supply = SeriesMath.sum(allMotorSupply);
                    source =
                            supply.isEmpty()
                                    ? partialCurrentSource(
                                            source,
                                            availableMotorSupply,
                                            fullyCoveredMotorSupply,
                                            registeredMotors.size())
                                    : "TALON_SUPPLY_CURRENT_SUM";
                } else {
                    supply = List.of();
                    source =
                            partialCurrentSource(
                                    source,
                                    availableMotorSupply,
                                    fullyCoveredMotorSupply,
                                    registeredMotors.size());
                }
            } else if (supply.isEmpty() && source.equalsIgnoreCase("UNAVAILABLE")) {
                if (completeMotorSupply) {
                    supply = SeriesMath.sum(allMotorSupply);
                    source =
                            supply.isEmpty()
                                    ? "UNAVAILABLE"
                                    : "TALON_SUPPLY_CURRENT_SUM";
                } else if (availableMotorSupply > 0) {
                    source =
                            partialCurrentSource(
                                    "TalonFXSupplyCurrent",
                                    availableMotorSupply,
                                    fullyCoveredMotorSupply,
                                    registeredMotors.size());
                }
            } else if (source.equalsIgnoreCase("UNAVAILABLE")) {
                source = "LOGGED_SUBSYSTEM_SUPPLY_CURRENT";
            }

            RuleConfig currentRule = rules.resolve(RuleType.HIGH_TOTAL_CURRENT, subsystem);
            double currentThreshold = currentRule.threshold("rawCurrent", 250.0);
            RuleConfig imbalanceRule = rules.resolve(RuleType.MOTOR_IMBALANCE, subsystem);
            double minimumCurrent = imbalanceRule.threshold("minCurrent", 10.0);
            double imbalanceThreshold = imbalanceRule.threshold("ratio", 0.45);
            List<Sample<Double>> imbalance =
                    completeMotorSupply && registeredMotors.size() >= 2
                            ? SeriesMath.imbalance(allMotorSupply, minimumCurrent)
                            : List.of();

            List<Sample<Boolean>> activeSeries =
                    log.booleanSeries("/Health/Subsystems/" + subsystem + "/Active");
            double activeSeconds = activeSeries.isEmpty()
                    ? Double.NaN
                    : SeriesMath.trueDuration(
                            activeSeries, log.startMicros(), log.endMicros());
            double energyWh =
                    coveredWattHours(
                            log,
                            log.doubleSeries("/Health/Power/Voltage"),
                            supply);
            double maximumTemperature =
                    motors.values().stream()
                            .mapToDouble(motor -> motor.temperature().maximum())
                            .filter(Double::isFinite)
                            .max()
                            .orElse(Double.NaN);
            double maximumTemperatureRise =
                    motors.values().stream()
                            .mapToDouble(MotorAnalysis::maximumTemperatureRiseCPerSecond)
                            .filter(Double::isFinite)
                            .max()
                            .orElse(Double.NaN);
            TrackingErrorSeries subsystemError =
                    TrackingErrorSeries.subsystem(log, index, subsystem);
            double meanReferenceError =
                    !subsystemError.available()
                            ? motors.values().stream()
                                    .mapToDouble(MotorAnalysis::meanAbsoluteReferenceError)
                                    .filter(Double::isFinite)
                                    .average()
                                    .orElse(Double.NaN)
                            : subsystemError.meanAbsolute();
            double maximumReferenceError =
                    !subsystemError.available()
                            ? motors.values().stream()
                                    .mapToDouble(MotorAnalysis::maximumAbsoluteReferenceError)
                                    .filter(Double::isFinite)
                                    .max()
                                    .orElse(Double.NaN)
                            : subsystemError.maximumAbsolute();
            double maximumHighCurrentLowVelocity =
                    motors.values().stream()
                            .mapToDouble(MotorAnalysis::highCurrentLowVelocitySeconds)
                            .filter(Double::isFinite)
                            .max()
                            .orElse(Double.NaN);
            result.put(
                    subsystem,
                    new SubsystemAnalysis(
                            subsystem,
                            activeSeconds,
                            source,
                            CurrentStatistics.of(
                                    "Supply Current",
                                    supply,
                                    currentThreshold,
                                    log.startMicros(),
                                    log.endMicros()),
                            energyWh,
                            SeriesMath.timeWeightedMean(imbalance),
                            SeriesMath.max(imbalance),
                            SeriesMath.durationAbove(imbalance, imbalanceThreshold),
                            maximumTemperature,
                            maximumTemperatureRise,
                            meanReferenceError,
                            maximumReferenceError,
                            maximumHighCurrentLowVelocity,
                            motors));
        }
        return Map.copyOf(result);
    }

    private static Map<String, MotorAnalysis> analyzeMotors(
            HealthLog log,
            HealthRules rules,
            HealthSignalIndex index,
            String subsystem) {
        Map<String, MotorAnalysis> result = new LinkedHashMap<>();
        for (MotorKey key : index.motors().keySet()) {
            if (!key.subsystem().equals(subsystem)) {
                continue;
            }
            RuleConfig stall = rules.resolve(RuleType.STALL, subsystem);
            double highCurrent = stall.threshold("minStatorCurrent", 60.0);
            double lowVelocity = stall.threshold("maxAbsVelocity", 1.0);
            List<Sample<Double>> supply = index.motor(key, "SupplyCurrent");
            List<Sample<Double>> stator = index.motor(key, "StatorCurrent");
            List<Sample<Double>> temperature = index.motor(key, "Temperature");
            TrackingErrorSeries error = TrackingErrorSeries.motor(log, index, key);
            result.put(
                    key.motor(),
                    new MotorAnalysis(
                            subsystem,
                            key.motor(),
                            CurrentStatistics.of(
                                    "Supply Current",
                                    supply,
                                    rules.resolve(RuleType.IDLE_DRAW, subsystem)
                                            .threshold("supplyCurrent", 8.0)),
                            CurrentStatistics.of(
                                    "Stator Current", stator, highCurrent),
                            SignalStatistics.of(
                                    "/Health/Motors/%s/%s/Temperature"
                                            .formatted(subsystem, key.motor()),
                                    temperature),
                            SeriesMath.maxPositiveSlopePerSecond(temperature),
                            error.meanAbsolute(),
                            error.maximumAbsolute(),
                            SeriesMath.highCurrentLowVelocityDuration(
                                    stator,
                                    index.motor(key, "Velocity"),
                                    highCurrent,
                                    lowVelocity)));
        }
        return Map.copyOf(result);
    }

    private static String subsystemFromPath(String path) {
        String prefix = "/Health/Subsystems/";
        if (!path.startsWith(prefix)) {
            return null;
        }
        int slash = path.indexOf('/', prefix.length());
        return slash < 0 ? null : path.substring(prefix.length(), slash);
    }

    private static String loggedCurrentSource(HealthLog log, String subsystem) {
        List<String> sources =
                log.stringSeries(
                                "/Health/Subsystems/" + subsystem + "/CurrentSource")
                        .stream()
                        .map(Sample::value)
                        .filter(value -> value != null && !value.isBlank())
                        .distinct()
                        .toList();
        if (sources.isEmpty()) {
            return "UNAVAILABLE";
        }
        return sources.size() == 1
                ? sources.get(0)
                : "MIXED[" + String.join("|", sources) + "]";
    }

    private static boolean isPureTalonCurrentSource(String source) {
        return source != null
                && source.trim().equalsIgnoreCase("TalonFXSupplyCurrent");
    }

    private static SignalStatistics signalStatisticsForSelectedRange(
            HealthLog log, String path, List<Sample<Double>> samples) {
        SignalStatistics statistics = SignalStatistics.of(path, samples);
        if (!isRangeScopedCurrentPath(path)
                || TelemetryCoverage.timelyAndContinuous(
                        samples, log.startMicros(), log.endMicros())) {
            return statistics;
        }
        return new SignalStatistics(
                statistics.path(),
                statistics.sampleCount(),
                statistics.startMicros(),
                statistics.endMicros(),
                statistics.minimum(),
                statistics.maximum(),
                Double.NaN,
                statistics.p95(),
                statistics.p99(),
                Double.NaN);
    }

    private static boolean isRangeScopedCurrentPath(String path) {
        if ("/Health/Power/TotalCurrent".equals(path)) {
            return true;
        }
        String subsystem = subsystemFromPath(path);
        return subsystem != null
                && path.equals(
                        "/Health/Subsystems/"
                                + subsystem
                                + "/SupplyCurrent");
    }

    private static double coveredWattHours(
            HealthLog log,
            List<Sample<Double>> voltage,
            List<Sample<Double>> current) {
        return TelemetryCoverage.timelyAndContinuous(
                                voltage,
                                log.startMicros(),
                                log.endMicros())
                        && TelemetryCoverage.timelyAndContinuous(
                                current,
                                log.startMicros(),
                                log.endMicros())
                ? SeriesMath.wattHours(voltage, current)
                : Double.NaN;
    }

    private static String partialCurrentSource(
            String source,
            int availableMotors,
            int fullyCoveredMotors,
            int registeredMotors) {
        if (registeredMotors <= 0) {
            return "UNAVAILABLE";
        }
        String base =
                source == null || source.isBlank() || source.equalsIgnoreCase("UNAVAILABLE")
                        ? "TalonFXSupplyCurrent"
                        : source;
        return "PARTIAL[%s;motorSeries=%d/%d;fullCoverage=%d/%d]"
                .formatted(
                        base,
                        availableMotors,
                        registeredMotors,
                        fullyCoveredMotors,
                        registeredMotors);
    }
}
