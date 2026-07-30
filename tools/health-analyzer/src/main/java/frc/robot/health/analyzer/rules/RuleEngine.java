package frc.robot.health.analyzer.rules;

import frc.robot.health.analyzer.analysis.HealthSignalIndex;
import frc.robot.health.analyzer.analysis.HealthSignalIndex.MotorKey;
import frc.robot.health.analyzer.analysis.SeriesMath;
import frc.robot.health.analyzer.analysis.TelemetryCoverage;
import frc.robot.health.analyzer.analysis.TrackingErrorSeries;
import frc.robot.health.analyzer.model.HealthEvent;
import frc.robot.health.analyzer.model.HealthLog;
import frc.robot.health.analyzer.model.Sample;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;
import java.util.LinkedHashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;
import java.util.function.LongPredicate;

/** Evaluates the ten configurable health rule families against stable /Health paths. */
public final class RuleEngine {
    private static final long BROWNOUT_DEDUP_TOLERANCE_MICROS = 100_000L;

    public List<Anomaly> evaluate(HealthLog log, HealthRules rules) {
        HealthSignalIndex index = new HealthSignalIndex(log);
        List<Anomaly> findings = new ArrayList<>();
        evaluatePower(log, rules, findings);
        evaluateMotors(log, rules, index, findings);
        evaluateFollowers(log, rules, index, findings);
        evaluateCan(log, rules, findings);
        findings.sort(Comparator.comparingLong(Anomaly::startMicros)
                .thenComparing(a -> a.rule().ordinal()));
        return List.copyOf(findings);
    }

    private static void evaluatePower(HealthLog log, HealthRules rules, List<Anomaly> findings) {
        RuleConfig lowVoltage = rules.resolve(RuleType.LOW_VOLTAGE, null);
        List<Sample<Double>> voltage = log.doubleSeries("/Health/Power/Voltage");
        if (lowVoltage.enabled()) {
            double threshold = lowVoltage.threshold("voltage", 7.0);
            addIntervals(
                    findings,
                    RuleType.LOW_VOLTAGE,
                    lowVoltage,
                    numericCondition(voltage, value -> value < threshold),
                    lastTimestamp(voltage, log.endMicros()),
                    "",
                    "",
                    "Battery voltage remained below %.2f V".formatted(threshold),
                    "Inspect battery charge, main connections, breaker, and high-current loads.",
                    Map.of("thresholdVolts", threshold));
        }

        RuleConfig brownout = rules.resolve(RuleType.BROWNOUT, null);
        if (brownout.enabled()) {
            evaluateBrownouts(log, brownout, findings);
        }

        RuleConfig highCurrent = rules.resolve(RuleType.HIGH_TOTAL_CURRENT, null);
        List<Sample<Double>> totalCurrent = log.doubleSeries("/Health/Power/TotalCurrent");
        if (highCurrent.enabled() && !totalCurrent.isEmpty()) {
            double raw = highCurrent.threshold("rawCurrent", 250.0);
            addIntervals(
                    findings,
                    RuleType.HIGH_TOTAL_CURRENT,
                    highCurrent,
                    numericCondition(totalCurrent, value -> value > raw),
                    lastTimestamp(totalCurrent, log.endMicros()),
                    "",
                    "",
                    "PDH total current exceeded the raw limit %.1f A".formatted(raw),
                    "Correlate the peak with subsystem supply currents and commanded activity.",
                    Map.of("rawCurrentThresholdAmps", raw));

            long windowMicros = Math.max(
                    1L,
                    Math.round(highCurrent.threshold("rollingWindowMs", 100.0) * 1_000.0));
            double rolling = highCurrent.threshold("rollingCurrent", 200.0);
            for (List<Sample<Double>> segment : continuousSegments(totalCurrent)) {
                List<Sample<Double>> rollingSeries =
                        SeriesMath.trailingRollingAverage(segment, windowMicros);
                addIntervals(
                        findings,
                        RuleType.HIGH_TOTAL_CURRENT,
                        highCurrent,
                        numericCondition(rollingSeries, value -> value > rolling),
                        lastTimestamp(rollingSeries, lastTimestamp(segment, log.endMicros())),
                        "",
                        "",
                        "PDH total current rolling mean exceeded %.1f A".formatted(rolling),
                        "Inspect sustained mechanism loads rather than a single inrush sample.",
                        Map.of(
                                "rollingCurrentThresholdAmps", rolling,
                                "rollingWindowMs", windowMicros / 1_000.0));
            }
        }
    }

    private static void evaluateBrownouts(
            HealthLog log, RuleConfig config, List<Anomaly> findings) {
        long minimumMicros = Math.round(config.durationSeconds() * 1_000_000.0);
        for (BrownoutIncident incident : brownoutIncidents(log)) {
            boolean sustainedSignal =
                    incident.booleanReported()
                            && incident.endMicros() - incident.startMicros()
                                    >= minimumMicros;
            if (!sustainedSignal && incident.eventTimestampMicros() == null) {
                continue;
            }
            long start =
                    sustainedSignal
                            ? incident.startMicros()
                            : incident.eventTimestampMicros();
            long end =
                    sustainedSignal ? incident.endMicros() : start;
            String reason =
                    incident.booleanReported() && incident.eventTimestampMicros() != null
                            ? "RobotController and log event reported a brownout"
                            : incident.booleanReported()
                                    ? "RobotController reported a brownout"
                                    : "Log event reported a brownout";
            findings.add(
                    new Anomaly(
                            RuleType.BROWNOUT,
                            config.severity(),
                            start,
                            end,
                            "",
                            "",
                            reason,
                            "Inspect the battery, power wiring, and load at this timeline location.",
                            Map.of()));
        }
    }

    /** Returns distinct brownout incidents from the boolean signal OR matching log events. */
    public static Integer brownoutIncidentCount(HealthLog log) {
        List<Sample<Boolean>> signal =
                log.booleanSeries("/Health/Robot/BrownedOut");
        List<HealthEvent> events = brownoutEvents(log);
        if (signal.isEmpty() && events.isEmpty()) {
            return null;
        }
        return brownoutIncidents(log, signal, events).size();
    }

    private static List<BrownoutIncident> brownoutIncidents(HealthLog log) {
        return brownoutIncidents(
                log,
                log.booleanSeries("/Health/Robot/BrownedOut"),
                brownoutEvents(log));
    }

    private static List<BrownoutIncident> brownoutIncidents(
            HealthLog log,
            List<Sample<Boolean>> signal,
            List<HealthEvent> events) {
        List<MutableBrownoutIncident> incidents = new ArrayList<>();
        for (TimeRange range : trueIntervals(signal, log.endMicros())) {
            incidents.add(
                    new MutableBrownoutIncident(
                            range.startMicros(), range.endMicros(), true, null));
        }
        for (HealthEvent event : events) {
            MutableBrownoutIncident match = null;
            long bestDistance = Long.MAX_VALUE;
            for (MutableBrownoutIncident candidate : incidents) {
                long distance =
                        distanceFromRange(
                                event.timestampMicros(),
                                candidate.startMicros,
                                candidate.endMicros);
                if (distance <= BROWNOUT_DEDUP_TOLERANCE_MICROS
                        && distance < bestDistance) {
                    match = candidate;
                    bestDistance = distance;
                }
            }
            if (match == null) {
                incidents.add(
                        new MutableBrownoutIncident(
                                event.timestampMicros(),
                                event.timestampMicros(),
                                false,
                                event.timestampMicros()));
            } else if (match.eventTimestampMicros == null) {
                match.eventTimestampMicros = event.timestampMicros();
            }
        }
        return incidents.stream()
                .sorted(Comparator.comparingLong(incident -> incident.startMicros))
                .map(
                        incident ->
                                new BrownoutIncident(
                                        incident.startMicros,
                                        incident.endMicros,
                                        incident.booleanReported,
                                        incident.eventTimestampMicros))
                .toList();
    }

    private static List<HealthEvent> brownoutEvents(HealthLog log) {
        return log.events().stream()
                .filter(
                        event ->
                                event.category()
                                                .toLowerCase(Locale.ROOT)
                                                .contains("brownout")
                                        || event.message()
                                                .toLowerCase(Locale.ROOT)
                                                .contains("brownout"))
                .toList();
    }

    private static long distanceFromRange(long timestamp, long start, long end) {
        if (timestamp < start) {
            return start - timestamp;
        }
        if (timestamp > end) {
            return timestamp - end;
        }
        return 0L;
    }

    private static void evaluateMotors(
            HealthLog log,
            HealthRules rules,
            HealthSignalIndex index,
            List<Anomaly> findings) {
        Map<String, List<MotorKey>> bySubsystem = new LinkedHashMap<>();
        for (MotorKey key : index.motors().keySet()) {
            bySubsystem.computeIfAbsent(key.subsystem(), ignored -> new ArrayList<>()).add(key);
            evaluateMotor(log, rules, index, key, findings);
        }

        Map<MotorKey, LinkedHashSet<MotorKey>> balanceGroups = new LinkedHashMap<>();
        for (HealthRules.FollowerPair pair : followerPairs(log, rules, index)) {
            MotorKey leader = new MotorKey(pair.subsystem(), pair.leader());
            LinkedHashSet<MotorKey> group =
                    balanceGroups.computeIfAbsent(
                            leader, ignored -> new LinkedHashSet<>());
            group.add(leader);
            group.add(new MotorKey(pair.subsystem(), pair.follower()));
        }

        for (Map.Entry<MotorKey, LinkedHashSet<MotorKey>> entry :
                balanceGroups.entrySet()) {
            MotorKey leader = entry.getKey();
            String subsystem = leader.subsystem();
            RuleConfig config = rules.resolve(RuleType.MOTOR_IMBALANCE, subsystem);
            if (!config.enabled()) {
                continue;
            }
            List<List<Sample<Double>>> currents = entry.getValue().stream()
                    .map(key -> index.motor(key, "SupplyCurrent"))
                    .toList();
            if (currents.size() < 2
                    || currents.stream()
                            .anyMatch(
                                    series ->
                                            !TelemetryCoverage.timelyAndContinuous(
                                                    series,
                                                    log.startMicros(),
                                                    log.endMicros()))) {
                continue;
            }
            List<Sample<Boolean>> active =
                    log.booleanSeries("/Health/Subsystems/" + subsystem + "/Active");
            if (active.isEmpty()) {
                continue;
            }
            double minCurrent = config.threshold("minCurrent", 10.0);
            double ratio = config.threshold("ratio", 0.45);
            List<Sample<Double>> imbalance = SeriesMath.imbalance(currents, minCurrent);
            List<Sample<Boolean>> activeImbalance =
                    condition(
                            imbalance.stream().map(Sample::timestampMicros).toList(),
                            time ->
                                    latestBoolean(active, time)
                                            && SeriesMath.valueAt(imbalance, time) > ratio);
            addIntervals(
                    findings,
                    RuleType.MOTOR_IMBALANCE,
                    config,
                    activeImbalance,
                    lastTimestamp(activeImbalance, log.endMicros()),
                    subsystem,
                    "",
                    "Follower-group supply-current imbalance for leader %s exceeded %.0f%%"
                            .formatted(leader.motor(), ratio * 100.0),
                    "Check mechanical drag, wiring, inversion, follower configuration, and gearing.",
                    Map.of("imbalanceRatio", ratio, "minimumActiveCurrentAmps", minCurrent));
        }

        Set<String> subsystemNames = new LinkedHashSet<>(index.subsystems().keySet());
        subsystemNames.addAll(bySubsystem.keySet());
        for (String subsystem : subsystemNames) {
            evaluateSubsystemTracking(log, rules, index, subsystem, findings);
        }
    }

    private static void evaluateSubsystemTracking(
            HealthLog log,
            HealthRules rules,
            HealthSignalIndex index,
            String subsystem,
            List<Anomaly> findings) {
        RuleConfig tracking = rules.resolve(RuleType.TRACKING_ERROR, subsystem);
        if (!tracking.enabled()) {
            return;
        }
        TrackingErrorSeries error =
                TrackingErrorSeries.subsystem(log, index, subsystem);
        if (!error.available()) {
            return;
        }
        double threshold = tracking.threshold("error", 5.0);
        addIntervals(
                findings,
                RuleType.TRACKING_ERROR,
                tracking,
                error.conditionAbove(threshold),
                error.endMicros(),
                subsystem,
                "",
                "Subsystem reference/measured error exceeded %.3f".formatted(threshold),
                "Inspect reference units, feedback sensor, gains, saturation, and mechanism load.",
                Map.of("absoluteErrorThreshold", threshold));
    }

    private static void evaluateMotor(
            HealthLog log,
            HealthRules rules,
            HealthSignalIndex index,
            MotorKey key,
            List<Anomaly> findings) {
        RuleConfig stall = rules.resolve(RuleType.STALL, key.subsystem());
        List<Sample<Double>> stator = index.motor(key, "StatorCurrent");
        List<Sample<Double>> velocity = index.motor(key, "Velocity");
        List<Sample<Double>> voltage = index.motor(key, "MotorVoltage");
        if (stall.enabled() && !stator.isEmpty() && !velocity.isEmpty() && !voltage.isEmpty()) {
            List<Sample<Double>> normalizedStator = SeriesMath.normalized(stator);
            List<Sample<Double>> normalizedVelocity = SeriesMath.normalized(velocity);
            List<Sample<Double>> normalizedVoltage = SeriesMath.normalized(voltage);
            double minOutput = stall.threshold("minOutputVolts", 2.0);
            double minStator = stall.threshold("minStatorCurrent", 60.0);
            double maxVelocity = stall.threshold("maxAbsVelocity", 1.0);
            List<List<Sample<Double>>> inputs =
                    List.of(normalizedStator, normalizedVelocity, normalizedVoltage);
            for (TimeRange coverage : commonCoverageSegments(inputs)) {
                List<Sample<Boolean>> condition = condition(
                        timelineWithin(inputs, coverage),
                        time -> Math.abs(SeriesMath.valueAt(normalizedVoltage, time))
                                                >= minOutput
                                && Math.abs(SeriesMath.valueAt(normalizedStator, time))
                                        >= minStator
                                && Math.abs(SeriesMath.valueAt(normalizedVelocity, time))
                                        <= maxVelocity);
                addIntervals(
                        findings,
                        RuleType.STALL,
                        stall,
                        condition,
                        coverage.endMicros(),
                        key.subsystem(),
                        key.motor(),
                        "High stator current with commanded voltage and near-zero velocity",
                        "Remove obstruction and inspect mechanism, current limit, sensor, and gearing.",
                        Map.of(
                                "minimumOutputVolts", minOutput,
                                "minimumStatorCurrentAmps", minStator,
                                "maximumAbsVelocity", maxVelocity));
            }
        }

        RuleConfig tracking = rules.resolve(RuleType.TRACKING_ERROR, key.subsystem());
        TrackingErrorSeries error = TrackingErrorSeries.motor(log, index, key);
        if (tracking.enabled() && error.available()) {
            double threshold = tracking.threshold("error", 5.0);
            addIntervals(
                    findings,
                    RuleType.TRACKING_ERROR,
                    tracking,
                    error.conditionAbove(threshold),
                    error.endMicros(),
                    key.subsystem(),
                    key.motor(),
                    "Closed-loop error exceeded %.3f".formatted(threshold),
                    "Inspect reference units, feedback sensor, gains, saturation, and mechanism load.",
                    Map.of("absoluteErrorThreshold", threshold));
        }

        RuleConfig temperature = rules.resolve(RuleType.TEMPERATURE, key.subsystem());
        List<Sample<Double>> temperatures = index.motor(key, "Temperature");
        if (temperature.enabled() && !temperatures.isEmpty()) {
            double high = temperature.threshold("temperature", 80.0);
            addIntervals(
                    findings,
                    RuleType.TEMPERATURE,
                    temperature,
                    numericCondition(temperatures, value -> value > high),
                    lastTimestamp(temperatures, log.endMicros()),
                    key.subsystem(),
                    key.motor(),
                    "Motor temperature exceeded %.1f C".formatted(high),
                    "Allow cooling and inspect load, friction, current limits, and airflow.",
                    Map.of("temperatureThresholdC", high));

            double rise = temperature.threshold("riseCPerSecond", 5.0);
            List<Sample<Double>> slopes = slopeSeries(temperatures);
            addIntervals(
                    findings,
                    RuleType.TEMPERATURE,
                    temperature,
                    numericCondition(slopes, value -> value > rise),
                    lastTimestamp(temperatures, log.endMicros()),
                    key.subsystem(),
                    key.motor(),
                    "Motor temperature rise exceeded %.2f C/s".formatted(rise),
                    "Inspect rapid load increase and compare against neighboring motors.",
                    Map.of("riseThresholdCPerSecond", rise));
        }

        RuleConfig idle = rules.resolve(RuleType.IDLE_DRAW, key.subsystem());
        if (idle.enabled()) {
            evaluateIdleDraw(log, index, key, idle, findings);
        }
    }

    private static void evaluateFollowers(
            HealthLog log,
            HealthRules rules,
            HealthSignalIndex index,
            List<Anomaly> findings) {
        for (HealthRules.FollowerPair pair : followerPairs(log, rules, index)) {
            RuleConfig config = rules.resolve(RuleType.FOLLOWER_FAULT, pair.subsystem());
            if (!config.enabled()) {
                continue;
            }
            MotorKey leader = new MotorKey(pair.subsystem(), pair.leader());
            MotorKey follower = new MotorKey(pair.subsystem(), pair.follower());
            List<Sample<Double>> leaderCurrent = index.motor(leader, "SupplyCurrent");
            List<Sample<Double>> leaderVoltage = index.motor(leader, "MotorVoltage");
            List<Sample<Double>> followerCurrent = index.motor(follower, "SupplyCurrent");
            List<Sample<Double>> followerVelocity = index.motor(follower, "Velocity");
            List<Sample<Boolean>> followerConnected =
                    log.booleanSeries(
                            "/Health/Motors/%s/%s/Connected"
                                    .formatted(pair.subsystem(), pair.follower()));
            boolean hasFollowerCurrent = !followerCurrent.isEmpty();
            boolean hasFollowerVelocity = !followerVelocity.isEmpty();
            boolean hasFollowerTelemetry = hasFollowerCurrent || hasFollowerVelocity;
            if (leaderCurrent.isEmpty()
                    || (!hasFollowerTelemetry && followerConnected.isEmpty())) {
                continue;
            }
            double leaderMinimum = config.threshold("leaderMinCurrent", 10.0);
            double leaderMinimumOutput =
                    config.threshold("leaderMinOutputVolts", 1.0);
            double followerCurrentMaximum = config.threshold("followerMaxCurrent", 2.0);
            double followerVelocityMaximum = config.threshold("followerMaxAbsVelocity", 0.5);
            long maximumSampleAgeMicros =
                    Math.max(
                            1L,
                            Math.round(
                                    config.threshold("maxSampleAgeSeconds", 0.75)
                                            * 1_000_000.0));
            TreeSet<Long> timeline = new TreeSet<>();
            addTimes(timeline, leaderCurrent);
            addTimes(timeline, leaderVoltage);
            addTimes(timeline, followerCurrent);
            addTimes(timeline, followerVelocity);
            followerConnected.forEach(sample -> timeline.add(sample.timestampMicros()));
            addFreshnessBoundaries(
                    timeline, leaderCurrent, maximumSampleAgeMicros, log.endMicros());
            addFreshnessBoundaries(
                    timeline, leaderVoltage, maximumSampleAgeMicros, log.endMicros());
            addFreshnessBoundaries(
                    timeline, followerCurrent, maximumSampleAgeMicros, log.endMicros());
            addFreshnessBoundaries(
                    timeline, followerVelocity, maximumSampleAgeMicros, log.endMicros());
            timeline.add(log.endMicros());
            List<Sample<Boolean>> condition = condition(
                    List.copyOf(timeline),
                    time -> {
                        double leaderCurrentValue =
                                freshValueAt(
                                        leaderCurrent,
                                        time,
                                        maximumSampleAgeMicros);
                        boolean leaderLoaded =
                                Double.isFinite(leaderCurrentValue)
                                        && Math.abs(leaderCurrentValue)
                                                > leaderMinimum;
                        double leaderVoltageValue =
                                freshValueAt(
                                        leaderVoltage,
                                        time,
                                        maximumSampleAgeMicros);
                        boolean leaderHasOutput =
                                leaderVoltage.isEmpty()
                                        || (Double.isFinite(leaderVoltageValue)
                                                && Math.abs(leaderVoltageValue)
                                                        > leaderMinimumOutput);
                        boolean disconnected =
                                hasBooleanAt(followerConnected, time)
                                        && !latestBoolean(followerConnected, time);
                        boolean stale =
                                (hasFollowerCurrent
                                                && hasDoubleAt(followerCurrent, time)
                                                && isStale(
                                                        followerCurrent,
                                                        time,
                                                        maximumSampleAgeMicros))
                                        || (hasFollowerVelocity
                                                && hasDoubleAt(followerVelocity, time)
                                                && isStale(
                                                        followerVelocity,
                                                        time,
                                                        maximumSampleAgeMicros));
                        double followerCurrentValue =
                                freshValueAt(
                                        followerCurrent,
                                        time,
                                        maximumSampleAgeMicros);
                        double followerVelocityValue =
                                freshValueAt(
                                        followerVelocity,
                                        time,
                                        maximumSampleAgeMicros);
                        boolean weak =
                                (hasFollowerCurrent
                                                && Double.isFinite(
                                                        followerCurrentValue)
                                                && Math.abs(followerCurrentValue)
                                                        < followerCurrentMaximum)
                                        || (hasFollowerVelocity
                                                && Double.isFinite(
                                                        followerVelocityValue)
                                                && Math.abs(followerVelocityValue)
                                                        < followerVelocityMaximum);
                        return leaderLoaded
                                && leaderHasOutput
                                && (disconnected || stale || weak);
                    });
            addIntervals(
                    findings,
                    RuleType.FOLLOWER_FAULT,
                    config,
                    condition,
                    log.endMicros(),
                    pair.subsystem(),
                    pair.follower(),
                    "Follower was disconnected, stale, or nearly unloaded while its leader was loaded",
                    "Check follower CAN status, wiring, inversion, leader ID, and mechanical coupling.",
                    Map.of(
                            "leaderMinimumCurrentAmps", leaderMinimum,
                            "leaderMinimumOutputVolts", leaderMinimumOutput,
                            "followerMaximumCurrentAmps", followerCurrentMaximum,
                            "followerMaximumAbsVelocity", followerVelocityMaximum,
                            "maximumSampleAgeSeconds",
                                    maximumSampleAgeMicros / 1_000_000.0));
        }
    }

    private static List<HealthRules.FollowerPair> followerPairs(
            HealthLog log, HealthRules rules, HealthSignalIndex index) {
        Set<HealthRules.FollowerPair> pairs = new LinkedHashSet<>(rules.followers());
        Map<MotorIdentity, String> motorsByIdentity = new LinkedHashMap<>();
        for (MotorKey key : index.motors().keySet()) {
            String base =
                    "/Health/Motors/%s/%s".formatted(key.subsystem(), key.motor());
            String canBus =
                    log.latestString(base + "/CANBus", log.endMicros())
                            .map(Sample::value)
                            .map(RuleEngine::normalizeCanBus)
                            .orElse("");
            log.latestDouble(base + "/DeviceId", log.endMicros())
                    .ifPresent(
                            sample ->
                                    motorsByIdentity.put(
                                            new MotorIdentity(
                                                    key.subsystem(),
                                                    canBus,
                                                    Math.round(sample.value())),
                                            key.motor()));
        }
        for (MotorKey key : index.motors().keySet()) {
            String base =
                    "/Health/Motors/%s/%s".formatted(key.subsystem(), key.motor());
            String role =
                    log.latestString(base + "/Role", log.endMicros())
                            .map(Sample::value)
                            .orElse("");
            if (!"FOLLOWER".equalsIgnoreCase(role)) {
                continue;
            }
            String canBus =
                    log.latestString(base + "/CANBus", log.endMicros())
                            .map(Sample::value)
                            .map(RuleEngine::normalizeCanBus)
                            .orElse("");
            log.latestDouble(base + "/Follower/LeaderDeviceId", log.endMicros())
                    .ifPresent(
                            sample -> {
                                 String leader =
                                        motorsByIdentity.get(
                                                new MotorIdentity(
                                                        key.subsystem(),
                                                        canBus,
                                                        Math.round(sample.value())));
                                if (leader != null && !leader.equals(key.motor())) {
                                    pairs.add(
                                            new HealthRules.FollowerPair(
                                                    key.subsystem(),
                                                    leader,
                                                    key.motor()));
                                }
                            });
        }
        return List.copyOf(pairs);
    }

    private static String normalizeCanBus(String canBus) {
        return canBus == null ? "" : canBus.trim().toLowerCase(Locale.ROOT);
    }

    private static void evaluateCan(HealthLog log, HealthRules rules, List<Anomaly> findings) {
        RuleConfig config = rules.resolve(RuleType.CAN_FAULT, null);
        if (!config.enabled()) {
            return;
        }
        double utilization = config.threshold("utilization", 0.85);
        evaluateCanUtilization(
                log,
                config,
                findings,
                "/Health/Robot/CAN/Utilization",
                "roboRIO",
                utilization);
        evaluateCanCounter(
                log,
                config,
                findings,
                "/Health/Robot/CAN/BusOffCount",
                "roboRIO",
                "BusOffCount");
        evaluateCanCounter(
                log,
                config,
                findings,
                "/Health/Robot/CAN/ReceiveErrors",
                "roboRIO",
                "ReceiveErrors");
        evaluateCanCounter(
                log,
                config,
                findings,
                "/Health/Robot/CAN/TransmitErrors",
                "roboRIO",
                "TransmitErrors");

        String busPrefix = "/Health/Robot/CAN/Buses/";
        TreeSet<String> busNames = new TreeSet<>();
        for (String path : log.signalPaths()) {
            if (path.startsWith(busPrefix)) {
                String remainder = path.substring(busPrefix.length());
                int slash = remainder.indexOf('/');
                if (slash > 0) {
                    busNames.add(remainder.substring(0, slash));
                }
            }
        }
        for (String bus : busNames) {
            String base = busPrefix + bus;
            evaluateCanUtilization(
                    log, config, findings, base + "/Utilization", bus, utilization);
            for (String counter :
                    List.of("BusOffCount", "ReceiveErrorCount", "TransmitErrorCount")) {
                evaluateCanCounter(
                        log, config, findings, base + "/" + counter, bus, counter);
            }
            List<Sample<Boolean>> connected = log.booleanSeries(base + "/Connected");
            if (!connected.isEmpty()) {
                addIntervals(
                        findings,
                        RuleType.CAN_FAULT,
                        config,
                        connected.stream()
                                .map(
                                        sample ->
                                                new Sample<>(
                                                        sample.timestampMicros(),
                                                        !sample.value()))
                                .toList(),
                        log.endMicros(),
                        "CAN/" + bus,
                        "",
                        "CAN bus status was not OK",
                        "Inspect CAN adapter power, wiring, termination, and device traffic.",
                        Map.of());
            }
        }
    }

    private static void evaluateCanUtilization(
            HealthLog log,
            RuleConfig config,
            List<Anomaly> findings,
            String path,
            String bus,
            double utilization) {
        List<Sample<Double>> values = log.doubleSeries(path);
        addIntervals(
                findings,
                RuleType.CAN_FAULT,
                config,
                numericCondition(values, value -> value > utilization),
                lastTimestamp(values, log.endMicros()),
                "CAN/" + bus,
                "",
                "%s CAN utilization exceeded %.0f%%"
                        .formatted(bus, utilization * 100.0),
                "Inspect status-signal frequencies, duplicate IDs, wiring, termination, and bus load.",
                Map.of("utilizationThreshold", utilization));
    }

    private static void evaluateCanCounter(
            HealthLog log,
            RuleConfig config,
            List<Anomaly> findings,
            String path,
            String bus,
            String counter) {
        List<Sample<Double>> values = SeriesMath.normalized(log.doubleSeries(path));
        for (int index = 1; index < values.size(); index++) {
            double delta = values.get(index).value() - values.get(index - 1).value();
            if (delta > 0.0) {
                long time = values.get(index).timestampMicros();
                findings.add(
                        new Anomaly(
                                RuleType.CAN_FAULT,
                                config.severity(),
                                time,
                                time,
                                "CAN/" + bus,
                                "",
                                counter + " increased by " + delta,
                                "Inspect CAN wiring, termination, IDs, device power, and status traffic.",
                                Map.of("delta", delta)));
            }
        }
    }

    private static void evaluateIdleDraw(
            HealthLog log,
            HealthSignalIndex index,
            MotorKey key,
            RuleConfig config,
            List<Anomaly> findings) {
        List<Sample<Boolean>> active =
                log.booleanSeries("/Health/Subsystems/" + key.subsystem() + "/Active");
        List<Sample<String>> state =
                log.stringSeries("/Health/Subsystems/" + key.subsystem() + "/State");
        if (active.isEmpty() && state.isEmpty()) {
            return;
        }
        List<Sample<Double>> supply = index.motor(key, "SupplyCurrent");
        List<Sample<Double>> stator = index.motor(key, "StatorCurrent");
        List<Sample<Double>> voltage = index.motor(key, "MotorVoltage");
        Collection<List<Sample<Double>>> numeric = List.of(supply, stator, voltage).stream()
                .filter(series -> !series.isEmpty())
                .toList();
        if (numeric.isEmpty()) {
            return;
        }
        double supplyThreshold = config.threshold("supplyCurrent", 8.0);
        double statorThreshold = config.threshold("statorCurrent", 10.0);
        double voltageThreshold = config.threshold("motorVoltage", 1.0);
        TreeSet<Long> timeline = new TreeSet<>();
        numeric.forEach(series -> addTimes(timeline, series));
        List<Sample<Boolean>> condition = condition(
                List.copyOf(timeline),
                time -> isIdle(active, state, time)
                        && ((!supply.isEmpty()
                                        && covers(supply, time)
                                        && Math.abs(SeriesMath.valueAt(supply, time)) > supplyThreshold)
                                || (!stator.isEmpty()
                                        && covers(stator, time)
                                        && Math.abs(SeriesMath.valueAt(stator, time)) > statorThreshold)
                                || (!voltage.isEmpty()
                                        && covers(voltage, time)
                                        && Math.abs(SeriesMath.valueAt(voltage, time)) > voltageThreshold)));
        addIntervals(
                findings,
                RuleType.IDLE_DRAW,
                config,
                condition,
                lastTimestamp(condition, log.endMicros()),
                key.subsystem(),
                key.motor(),
                "Motor consumed current or voltage while the subsystem reported Idle",
                "Check command cleanup, neutral mode, mechanical preload, and state reporting.",
                Map.of(
                        "supplyCurrentThresholdAmps", supplyThreshold,
                        "statorCurrentThresholdAmps", statorThreshold,
                        "motorVoltageThresholdVolts", voltageThreshold));
    }

    private static boolean isIdle(
            List<Sample<Boolean>> active, List<Sample<String>> state, long timestampMicros) {
        if (hasBooleanAt(active, timestampMicros)
                && !latestBoolean(active, timestampMicros)) {
            return true;
        }
        String latestState =
                state.isEmpty()
                                || state.get(0).timestampMicros() > timestampMicros
                        ? ""
                        : latestString(state, timestampMicros);
        return "idle".equalsIgnoreCase(latestState);
    }

    private static boolean covers(
            List<Sample<Double>> samples, long timestampMicros) {
        return !samples.isEmpty()
                && timestampMicros >= samples.get(0).timestampMicros()
                && timestampMicros
                        <= samples.get(samples.size() - 1).timestampMicros();
    }

    private static long lastTimestamp(
            List<? extends Sample<?>> samples, long fallback) {
        return samples == null || samples.isEmpty()
                ? fallback
                : samples.get(samples.size() - 1).timestampMicros();
    }

    private static List<Sample<Double>> slopeSeries(List<Sample<Double>> input) {
        List<Sample<Double>> values = SeriesMath.normalized(input);
        List<Sample<Double>> slopes = new ArrayList<>();
        for (int i = 1; i < values.size(); i++) {
            Sample<Double> a = values.get(i - 1);
            Sample<Double> b = values.get(i);
            double seconds = (b.timestampMicros() - a.timestampMicros()) / 1_000_000.0;
            if (seconds > 0.0
                    && b.timestampMicros() - a.timestampMicros()
                            <= SeriesMath.MAX_CONTINUOUS_GAP_MICROS) {
                slopes.add(new Sample<>(a.timestampMicros(), (b.value() - a.value()) / seconds));
            } else if (seconds > 0.0) {
                slopes.add(new Sample<>(a.timestampMicros(), -Double.MAX_VALUE));
            }
        }
        return List.copyOf(slopes);
    }

    private static List<Sample<Boolean>> numericCondition(
            List<Sample<Double>> input, java.util.function.DoublePredicate predicate) {
        List<Sample<Double>> samples = SeriesMath.normalized(input);
        if (samples.isEmpty()) {
            return List.of();
        }
        List<Sample<Boolean>> result = new ArrayList<>(samples.size());
        for (int i = 0; i < samples.size(); i++) {
            Sample<Double> sample = samples.get(i);
            if (i > 0) {
                long previous = samples.get(i - 1).timestampMicros();
                long current = sample.timestampMicros();
                if (current - previous > SeriesMath.MAX_CONTINUOUS_GAP_MICROS) {
                    result.add(new Sample<>(previous + 1L, false));
                    result.add(new Sample<>(current - 1L, false));
                }
            }
            result.add(
                    new Sample<>(
                            sample.timestampMicros(),
                            predicate.test(sample.value())));
        }
        return List.copyOf(result);
    }

    private static List<Sample<Boolean>> condition(List<Long> times, LongPredicate predicate) {
        return times.stream().map(time -> new Sample<>(time, predicate.test(time))).toList();
    }

    private static void addTimes(TreeSet<Long> destination, List<Sample<Double>> samples) {
        for (int i = 0; i < samples.size(); i++) {
            Sample<Double> sample = samples.get(i);
            if (i > 0) {
                long previous = samples.get(i - 1).timestampMicros();
                long current = sample.timestampMicros();
                if (current - previous > SeriesMath.MAX_CONTINUOUS_GAP_MICROS) {
                    destination.add(previous + 1L);
                    destination.add(current - 1L);
                }
            }
            destination.add(sample.timestampMicros());
        }
    }

    private static void addFreshnessBoundaries(
            TreeSet<Long> destination,
            List<Sample<Double>> samples,
            long maximumAgeMicros,
            long timelineEndMicros) {
        for (Sample<Double> sample : samples) {
            long timestamp = sample.timestampMicros();
            long freshThrough = timestamp > Long.MAX_VALUE - maximumAgeMicros
                    ? Long.MAX_VALUE
                    : timestamp + maximumAgeMicros;
            if (freshThrough <= timelineEndMicros) {
                destination.add(freshThrough);
            }
            if (freshThrough < Long.MAX_VALUE && freshThrough + 1L <= timelineEndMicros) {
                // Freshness is inclusive at freshThrough; the first stale instant is one
                // microsecond later because log timestamps use integer microseconds.
                destination.add(freshThrough + 1L);
            }
        }
    }

    private static List<List<Sample<Double>>> continuousSegments(
            List<Sample<Double>> input) {
        List<Sample<Double>> samples = SeriesMath.normalized(input);
        if (samples.isEmpty()) {
            return List.of();
        }
        List<List<Sample<Double>>> segments = new ArrayList<>();
        int segmentStart = 0;
        for (int i = 1; i < samples.size(); i++) {
            if (samples.get(i).timestampMicros()
                            - samples.get(i - 1).timestampMicros()
                    > SeriesMath.MAX_CONTINUOUS_GAP_MICROS) {
                segments.add(List.copyOf(samples.subList(segmentStart, i)));
                segmentStart = i;
            }
        }
        segments.add(List.copyOf(samples.subList(segmentStart, samples.size())));
        return List.copyOf(segments);
    }

    private static List<TimeRange> commonCoverageSegments(
            Collection<List<Sample<Double>>> inputs) {
        List<List<TimeRange>> coverage = inputs.stream()
                .map(RuleEngine::coverageSegments)
                .toList();
        if (coverage.isEmpty() || coverage.stream().anyMatch(List::isEmpty)) {
            return List.of();
        }
        List<TimeRange> common = coverage.get(0);
        for (int i = 1; i < coverage.size() && !common.isEmpty(); i++) {
            List<TimeRange> intersections = new ArrayList<>();
            for (TimeRange left : common) {
                for (TimeRange right : coverage.get(i)) {
                    long start = Math.max(left.startMicros(), right.startMicros());
                    long end = Math.min(left.endMicros(), right.endMicros());
                    if (start <= end) {
                        intersections.add(new TimeRange(start, end));
                    }
                }
            }
            common = List.copyOf(intersections);
        }
        return List.copyOf(common);
    }

    private static List<TimeRange> coverageSegments(List<Sample<Double>> input) {
        return continuousSegments(input).stream()
                .map(
                        segment ->
                                new TimeRange(
                                        segment.get(0).timestampMicros(),
                                        segment.get(segment.size() - 1).timestampMicros()))
                .toList();
    }

    private static List<Long> timelineWithin(
            Collection<List<Sample<Double>>> inputs, TimeRange coverage) {
        TreeSet<Long> times = new TreeSet<>();
        times.add(coverage.startMicros());
        times.add(coverage.endMicros());
        for (List<Sample<Double>> input : inputs) {
            for (Sample<Double> sample : input) {
                if (sample.timestampMicros() >= coverage.startMicros()
                        && sample.timestampMicros() <= coverage.endMicros()) {
                    times.add(sample.timestampMicros());
                }
            }
        }
        return List.copyOf(times);
    }

    private static boolean isStale(
            List<Sample<Double>> samples, long timestampMicros, long maximumAgeMicros) {
        if (Double.isFinite(SeriesMath.valueAt(samples, timestampMicros))) {
            // A real sample on each side provides continuous offline coverage; freshness
            // applies only where analysis would otherwise hold a previous sample.
            return false;
        }
        int index = latestIndexAtOrBefore(samples, timestampMicros);
        if (index < 0) {
            return false;
        }
        return timestampMicros - samples.get(index).timestampMicros()
                > maximumAgeMicros;
    }

    private static boolean hasDoubleAt(
            List<Sample<Double>> samples, long timestampMicros) {
        return latestIndexAtOrBefore(samples, timestampMicros) >= 0;
    }

    private static double freshValueAt(
            List<Sample<Double>> samples,
            long timestampMicros,
            long maximumAgeMicros) {
        double interpolated = SeriesMath.valueAt(samples, timestampMicros);
        if (Double.isFinite(interpolated)) {
            return interpolated;
        }
        int index = latestIndexAtOrBefore(samples, timestampMicros);
        if (index < 0
                || timestampMicros - samples.get(index).timestampMicros()
                        > maximumAgeMicros) {
            return Double.NaN;
        }
        return samples.get(index).value();
    }

    private static boolean hasBooleanAt(
            List<Sample<Boolean>> samples, long timestampMicros) {
        return !samples.isEmpty()
                && samples.get(0).timestampMicros() <= timestampMicros;
    }

    private static void addIntervals(
            List<Anomaly> findings,
            RuleType type,
            RuleConfig config,
            List<Sample<Boolean>> condition,
            long logEndMicros,
            String subsystem,
            String motor,
            String reason,
            String recommendation,
            Map<String, Double> observed) {
        long minimumMicros = Math.round(config.durationSeconds() * 1_000_000.0);
        for (TimeRange range : trueIntervals(condition, logEndMicros)) {
            if (range.endMicros() - range.startMicros() >= minimumMicros) {
                findings.add(new Anomaly(
                        type,
                        config.severity(),
                        range.startMicros(),
                        range.endMicros(),
                        subsystem,
                        motor,
                        reason,
                        recommendation,
                        observed));
            }
        }
    }

    static List<TimeRange> trueIntervals(List<Sample<Boolean>> input, long logEndMicros) {
        if (input == null || input.isEmpty()) {
            return List.of();
        }
        List<Sample<Boolean>> samples = input.stream()
                .filter(sample -> sample != null && sample.value() != null)
                .sorted(Comparator.comparingLong(Sample::timestampMicros))
                .toList();
        List<TimeRange> ranges = new ArrayList<>();
        Long start = null;
        for (Sample<Boolean> sample : samples) {
            if (sample.value() && start == null) {
                start = sample.timestampMicros();
            } else if (!sample.value() && start != null) {
                ranges.add(new TimeRange(start, sample.timestampMicros()));
                start = null;
            }
        }
        if (start != null) {
            ranges.add(new TimeRange(start, Math.max(start, logEndMicros)));
        }
        return List.copyOf(ranges);
    }

    private static boolean latestBoolean(List<Sample<Boolean>> values, long timestampMicros) {
        int index = latestIndexAtOrBefore(values, timestampMicros);
        return index >= 0 && values.get(index).value();
    }

    private static String latestString(List<Sample<String>> values, long timestampMicros) {
        int index = latestIndexAtOrBefore(values, timestampMicros);
        return index < 0 ? "" : values.get(index).value();
    }

    private static <T> int latestIndexAtOrBefore(
            List<Sample<T>> samples, long timestampMicros) {
        int low = 0;
        int high = samples.size() - 1;
        int result = -1;
        while (low <= high) {
            int middle = (low + high) >>> 1;
            if (samples.get(middle).timestampMicros() <= timestampMicros) {
                result = middle;
                low = middle + 1;
            } else {
                high = middle - 1;
            }
        }
        return result;
    }

    record TimeRange(long startMicros, long endMicros) {}

    private record BrownoutIncident(
            long startMicros,
            long endMicros,
            boolean booleanReported,
            Long eventTimestampMicros) {}

    private static final class MutableBrownoutIncident {
        private final long startMicros;
        private final long endMicros;
        private final boolean booleanReported;
        private Long eventTimestampMicros;

        private MutableBrownoutIncident(
                long startMicros,
                long endMicros,
                boolean booleanReported,
                Long eventTimestampMicros) {
            this.startMicros = startMicros;
            this.endMicros = endMicros;
            this.booleanReported = booleanReported;
            this.eventTimestampMicros = eventTimestampMicros;
        }
    }

    private record MotorIdentity(String subsystem, String canBus, long deviceId) {}
}
