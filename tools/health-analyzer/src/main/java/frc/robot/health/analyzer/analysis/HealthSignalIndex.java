package frc.robot.health.analyzer.analysis;

import frc.robot.health.analyzer.model.HealthLog;
import frc.robot.health.analyzer.model.Sample;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/** Parsed view of the stable /Health path convention. */
public final class HealthSignalIndex {
    private static final Pattern MOTOR =
            Pattern.compile("^/Health/Motors/([^/]+)/([^/]+)/([^/]+)$");
    private static final Pattern SUBSYSTEM =
            Pattern.compile("^/Health/Subsystems/([^/]+)/([^/]+)$");

    private final Map<MotorKey, Map<String, List<Sample<Double>>>> motors = new LinkedHashMap<>();
    private final Map<String, Map<String, List<Sample<Double>>>> subsystems = new LinkedHashMap<>();

    public HealthSignalIndex(HealthLog log) {
        for (String path : log.signalPaths()) {
            Matcher motor = MOTOR.matcher(path);
            if (motor.matches()) {
                motors.computeIfAbsent(
                        new MotorKey(motor.group(1), motor.group(2)),
                        ignored -> new LinkedHashMap<>());
                continue;
            }
            Matcher subsystem = SUBSYSTEM.matcher(path);
            if (subsystem.matches()) {
                subsystems.computeIfAbsent(
                        subsystem.group(1), ignored -> new LinkedHashMap<>());
            }
        }
        for (Map.Entry<String, List<Sample<Double>>> entry : log.doubleSignals().entrySet()) {
            Matcher motor = MOTOR.matcher(entry.getKey());
            if (motor.matches()) {
                motors.computeIfAbsent(new MotorKey(motor.group(1), motor.group(2)), ignored -> new LinkedHashMap<>())
                        .put(motor.group(3), entry.getValue());
                continue;
            }
            Matcher subsystem = SUBSYSTEM.matcher(entry.getKey());
            if (subsystem.matches()) {
                subsystems.computeIfAbsent(subsystem.group(1), ignored -> new LinkedHashMap<>())
                        .put(subsystem.group(2), entry.getValue());
            }
        }
    }

    public Map<MotorKey, Map<String, List<Sample<Double>>>> motors() {
        return motors;
    }

    public Map<String, Map<String, List<Sample<Double>>>> subsystems() {
        return subsystems;
    }

    public List<Sample<Double>> motor(MotorKey key, String signal) {
        return Optional.ofNullable(motors.get(key))
                .map(m -> m.get(signal))
                .orElse(List.of());
    }

    public List<Sample<Double>> subsystem(String subsystem, String signal) {
        return Optional.ofNullable(subsystems.get(subsystem))
                .map(m -> m.get(signal))
                .orElse(List.of());
    }

    public record MotorKey(String subsystem, String motor) {}
}
