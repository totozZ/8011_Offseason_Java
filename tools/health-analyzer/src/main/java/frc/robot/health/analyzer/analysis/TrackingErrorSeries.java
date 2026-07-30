package frc.robot.health.analyzer.analysis;

import frc.robot.health.analyzer.analysis.HealthSignalIndex.MotorKey;
import frc.robot.health.analyzer.model.HealthLog;
import frc.robot.health.analyzer.model.Sample;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.OptionalDouble;
import java.util.TreeSet;

/**
 * A tracking-error timeline whose values exist only while the subsystem is active and the motor
 * is in a compatible closed-loop control mode.
 *
 * <p>Missing Active telemetry is treated as an older-log compatibility case. Direct
 * ClosedLoopError telemetry is also accepted when ControlMode was not logged. When either state
 * series does exist, timestamps before its first sample are unknown rather than implicitly false
 * or active.
 */
public final class TrackingErrorSeries {
    private final List<TimedError> points;
    private final double meanAbsolute;
    private final double maximumAbsolute;
    private final long endMicros;

    private TrackingErrorSeries(
            List<TimedError> points,
            double meanAbsolute,
            double maximumAbsolute,
            long endMicros) {
        this.points = List.copyOf(points);
        this.meanAbsolute = meanAbsolute;
        this.maximumAbsolute = maximumAbsolute;
        this.endMicros = endMicros;
    }

    public static TrackingErrorSeries motor(
            HealthLog log, HealthSignalIndex index, MotorKey key) {
        List<Sample<Boolean>> active =
                log.booleanSeries("/Health/Subsystems/" + key.subsystem() + "/Active");
        List<Sample<String>> modes =
                log.stringSeries(
                        "/Health/Motors/%s/%s/ControlMode"
                                .formatted(key.subsystem(), key.motor()));
        List<Sample<Double>> direct =
                SeriesMath.normalized(index.motor(key, "ClosedLoopError"));
        if (!direct.isEmpty()) {
            return build(
                    List.of(direct),
                    active,
                    modes,
                    false,
                    (timestampMicros, ignored) ->
                            absoluteValueWithinCoverage(direct, timestampMicros));
        }

        List<Sample<Double>> reference =
                SeriesMath.normalized(index.motor(key, "Reference"));
        List<Sample<Double>> position =
                SeriesMath.normalized(index.motor(key, "Position"));
        List<Sample<Double>> velocity =
                SeriesMath.normalized(index.motor(key, "Velocity"));
        if (reference.isEmpty() || modes.isEmpty()) {
            return unavailable();
        }
        return build(
                List.of(reference, position, velocity),
                active,
                modes,
                true,
                (timestampMicros, mode) -> {
                    List<Sample<Double>> measured = measuredSeries(mode, position, velocity);
                    OptionalDouble target = rawValueWithinCoverage(reference, timestampMicros);
                    OptionalDouble actual = rawValueWithinCoverage(measured, timestampMicros);
                    return target.isPresent() && actual.isPresent()
                            ? OptionalDouble.of(
                                    Math.abs(target.getAsDouble() - actual.getAsDouble()))
                            : OptionalDouble.empty();
                });
    }

    public static TrackingErrorSeries subsystem(
            HealthLog log, HealthSignalIndex index, String subsystem) {
        List<Sample<Boolean>> active =
                log.booleanSeries("/Health/Subsystems/" + subsystem + "/Active");
        List<Sample<Double>> direct =
                SeriesMath.normalized(index.subsystem(subsystem, "ClosedLoopError"));
        if (!direct.isEmpty()) {
            return build(
                    List.of(direct),
                    active,
                    List.of(),
                    false,
                    (timestampMicros, ignored) ->
                            absoluteValueWithinCoverage(direct, timestampMicros));
        }

        List<Sample<Double>> reference =
                SeriesMath.normalized(index.subsystem(subsystem, "Reference"));
        List<Sample<Double>> measured =
                SeriesMath.normalized(index.subsystem(subsystem, "Measured"));
        if (reference.isEmpty() || measured.isEmpty()) {
            return unavailable();
        }
        return build(
                List.of(reference, measured),
                active,
                List.of(),
                false,
                (timestampMicros, ignored) -> {
                    OptionalDouble target = rawValueWithinCoverage(reference, timestampMicros);
                    OptionalDouble actual = rawValueWithinCoverage(measured, timestampMicros);
                    return target.isPresent() && actual.isPresent()
                            ? OptionalDouble.of(
                                    Math.abs(target.getAsDouble() - actual.getAsDouble()))
                            : OptionalDouble.empty();
                });
    }

    public boolean available() {
        return Double.isFinite(maximumAbsolute);
    }

    public double meanAbsolute() {
        return meanAbsolute;
    }

    public double maximumAbsolute() {
        return maximumAbsolute;
    }

    /** Returns a state timeline suitable for duration-based rule evaluation. */
    public List<Sample<Boolean>> conditionAbove(double threshold) {
        if (points.isEmpty()) {
            return List.of();
        }
        return points.stream()
                .map(
                        point ->
                                new Sample<>(
                                        point.timestampMicros(),
                                        point.error().isPresent()
                                                && point.error().getAsDouble()
                                                        > Math.abs(threshold)))
                .toList();
    }

    public long endMicros() {
        return endMicros;
    }

    private static TrackingErrorSeries build(
            List<List<Sample<Double>>> numericInputs,
            List<Sample<Boolean>> active,
            List<Sample<String>> modes,
            boolean modeRequired,
            ErrorProvider provider) {
        List<List<Sample<Double>>> numeric =
                numericInputs.stream().filter(series -> !series.isEmpty()).toList();
        if (numeric.isEmpty()) {
            return unavailable();
        }

        long first =
                numeric.stream()
                        .mapToLong(series -> series.get(0).timestampMicros())
                        .min()
                        .orElseThrow();
        long last =
                numeric.stream()
                        .mapToLong(series -> series.get(series.size() - 1).timestampMicros())
                        .max()
                        .orElseThrow();
        TreeSet<Long> timeline = new TreeSet<>();
        numeric.forEach(
                series ->
                        series.forEach(sample -> timeline.add(sample.timestampMicros())));
        numeric.forEach(series -> addGapBoundaries(timeline, series));
        active.stream()
                .mapToLong(Sample::timestampMicros)
                .filter(timestamp -> timestamp >= first && timestamp <= last)
                .forEach(timeline::add);
        modes.stream()
                .mapToLong(Sample::timestampMicros)
                .filter(timestamp -> timestamp >= first && timestamp <= last)
                .forEach(timeline::add);
        timeline.add(first);
        timeline.add(last);

        List<Long> times = List.copyOf(timeline);
        List<TimedError> points = new ArrayList<>(times.size());
        double maximum = Double.NaN;
        for (long time : times) {
            OptionalDouble error =
                    gatedValueAt(time, active, modes, modeRequired, provider);
            points.add(new TimedError(time, error));
            if (error.isPresent()) {
                maximum =
                        Double.isFinite(maximum)
                                ? Math.max(maximum, error.getAsDouble())
                                : error.getAsDouble();
            }
        }

        double integral = 0.0;
        long durationMicros = 0L;
        for (int i = 1; i < times.size(); i++) {
            long a = times.get(i - 1);
            long b = times.get(i);
            if (b <= a || !activeAt(active, a)) {
                continue;
            }
            String mode = latestMode(modes, a);
            if (!modeAllowed(modes, modeRequired, mode)) {
                continue;
            }
            OptionalDouble firstValue = provider.valueAt(a, mode);
            OptionalDouble lastValue = provider.valueAt(b, mode);
            OptionalDouble middleValue =
                    provider.valueAt(a + (b - a) / 2L, mode);
            if (firstValue.isEmpty()
                    || middleValue.isEmpty()
                    || lastValue.isEmpty()) {
                continue;
            }
            long intervalMicros = b - a;
            integral +=
                    (firstValue.getAsDouble() + lastValue.getAsDouble())
                            * 0.5
                            * (intervalMicros / 1_000_000.0);
            durationMicros += intervalMicros;
            maximum =
                    maxFinite(maximum, firstValue.getAsDouble(), lastValue.getAsDouble());
        }

        double mean =
                durationMicros > 0
                        ? integral / (durationMicros / 1_000_000.0)
                        : Double.NaN;
        return new TrackingErrorSeries(points, mean, maximum, last);
    }

    private static OptionalDouble gatedValueAt(
            long timestampMicros,
            List<Sample<Boolean>> active,
            List<Sample<String>> modes,
            boolean modeRequired,
            ErrorProvider provider) {
        if (!activeAt(active, timestampMicros)) {
            return OptionalDouble.empty();
        }
        String mode = latestMode(modes, timestampMicros);
        if (!modeAllowed(modes, modeRequired, mode)) {
            return OptionalDouble.empty();
        }
        return provider.valueAt(timestampMicros, mode);
    }

    private static boolean activeAt(
            List<Sample<Boolean>> active, long timestampMicros) {
        if (active.isEmpty()) {
            return true;
        }
        int index = latestIndexAtOrBefore(active, timestampMicros);
        return index >= 0 && active.get(index).value();
    }

    private static boolean modeAllowed(
            List<Sample<String>> modes, boolean modeRequired, String mode) {
        if (modes.isEmpty()) {
            return !modeRequired;
        }
        return isClosedLoopMode(mode);
    }

    private static String latestMode(
            List<Sample<String>> modes, long timestampMicros) {
        int index = latestIndexAtOrBefore(modes, timestampMicros);
        return index < 0 ? "" : modes.get(index).value();
    }

    private static List<Sample<Double>> measuredSeries(
            String mode,
            List<Sample<Double>> position,
            List<Sample<Double>> velocity) {
        String normalized = normalizeMode(mode);
        if (normalized.contains("velocity")) {
            return velocity;
        }
        if (normalized.contains("position") || normalized.contains("motionmagic")) {
            return position;
        }
        return List.of();
    }

    static boolean isClosedLoopMode(String mode) {
        String normalized = normalizeMode(mode);
        return normalized.contains("velocity")
                || normalized.contains("position")
                || normalized.contains("motionmagic");
    }

    private static String normalizeMode(String mode) {
        return mode == null ? "" : mode.trim().toLowerCase(Locale.ROOT);
    }

    private static OptionalDouble absoluteValueWithinCoverage(
            List<Sample<Double>> values, long timestampMicros) {
        OptionalDouble value = rawValueWithinCoverage(values, timestampMicros);
        return value.isPresent()
                ? OptionalDouble.of(Math.abs(value.getAsDouble()))
                : OptionalDouble.empty();
    }

    private static OptionalDouble rawValueWithinCoverage(
            List<Sample<Double>> values, long timestampMicros) {
        if (values.isEmpty()
                || timestampMicros < values.get(0).timestampMicros()
                || timestampMicros
                        > values.get(values.size() - 1).timestampMicros()) {
            return OptionalDouble.empty();
        }
        double value = SeriesMath.valueAt(values, timestampMicros);
        return Double.isFinite(value)
                ? OptionalDouble.of(value)
                : OptionalDouble.empty();
    }

    private static void addGapBoundaries(
            TreeSet<Long> timeline, List<Sample<Double>> values) {
        for (int i = 1; i < values.size(); i++) {
            long left = values.get(i - 1).timestampMicros();
            long right = values.get(i).timestampMicros();
            if (right - left > SeriesMath.MAX_CONTINUOUS_GAP_MICROS) {
                timeline.add(left + 1L);
                timeline.add(right - 1L);
            }
        }
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

    private static double maxFinite(double current, double... values) {
        double result = current;
        for (double value : values) {
            result = Double.isFinite(result) ? Math.max(result, value) : value;
        }
        return result;
    }

    private static TrackingErrorSeries unavailable() {
        return new TrackingErrorSeries(
                List.of(), Double.NaN, Double.NaN, 0L);
    }

    @FunctionalInterface
    private interface ErrorProvider {
        OptionalDouble valueAt(long timestampMicros, String mode);
    }

    private record TimedError(long timestampMicros, OptionalDouble error) {}
}
