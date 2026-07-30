package frc.robot.health.analyzer.analysis;

import frc.robot.health.analyzer.model.Sample;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeSet;

/**
 * Timestamp-aware numerical operations. Times are microseconds and values are piecewise linear
 * only while continuous telemetry coverage exists.
 */
public final class SeriesMath {
    private static final double MICROS_PER_SECOND = 1_000_000.0;

    /**
     * Largest interval treated as continuous numeric telemetry.
     *
     * <p>One second spans five sample periods from the logger's slowest default continuous stream
     * (5 Hz). A larger interval is a data gap, not evidence that the value changed linearly or
     * remained constant.
     */
    public static final long MAX_CONTINUOUS_GAP_MICROS = 1_000_000L;

    private SeriesMath() {}

    public static List<Sample<Double>> normalized(List<Sample<Double>> input) {
        if (input == null || input.isEmpty()) {
            return List.of();
        }
        List<Sample<Double>> sorted = input.stream()
                .filter(s -> s != null && s.value() != null && Double.isFinite(s.value()))
                .sorted(Comparator.comparingLong(Sample::timestampMicros))
                .toList();
        Map<Long, Sample<Double>> unique = new LinkedHashMap<>();
        for (Sample<Double> sample : sorted) {
            unique.put(sample.timestampMicros(), sample);
        }
        return List.copyOf(unique.values());
    }

    /** Integral in value-seconds using actual timestamps and the trapezoid rule. */
    public static double integrate(List<Sample<Double>> input) {
        List<Sample<Double>> samples = normalized(input);
        if (samples.size() < 2 || !hasContinuousCoverageNormalized(samples)) {
            return Double.NaN;
        }
        double result = 0.0;
        for (int i = 1; i < samples.size(); i++) {
            Sample<Double> a = samples.get(i - 1);
            Sample<Double> b = samples.get(i);
            double dt = (b.timestampMicros() - a.timestampMicros()) / MICROS_PER_SECOND;
            result += (a.value() + b.value()) * 0.5 * dt;
        }
        return result;
    }

    public static double timeWeightedMean(List<Sample<Double>> input) {
        List<Sample<Double>> samples = normalized(input);
        if (samples.isEmpty()) {
            return Double.NaN;
        }
        if (samples.size() == 1) {
            return samples.get(0).value();
        }
        if (!hasContinuousCoverageNormalized(samples)) {
            return Double.NaN;
        }
        double seconds = (samples.get(samples.size() - 1).timestampMicros()
                        - samples.get(0).timestampMicros())
                / MICROS_PER_SECOND;
        return seconds <= 0.0 ? samples.get(samples.size() - 1).value() : integrate(samples) / seconds;
    }

    public static double min(List<Sample<Double>> input) {
        return normalized(input).stream().mapToDouble(Sample::value).min().orElse(Double.NaN);
    }

    /** First finite value in timestamp order. */
    public static double first(List<Sample<Double>> input) {
        List<Sample<Double>> samples = normalized(input);
        return samples.isEmpty() ? Double.NaN : samples.get(0).value();
    }

    public static double max(List<Sample<Double>> input) {
        return normalized(input).stream().mapToDouble(Sample::value).max().orElse(Double.NaN);
    }

    /** Linear-interpolated sample percentile (R-7/NumPy default convention). */
    public static double percentile(List<Sample<Double>> input, double probability) {
        if (probability < 0.0 || probability > 1.0) {
            throw new IllegalArgumentException("probability must be in [0,1]");
        }
        double[] values = normalized(input).stream().mapToDouble(Sample::value).sorted().toArray();
        if (values.length == 0) {
            return Double.NaN;
        }
        double index = probability * (values.length - 1);
        int lower = (int) Math.floor(index);
        int upper = (int) Math.ceil(index);
        if (lower == upper) {
            return values[lower];
        }
        double fraction = index - lower;
        return values[lower] + (values[upper] - values[lower]) * fraction;
    }

    /**
     * Peak trailing rolling mean evaluated at input timestamps. A full window is used when
     * available; for a recording shorter than the window, the whole recording mean is returned.
     */
    public static double rollingAveragePeak(List<Sample<Double>> input, long windowMicros) {
        if (windowMicros <= 0) {
            throw new IllegalArgumentException("windowMicros must be positive");
        }
        List<Sample<Double>> samples = normalized(input);
        if (samples.isEmpty()) {
            return Double.NaN;
        }
        if (samples.size() == 1) {
            return samples.get(0).value();
        }
        if (!hasContinuousCoverageNormalized(samples)) {
            return Double.NaN;
        }
        IntegralLookup integral = new IntegralLookup(samples);
        long first = samples.get(0).timestampMicros();
        double peak = Double.NEGATIVE_INFINITY;
        boolean hadFullWindow = false;
        for (Sample<Double> sample : samples) {
            long end = sample.timestampMicros();
            if (end - first < windowMicros) {
                continue;
            }
            hadFullWindow = true;
            double windowIntegral = integral.between(end - windowMicros, end);
            peak = Math.max(peak, windowIntegral / (windowMicros / MICROS_PER_SECOND));
        }
        return hadFullWindow ? peak : timeWeightedMean(samples);
    }

    public static List<Sample<Double>> trailingRollingAverage(
            List<Sample<Double>> input, long windowMicros) {
        if (windowMicros <= 0) {
            throw new IllegalArgumentException("windowMicros must be positive");
        }
        List<Sample<Double>> samples = normalized(input);
        if (samples.isEmpty()) {
            return List.of();
        }
        if (!hasContinuousCoverageNormalized(samples)) {
            return List.of();
        }
        IntegralLookup integral = new IntegralLookup(samples);
        List<Sample<Double>> result = new ArrayList<>();
        long first = samples.get(0).timestampMicros();
        for (Sample<Double> sample : samples) {
            long end = sample.timestampMicros();
            long start = Math.max(first, end - windowMicros);
            if (end == start) {
                result.add(new Sample<>(end, sample.value()));
            } else {
                double average = integral.between(start, end)
                        / ((end - start) / MICROS_PER_SECOND);
                result.add(new Sample<>(end, average));
            }
        }
        return List.copyOf(result);
    }

    /** Exact duration above a threshold for linearly interpolated samples. */
    public static double durationAbove(List<Sample<Double>> input, double threshold) {
        List<Sample<Double>> samples = normalized(input);
        if (samples.size() < 2) {
            return Double.NaN;
        }
        if (!hasContinuousCoverageNormalized(samples)) {
            return Double.NaN;
        }
        double micros = 0.0;
        for (int i = 1; i < samples.size(); i++) {
            Sample<Double> a = samples.get(i - 1);
            Sample<Double> b = samples.get(i);
            double dt = b.timestampMicros() - a.timestampMicros();
            double y0 = a.value() - threshold;
            double y1 = b.value() - threshold;
            if (y0 > 0.0 && y1 > 0.0) {
                micros += dt;
            } else if (y0 > 0.0 && y1 <= 0.0) {
                micros += dt * y0 / (y0 - y1);
            } else if (y0 <= 0.0 && y1 > 0.0) {
                micros += dt * y1 / (y1 - y0);
            }
        }
        return micros / MICROS_PER_SECOND;
    }

    /** Step-held duration of a boolean state. */
    public static double trueDuration(
            List<Sample<Boolean>> input, long rangeStartMicros, long rangeEndMicros) {
        if (input == null || input.isEmpty() || rangeEndMicros <= rangeStartMicros) {
            return 0.0;
        }
        List<Sample<Boolean>> sorted = input.stream()
                .filter(s -> s != null && s.value() != null)
                .sorted(Comparator.comparingLong(Sample::timestampMicros))
                .toList();
        boolean state = false;
        int index = 0;
        while (index < sorted.size() && sorted.get(index).timestampMicros() <= rangeStartMicros) {
            state = sorted.get(index++).value();
        }
        long cursor = rangeStartMicros;
        long trueMicros = 0L;
        while (index < sorted.size() && sorted.get(index).timestampMicros() < rangeEndMicros) {
            long next = Math.max(cursor, sorted.get(index).timestampMicros());
            if (state) {
                trueMicros += next - cursor;
            }
            cursor = next;
            state = sorted.get(index++).value();
        }
        if (state) {
            trueMicros += rangeEndMicros - cursor;
        }
        return trueMicros / MICROS_PER_SECOND;
    }

    /** Ah from battery-side current. */
    public static double ampHours(List<Sample<Double>> current) {
        List<Sample<Double>> samples = normalized(current);
        return samples.size() < 2 ? Double.NaN : integrate(samples) / 3600.0;
    }

    /**
     * Wh using a merged voltage/current timestamp grid and trapezoidal integration of V*I.
     */
    public static double wattHours(
            List<Sample<Double>> voltageInput, List<Sample<Double>> currentInput) {
        List<Sample<Double>> voltage = normalized(voltageInput);
        List<Sample<Double>> current = normalized(currentInput);
        current =
                current.stream()
                        .map(
                                sample ->
                                        new Sample<>(
                                                sample.timestampMicros(),
                                                Math.abs(sample.value())))
                        .toList();
        List<Long> times = commonTimeline(List.of(voltage, current));
        if (times.size() < 2) {
            return Double.NaN;
        }
        double wattSeconds = 0.0;
        long previousTime = times.get(0);
        double previousPower = valueAt(voltage, previousTime) * valueAt(current, previousTime);
        for (int i = 1; i < times.size(); i++) {
            long time = times.get(i);
            double power = valueAt(voltage, time) * valueAt(current, time);
            wattSeconds += (previousPower + power)
                    * 0.5
                    * ((time - previousTime) / MICROS_PER_SECOND);
            previousTime = time;
            previousPower = power;
        }
        return wattSeconds / 3600.0;
    }

    public static double maxPositiveSlopePerSecond(List<Sample<Double>> input) {
        List<Sample<Double>> samples = normalized(input);
        if (samples.size() < 2) {
            return Double.NaN;
        }
        if (!hasContinuousCoverageNormalized(samples)) {
            return Double.NaN;
        }
        double maxSlope = 0.0;
        for (int i = 1; i < samples.size(); i++) {
            double dt = (samples.get(i).timestampMicros() - samples.get(i - 1).timestampMicros())
                    / MICROS_PER_SECOND;
            if (dt > 0.0) {
                maxSlope = Math.max(maxSlope, (samples.get(i).value() - samples.get(i - 1).value()) / dt);
            }
        }
        return maxSlope;
    }

    public static List<Sample<Double>> absoluteError(
            List<Sample<Double>> referenceInput, List<Sample<Double>> measuredInput) {
        List<Sample<Double>> reference = normalized(referenceInput);
        List<Sample<Double>> measured = normalized(measuredInput);
        List<Sample<Double>> result = new ArrayList<>();
        for (long time : commonTimeline(List.of(reference, measured))) {
            result.add(new Sample<>(
                    time, Math.abs(valueAt(reference, time) - valueAt(measured, time))));
        }
        return List.copyOf(result);
    }

    public static double highCurrentLowVelocityDuration(
            List<Sample<Double>> currentInput,
            List<Sample<Double>> velocityInput,
            double minimumCurrent,
            double maximumAbsVelocity) {
        List<Sample<Double>> current = normalized(currentInput);
        current =
                current.stream()
                        .map(
                                sample ->
                                        new Sample<>(
                                                sample.timestampMicros(),
                                                Math.abs(sample.value())))
                        .toList();
        List<Sample<Double>> velocity = normalized(velocityInput);
        List<Long> times = commonTimeline(List.of(current, velocity));
        if (times.size() < 2) {
            return Double.NaN;
        }
        double micros = 0.0;
        for (int i = 1; i < times.size(); i++) {
            long t0 = times.get(i - 1);
            long t1 = times.get(i);
            List<FractionRange> currentRange =
                    linearGreaterRanges(valueAt(current, t0), valueAt(current, t1), minimumCurrent);
            List<FractionRange> velocityRange =
                    linearAbsLessRanges(valueAt(velocity, t0), valueAt(velocity, t1), maximumAbsVelocity);
            for (FractionRange a : currentRange) {
                for (FractionRange b : velocityRange) {
                    double start = Math.max(a.start(), b.start());
                    double end = Math.min(a.end(), b.end());
                    if (end > start) {
                        micros += (t1 - t0) * (end - start);
                    }
                }
            }
        }
        return micros / MICROS_PER_SECOND;
    }

    /**
     * Builds an imbalance ratio series: (max current - min current) / max current. Values below
     * minimumCurrent are treated as inactive and yield zero.
     */
    public static List<Sample<Double>> imbalance(
            Collection<List<Sample<Double>>> inputs, double minimumCurrent) {
        List<List<Sample<Double>>> series = inputs.stream()
                .map(SeriesMath::normalized)
                .filter(s -> !s.isEmpty())
                .toList();
        if (series.size() < 2) {
            return List.of();
        }
        List<Sample<Double>> result = new ArrayList<>();
        for (long time : commonTimeline(series)) {
            double min = Double.POSITIVE_INFINITY;
            double max = 0.0;
            for (List<Sample<Double>> values : series) {
                double value = Math.abs(valueAt(values, time));
                min = Math.min(min, value);
                max = Math.max(max, value);
            }
            double ratio = max < minimumCurrent || max == 0.0 ? 0.0 : (max - min) / max;
            result.add(new Sample<>(time, ratio));
        }
        return List.copyOf(result);
    }

    /** Sum simultaneously sampled/interpolated series over their common time range. */
    public static List<Sample<Double>> sum(Collection<List<Sample<Double>>> inputs) {
        List<List<Sample<Double>>> series = inputs.stream()
                .map(SeriesMath::normalized)
                .filter(s -> !s.isEmpty())
                .toList();
        if (series.isEmpty()) {
            return List.of();
        }
        List<Sample<Double>> result = new ArrayList<>();
        for (long time : commonTimeline(series)) {
            double value = 0.0;
            for (List<Sample<Double>> values : series) {
                value += valueAt(values, time);
            }
            result.add(new Sample<>(time, value));
        }
        return List.copyOf(result);
    }

    public static List<Long> timeline(Collection<List<Sample<Double>>> inputs) {
        List<List<Sample<Double>>> series =
                inputs.stream().map(SeriesMath::normalized).filter(s -> !s.isEmpty()).toList();
        return commonTimeline(series);
    }

    /**
     * Returns whether every adjacent numeric sample belongs to one continuous coverage segment.
     *
     * <p>A single finite sample is covered at its timestamp, but it still cannot define an
     * integral or duration.
     */
    public static boolean hasContinuousCoverage(List<Sample<Double>> input) {
        return hasContinuousCoverageNormalized(normalized(input));
    }

    /**
     * Returns whether the complete closed range is covered by finite samples without a telemetry
     * gap. The requested range must lie inside the series' real sample bounds.
     */
    public static boolean hasContinuousCoverage(
            List<Sample<Double>> input, long startMicros, long endMicros) {
        return hasContinuousCoverageNormalized(
                normalized(input), startMicros, endMicros);
    }

    /**
     * Returns whether a previously observed numeric value is still fresh at {@code
     * timestampMicros}. This is intended for cursor snapshots, not interpolation or integration.
     */
    public static boolean hasFreshSampleAt(
            List<Sample<Double>> input, long timestampMicros) {
        List<Sample<Double>> samples = normalized(input);
        if (samples.isEmpty() || timestampMicros < samples.get(0).timestampMicros()) {
            return false;
        }
        int index = floorIndex(samples, timestampMicros);
        return index >= 0
                && timestampMicros - samples.get(index).timestampMicros()
                        <= MAX_CONTINUOUS_GAP_MICROS;
    }

    /**
     * Returns a linearly interpolated value only inside real, continuous sample coverage.
     *
     * <p>Before the first sample, after the last sample, and inside a gap larger than {@link
     * #MAX_CONTINUOUS_GAP_MICROS}, the result is {@link Double#NaN}.
     */
    public static double valueAt(List<Sample<Double>> input, long timestampMicros) {
        List<Sample<Double>> samples = input;
        if (samples.isEmpty()) {
            return Double.NaN;
        }
        if (timestampMicros < samples.get(0).timestampMicros()) {
            return Double.NaN;
        }
        if (timestampMicros == samples.get(0).timestampMicros()) {
            return samples.get(0).value();
        }
        int last = samples.size() - 1;
        if (timestampMicros > samples.get(last).timestampMicros()) {
            return Double.NaN;
        }
        if (timestampMicros == samples.get(last).timestampMicros()) {
            return samples.get(last).value();
        }
        int low = floorIndex(samples, timestampMicros);
        int high = low + 1;
        Sample<Double> a = samples.get(low);
        if (a.timestampMicros() == timestampMicros) {
            return a.value();
        }
        Sample<Double> b = samples.get(high);
        if (!isContinuousInterval(a.timestampMicros(), b.timestampMicros())) {
            return Double.NaN;
        }
        double fraction = (double) (timestampMicros - a.timestampMicros())
                / (b.timestampMicros() - a.timestampMicros());
        return a.value() + (b.value() - a.value()) * fraction;
    }

    private static List<Long> commonTimeline(List<List<Sample<Double>>> series) {
        if (series.isEmpty() || series.stream().anyMatch(List::isEmpty)) {
            return List.of();
        }
        long start = series.stream()
                .mapToLong(s -> s.get(0).timestampMicros())
                .max()
                .orElse(0);
        long end = series.stream()
                .mapToLong(s -> s.get(s.size() - 1).timestampMicros())
                .min()
                .orElse(-1);
        if (end < start) {
            return List.of();
        }
        if (series.stream()
                .anyMatch(values -> !hasContinuousCoverageNormalized(values, start, end))) {
            return List.of();
        }
        TreeSet<Long> times = new TreeSet<>();
        times.add(start);
        times.add(end);
        for (List<Sample<Double>> values : series) {
            for (Sample<Double> sample : values) {
                if (sample.timestampMicros() >= start && sample.timestampMicros() <= end) {
                    times.add(sample.timestampMicros());
                }
            }
        }
        return List.copyOf(times);
    }

    private static boolean hasContinuousCoverageNormalized(
            List<Sample<Double>> samples) {
        if (samples.isEmpty()) {
            return false;
        }
        for (int i = 1; i < samples.size(); i++) {
            if (!isContinuousInterval(
                    samples.get(i - 1).timestampMicros(),
                    samples.get(i).timestampMicros())) {
                return false;
            }
        }
        return true;
    }

    private static boolean hasContinuousCoverageNormalized(
            List<Sample<Double>> samples, long startMicros, long endMicros) {
        if (samples.isEmpty()
                || endMicros < startMicros
                || !Double.isFinite(valueAt(samples, startMicros))
                || !Double.isFinite(valueAt(samples, endMicros))) {
            return false;
        }
        for (int i = 1; i < samples.size(); i++) {
            Sample<Double> a = samples.get(i - 1);
            Sample<Double> b = samples.get(i);
            if (b.timestampMicros() <= startMicros || a.timestampMicros() >= endMicros) {
                continue;
            }
            if (!isContinuousInterval(a.timestampMicros(), b.timestampMicros())) {
                return false;
            }
        }
        return true;
    }

    private static boolean isContinuousInterval(long startMicros, long endMicros) {
        return endMicros >= startMicros
                && endMicros - startMicros <= MAX_CONTINUOUS_GAP_MICROS;
    }

    private static int floorIndex(
            List<Sample<Double>> samples, long timestampMicros) {
        int low = 0;
        int high = samples.size() - 1;
        while (low < high) {
            int middle = (low + high + 1) >>> 1;
            if (samples.get(middle).timestampMicros() <= timestampMicros) {
                low = middle;
            } else {
                high = middle - 1;
            }
        }
        return low;
    }

    private static List<FractionRange> linearGreaterRanges(double start, double end, double threshold) {
        double a = start - threshold;
        double b = end - threshold;
        if (a > 0.0 && b > 0.0) {
            return List.of(new FractionRange(0.0, 1.0));
        }
        if (a <= 0.0 && b <= 0.0) {
            return List.of();
        }
        double crossing = a / (a - b);
        return a > 0.0
                ? List.of(new FractionRange(0.0, crossing))
                : List.of(new FractionRange(crossing, 1.0));
    }

    private static List<FractionRange> linearAbsLessRanges(double start, double end, double threshold) {
        if (threshold < 0.0) {
            return List.of();
        }
        double delta = end - start;
        if (Math.abs(delta) < 1e-12) {
            return Math.abs(start) < threshold
                    ? List.of(new FractionRange(0.0, 1.0))
                    : List.of();
        }
        double atNegative = (-threshold - start) / delta;
        double atPositive = (threshold - start) / delta;
        double lower = Math.max(0.0, Math.min(atNegative, atPositive));
        double upper = Math.min(1.0, Math.max(atNegative, atPositive));
        return upper > lower ? List.of(new FractionRange(lower, upper)) : List.of();
    }

    private record FractionRange(double start, double end) {}

    /** Prefix trapezoid integrals with O(log n) arbitrary-time queries. */
    private static final class IntegralLookup {
        private final List<Sample<Double>> samples;
        private final double[] prefix;

        IntegralLookup(List<Sample<Double>> samples) {
            this.samples = samples;
            prefix = new double[samples.size()];
            for (int i = 1; i < samples.size(); i++) {
                Sample<Double> a = samples.get(i - 1);
                Sample<Double> b = samples.get(i);
                prefix[i] = prefix[i - 1]
                        + (a.value() + b.value())
                                * 0.5
                                * ((b.timestampMicros() - a.timestampMicros())
                                        / MICROS_PER_SECOND);
            }
        }

        double between(long start, long end) {
            return end <= start ? 0.0 : integralTo(end) - integralTo(start);
        }

        private double integralTo(long timestampMicros) {
            if (samples.size() < 2 || timestampMicros <= samples.get(0).timestampMicros()) {
                return 0.0;
            }
            int last = samples.size() - 1;
            if (timestampMicros >= samples.get(last).timestampMicros()) {
                return prefix[last];
            }
            int low = 0;
            int high = last;
            while (low + 1 < high) {
                int mid = (low + high) >>> 1;
                if (samples.get(mid).timestampMicros() <= timestampMicros) {
                    low = mid;
                } else {
                    high = mid;
                }
            }
            Sample<Double> a = samples.get(low);
            double value = valueAt(samples, timestampMicros);
            return prefix[low]
                    + (a.value() + value)
                            * 0.5
                            * ((timestampMicros - a.timestampMicros()) / MICROS_PER_SECOND);
        }
    }
}
