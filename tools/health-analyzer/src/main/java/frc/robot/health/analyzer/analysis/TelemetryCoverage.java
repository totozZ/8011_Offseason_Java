package frc.robot.health.analyzer.analysis;

import frc.robot.health.analyzer.model.Sample;
import java.util.ArrayList;
import java.util.List;

/** Shared completeness checks for periodic numeric telemetry. */
public final class TelemetryCoverage {
    private static final long ROBOT_LOOP_BOUNDARY_GRACE_MICROS = 20_000L;

    private TelemetryCoverage() {}

    /**
     * Returns whether a periodic series continuously covers a selected log range, allowing at
     * most one observed continuous sampling interval plus one normal 20 ms robot loop at either
     * boundary.
     *
     * <p>The tolerance does not extrapolate values. Downstream math still integrates only the
     * common real sample range. Using the largest observed continuous interval plus one robot-loop
     * grace handles session-start ordering and real CAN/scheduler jitter without allowing a
     * high-rate stream to remain "current" for nearly a second merely because the global gap limit
     * is one second.
     */
    public static boolean timelyAndContinuous(
            List<Sample<Double>> input, long rangeStartMicros, long rangeEndMicros) {
        List<Sample<Double>> samples = SeriesMath.normalized(input);
        if (samples.size() < 2 || rangeEndMicros < rangeStartMicros) {
            return false;
        }
        long boundaryToleranceMicros = inferredBoundaryToleranceMicros(samples);
        if (boundaryToleranceMicros <= 0) {
            return false;
        }
        long first = samples.get(0).timestampMicros();
        long last = samples.get(samples.size() - 1).timestampMicros();
        return first <= rangeEndMicros
                && last >= rangeStartMicros
                && first <= saturatedAdd(
                        rangeStartMicros,
                        boundaryToleranceMicros)
                && last
                        >= Math.max(
                                rangeStartMicros,
                                rangeEndMicros
                                        - boundaryToleranceMicros)
                && SeriesMath.hasContinuousCoverage(samples);
    }

    /**
     * Uses the largest actually observed adjacent interval plus one default 20 ms robot loop,
     * capped at the continuity limit. This lets a range boundary tolerate session-start ordering
     * and the scheduler/CAN jitter already present inside the selected series, while a steady
     * high-rate series keeps a small tolerance.
     */
    private static long inferredBoundaryToleranceMicros(List<Sample<Double>> samples) {
        List<Long> intervals = new ArrayList<>(samples.size() - 1);
        for (int i = 1; i < samples.size(); i++) {
            long interval =
                    samples.get(i).timestampMicros()
                            - samples.get(i - 1).timestampMicros();
            if (interval > 0) {
                intervals.add(interval);
            }
        }
        if (intervals.isEmpty()) {
            return 0;
        }
        long maximumObservedInterval =
                intervals.stream().mapToLong(Long::longValue).max().orElse(0L);
        return Math.min(
                saturatedAdd(
                        maximumObservedInterval,
                        ROBOT_LOOP_BOUNDARY_GRACE_MICROS),
                SeriesMath.MAX_CONTINUOUS_GAP_MICROS);
    }

    private static long saturatedAdd(long value, long increment) {
        return value > Long.MAX_VALUE - increment
                ? Long.MAX_VALUE
                : value + increment;
    }
}
