package frc.robot.health.analyzer.analysis;

import frc.robot.health.analyzer.model.Sample;
import java.util.List;

/** Battery-side or motor-side current metrics; {@code currentKind} prevents unit conflation. */
public record CurrentStatistics(
        String currentKind,
        double averageAmps,
        double rawPeakAmps,
        double rolling100msPeakAmps,
        double rolling500msPeakAmps,
        double p95Amps,
        double p99Amps,
        double secondsAboveThreshold,
        double ampHours) {

    public static CurrentStatistics of(
            String currentKind, List<Sample<Double>> samples, double thresholdAmps) {
        return of(currentKind, samples, thresholdAmps, true);
    }

    /**
     * Computes current statistics for a selected range. Sample-only statistics remain available
     * when the series does not cover the range, while metrics that imply continuous time coverage
     * are unavailable.
     */
    public static CurrentStatistics of(
            String currentKind,
            List<Sample<Double>> samples,
            double thresholdAmps,
            long rangeStartMicros,
            long rangeEndMicros) {
        return of(
                currentKind,
                samples,
                thresholdAmps,
                TelemetryCoverage.timelyAndContinuous(
                        samples, rangeStartMicros, rangeEndMicros));
    }

    private static CurrentStatistics of(
            String currentKind,
            List<Sample<Double>> input,
            double thresholdAmps,
            boolean completeTimeCoverage) {
        List<Sample<Double>> samples = SeriesMath.normalized(input);
        if (samples.isEmpty()) {
            return unavailable(currentKind);
        }
        List<Sample<Double>> magnitudes =
                samples.stream()
                        .map(
                                sample ->
                                        new Sample<>(
                                                sample.timestampMicros(),
                                                Math.abs(sample.value())))
                        .toList();
        boolean batterySide =
                currentKind != null
                        && (currentKind.contains("Supply")
                                || currentKind.contains("PDH"));
        return new CurrentStatistics(
                currentKind,
                completeTimeCoverage
                        ? SeriesMath.timeWeightedMean(magnitudes)
                        : Double.NaN,
                SeriesMath.max(magnitudes),
                completeTimeCoverage
                        ? SeriesMath.rollingAveragePeak(magnitudes, 100_000)
                        : Double.NaN,
                completeTimeCoverage
                        ? SeriesMath.rollingAveragePeak(magnitudes, 500_000)
                        : Double.NaN,
                SeriesMath.percentile(magnitudes, 0.95),
                SeriesMath.percentile(magnitudes, 0.99),
                completeTimeCoverage
                        ? SeriesMath.durationAbove(
                                magnitudes, Math.abs(thresholdAmps))
                        : Double.NaN,
                batterySide && completeTimeCoverage
                        ? SeriesMath.ampHours(magnitudes)
                        : Double.NaN);
    }

    public static CurrentStatistics unavailable(String currentKind) {
        return new CurrentStatistics(
                currentKind,
                Double.NaN,
                Double.NaN,
                Double.NaN,
                Double.NaN,
                Double.NaN,
                Double.NaN,
                Double.NaN,
                Double.NaN);
    }
}
