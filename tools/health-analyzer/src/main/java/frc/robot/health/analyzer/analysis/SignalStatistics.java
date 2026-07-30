package frc.robot.health.analyzer.analysis;

import frc.robot.health.analyzer.model.Sample;
import java.util.List;

/** Generic statistics for one numeric signal. */
public record SignalStatistics(
        String path,
        int sampleCount,
        long startMicros,
        long endMicros,
        double minimum,
        double maximum,
        double timeWeightedMean,
        double p95,
        double p99,
        double integralValueSeconds) {

    public static SignalStatistics of(String path, List<Sample<Double>> input) {
        List<Sample<Double>> samples = SeriesMath.normalized(input);
        if (samples.isEmpty()) {
            return unavailable(path);
        }
        return new SignalStatistics(
                path,
                samples.size(),
                samples.get(0).timestampMicros(),
                samples.get(samples.size() - 1).timestampMicros(),
                SeriesMath.min(samples),
                SeriesMath.max(samples),
                SeriesMath.timeWeightedMean(samples),
                SeriesMath.percentile(samples, 0.95),
                SeriesMath.percentile(samples, 0.99),
                SeriesMath.integrate(samples));
    }

    public static SignalStatistics unavailable(String path) {
        return new SignalStatistics(
                path,
                0,
                0,
                0,
                Double.NaN,
                Double.NaN,
                Double.NaN,
                Double.NaN,
                Double.NaN,
                Double.NaN);
    }

    public boolean available() {
        return sampleCount > 0;
    }
}
