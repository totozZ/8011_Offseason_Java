package frc.robot.health.analyzer.analysis;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.health.analyzer.model.Sample;
import java.util.List;
import org.junit.jupiter.api.Test;

class SeriesMathTest {
    private static final double EPSILON = 1e-9;

    @Test
    void integratesUsingIrregularTimestamps() {
        List<Sample<Double>> current = List.of(
                sample(0, 0.0),
                sample(1_000_000, 10.0),
                sample(2_000_000, 10.0),
                sample(3_000_000, 10.0));
        List<Sample<Double>> voltage = List.of(
                sample(0, 12.0),
                sample(1_000_000, 12.0),
                sample(2_000_000, 12.0),
                sample(3_000_000, 12.0));

        assertEquals(25.0, SeriesMath.integrate(current), EPSILON);
        assertEquals(25.0 / 3600.0, SeriesMath.ampHours(current), EPSILON);
        assertEquals(300.0 / 3600.0, SeriesMath.wattHours(voltage, current), EPSILON);
        assertEquals(25.0 / 3.0, SeriesMath.timeWeightedMean(current), EPSILON);
    }

    @Test
    void returnsFirstFiniteValueInTimestampOrder() {
        assertEquals(
                10.0,
                SeriesMath.first(
                        List.of(
                                new Sample<>(2_000_000L, 20.0),
                                new Sample<>(1_000_000L, 10.0))),
                EPSILON);
        assertTrue(Double.isNaN(SeriesMath.first(List.of())));
    }

    @Test
    void computesPercentilesWithLinearInterpolation() {
        List<Sample<Double>> values = List.of(
                sample(0, 0.0),
                sample(1, 10.0),
                sample(2, 20.0),
                sample(3, 30.0),
                sample(4, 40.0));

        assertEquals(38.0, SeriesMath.percentile(values, 0.95), EPSILON);
        assertEquals(39.6, SeriesMath.percentile(values, 0.99), EPSILON);
    }

    @Test
    void marksEnergyUnavailableWithoutATimestampInterval() {
        assertTrue(Double.isNaN(SeriesMath.ampHours(List.of())));
        assertTrue(Double.isNaN(SeriesMath.ampHours(List.of(sample(0, 10.0)))));
        assertTrue(Double.isNaN(SeriesMath.wattHours(List.of(), List.of())));
        assertTrue(
                Double.isNaN(
                        SeriesMath.wattHours(
                                List.of(sample(0, 12.0)),
                                List.of(sample(0, 10.0)))));
        assertTrue(Double.isNaN(SeriesMath.durationAbove(List.of(), 1.0)));
        assertTrue(
                Double.isNaN(
                        SeriesMath.durationAbove(
                                List.of(sample(0, 2.0)),
                                1.0)));
        assertTrue(Double.isNaN(SeriesMath.maxPositiveSlopePerSecond(List.of())));
        assertTrue(
                Double.isNaN(
                        SeriesMath.highCurrentLowVelocityDuration(
                                List.of(sample(0, 20.0)),
                                List.of(),
                                10.0,
                                1.0)));
    }

    @Test
    void computesTimestampBasedRollingPeaks() {
        List<Sample<Double>> values = List.of(
                sample(0, 0.0),
                sample(100_000, 10.0),
                sample(200_000, 10.0),
                sample(300_000, 0.0));

        assertEquals(10.0, SeriesMath.rollingAveragePeak(values, 100_000), EPSILON);
        assertEquals(20.0 / 3.0, SeriesMath.rollingAveragePeak(values, 500_000), EPSILON);
    }

    @Test
    void computesThresholdAndBooleanDurations() {
        List<Sample<Double>> values = List.of(
                sample(0, 0.0),
                sample(1_000_000, 10.0),
                sample(2_000_000, 0.0));
        List<Sample<Boolean>> active = List.of(
                new Sample<>(0, false),
                new Sample<>(250_000, true),
                new Sample<>(1_250_000, false));

        assertEquals(1.0, SeriesMath.durationAbove(values, 5.0), EPSILON);
        assertEquals(1.0, SeriesMath.trueDuration(active, 0, 2_000_000), EPSILON);
    }

    @Test
    void computesReferenceErrorStallDurationAndImbalance() {
        List<Sample<Double>> current =
                List.of(
                        sample(0, 20.0),
                        sample(1_000_000, 20.0),
                        sample(2_000_000, 20.0));
        List<Sample<Double>> velocity =
                List.of(
                        sample(0, 0.0),
                        sample(1_000_000, 0.0),
                        sample(2_000_000, 0.0));
        List<Sample<Double>> reference =
                List.of(
                        sample(0, 10.0),
                        sample(1_000_000, 10.0),
                        sample(2_000_000, 10.0));
        List<Sample<Double>> measured =
                List.of(
                        sample(0, 8.0),
                        sample(1_000_000, 7.0),
                        sample(2_000_000, 6.0));
        List<Sample<Double>> secondCurrent =
                List.of(
                        sample(0, 10.0),
                        sample(1_000_000, 10.0),
                        sample(2_000_000, 10.0));

        assertEquals(
                2.0,
                SeriesMath.highCurrentLowVelocityDuration(current, velocity, 15.0, 1.0),
                EPSILON);
        assertEquals(
                3.0,
                SeriesMath.timeWeightedMean(SeriesMath.absoluteError(reference, measured)),
                EPSILON);
        List<Sample<Double>> imbalance =
                SeriesMath.imbalance(List.of(current, secondCurrent), 5.0);
        assertEquals(0.5, SeriesMath.max(imbalance), EPSILON);
    }

    @Test
    void treatsCurrentMagnitudeAsLoadForNegativeMotorDirection() {
        List<Sample<Double>> current =
                List.of(
                        sample(0, -20.0),
                        sample(1_000_000, -20.0),
                        sample(2_000_000, -20.0));
        List<Sample<Double>> velocity =
                List.of(
                        sample(0, 0.0),
                        sample(1_000_000, 0.0),
                        sample(2_000_000, 0.0));

        CurrentStatistics statistics =
                CurrentStatistics.of("Stator Current", current, 15.0);

        assertEquals(20.0, statistics.averageAmps(), EPSILON);
        assertEquals(20.0, statistics.rawPeakAmps(), EPSILON);
        assertEquals(2.0, statistics.secondsAboveThreshold(), EPSILON);
        assertTrue(Double.isNaN(statistics.ampHours()));
        assertEquals(
                2.0,
                SeriesMath.highCurrentLowVelocityDuration(
                        current, velocity, 15.0, 1.0),
                EPSILON);
    }

    @Test
    void refusesToInterpolateIntegrateOrMergeAcrossLongTelemetryGaps() {
        List<Sample<Double>> gapped =
                List.of(sample(0, 10.0), sample(10_000_000, 20.0));
        List<Sample<Double>> continuous =
                List.of(
                        sample(0, 1.0),
                        sample(1_000_000, 1.0),
                        sample(2_000_000, 1.0),
                        sample(3_000_000, 1.0),
                        sample(4_000_000, 1.0),
                        sample(5_000_000, 1.0),
                        sample(6_000_000, 1.0),
                        sample(7_000_000, 1.0),
                        sample(8_000_000, 1.0),
                        sample(9_000_000, 1.0),
                        sample(10_000_000, 1.0));

        assertFalse(SeriesMath.hasContinuousCoverage(gapped));
        assertFalse(
                SeriesMath.hasContinuousCoverage(
                        gapped, 4_000_000, 6_000_000));
        assertTrue(Double.isNaN(SeriesMath.valueAt(gapped, 5_000_000)));
        assertTrue(Double.isNaN(SeriesMath.integrate(gapped)));
        assertTrue(Double.isNaN(SeriesMath.timeWeightedMean(gapped)));
        assertTrue(Double.isNaN(SeriesMath.durationAbove(gapped, 5.0)));
        assertTrue(Double.isNaN(SeriesMath.rollingAveragePeak(gapped, 100_000)));
        assertTrue(SeriesMath.trailingRollingAverage(gapped, 100_000).isEmpty());
        assertTrue(Double.isNaN(SeriesMath.maxPositiveSlopePerSecond(gapped)));
        assertTrue(Double.isNaN(SeriesMath.ampHours(gapped)));
        assertTrue(Double.isNaN(SeriesMath.wattHours(continuous, gapped)));
        assertTrue(
                Double.isNaN(
                        SeriesMath.highCurrentLowVelocityDuration(
                                gapped, continuous, 5.0, 2.0)));
        assertTrue(SeriesMath.absoluteError(gapped, continuous).isEmpty());
        assertTrue(SeriesMath.sum(List.of(gapped, continuous)).isEmpty());
        assertTrue(SeriesMath.imbalance(List.of(gapped, continuous), 1.0).isEmpty());
        assertTrue(SeriesMath.timeline(List.of(gapped, continuous)).isEmpty());

        // Sample-only statistics remain valid because they make no continuity claim.
        assertEquals(10.0, SeriesMath.min(gapped), EPSILON);
        assertEquals(20.0, SeriesMath.max(gapped), EPSILON);
        assertEquals(19.5, SeriesMath.percentile(gapped, 0.95), EPSILON);
    }

    @Test
    void keepsShortInterpolationButNeverExtrapolatesBeyondRealSamples() {
        List<Sample<Double>> values =
                List.of(sample(0, 10.0), sample(1_000_000, 20.0));

        assertTrue(SeriesMath.hasContinuousCoverage(values));
        assertEquals(15.0, SeriesMath.valueAt(values, 500_000), EPSILON);
        assertEquals(15.0, SeriesMath.integrate(values), EPSILON);
        assertTrue(Double.isNaN(SeriesMath.valueAt(values, -1)));
        assertTrue(Double.isNaN(SeriesMath.valueAt(values, 1_000_001)));
        assertTrue(SeriesMath.hasFreshSampleAt(values, 1_500_000));
        assertFalse(SeriesMath.hasFreshSampleAt(values, 2_000_001));
    }

    private static Sample<Double> sample(long timestampMicros, double value) {
        return new Sample<>(timestampMicros, value);
    }
}
