package frc.robot.health.analyzer.analysis;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.health.analyzer.model.Sample;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class TelemetryCoverageTest {
    @Test
    void rejectsASingleSampleInAShortSelectedRange() {
        assertFalse(
                TelemetryCoverage.timelyAndContinuous(
                        List.of(new Sample<>(1_000_000L, 10.0)),
                        0,
                        2_000_000));
    }

    @Test
    void rejectsHighRateTelemetryThatStopsPointNineNineSecondsBeforeTheEnd() {
        List<Sample<Double>> samples = new ArrayList<>();
        for (long timestamp = 10_000;
                timestamp <= 1_010_000;
                timestamp += 20_000) {
            samples.add(new Sample<>(timestamp, 10.0));
        }

        assertFalse(
                TelemetryCoverage.timelyAndContinuous(
                        samples, 0, 2_000_000));
    }

    @Test
    void acceptsOneNormalSamplePhaseAtEachBoundary() {
        List<Sample<Double>> samples = new ArrayList<>();
        for (long timestamp = 20_000;
                timestamp <= 1_980_000;
                timestamp += 20_000) {
            samples.add(new Sample<>(timestamp, 10.0));
        }

        assertTrue(
                TelemetryCoverage.timelyAndContinuous(
                        samples, 0, 2_000_000));
    }

    @Test
    void acceptsBoundaryDelayWithinObservedRobotLoopJitter() {
        List<Sample<Double>> samples =
                List.of(
                        new Sample<>(170_000L, 10.0),
                        new Sample<>(350_000L, 10.0),
                        new Sample<>(400_000L, 10.0),
                        new Sample<>(450_000L, 10.0),
                        new Sample<>(1_950_000L, 10.0));

        assertFalse(
                TelemetryCoverage.timelyAndContinuous(
                        samples, 0, 2_000_000));

        List<Sample<Double>> continuousSamples = new ArrayList<>();
        continuousSamples.add(new Sample<>(170_000L, 10.0));
        continuousSamples.add(new Sample<>(350_000L, 10.0));
        for (long timestamp = 400_000;
                timestamp <= 1_950_000;
                timestamp += 50_000) {
            continuousSamples.add(new Sample<>(timestamp, 10.0));
        }

        assertTrue(
                TelemetryCoverage.timelyAndContinuous(
                        continuousSamples, 0, 2_000_000));
    }

    @Test
    void acceptsOneRobotLoopOfBoundaryGraceBeyondObservedJitter() {
        List<Sample<Double>> samples = new ArrayList<>();
        samples.add(new Sample<>(148_790L, 10.0));
        samples.add(new Sample<>(292_219L, 10.0));
        for (long timestamp = 342_219L;
                timestamp <= 992_219L;
                timestamp += 50_000L) {
            samples.add(new Sample<>(timestamp, 10.0));
        }

        assertTrue(
                TelemetryCoverage.timelyAndContinuous(
                        samples, 0, 1_000_000L));
    }

    @Test
    void rejectsBoundaryDelayBeyondObservedJitterPlusOneRobotLoop() {
        List<Sample<Double>> samples = new ArrayList<>();
        samples.add(new Sample<>(163_430L, 10.0));
        samples.add(new Sample<>(306_859L, 10.0));
        for (long timestamp = 356_859L;
                timestamp <= 956_859L;
                timestamp += 50_000L) {
            samples.add(new Sample<>(timestamp, 10.0));
        }

        assertFalse(
                TelemetryCoverage.timelyAndContinuous(
                        samples, 0, 1_000_000L));
    }
}
