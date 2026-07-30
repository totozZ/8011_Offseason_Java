package frc.robot.health.analyzer.analysis;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.health.analyzer.model.HealthLog;
import frc.robot.health.analyzer.model.Sample;
import frc.robot.health.analyzer.rules.HealthRules;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

class HealthAnalyzerTest {
    @Test
    void producesSummarySignalSubsystemAndMotorStatistics() throws IOException {
        HealthLog log = new HealthLog(
                Map.of(
                        "/Health/Robot/Enabled",
                        List.of(bool(0, false), bool(1_000_000, true), bool(3_000_000, false)),
                        "/Health/Subsystems/Drive/Active",
                        List.of(bool(0, false), bool(500_000, true), bool(2_500_000, false))),
                Map.ofEntries(
                        Map.entry("/Health/Power/Voltage", values(12, 10, 10, 12)),
                        Map.entry("/Health/Power/TotalCurrent", values(0, 10, 10, 0)),
                        Map.entry("/Health/Motors/Drive/Left/SupplyCurrent", values(0, 6, 6, 0)),
                        Map.entry("/Health/Motors/Drive/Right/SupplyCurrent", values(0, 4, 4, 0)),
                        Map.entry("/Health/Motors/Drive/Left/StatorCurrent", values(0, 20, 20, 0)),
                        Map.entry("/Health/Motors/Drive/Left/Velocity", values(0, 5, 5, 0)),
                        Map.entry("/Health/Motors/Drive/Left/Reference", values(0, 6, 6, 0)),
                        Map.entry("/Health/Motors/Drive/Left/Temperature", values(20, 21, 22, 23)),
                        Map.entry("/Health/Robot/CAN/Utilization", values(.1, .2, .3, .2))),
                Map.of(
                        "/Health/Motors/Drive/Left/ControlMode",
                        List.of(new Sample<>(0L, "VelocityVoltage"))),
                0,
                3_000_000);

        AnalysisResult result = HealthAnalyzer.analyze(log, HealthRules.loadDefaults());

        assertEquals(3.0, result.summary().testDurationSeconds(), 1e-9);
        assertEquals(2.0, result.summary().enabledSeconds(), 1e-9);
        assertEquals(12.0, result.summary().startingVoltage(), 1e-9);
        assertEquals(10.0, result.summary().minimumVoltage(), 1e-9);
        assertEquals(10.0, result.summary().pdhTotalCurrent().rawPeakAmps(), 1e-9);
        assertEquals(20.0 / 3600.0, result.summary().consumedAmpHours(), 1e-9);
        assertTrue(result.signalStatistics().containsKey("/Health/Power/Voltage"));
        SubsystemAnalysis drive = result.subsystemStatistics().get("Drive");
        assertEquals("TALON_SUPPLY_CURRENT_SUM", drive.supplyCurrentSource());
        assertEquals(2.0, drive.activeSeconds(), 1e-9);
        assertTrue(drive.motors().containsKey("Left"));
        assertEquals(1.0, drive.motors().get("Left").maximumAbsoluteReferenceError(), 1e-9);
        assertTrue(result.currentDefinitions().containsKey("Stator Current"));
    }

    @Test
    void treatsConsumedCurrentAsMagnitudeAndLeavesMissingMetricsUnavailable()
            throws IOException {
        HealthLog log =
                new HealthLog(
                        Map.of(),
                        Map.of(
                                "/Health/Power/Voltage",
                                List.of(
                                        new Sample<>(0L, 12.0),
                                        new Sample<>(1_000_000L, 12.0)),
                                "/Health/Power/TotalCurrent",
                                List.of(
                                        new Sample<>(0L, -10.0),
                                        new Sample<>(1_000_000L, -10.0))),
                        Map.of(),
                        0,
                        1_000_000);

        AnalysisResult result =
                HealthAnalyzer.analyze(log, HealthRules.loadDefaults());

        assertEquals(10.0 / 3600.0, result.summary().consumedAmpHours(), 1e-9);
        assertEquals(
                result.summary().pdhTotalCurrent().ampHours(),
                result.summary().consumedAmpHours(),
                1e-12);
        assertNull(result.summary().brownoutCount());
        assertTrue(
                Double.isNaN(
                        result.summary().maximumTemperatureRiseCPerSecond()));
        assertTrue(
                Double.isNaN(
                        result.summary().maximumHighCurrentLowVelocitySeconds()));
    }

    @Test
    void derivesSubsystemErrorAndUsesControlModeAtEachMotorTimestamp()
            throws IOException {
        HealthLog log =
                new HealthLog(
                        Map.of(),
                        Map.ofEntries(
                                Map.entry(
                                         "/Health/Subsystems/Arm/Reference",
                                         List.of(
                                                 new Sample<>(0L, 10.0),
                                                 new Sample<>(1_000_000L, 10.0),
                                                 new Sample<>(2_000_000L, 10.0))),
                                Map.entry(
                                         "/Health/Subsystems/Arm/Measured",
                                         List.of(
                                                 new Sample<>(0L, 0.0),
                                                 new Sample<>(1_000_000L, 0.0),
                                                 new Sample<>(2_000_000L, 0.0))),
                                Map.entry(
                                        "/Health/Motors/Arm/Main/Reference",
                                        values(10, 10, 10, 10)),
                                Map.entry(
                                        "/Health/Motors/Arm/Main/Velocity",
                                        values(7, 7, 7, 7)),
                                Map.entry(
                                        "/Health/Motors/Arm/Main/Position",
                                        values(8, 8, 8, 8))),
                        Map.of(
                                "/Health/Motors/Arm/Main/ControlMode",
                                List.of(
                                        new Sample<>(0L, "VelocityVoltage"),
                                        new Sample<>(1_000_000L, "PositionVoltage"))),
                        0,
                        3_000_000);

        AnalysisResult result =
                HealthAnalyzer.analyze(log, HealthRules.loadDefaults());

        SubsystemAnalysis arm = result.subsystemStatistics().get("Arm");
        assertEquals(10.0, arm.meanAbsoluteReferenceError(), 1e-9);
        assertEquals(10.0, arm.maximumAbsoluteReferenceError(), 1e-9);
        assertEquals(
                3.0,
                arm.motors().get("Main").maximumAbsoluteReferenceError(),
                1e-9);
        assertTrue(
                result.anomalies().stream()
                        .anyMatch(
                                anomaly ->
                                        anomaly.rule()
                                                == frc.robot.health.analyzer.rules.RuleType
                                                        .TRACKING_ERROR
                                                && anomaly.subsystem().equals("Arm")
                                                && anomaly.motor().isBlank()));
    }

    @Test
    void doesNotReplaceAnUnavailableMappedPdhSourceWithMotorCurrent()
            throws IOException {
        HealthLog log =
                new HealthLog(
                        Map.of(),
                        Map.of(
                                "/Health/Motors/Drive/Left/SupplyCurrent",
                                List.of(
                                        new Sample<>(0L, 10.0),
                                        new Sample<>(1_000_000L, 10.0))),
                        Map.of(
                                "/Health/Subsystems/Drive/CurrentSource",
                                List.of(new Sample<>(0L, "PDHChannels"))),
                        0,
                        1_000_000);

        SubsystemAnalysis drive =
                HealthAnalyzer.analyze(log, HealthRules.loadDefaults())
                        .subsystemStatistics()
                        .get("Drive");

        assertEquals("PDHChannels", drive.supplyCurrentSource());
        assertTrue(Double.isNaN(drive.supplyCurrent().averageAmps()));
        assertTrue(Double.isNaN(drive.energyWattHours()));
    }

    @Test
    void marksTalonSubsystemCurrentPartialWhenAnyRegisteredMotorIsMissing()
            throws IOException {
        HealthLog log =
                new HealthLog(
                        Map.of(
                                "/Health/Subsystems/Drive/Active",
                                List.of(bool(0, true), bool(1_000_000, true))),
                        Map.ofEntries(
                                Map.entry(
                                        "/Health/Power/Voltage",
                                        List.of(
                                                new Sample<>(0L, 12.0),
                                                new Sample<>(1_000_000L, 12.0))),
                                Map.entry(
                                        "/Health/Subsystems/Drive/SupplyCurrent",
                                        List.of(
                                                new Sample<>(0L, 10.0),
                                                new Sample<>(1_000_000L, 10.0))),
                                Map.entry(
                                        "/Health/Motors/Drive/Left/SupplyCurrent",
                                        List.of(
                                                new Sample<>(0L, 10.0),
                                                new Sample<>(1_000_000L, 10.0))),
                                Map.entry(
                                        "/Health/Motors/Drive/Right/Temperature",
                                        List.of(
                                                new Sample<>(0L, 25.0),
                                                new Sample<>(1_000_000L, 25.0)))),
                        Map.of(
                                "/Health/Subsystems/Drive/CurrentSource",
                                List.of(new Sample<>(0L, "TalonFXSupplyCurrent"))),
                        0,
                        1_000_000);

        SubsystemAnalysis drive =
                HealthAnalyzer.analyze(log, HealthRules.loadDefaults())
                        .subsystemStatistics()
                        .get("Drive");

        assertEquals(
                "PARTIAL[TalonFXSupplyCurrent;motorSeries=1/2;fullCoverage=1/2]",
                drive.supplyCurrentSource());
        assertTrue(Double.isNaN(drive.supplyCurrent().averageAmps()));
        assertTrue(Double.isNaN(drive.energyWattHours()));
        assertTrue(Double.isNaN(drive.maximumMotorImbalanceRatio()));
    }

    @Test
    void trackingMetricsUseActiveAndControlModeAtEachTimestamp()
            throws IOException {
        HealthLog log =
                new HealthLog(
                        Map.of(
                                "/Health/Subsystems/Arm/Active",
                                List.of(
                                        bool(0, false),
                                        bool(1_500_000, true),
                                        bool(3_000_000, true))),
                        Map.of(
                                "/Health/Motors/Arm/Main/ClosedLoopError",
                                values(100, 100, 2, 2)),
                        Map.of(
                                "/Health/Motors/Arm/Main/ControlMode",
                                List.of(
                                        new Sample<>(0L, "VelocityVoltage"),
                                        new Sample<>(1_000_000L, "DutyCycleOut"),
                                        new Sample<>(2_000_000L, "PositionVoltage"))),
                        0,
                        3_000_000);

        AnalysisResult result =
                HealthAnalyzer.analyze(log, HealthRules.loadDefaults());

        MotorAnalysis main =
                result.subsystemStatistics().get("Arm").motors().get("Main");
        assertEquals(2.0, main.meanAbsoluteReferenceError(), 1e-9);
        assertEquals(2.0, main.maximumAbsoluteReferenceError(), 1e-9);
        assertEquals(
                0,
                result.anomalies().stream()
                        .filter(
                                anomaly ->
                                        anomaly.rule()
                                                        == frc.robot.health.analyzer.rules.RuleType
                                                                .TRACKING_ERROR
                                                && anomaly.motor().equals("Main"))
                        .count());
    }

    @Test
    void directTrackingErrorRemainsCompatibleWhenStateTelemetryIsMissing()
            throws IOException {
        HealthLog log =
                new HealthLog(
                        Map.of(),
                        Map.of(
                                "/Health/Motors/Legacy/Main/ClosedLoopError",
                                List.of(
                                        new Sample<>(0L, 10.0),
                                        new Sample<>(1_000_000L, 10.0))),
                        Map.of(),
                        0,
                        1_000_000);

        AnalysisResult result =
                HealthAnalyzer.analyze(log, HealthRules.loadDefaults());

        assertEquals(
                10.0,
                result.subsystemStatistics()
                        .get("Legacy")
                        .motors()
                        .get("Main")
                        .maximumAbsoluteReferenceError(),
                1e-9);
        assertTrue(
                result.anomalies().stream()
                        .anyMatch(
                                anomaly ->
                                        anomaly.rule()
                                                        == frc.robot.health.analyzer.rules.RuleType
                                                                .TRACKING_ERROR
                                                && anomaly.motor().equals("Main")));
    }

    @Test
    void doesNotBridgeTrackingRulesAcrossAnInternalTelemetryGap()
            throws IOException {
        HealthLog log =
                new HealthLog(
                        Map.of(),
                        Map.of(
                                "/Health/Motors/Legacy/Main/ClosedLoopError",
                                List.of(
                                        new Sample<>(0L, 10.0),
                                        new Sample<>(3_000_000L, 10.0))),
                        Map.of(),
                        0,
                        3_000_000);

        AnalysisResult result =
                HealthAnalyzer.analyze(log, HealthRules.loadDefaults());

        assertEquals(
                0,
                result.anomalies().stream()
                        .filter(
                                anomaly ->
                                        anomaly.rule()
                                                        == frc.robot.health.analyzer.rules.RuleType
                                                                .TRACKING_ERROR
                                                && anomaly.motor().equals("Main"))
                        .count());
    }

    @Test
    void marksTalonCurrentPartialWhenAMotorDoesNotCoverTheSelectedLog()
            throws IOException {
        HealthLog log =
                new HealthLog(
                        Map.of(),
                        Map.ofEntries(
                                Map.entry(
                                        "/Health/Subsystems/Drive/SupplyCurrent",
                                        List.of(
                                                new Sample<>(0L, 20.0),
                                                new Sample<>(2_000_000L, 20.0))),
                                Map.entry(
                                        "/Health/Motors/Drive/Left/SupplyCurrent",
                                        List.of(
                                                new Sample<>(20_000L, 10.0),
                                                new Sample<>(1_020_000L, 10.0),
                                                new Sample<>(1_980_000L, 10.0))),
                                Map.entry(
                                        "/Health/Motors/Drive/Right/SupplyCurrent",
                                        List.of(
                                                new Sample<>(20_000L, 10.0),
                                                new Sample<>(1_500_000L, 10.0),
                                                new Sample<>(1_980_000L, 10.0)))),
                        Map.of(
                                "/Health/Subsystems/Drive/CurrentSource",
                                List.of(new Sample<>(0L, "TalonFXSupplyCurrent"))),
                        0,
                        2_000_000);

        SubsystemAnalysis drive =
                HealthAnalyzer.analyze(log, HealthRules.loadDefaults())
                        .subsystemStatistics()
                        .get("Drive");

        assertEquals(
                "PARTIAL[TalonFXSupplyCurrent;motorSeries=2/2;fullCoverage=1/2]",
                drive.supplyCurrentSource());
        assertTrue(Double.isNaN(drive.supplyCurrent().rawPeakAmps()));
        assertTrue(Double.isNaN(drive.energyWattHours()));
    }

    @Test
    void acceptsNormalSamplingPhaseAtTalonCurrentBoundaries()
            throws IOException {
        List<Sample<Double>> periodic =
                List.of(
                        new Sample<>(20_000L, 10.0),
                        new Sample<>(1_020_000L, 10.0),
                        new Sample<>(1_980_000L, 10.0));
        HealthLog log =
                new HealthLog(
                        Map.of(),
                        Map.of(
                                "/Health/Motors/Drive/Left/SupplyCurrent",
                                periodic,
                                "/Health/Motors/Drive/Right/SupplyCurrent",
                                periodic),
                        Map.of(
                                "/Health/Subsystems/Drive/CurrentSource",
                                List.of(new Sample<>(0L, "TalonFXSupplyCurrent"))),
                        0,
                        2_000_000);

        SubsystemAnalysis drive =
                HealthAnalyzer.analyze(log, HealthRules.loadDefaults())
                        .subsystemStatistics()
                        .get("Drive");

        assertEquals("TALON_SUPPLY_CURRENT_SUM", drive.supplyCurrentSource());
        assertEquals(20.0, drive.supplyCurrent().averageAmps(), 1e-9);
    }

    @Test
    void keepsLoggedSubsystemCurrentWhenSourceChangesBetweenPdhAndTalon()
            throws IOException {
        HealthLog log =
                new HealthLog(
                        Map.of(),
                        Map.of(
                                "/Health/Power/Voltage",
                                values(12.0, 12.0, 12.0, 12.0),
                                "/Health/Subsystems/Drive/SupplyCurrent",
                                values(5.0, 6.0, 7.0, 8.0),
                                "/Health/Motors/Drive/Left/SupplyCurrent",
                                values(100.0, 100.0, 100.0, 100.0)),
                        Map.of(
                                "/Health/Subsystems/Drive/CurrentSource",
                                List.of(
                                        new Sample<>(0L, "PDHChannels"),
                                        new Sample<>(
                                                2_000_000L,
                                                "TalonFXSupplyCurrent"))),
                        0,
                        3_000_000);

        SubsystemAnalysis drive =
                HealthAnalyzer.analyze(log, HealthRules.loadDefaults())
                        .subsystemStatistics()
                        .get("Drive");

        assertEquals(
                "MIXED[PDHChannels|TalonFXSupplyCurrent]",
                drive.supplyCurrentSource());
        assertEquals(6.5, drive.supplyCurrent().averageAmps(), 1e-9);
        assertEquals(8.0, drive.supplyCurrent().rawPeakAmps(), 1e-9);
        assertTrue(Double.isFinite(drive.energyWattHours()));
    }

    @Test
    void boundaryIncompleteLoggedCurrentsKeepRawSamplesButHideTimeMetrics()
            throws IOException {
        List<Sample<Double>> totalCurrent =
                List.of(
                        new Sample<>(1_100_000L, 10.0),
                        new Sample<>(2_100_000L, 20.0),
                        new Sample<>(3_100_000L, 30.0));
        List<Sample<Double>> subsystemCurrent =
                List.of(
                        new Sample<>(1_100_000L, 5.0),
                        new Sample<>(2_100_000L, 10.0),
                        new Sample<>(3_100_000L, 15.0));
        HealthLog log =
                new HealthLog(
                        Map.of(),
                        Map.of(
                                "/Health/Power/Voltage",
                                constantSeries(5, 12.0),
                                "/Health/Power/TotalCurrent",
                                totalCurrent,
                                "/Health/Subsystems/Drive/SupplyCurrent",
                                subsystemCurrent),
                        Map.of(
                                "/Health/Subsystems/Drive/CurrentSource",
                                List.of(new Sample<>(0L, "PDHChannels"))),
                        0,
                        5_000_000);

        AnalysisResult result =
                HealthAnalyzer.analyze(log, HealthRules.loadDefaults());
        CurrentStatistics total = result.summary().pdhTotalCurrent();
        SubsystemAnalysis drive = result.subsystemStatistics().get("Drive");

        assertEquals(30.0, total.rawPeakAmps(), 1e-9);
        assertEquals(29.0, total.p95Amps(), 1e-9);
        assertEquals(29.8, total.p99Amps(), 1e-9);
        assertTrue(Double.isNaN(total.averageAmps()));
        assertTrue(Double.isNaN(total.rolling100msPeakAmps()));
        assertTrue(Double.isNaN(total.rolling500msPeakAmps()));
        assertTrue(Double.isNaN(total.secondsAboveThreshold()));
        assertTrue(Double.isNaN(total.ampHours()));
        assertTrue(Double.isNaN(result.summary().consumedAmpHours()));
        assertTrue(Double.isNaN(result.summary().consumedWattHours()));

        assertEquals("PDHChannels", drive.supplyCurrentSource());
        assertEquals(15.0, drive.supplyCurrent().rawPeakAmps(), 1e-9);
        assertEquals(14.5, drive.supplyCurrent().p95Amps(), 1e-9);
        assertTrue(Double.isNaN(drive.supplyCurrent().averageAmps()));
        assertTrue(Double.isNaN(drive.supplyCurrent().rolling100msPeakAmps()));
        assertTrue(Double.isNaN(drive.supplyCurrent().secondsAboveThreshold()));
        assertTrue(Double.isNaN(drive.supplyCurrent().ampHours()));
        assertTrue(Double.isNaN(drive.energyWattHours()));

        SignalStatistics totalSignal =
                result.signalStatistics().get("/Health/Power/TotalCurrent");
        assertEquals(10.0, totalSignal.minimum(), 1e-9);
        assertEquals(30.0, totalSignal.maximum(), 1e-9);
        assertEquals(29.0, totalSignal.p95(), 1e-9);
        assertTrue(Double.isNaN(totalSignal.timeWeightedMean()));
        assertTrue(Double.isNaN(totalSignal.integralValueSeconds()));

        SignalStatistics subsystemSignal =
                result.signalStatistics()
                        .get("/Health/Subsystems/Drive/SupplyCurrent");
        assertEquals(5.0, subsystemSignal.minimum(), 1e-9);
        assertEquals(15.0, subsystemSignal.maximum(), 1e-9);
        assertTrue(Double.isNaN(subsystemSignal.timeWeightedMean()));
        assertTrue(Double.isNaN(subsystemSignal.integralValueSeconds()));
    }

    @Test
    void boundaryIncompleteVoltageMakesWholeRobotAndSubsystemEnergyUnavailable()
            throws IOException {
        List<Sample<Double>> incompleteVoltage =
                List.of(
                        new Sample<>(1_100_000L, 12.0),
                        new Sample<>(2_100_000L, 12.0),
                        new Sample<>(3_100_000L, 12.0));
        HealthLog log =
                new HealthLog(
                        Map.of(),
                        Map.of(
                                "/Health/Power/Voltage",
                                incompleteVoltage,
                                "/Health/Power/TotalCurrent",
                                constantSeries(5, 10.0),
                                "/Health/Subsystems/Drive/SupplyCurrent",
                                constantSeries(5, 5.0)),
                        Map.of(
                                "/Health/Subsystems/Drive/CurrentSource",
                                List.of(new Sample<>(0L, "PDHChannels"))),
                        0,
                        5_000_000);

        AnalysisResult result =
                HealthAnalyzer.analyze(log, HealthRules.loadDefaults());
        SubsystemAnalysis drive = result.subsystemStatistics().get("Drive");

        assertEquals(10.0, result.summary().pdhTotalCurrent().averageAmps(), 1e-9);
        assertTrue(
                Double.isFinite(
                        result.summary().pdhTotalCurrent().ampHours()));
        assertTrue(Double.isNaN(result.summary().consumedWattHours()));
        assertEquals(5.0, drive.supplyCurrent().averageAmps(), 1e-9);
        assertTrue(Double.isFinite(drive.supplyCurrent().ampHours()));
        assertTrue(Double.isNaN(drive.energyWattHours()));
    }

    @Test
    void countsTheDeduplicatedUnionOfBrownoutSignalAndEvents()
            throws IOException {
        HealthLog log =
                new HealthLog(
                        Map.of(
                                "/Health/Robot/BrownedOut",
                                List.of(
                                        bool(0, false),
                                        bool(1_000_000, true),
                                        bool(1_500_000, false),
                                        bool(3_000_000, true),
                                        bool(3_500_000, false))),
                        Map.of(),
                        Map.of(
                                HealthLog.EVENT_CATEGORY_PATH,
                                List.of(
                                        new Sample<>(1_100_000L, "Brownout"),
                                        new Sample<>(5_000_000L, "Brownout")),
                                HealthLog.EVENT_MESSAGE_PATH,
                                List.of(
                                        new Sample<>(1_100_000L, "Controller event"),
                                        new Sample<>(5_000_000L, "Controller event"))),
                        0,
                        6_000_000);

        AnalysisResult result =
                HealthAnalyzer.analyze(log, HealthRules.loadDefaults());

        assertEquals(3, result.summary().brownoutCount());
        assertEquals(
                3,
                result.anomalies().stream()
                        .filter(
                                anomaly ->
                                        anomaly.rule()
                                                == frc.robot.health.analyzer.rules.RuleType
                                                        .BROWNOUT)
                        .count());
    }

    private static List<Sample<Double>> values(double a, double b, double c, double d) {
        return List.of(
                new Sample<>(0L, a),
                new Sample<>(1_000_000L, b),
                new Sample<>(2_000_000L, c),
                new Sample<>(3_000_000L, d));
    }

    private static List<Sample<Double>> constantSeries(
            int endSecond, double value) {
        List<Sample<Double>> samples = new ArrayList<>();
        for (int second = 0; second <= endSecond; second++) {
            samples.add(new Sample<>(second * 1_000_000L, value));
        }
        return List.copyOf(samples);
    }

    private static Sample<Boolean> bool(long timestamp, boolean value) {
        return new Sample<>(timestamp, value);
    }
}
