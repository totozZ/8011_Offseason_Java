package frc.robot.health.analyzer.ui;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.health.analyzer.model.HealthLog;
import frc.robot.health.analyzer.model.Sample;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

class SnapshotPaneTest {
    @Test
    void displaysPreCoverageAsUnavailableAndExpiredTelemetryAsStale() {
        String voltage = "/Health/Power/Voltage";
        HealthLog log =
                new HealthLog(
                        Map.of(),
                        Map.of(
                                voltage,
                                List.of(
                                        new Sample<>(1_000_000, 12.0),
                                        new Sample<>(2_000_000, 11.0))),
                        Map.of(),
                        0,
                        4_000_001);

        assertEquals("Unavailable", SnapshotPane.numericDisplay(log, voltage, 500_000));
        assertEquals("11.5000", SnapshotPane.numericDisplay(log, voltage, 1_500_000));
        assertEquals("11.0000", SnapshotPane.numericDisplay(log, voltage, 2_500_000));
        assertEquals(
                "Unavailable (stale)",
                SnapshotPane.numericDisplay(log, voltage, 3_000_001));
    }

    @Test
    void keepsExplicitStepNumericMetadataAvailable() {
        String deviceId = "/Health/Motors/Drive/Left/DeviceId";
        HealthLog log =
                new HealthLog(
                        Map.of(),
                        Map.of(deviceId, List.of(new Sample<>(0, 8.0))),
                        Map.of(),
                        0,
                        10_000_000);

        assertEquals(
                "8.0000",
                SnapshotPane.numericDisplay(log, deviceId, 10_000_000));
    }
}
