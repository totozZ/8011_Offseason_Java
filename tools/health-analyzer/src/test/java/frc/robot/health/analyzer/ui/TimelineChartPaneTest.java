package frc.robot.health.analyzer.ui;

import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.health.analyzer.model.HealthLog;
import frc.robot.health.analyzer.model.Sample;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

class TimelineChartPaneTest {
  @Test
  void hoverDetailsShowsOneTimestampAndEverySelectedSignalValue() {
    String voltage = "/Health/Power/Voltage";
    String current = "/Health/Motors/Drive/Left/SupplyCurrent";
    HealthLog log =
        new HealthLog(
            Map.of(),
            Map.of(
                voltage,
                List.of(new Sample<>(1_000_000L, 12.0), new Sample<>(2_000_000L, 10.0)),
                current,
                List.of(new Sample<>(1_000_000L, 4.0), new Sample<>(2_000_000L, 8.0))),
            Map.of(),
            1_000_000L,
            2_000_000L);

    String details =
        TimelineChartPane.hoverDetails(
            log, List.of(voltage, current), 1_500_000L, null);

    assertTrue(details.contains("时间 0.500 s"));
    assertTrue(details.contains("Power/Voltage = 11.0000"));
    assertTrue(details.contains("Motors/Drive/Left/SupplyCurrent = 6.0000"));
  }

  @Test
  void hoverDetailsIncludesNearbyMarkerContext() {
    HealthLog log =
        new HealthLog(Map.of(), Map.of(), Map.of(), 0L, 1_000_000L);
    TimelineMarker marker =
        new TimelineMarker(
            500_000L,
            TimelineMarker.Kind.WARNING,
            "Low voltage",
            "Voltage remained below threshold");

    String details =
        TimelineChartPane.hoverDetails(log, List.of(), 500_000L, marker);

    assertTrue(details.contains("未选择数值信号"));
    assertTrue(details.contains("Low voltage"));
    assertTrue(details.contains("Voltage remained below threshold"));
  }
}
