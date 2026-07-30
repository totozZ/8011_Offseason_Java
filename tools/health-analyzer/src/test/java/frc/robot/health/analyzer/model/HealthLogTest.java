package frc.robot.health.analyzer.model;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

class HealthLogTest {
  @Test
  void derivesNamedSessionsAndSlicesStateNumericAndDiscreteEvents() {
    HealthLog log =
        new HealthLog(
            Map.of(
                HealthLog.TEST_SESSION_ACTIVE_PATH,
                List.of(
                    bool(0, false),
                    bool(1_000_000, true),
                    bool(3_000_000, false),
                    bool(4_000_000, true),
                    bool(5_000_000, false))),
            Map.of(
                "/Health/Power/Voltage",
                List.of(
                    number(0, 12),
                    number(1_000_000, 11),
                    number(2_000_000, 10),
                    number(3_000_000, 9),
                    number(4_000_000, 8),
                    number(5_000_000, 10),
                    number(6_000_000, 12))),
            Map.of(
                HealthLog.TEST_SESSION_NAME_PATH,
                List.of(text(1_000_000, "Auto test"), text(4_000_000, "Teleop test")),
                HealthLog.ROBOT_MODE_PATH,
                List.of(text(0, "Disabled"), text(1_000_000, "Autonomous")),
                HealthLog.EVENT_CATEGORY_PATH,
                List.of(text(500_000, "Before"), text(2_000_000, "Inside")),
                HealthLog.EVENT_MESSAGE_PATH,
                List.of(text(500_000, "old"), text(2_000_000, "kept"))),
            0,
            6_000_000);

    assertEquals(
        List.of("Auto test", "Teleop test"),
        log.testSessionIntervals().stream().map(TestSessionInterval::name).toList());

    HealthLog sliced = log.slice(1_000_000, 3_000_000);

    assertEquals(1_000_000, sliced.startMicros());
    assertEquals(3_000_000, sliced.endMicros());
    assertEquals(
        List.of(
            number(1_000_000, 11),
            number(2_000_000, 10),
            number(3_000_000, 9)),
        sliced.doubleSeries("/Health/Power/Voltage"));
    assertEquals(
        List.of(new HealthEvent(2_000_000, "Inside", "kept")),
        sliced.events());
    assertEquals(
        "Autonomous",
        sliced.latestString(HealthLog.ROBOT_MODE_PATH, 1_000_000).orElseThrow().value());
  }

  @Test
  void doesNotCarryExpiredTelemetryIntoALaterSessionButKeepsStaticIds() {
    HealthLog log =
        new HealthLog(
            Map.of(),
            Map.of(
                "/Health/Motors/Drive/Left/SupplyCurrent",
                List.of(number(0, 5), number(1_000_000, 6)),
                "/Health/Motors/Drive/Left/Temperature",
                List.of(number(0, 30), number(1_000_000, 31)),
                "/Health/Motors/Drive/Left/Faults",
                List.of(number(0, 0), number(1_000_000, 1)),
                "/Health/Motors/Drive/Left/DeviceId",
                List.of(number(0, 8))),
            Map.of(),
            0,
            3_000_000);

    HealthLog sliced = log.slice(2_000_000, 3_000_000);

    assertTrue(
        sliced
            .doubleSeries("/Health/Motors/Drive/Left/SupplyCurrent")
            .isEmpty());
    assertTrue(
        sliced
            .doubleSeries("/Health/Motors/Drive/Left/Temperature")
            .isEmpty());
    assertEquals(
        List.of(number(2_000_000, 1)),
        sliced.doubleSeries("/Health/Motors/Drive/Left/Faults"));
    assertEquals(
        List.of(number(2_000_000, 8)),
        sliced.doubleSeries("/Health/Motors/Drive/Left/DeviceId"));
  }

  @Test
  void doesNotFabricateNumericSliceBoundariesInsideATelemetryGap() {
    String path = "/Health/Power/Voltage";
    HealthLog log =
        new HealthLog(
            Map.of(),
            Map.of(path, List.of(number(0, 10), number(10_000_000, 20))),
            Map.of(),
            0,
            10_000_000);

    HealthLog insideGap = log.slice(4_000_000, 6_000_000);

    assertTrue(insideGap.doubleSeries(path).isEmpty());
  }

  @Test
  void stillInterpolatesSliceBoundariesInsideShortContinuousCoverage() {
    String path = "/Health/Power/Voltage";
    HealthLog log =
        new HealthLog(
            Map.of(),
            Map.of(
                path,
                List.of(
                    number(0, 10),
                    number(1_000_000, 20))),
            Map.of(),
            0,
            1_000_000);

    HealthLog sliced = log.slice(250_000, 750_000);

    assertEquals(
        List.of(number(250_000, 12.5), number(750_000, 17.5)),
        sliced.doubleSeries(path));
  }

  private static Sample<Boolean> bool(long timestamp, boolean value) {
    return new Sample<>(timestamp, value);
  }

  private static Sample<Double> number(long timestamp, double value) {
    return new Sample<>(timestamp, value);
  }

  private static Sample<String> text(long timestamp, String value) {
    return new Sample<>(timestamp, value);
  }
}
