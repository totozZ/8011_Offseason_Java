package frc.robot.health.analyzer.app;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;

import frc.robot.health.analyzer.model.HealthLog;
import frc.robot.health.analyzer.model.Sample;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

class HealthAnalyzerLauncherTest {
  @Test
  void selectsFullLastAndOrdinalSessionsDeterministically() {
    HealthLog source =
        new HealthLog(
            Map.of(
                HealthLog.TEST_SESSION_ACTIVE_PATH,
                List.of(
                    bool(0, false),
                    bool(1_000_000, true),
                    bool(2_000_000, false),
                    bool(3_000_000, true),
                    bool(5_000_000, false))),
            Map.of(
                "/Health/Power/Voltage",
                List.of(number(0, 12), number(5_000_000, 11))),
            Map.of(
                HealthLog.TEST_SESSION_NAME_PATH,
                List.of(text(1_000_000, "First"), text(3_000_000, "Last"))),
            0,
            5_000_000);

    assertSame(source, HealthAnalyzerLauncher.selectSession(source, "full"));
    HealthLog last = HealthAnalyzerLauncher.selectSession(source, "last");
    assertEquals(3_000_000, last.startMicros());
    assertEquals(5_000_000, last.endMicros());
    HealthLog first = HealthAnalyzerLauncher.selectSession(source, "1");
    assertEquals(1_000_000, first.startMicros());
    assertEquals(2_000_000, first.endMicros());
    assertThrows(
        IllegalArgumentException.class,
        () -> HealthAnalyzerLauncher.selectSession(source, "3"));
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
