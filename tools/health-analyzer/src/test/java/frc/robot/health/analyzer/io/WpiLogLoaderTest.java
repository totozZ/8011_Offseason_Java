package frc.robot.health.analyzer.io;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.health.analyzer.io.wpilog.PureJavaWpiLogWriter;
import frc.robot.health.analyzer.model.HealthEvent;
import frc.robot.health.analyzer.model.HealthLog;
import frc.robot.health.analyzer.model.RobotModeInterval;
import frc.robot.health.analyzer.model.Sample;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class WpiLogLoaderTest {
  @TempDir Path tempDirectory;

  @Test
  void loadsTypedHealthSeriesAndIgnoresOtherNamespaces() throws Exception {
    Path file = tempDirectory.resolve("health.wpilog");
    writeRepresentativeHealthLog(file);

    HealthLog log = new WpiLogLoader().load(file);

    assertEquals(1_000_000, log.startMicros());
    assertEquals(4_000_000, log.endMicros());
    assertEquals(3_000_000, log.durationMicros());
    assertEquals(
        List.of(new Sample<>(1_000_000L, true)),
        log.booleanSeries("/Health/Robot/Enabled"));
    assertEquals(
        List.of(new Sample<>(1_000_000L, 12.4), new Sample<>(4_000_000L, 10.8)),
        log.doubleSeries("/Health/Power/Voltage"));
    assertEquals(
        List.of(new Sample<>(1_000_000L, 0.0), new Sample<>(3_000_000L, 2.0)),
        log.doubleSeries("/Health/Robot/CAN/BusOffCount"));
    assertFalse(log.signalPaths().contains("/Unrelated/Number"));
    assertTrue(log.doubleSignals().containsKey("/Health/Power/Voltage"));
    assertTrue(log.booleanSignals().containsKey("/Health/Robot/Enabled"));
    assertTrue(log.stringSignals().containsKey("/Health/Robot/Mode"));
  }

  @Test
  void derivesEventsAndCollapsedRobotModeIntervals() throws Exception {
    Path file = tempDirectory.resolve("timeline.wpilog");
    writeRepresentativeHealthLog(file);

    HealthLog log = new WpiLogLoader().load(file);

    assertEquals(
        List.of(
            new HealthEvent(1_500_000, "Session", "Started"),
            new HealthEvent(3_250_000, "Fault", "Drive current mismatch")),
        log.events());
    assertEquals(
        List.of(
            new RobotModeInterval("Disabled", 1_000_000, 2_000_000),
            new RobotModeInterval("Auto", 2_000_000, 3_000_000),
            new RobotModeInterval("Teleop", 3_000_000, 4_000_000)),
        log.modeIntervals());
  }

  @Test
  void supportsLatestAndNearestTimelineQueries() throws Exception {
    Path file = tempDirectory.resolve("queries.wpilog");
    writeRepresentativeHealthLog(file);
    HealthLog log = new WpiLogLoader().load(file);

    assertTrue(log.latestDouble("/Health/Power/Voltage", 999_999).isEmpty());
    assertEquals(
        new Sample<>(1_000_000L, 12.4),
        log.latestDouble("/Health/Power/Voltage", 3_999_999).orElseThrow());
    assertEquals(
        new Sample<>(1_000_000L, 12.4),
        log.nearestDouble("/Health/Power/Voltage", 2_500_000).orElseThrow());
    assertEquals(
        new Sample<>(4_000_000L, 10.8),
        log.nearestDouble("/Health/Power/Voltage", 2_500_001).orElseThrow());
    assertTrue(log.nearestString("/Health/DoesNotExist", 0).isEmpty());
  }

  @Test
  void returnsAnEmptyDatasetForAValidLogWithoutHealthData() throws Exception {
    Path file = tempDirectory.resolve("no-health.wpilog");
    try (PureJavaWpiLogWriter writer = new PureJavaWpiLogWriter(file)) {
      int entry = writer.startDouble("/Unrelated/Number", 0);
      writer.appendDouble(entry, 42.0, 500_000);
    }

    HealthLog log = new WpiLogLoader().load(file);
    assertEquals(0, log.startMicros());
    assertEquals(0, log.endMicros());
    assertTrue(log.signalPaths().isEmpty());
    assertTrue(log.events().isEmpty());
    assertTrue(log.modeIntervals().isEmpty());
  }

  @Test
  void rejectsMissingAndInvalidFiles() throws Exception {
    Path missing = tempDirectory.resolve("missing.wpilog");
    assertThrows(IOException.class, () -> new WpiLogLoader().load(missing));

    Path invalid = tempDirectory.resolve("invalid.wpilog");
    Files.write(invalid, new byte[] {1, 2, 3, 4});
    assertThrows(IOException.class, () -> new WpiLogLoader().load(invalid));

    Path invalidExtraHeader = tempDirectory.resolve("invalid-extra-header.wpilog");
    Files.write(
        invalidExtraHeader,
        new byte[] {'W', 'P', 'I', 'L', 'O', 'G', 0, 1, 100, 0, 0, 0});
    assertThrows(IOException.class, () -> new WpiLogLoader().load(invalidExtraHeader));

    Path overLimit = tempDirectory.resolve("over-limit.wpilog");
    writeRepresentativeHealthLog(overLimit);
    assertThrows(IOException.class, () -> new WpiLogLoader(16).load(overLimit));
  }

  @Test
  void exposesImmutableMapsAndSeries() throws Exception {
    Path file = tempDirectory.resolve("immutable.wpilog");
    writeRepresentativeHealthLog(file);
    HealthLog log = new WpiLogLoader().load(file);

    assertThrows(
        UnsupportedOperationException.class,
        () -> log.doubleSignals().put("/Health/New", List.of()));
    assertThrows(
        UnsupportedOperationException.class,
        () -> log.doubleSeries("/Health/Power/Voltage").add(new Sample<>(5_000_000L, 0.0)));
  }

  private static void writeRepresentativeHealthLog(Path file) throws IOException {
    try (PureJavaWpiLogWriter writer = new PureJavaWpiLogWriter(file)) {
      int voltage = writer.startDouble("/Health/Power/Voltage", 0);
      int enabled = writer.startBoolean("/Health/Robot/Enabled", 0);
      int mode = writer.startString("/Health/Robot/Mode", 0);
      int eventCategory = writer.startString("/Health/Events/Category", 0);
      int eventMessage = writer.startString("/Health/Events/Message", 0);
      int busOffCount = writer.startInteger("/Health/Robot/CAN/BusOffCount", 0);
      int unrelated = writer.startDouble("/Unrelated/Number", 0);

      writer.appendDouble(unrelated, 99.0, 500_000);
      writer.appendDouble(voltage, 12.4, 1_000_000);
      writer.appendBoolean(enabled, true, 1_000_000);
      writer.appendString(mode, "Disabled", 1_000_000);
      writer.appendInteger(busOffCount, 0, 1_000_000);

      writer.appendString(eventCategory, "Session", 1_500_000);
      writer.appendString(eventMessage, "Started", 1_500_000);
      writer.appendString(mode, "Auto", 2_000_000);
      writer.appendString(mode, "Auto", 2_250_000);
      writer.appendString(mode, "Teleop", 3_000_000);
      writer.appendInteger(busOffCount, 2, 3_000_000);
      writer.appendString(eventCategory, "Fault", 3_250_000);
      writer.appendString(eventMessage, "Drive current mismatch", 3_250_000);
      writer.appendDouble(voltage, 10.8, 4_000_000);
    }
  }
}
