package frc.robot.health.analyzer.sample;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.health.analyzer.analysis.HealthAnalyzer;
import frc.robot.health.analyzer.io.WpiLogLoader;
import frc.robot.health.analyzer.model.HealthLog;
import frc.robot.health.analyzer.rules.HealthRules;
import frc.robot.health.analyzer.rules.RuleType;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.EnumSet;
import java.util.List;
import java.util.stream.Collectors;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class SampleHealthLogGeneratorTest {
  @TempDir Path tempDirectory;

  @Test
  void generatesDeterministicReadableLogWithFaultsAndDefaultRuleFindings() throws Exception {
    Path first = SampleHealthLogGenerator.generate(tempDirectory.resolve("first.wpilog"));
    Path second = SampleHealthLogGenerator.generate(tempDirectory.resolve("second.wpilog"));

    assertArrayEquals(Files.readAllBytes(first), Files.readAllBytes(second));
    assertTrue(Files.size(first) < 250_000, "sample should remain small enough to share");

    HealthLog log = new WpiLogLoader().load(first);
    assertEquals(15_000_000, log.durationMicros());
    assertEquals(
        List.of("Disabled", "Autonomous", "Teleop", "Disabled"),
        log.modeIntervals().stream().map(interval -> interval.mode()).toList());
    assertEquals(8, log.events().size());

    for (String motor : List.of("Left", "Right")) {
      String base = "/Health/Motors/Drive/" + motor;
      assertTrue(log.doubleSignals().containsKey(base + "/Reference"));
      assertTrue(log.doubleSignals().containsKey(base + "/Velocity"));
      assertTrue(log.doubleSignals().containsKey(base + "/Temperature"));
      assertTrue(log.booleanSignals().containsKey(base + "/Connected"));
    }
    assertTrue(
        log.booleanSeries("/Health/Motors/Drive/Left/StatorCurrentLimited").stream()
            .anyMatch(sample -> sample.value()));
    assertTrue(
        log.booleanSeries("/Health/Motors/Drive/Right/Connected").stream()
            .anyMatch(sample -> !sample.value()));
    assertTrue(
        log.doubleSeries("/Health/Robot/CAN/BusOffCount").stream()
            .anyMatch(sample -> sample.value() > 0.0));
    assertTrue(
        log.doubleSeries("/Health/Motors/Drive/Left/Faults").stream()
            .anyMatch(sample -> sample.value() > 0.0));

    EnumSet<RuleType> detected =
        HealthAnalyzer.analyze(log, HealthRules.loadDefaults()).anomalies().stream()
            .map(anomaly -> anomaly.rule())
            .collect(Collectors.toCollection(() -> EnumSet.noneOf(RuleType.class)));
    assertTrue(
        detected.containsAll(
            EnumSet.of(
                RuleType.LOW_VOLTAGE,
                RuleType.BROWNOUT,
                RuleType.HIGH_TOTAL_CURRENT,
                RuleType.STALL,
                RuleType.TRACKING_ERROR,
                RuleType.TEMPERATURE,
                RuleType.CAN_FAULT,
                RuleType.IDLE_DRAW)));
  }
}
