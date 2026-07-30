package frc.robot.health.analyzer.app;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.OptionalLong;
import org.junit.jupiter.api.Test;

class HealthAnalyzerAppTest {
  @Test
  void exportCoordinatorRejectsOverlapAndMarksInvalidatedCompletionStale() {
    HealthAnalyzerApp.ExportCoordinator coordinator =
        new HealthAnalyzerApp.ExportCoordinator();

    OptionalLong first = coordinator.tryBegin();
    assertTrue(first.isPresent());
    assertTrue(coordinator.isInProgress());
    assertTrue(coordinator.tryBegin().isEmpty());

    coordinator.invalidate();

    assertFalse(coordinator.finish(first.getAsLong()));
    assertFalse(coordinator.isInProgress());
  }

  @Test
  void staleCallbackCannotClearNewerExport() {
    HealthAnalyzerApp.ExportCoordinator coordinator =
        new HealthAnalyzerApp.ExportCoordinator();
    long first = coordinator.tryBegin().orElseThrow();
    assertTrue(coordinator.finish(first));

    long second = coordinator.tryBegin().orElseThrow();
    assertFalse(coordinator.finish(first));
    assertTrue(coordinator.isInProgress());
    assertTrue(coordinator.finish(second));
    assertFalse(coordinator.isInProgress());
  }
}
