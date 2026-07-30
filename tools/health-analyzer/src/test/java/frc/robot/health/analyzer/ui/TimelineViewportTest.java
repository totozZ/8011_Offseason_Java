package frc.robot.health.analyzer.ui;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class TimelineViewportTest {
  @Test
  void zoomKeepsAnchorAndClampsToData() {
    TimelineViewport viewport = new TimelineViewport(1_000_000L, 11_000_000L);

    viewport.zoom(0.5, 6_000_000L);

    assertEquals(5_000_000L, viewport.visibleDurationMicros());
    assertEquals(6_000_000L, viewport.timestampAtFraction(0.5));

    viewport.pan(-100_000_000L);
    assertEquals(1_000_000L, viewport.viewStartMicros());
  }

  @Test
  void panAndJumpShareOneGlobalRange() {
    TimelineViewport viewport = new TimelineViewport(0L, 20_000_000L);
    viewport.zoom(0.25, 10_000_000L);
    viewport.panFraction(0.5);

    assertEquals(5_000_000L, viewport.visibleDurationMicros());
    assertEquals(10_000_000L, viewport.viewStartMicros());

    viewport.jumpTo(18_000_000L);
    assertEquals(18_000_000L, viewport.cursorMicros());
    assertTrue(viewport.viewStartMicros() <= 18_000_000L);
    assertTrue(viewport.viewEndMicros() >= 18_000_000L);
  }

  @Test
  void cursorAndPointerConversionsClampSafely() {
    TimelineViewport viewport = new TimelineViewport(5_000L, 10_000L);

    viewport.setCursorMicros(50_000L);

    assertEquals(10_000L, viewport.cursorMicros());
    assertEquals(5_000L, viewport.timestampAtFraction(-2.0));
    assertEquals(10_000L, viewport.timestampAtFraction(2.0));
  }
}
