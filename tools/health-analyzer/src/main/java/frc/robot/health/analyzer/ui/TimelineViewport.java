package frc.robot.health.analyzer.ui;

/**
 * Mutable time viewport shared by every timeline lane.
 *
 * <p>The class deliberately has no JavaFX dependency so zoom, pan, cursor and
 * anomaly-jump behavior can be tested headlessly.
 */
public final class TimelineViewport {
  private static final long MIN_VISIBLE_MICROS = 1_000L;

  private long dataStartMicros;
  private long dataEndMicros;
  private long viewStartMicros;
  private long viewEndMicros;
  private long cursorMicros;

  public TimelineViewport(long dataStartMicros, long dataEndMicros) {
    setDataRange(dataStartMicros, dataEndMicros);
  }

  public void setDataRange(long startMicros, long endMicros) {
    if (startMicros < 0L || endMicros < startMicros) {
      throw new IllegalArgumentException("invalid timeline data range");
    }
    dataStartMicros = startMicros;
    dataEndMicros = endMicros;
    viewStartMicros = startMicros;
    viewEndMicros = endMicros;
    cursorMicros = startMicros;
  }

  public void showAll() {
    viewStartMicros = dataStartMicros;
    viewEndMicros = dataEndMicros;
  }

  /**
   * Zooms around an absolute timestamp.
   *
   * @param scale visible-duration multiplier; values below one zoom in
   * @param anchorMicros timestamp that should stay under the pointer
   */
  public void zoom(double scale, long anchorMicros) {
    if (!Double.isFinite(scale) || scale <= 0.0) {
      throw new IllegalArgumentException("zoom scale must be finite and positive");
    }
    long dataSpan = dataSpanMicros();
    if (dataSpan <= 0L) {
      return;
    }

    long currentSpan = visibleDurationMicros();
    long desiredSpan =
        clamp(
            Math.round(currentSpan * scale),
            Math.min(MIN_VISIBLE_MICROS, dataSpan),
            dataSpan);
    long anchor = clamp(anchorMicros, viewStartMicros, viewEndMicros);
    double ratio =
        currentSpan <= 0L ? 0.5 : (double) (anchor - viewStartMicros) / (double) currentSpan;
    long desiredStart = anchor - Math.round(desiredSpan * ratio);
    setVisibleRange(desiredStart, desiredStart + desiredSpan);
  }

  /** Pans the visible range by an absolute time delta. */
  public void pan(long deltaMicros) {
    if (deltaMicros == 0L || dataSpanMicros() <= visibleDurationMicros()) {
      return;
    }
    setVisibleRange(saturatingAdd(viewStartMicros, deltaMicros),
        saturatingAdd(viewEndMicros, deltaMicros));
  }

  /** Pans by a fraction of the currently visible duration. */
  public void panFraction(double fraction) {
    if (!Double.isFinite(fraction)) {
      throw new IllegalArgumentException("pan fraction must be finite");
    }
    pan(Math.round(visibleDurationMicros() * fraction));
  }

  public void setVisibleRange(long requestedStartMicros, long requestedEndMicros) {
    if (requestedEndMicros < requestedStartMicros) {
      throw new IllegalArgumentException("visible range end precedes start");
    }
    long dataSpan = dataSpanMicros();
    if (dataSpan <= 0L) {
      viewStartMicros = dataStartMicros;
      viewEndMicros = dataEndMicros;
      return;
    }

    long requestedSpan =
        clamp(
            requestedEndMicros - requestedStartMicros,
            Math.min(MIN_VISIBLE_MICROS, dataSpan),
            dataSpan);
    long start = requestedStartMicros;
    if (start < dataStartMicros) {
      start = dataStartMicros;
    }
    long latestStart = dataEndMicros - requestedSpan;
    if (start > latestStart) {
      start = latestStart;
    }
    viewStartMicros = start;
    viewEndMicros = start + requestedSpan;
  }

  /** Moves the cursor without changing the visible range. */
  public void setCursorMicros(long timestampMicros) {
    cursorMicros = clamp(timestampMicros, dataStartMicros, dataEndMicros);
  }

  /**
   * Moves the cursor to an event/anomaly and scrolls only when it is outside
   * the current view.
   */
  public void jumpTo(long timestampMicros) {
    setCursorMicros(timestampMicros);
    if (cursorMicros >= viewStartMicros && cursorMicros <= viewEndMicros) {
      return;
    }
    long span = visibleDurationMicros();
    setVisibleRange(cursorMicros - span / 2L, cursorMicros - span / 2L + span);
  }

  public long timestampAtFraction(double fraction) {
    if (!Double.isFinite(fraction)) {
      throw new IllegalArgumentException("timeline fraction must be finite");
    }
    double clamped = Math.max(0.0, Math.min(1.0, fraction));
    return viewStartMicros + Math.round(visibleDurationMicros() * clamped);
  }

  public double fractionAtTimestamp(long timestampMicros) {
    long span = visibleDurationMicros();
    if (span <= 0L) {
      return 0.0;
    }
    return (double) (timestampMicros - viewStartMicros) / (double) span;
  }

  public long dataStartMicros() {
    return dataStartMicros;
  }

  public long dataEndMicros() {
    return dataEndMicros;
  }

  public long viewStartMicros() {
    return viewStartMicros;
  }

  public long viewEndMicros() {
    return viewEndMicros;
  }

  public long cursorMicros() {
    return cursorMicros;
  }

  public long dataSpanMicros() {
    return dataEndMicros - dataStartMicros;
  }

  public long visibleDurationMicros() {
    return viewEndMicros - viewStartMicros;
  }

  private static long clamp(long value, long minimum, long maximum) {
    return Math.max(minimum, Math.min(maximum, value));
  }

  private static long saturatingAdd(long left, long right) {
    if (right > 0L && left > Long.MAX_VALUE - right) {
      return Long.MAX_VALUE;
    }
    if (right < 0L && left < Long.MIN_VALUE - right) {
      return Long.MIN_VALUE;
    }
    return left + right;
  }
}
