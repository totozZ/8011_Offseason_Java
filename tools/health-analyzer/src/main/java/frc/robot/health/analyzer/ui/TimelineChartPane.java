package frc.robot.health.analyzer.ui;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import java.util.Objects;
import java.util.Optional;
import java.util.function.LongConsumer;

import javafx.beans.InvalidationListener;
import javafx.util.Duration;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.input.MouseButton;
import javafx.scene.control.Tooltip;
import javafx.scene.layout.Region;
import javafx.scene.paint.Color;
import javafx.scene.text.Font;
import javafx.scene.text.FontWeight;

import frc.robot.health.analyzer.model.HealthEvent;
import frc.robot.health.analyzer.model.HealthLog;
import frc.robot.health.analyzer.model.RobotModeInterval;
import frc.robot.health.analyzer.model.Sample;

/**
 * Canvas-based multi-lane chart with one global viewport and cursor.
 *
 * <p>Each signal receives its own automatically scaled lane, while zoom, pan
 * and cursor state are shared. Large series are reduced into per-pixel
 * min/max envelopes before drawing.
 */
public final class TimelineChartPane extends Region {
  private static final double LEFT_GUTTER = 190.0;
  private static final double RIGHT_GUTTER = 18.0;
  private static final double MODE_HEIGHT = 28.0;
  private static final double AXIS_HEIGHT = 30.0;
  private static final double MIN_LANE_HEIGHT = 78.0;
  private static final String POINTER_HELP = "悬停查看数值；滚轮缩放；拖动平移；双击显示全部";
  private static final Color BACKGROUND = Color.web("#101722");
  private static final Color GRID = Color.web("#2a3545");
  private static final Color TEXT = Color.web("#dbe7f4");
  private static final Color MUTED = Color.web("#8190a5");
  private static final Color CURSOR = Color.web("#f7d154");
  private static final Color[] SERIES_COLORS = {
    Color.web("#58a6ff"),
    Color.web("#3ddc97"),
    Color.web("#ff9f43"),
    Color.web("#d486ff"),
    Color.web("#4dd9e6"),
    Color.web("#ff6b8a")
  };

  private final Canvas canvas = new Canvas();
  private final Tooltip markerTooltip = new Tooltip(POINTER_HELP);
  private final InvalidationListener redrawListener = ignored -> draw();
  private HealthLog log;
  private TimelineViewport viewport = new TimelineViewport(0L, 0L);
  private List<String> selectedSignals = List.of();
  private List<TimelineMarker> markers = List.of();
  private LongConsumer cursorListener = ignored -> {};
  private double pressX;
  private long pressViewStart;
  private long pressViewEnd;
  private boolean dragged;

  public TimelineChartPane() {
    getChildren().add(canvas);
    markerTooltip.setShowDelay(Duration.millis(80.0));
    markerTooltip.setHideDelay(Duration.millis(80.0));
    markerTooltip.setShowDuration(Duration.INDEFINITE);
    Tooltip.install(canvas, markerTooltip);
    widthProperty().addListener(redrawListener);
    heightProperty().addListener(redrawListener);

    canvas.setOnScroll(
        event -> {
          if (log == null || plotWidth() <= 0.0) {
            return;
          }
          double fraction = (event.getX() - LEFT_GUTTER) / plotWidth();
          long anchor = viewport.timestampAtFraction(fraction);
          viewport.zoom(event.getDeltaY() > 0.0 ? 0.80 : 1.25, anchor);
          draw();
          event.consume();
        });

    canvas.setOnMousePressed(
        event -> {
          if (event.getButton() != MouseButton.PRIMARY || log == null) {
            return;
          }
          pressX = event.getX();
          pressViewStart = viewport.viewStartMicros();
          pressViewEnd = viewport.viewEndMicros();
          dragged = false;
        });

    canvas.setOnMouseDragged(
        event -> {
          if (!event.isPrimaryButtonDown() || log == null || plotWidth() <= 0.0) {
            return;
          }
          double deltaPixels = event.getX() - pressX;
          if (Math.abs(deltaPixels) > 2.0) {
            dragged = true;
          }
          long deltaMicros =
              Math.round(
                  -deltaPixels
                      / plotWidth()
                      * (pressViewEnd - pressViewStart));
          viewport.setVisibleRange(
              pressViewStart + deltaMicros, pressViewEnd + deltaMicros);
          draw();
        });

    canvas.setOnMouseReleased(
        event -> {
          if (event.getButton() != MouseButton.PRIMARY || log == null || dragged) {
            return;
          }
          setCursorFromX(event.getX());
        });

    canvas.setOnMouseClicked(
        event -> {
          if (event.getClickCount() == 2 && event.getButton() == MouseButton.PRIMARY) {
            viewport.showAll();
            draw();
          }
        });
    canvas.setOnMouseMoved(
        event -> {
          TimelineMarker nearest = nearestMarker(event.getX());
          if (isInsidePlot(event.getX(), event.getY())) {
            setCursorFromX(event.getX());
            markerTooltip.setText(
                hoverDetails(log, selectedSignals, viewport.cursorMicros(), nearest));
          } else {
            markerTooltip.setText(
                nearest == null
                    ? POINTER_HELP
                    : nearest.title() + "\n" + nearest.detail());
          }
        });
  }

  public void setLog(HealthLog value) {
    log = value;
    if (value == null) {
      viewport = new TimelineViewport(0L, 0L);
      selectedSignals = List.of();
      markers = List.of();
    } else {
      viewport = new TimelineViewport(value.startMicros(), value.endMicros());
    }
    requestLayout();
    draw();
  }

  public void setSelectedSignals(List<String> signalPaths) {
    selectedSignals =
        signalPaths == null
            ? List.of()
            : signalPaths.stream().filter(Objects::nonNull).distinct().toList();
    requestLayout();
    draw();
  }

  public void setMarkers(List<TimelineMarker> value) {
    markers = value == null ? List.of() : List.copyOf(value);
    draw();
  }

  public void setOnCursorChanged(LongConsumer listener) {
    cursorListener = listener == null ? ignored -> {} : listener;
  }

  public TimelineViewport viewport() {
    return viewport;
  }

  public void jumpTo(long timestampMicros) {
    viewport.jumpTo(timestampMicros);
    draw();
    cursorListener.accept(viewport.cursorMicros());
  }

  public void redraw() {
    draw();
  }

  @Override
  protected void layoutChildren() {
    canvas.setWidth(getWidth());
    canvas.setHeight(getHeight());
    draw();
  }

  @Override
  protected double computePrefHeight(double width) {
    return MODE_HEIGHT
        + AXIS_HEIGHT
        + Math.max(1, selectedSignals.size()) * MIN_LANE_HEIGHT;
  }

  private void setCursorFromX(double x) {
    if (plotWidth() <= 0.0) {
      return;
    }
    viewport.setCursorMicros(
        viewport.timestampAtFraction((x - LEFT_GUTTER) / plotWidth()));
    draw();
    cursorListener.accept(viewport.cursorMicros());
  }

  private void draw() {
    double width = canvas.getWidth();
    double height = canvas.getHeight();
    GraphicsContext graphics = canvas.getGraphicsContext2D();
    graphics.setFill(BACKGROUND);
    graphics.fillRect(0.0, 0.0, width, height);
    if (log == null || width <= LEFT_GUTTER + RIGHT_GUTTER || height <= 40.0) {
      drawEmpty(graphics, width, height);
      return;
    }

    double plotWidth = plotWidth();
    drawModeIntervals(graphics, plotWidth);

    int laneCount = Math.max(1, selectedSignals.size());
    double lanesTop = MODE_HEIGHT;
    double laneHeight = Math.max(
        36.0, (height - MODE_HEIGHT - AXIS_HEIGHT) / laneCount);
    for (int index = 0; index < laneCount; index++) {
      double top = lanesTop + index * laneHeight;
      drawLaneBackground(graphics, top, laneHeight, index);
      if (index < selectedSignals.size()) {
        drawSeries(
            graphics,
            selectedSignals.get(index),
            top,
            laneHeight,
            SERIES_COLORS[index % SERIES_COLORS.length],
            plotWidth);
      } else {
        graphics.setFill(MUTED);
        graphics.fillText("请选择至少一个 double 信号", LEFT_GUTTER + 12.0, top + 26.0);
      }
    }

    drawMarkers(graphics, height - AXIS_HEIGHT, plotWidth);
    drawCursor(graphics, height - AXIS_HEIGHT, plotWidth);
    drawTimeAxis(graphics, height, plotWidth);
  }

  private void drawEmpty(GraphicsContext graphics, double width, double height) {
    graphics.setFill(MUTED);
    graphics.setFont(Font.font("System", FontWeight.SEMI_BOLD, 15.0));
    graphics.fillText(
        "打开或拖入 .wpilog 后在左侧勾选信号",
        Math.max(18.0, width / 2.0 - 150.0),
        Math.max(30.0, height / 2.0));
  }

  private void drawModeIntervals(GraphicsContext graphics, double plotWidth) {
    graphics.setFill(Color.web("#151f2d"));
    graphics.fillRect(LEFT_GUTTER, 0.0, plotWidth, MODE_HEIGHT);
    for (RobotModeInterval interval : log.modeIntervals()) {
      long start = Math.max(interval.startMicros(), viewport.viewStartMicros());
      long end = Math.min(interval.endMicros(), viewport.viewEndMicros());
      if (end < start) {
        continue;
      }
      double x = xForTime(start, plotWidth);
      double endX = xForTime(end, plotWidth);
      graphics.setFill(modeColor(interval.mode()));
      graphics.fillRect(x, 0.0, Math.max(1.0, endX - x), MODE_HEIGHT);
      if (endX - x > 42.0) {
        graphics.setFill(TEXT);
        graphics.setFont(Font.font("System", 10.0));
        graphics.fillText(interval.mode(), x + 5.0, 18.0);
      }
    }
    graphics.setFill(TEXT);
    graphics.setFont(Font.font("System", FontWeight.BOLD, 11.0));
    graphics.fillText("机器人模式", 12.0, 18.0);
  }

  private void drawLaneBackground(
      GraphicsContext graphics, double top, double laneHeight, int index) {
    graphics.setFill(index % 2 == 0 ? Color.web("#101722") : Color.web("#121b27"));
    graphics.fillRect(0.0, top, canvas.getWidth(), laneHeight);
    graphics.setStroke(GRID);
    graphics.setLineWidth(1.0);
    graphics.strokeLine(LEFT_GUTTER, top, canvas.getWidth() - RIGHT_GUTTER, top);
    graphics.strokeLine(
        LEFT_GUTTER,
        top + laneHeight / 2.0,
        canvas.getWidth() - RIGHT_GUTTER,
        top + laneHeight / 2.0);
  }

  private void drawSeries(
      GraphicsContext graphics,
      String path,
      double top,
      double laneHeight,
      Color color,
      double plotWidth) {
    List<Sample<Double>> samples = log.doubleSamples(path);
    int from = Math.max(0, lowerBound(samples, viewport.viewStartMicros()) - 1);
    int to = Math.min(samples.size(), upperBound(samples, viewport.viewEndMicros()) + 1);
    double minimum = Double.POSITIVE_INFINITY;
    double maximum = Double.NEGATIVE_INFINITY;
    for (int index = from; index < to; index++) {
      double value = samples.get(index).value();
      if (Double.isFinite(value)) {
        minimum = Math.min(minimum, value);
        maximum = Math.max(maximum, value);
      }
    }

    graphics.setFill(TEXT);
    graphics.setFont(Font.font("System", FontWeight.SEMI_BOLD, 11.0));
    graphics.fillText(shortPath(path, 27), 10.0, top + 18.0);
    if (!Double.isFinite(minimum) || !Double.isFinite(maximum)) {
      graphics.setFill(MUTED);
      graphics.fillText("Unavailable", 10.0, top + 38.0);
      return;
    }

    if (Math.abs(maximum - minimum) < 1e-12) {
      double padding = Math.max(1.0, Math.abs(maximum) * 0.1);
      minimum -= padding;
      maximum += padding;
    } else {
      double padding = (maximum - minimum) * 0.08;
      minimum -= padding;
      maximum += padding;
    }
    graphics.setFill(MUTED);
    graphics.setFont(Font.font("Monospaced", 9.0));
    graphics.fillText(formatValue(maximum), 10.0, top + 34.0);
    graphics.fillText(formatValue(minimum), 10.0, top + laneHeight - 8.0);

    int pixels = Math.max(1, (int) Math.floor(plotWidth));
    double[] bucketMin = new double[pixels];
    double[] bucketMax = new double[pixels];
    Arrays.fill(bucketMin, Double.POSITIVE_INFINITY);
    Arrays.fill(bucketMax, Double.NEGATIVE_INFINITY);
    for (int index = from; index < to; index++) {
      Sample<Double> sample = samples.get(index);
      double value = sample.value();
      if (!Double.isFinite(value)) {
        continue;
      }
      int pixel =
          Math.max(
              0,
              Math.min(
                  pixels - 1,
                  (int)
                      Math.floor(
                          viewport.fractionAtTimestamp(sample.timestampMicros()) * pixels)));
      bucketMin[pixel] = Math.min(bucketMin[pixel], value);
      bucketMax[pixel] = Math.max(bucketMax[pixel], value);
    }

    graphics.setStroke(color);
    graphics.setLineWidth(1.2);
    double plotTop = top + 6.0;
    double plotHeight = Math.max(1.0, laneHeight - 12.0);
    double previousX = Double.NaN;
    double previousY = Double.NaN;
    for (int pixel = 0; pixel < pixels; pixel++) {
      if (!Double.isFinite(bucketMin[pixel])) {
        continue;
      }
      double x = LEFT_GUTTER + pixel + 0.5;
      double minY = yForValue(bucketMin[pixel], minimum, maximum, plotTop, plotHeight);
      double maxY = yForValue(bucketMax[pixel], minimum, maximum, plotTop, plotHeight);
      graphics.strokeLine(x, minY, x, maxY);
      double centerY = (minY + maxY) / 2.0;
      if (Double.isFinite(previousX) && x - previousX < 4.0) {
        graphics.strokeLine(previousX, previousY, x, centerY);
      }
      previousX = x;
      previousY = centerY;
    }
    drawCursorValue(graphics, path, top, laneHeight, color, minimum, maximum, plotWidth);
  }

  private void drawCursorValue(
      GraphicsContext graphics,
      String path,
      double top,
      double laneHeight,
      Color color,
      double minimum,
      double maximum,
      double plotWidth) {
    long cursor = viewport.cursorMicros();
    String display = SnapshotPane.numericDisplay(log, path, cursor);
    boolean available = !display.startsWith("Unavailable");
    graphics.setFill(available ? color : MUTED);
    graphics.setFont(Font.font("Monospaced", FontWeight.BOLD, 10.0));
    graphics.fillText("当前 " + display, 10.0, top + Math.min(51.0, laneHeight - 21.0));

    Optional<Double> value = SnapshotPane.numericValue(log, path, cursor);
    if (value.isEmpty()
        || cursor < viewport.viewStartMicros()
        || cursor > viewport.viewEndMicros()) {
      return;
    }
    double x = xForTime(cursor, plotWidth);
    double y =
        yForValue(
            value.get(),
            minimum,
            maximum,
            top + 6.0,
            Math.max(1.0, laneHeight - 12.0));
    graphics.setFill(color);
    graphics.fillOval(x - 3.5, y - 3.5, 7.0, 7.0);
    graphics.setStroke(Color.WHITE);
    graphics.setLineWidth(1.0);
    graphics.strokeOval(x - 3.5, y - 3.5, 7.0, 7.0);
  }

  private void drawMarkers(GraphicsContext graphics, double bottom, double plotWidth) {
    double lastLabelX = Double.NEGATIVE_INFINITY;
    for (TimelineMarker marker : combinedMarkers()) {
      if (marker.timestampMicros() < viewport.viewStartMicros()
          || marker.timestampMicros() > viewport.viewEndMicros()) {
        continue;
      }
      double x = xForTime(marker.timestampMicros(), plotWidth);
      Color color = markerColor(marker.kind());
      graphics.setStroke(color.deriveColor(0.0, 1.0, 1.0, 0.55));
      graphics.setLineWidth(marker.kind() == TimelineMarker.Kind.EVENT ? 1.0 : 1.6);
      graphics.strokeLine(x, MODE_HEIGHT, x, bottom);
      graphics.setFill(color);
      graphics.fillPolygon(
          new double[] {x - 5.0, x + 5.0, x},
          new double[] {MODE_HEIGHT + 2.0, MODE_HEIGHT + 2.0, MODE_HEIGHT + 10.0},
          3);
      if (x - lastLabelX > 95.0) {
        graphics.setFill(color);
        graphics.setFont(Font.font("System", FontWeight.SEMI_BOLD, 9.0));
        graphics.fillText(shortPath(marker.title(), 16), x + 4.0, MODE_HEIGHT + 12.0);
        lastLabelX = x;
      }
    }
  }

  private List<TimelineMarker> combinedMarkers() {
    ArrayList<TimelineMarker> combined = new ArrayList<>(markers);
    if (log != null) {
      for (HealthEvent event : log.events()) {
        combined.add(
            new TimelineMarker(
                event.timestampMicros(),
                TimelineMarker.Kind.EVENT,
                event.category(),
                event.message()));
      }
    }
    return combined;
  }

  private TimelineMarker nearestMarker(double mouseX) {
    if (log == null || plotWidth() <= 0.0) {
      return null;
    }
    TimelineMarker nearest = null;
    double nearestDistance = 8.0;
    for (TimelineMarker marker : combinedMarkers()) {
      if (marker.timestampMicros() < viewport.viewStartMicros()
          || marker.timestampMicros() > viewport.viewEndMicros()) {
        continue;
      }
      double distance = Math.abs(mouseX - xForTime(marker.timestampMicros(), plotWidth()));
      if (distance < nearestDistance) {
        nearestDistance = distance;
        nearest = marker;
      }
    }
    return nearest;
  }

  private boolean isInsidePlot(double x, double y) {
    return log != null
        && x >= LEFT_GUTTER
        && x <= canvas.getWidth() - RIGHT_GUTTER
        && y >= 0.0
        && y <= canvas.getHeight() - AXIS_HEIGHT;
  }

  static String hoverDetails(
      HealthLog log,
      List<String> signalPaths,
      long timestampMicros,
      TimelineMarker nearestMarker) {
    if (log == null) {
      return POINTER_HELP;
    }
    double relativeSeconds = (timestampMicros - log.startMicros()) / 1_000_000.0;
    StringBuilder text =
        new StringBuilder(String.format(Locale.ROOT, "时间 %,.3f s", relativeSeconds));
    List<String> paths = signalPaths == null ? List.of() : signalPaths;
    if (paths.isEmpty()) {
      text.append("\n未选择数值信号");
    } else {
      for (String path : paths) {
        text.append('\n')
            .append(shortPath(path, 42))
            .append(" = ")
            .append(SnapshotPane.numericDisplay(log, path, timestampMicros));
      }
    }
    if (nearestMarker != null) {
      text.append("\n\n")
          .append(nearestMarker.title())
          .append('\n')
          .append(nearestMarker.detail());
    }
    return text.toString();
  }

  private void drawCursor(GraphicsContext graphics, double bottom, double plotWidth) {
    long cursor = viewport.cursorMicros();
    if (cursor < viewport.viewStartMicros() || cursor > viewport.viewEndMicros()) {
      return;
    }
    double x = xForTime(cursor, plotWidth);
    graphics.setStroke(CURSOR);
    graphics.setLineWidth(1.2);
    graphics.strokeLine(x, 0.0, x, bottom);
    String label = formatRelativeSeconds(cursor);
    graphics.setFont(Font.font("Monospaced", FontWeight.BOLD, 10.0));
    double labelWidth = Math.max(56.0, label.length() * 7.0);
    double labelX =
        Math.max(
            LEFT_GUTTER,
            Math.min(canvas.getWidth() - RIGHT_GUTTER - labelWidth, x - labelWidth / 2.0));
    graphics.setFill(Color.web("#4b421d"));
    graphics.fillRoundRect(labelX, 3.0, labelWidth, 20.0, 5.0, 5.0);
    graphics.setFill(Color.WHITE);
    graphics.fillText(label, labelX + 6.0, 17.0);
  }

  private void drawTimeAxis(GraphicsContext graphics, double height, double plotWidth) {
    double y = height - AXIS_HEIGHT;
    graphics.setFill(Color.web("#0c1119"));
    graphics.fillRect(0.0, y, canvas.getWidth(), AXIS_HEIGHT);
    graphics.setStroke(GRID);
    graphics.strokeLine(LEFT_GUTTER, y, canvas.getWidth() - RIGHT_GUTTER, y);
    graphics.setFill(MUTED);
    graphics.setFont(Font.font("Monospaced", 10.0));
    int ticks = Math.max(2, Math.min(10, (int) (plotWidth / 120.0)));
    for (int index = 0; index <= ticks; index++) {
      double fraction = (double) index / ticks;
      double x = LEFT_GUTTER + fraction * plotWidth;
      long timestamp = viewport.timestampAtFraction(fraction);
      graphics.setStroke(GRID);
      graphics.strokeLine(x, y, x, y + 5.0);
      String label = formatRelativeSeconds(timestamp);
      graphics.fillText(label, x - 22.0, y + 19.0);
    }
  }

  private double plotWidth() {
    return Math.max(0.0, canvas.getWidth() - LEFT_GUTTER - RIGHT_GUTTER);
  }

  private double xForTime(long timestampMicros, double plotWidth) {
    return LEFT_GUTTER + viewport.fractionAtTimestamp(timestampMicros) * plotWidth;
  }

  private String formatRelativeSeconds(long timestampMicros) {
    double seconds = (timestampMicros - log.startMicros()) / 1_000_000.0;
    return String.format(Locale.ROOT, "%.3fs", seconds);
  }

  private static double yForValue(
      double value,
      double minimum,
      double maximum,
      double plotTop,
      double plotHeight) {
    double normalized = (value - minimum) / (maximum - minimum);
    return plotTop + (1.0 - normalized) * plotHeight;
  }

  private static int lowerBound(List<Sample<Double>> samples, long timestampMicros) {
    int low = 0;
    int high = samples.size();
    while (low < high) {
      int middle = (low + high) >>> 1;
      if (samples.get(middle).timestampMicros() < timestampMicros) {
        low = middle + 1;
      } else {
        high = middle;
      }
    }
    return low;
  }

  private static int upperBound(List<Sample<Double>> samples, long timestampMicros) {
    int low = 0;
    int high = samples.size();
    while (low < high) {
      int middle = (low + high) >>> 1;
      if (samples.get(middle).timestampMicros() <= timestampMicros) {
        low = middle + 1;
      } else {
        high = middle;
      }
    }
    return low;
  }

  private static Color modeColor(String mode) {
    String normalized = mode == null ? "" : mode.toLowerCase(Locale.ROOT);
    if (normalized.contains("auto")) {
      return Color.web("#4d2f72", 0.92);
    }
    if (normalized.contains("teleop")) {
      return Color.web("#164d44", 0.92);
    }
    if (normalized.contains("test")) {
      return Color.web("#6a481a", 0.92);
    }
    return Color.web("#303a49", 0.92);
  }

  private static Color markerColor(TimelineMarker.Kind kind) {
    return switch (kind) {
      case EVENT -> Color.web("#7ca9d8");
      case WARNING -> Color.web("#f5b942");
      case ERROR -> Color.web("#ff765f");
      case CRITICAL -> Color.web("#ff3b5f");
    };
  }

  private static String shortPath(String path, int maximumCharacters) {
    if (path.length() <= maximumCharacters) {
      return path;
    }
    return "…" + path.substring(path.length() - maximumCharacters + 1);
  }

  private static String formatValue(double value) {
    double absolute = Math.abs(value);
    if (absolute >= 1_000.0 || (absolute > 0.0 && absolute < 0.01)) {
      return String.format(Locale.ROOT, "%.2e", value);
    }
    return String.format(Locale.ROOT, "%.2f", value);
  }
}
