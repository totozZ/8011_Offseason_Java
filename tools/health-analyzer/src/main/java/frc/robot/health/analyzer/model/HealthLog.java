package frc.robot.health.analyzer.model;

import frc.robot.health.analyzer.analysis.SeriesMath;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;
import java.util.TreeSet;

/** Immutable, typed view of the scalar {@code /Health/...} signals in a WPILOG. */
public final class HealthLog {
  public static final String EVENT_CATEGORY_PATH = "/Health/Events/Category";
  public static final String EVENT_MESSAGE_PATH = "/Health/Events/Message";
  public static final String ROBOT_MODE_PATH = "/Health/Robot/Mode";
  public static final String TEST_SESSION_ACTIVE_PATH = "/Health/TestSession/Active";
  public static final String TEST_SESSION_NAME_PATH = "/Health/TestSession/Name";

  private final Map<String, List<Sample<Boolean>>> booleanSignals;
  private final Map<String, List<Sample<Double>>> doubleSignals;
  private final Map<String, List<Sample<String>>> stringSignals;
  private final long startMicros;
  private final long endMicros;
  private final List<HealthEvent> events;
  private final List<RobotModeInterval> modeIntervals;
  private final List<TestSessionInterval> testSessionIntervals;

  public HealthLog(
      Map<String, List<Sample<Boolean>>> booleanSignals,
      Map<String, List<Sample<Double>>> doubleSignals,
      Map<String, List<Sample<String>>> stringSignals,
      long startMicros,
      long endMicros) {
    if (startMicros < 0) {
      throw new IllegalArgumentException("startMicros must be non-negative");
    }
    if (endMicros < startMicros) {
      throw new IllegalArgumentException("endMicros must not precede startMicros");
    }

    this.booleanSignals = immutableSignals(booleanSignals, "booleanSignals");
    this.doubleSignals = immutableSignals(doubleSignals, "doubleSignals");
    this.stringSignals = immutableSignals(stringSignals, "stringSignals");
    this.startMicros = startMicros;
    this.endMicros = endMicros;
    events = deriveEvents();
    modeIntervals = deriveModeIntervals();
    testSessionIntervals = deriveTestSessionIntervals();
  }

  /** Returns every boolean series, keyed by its stable WPILOG path. */
  public Map<String, List<Sample<Boolean>>> booleanSignals() {
    return booleanSignals;
  }

  /** Returns every double series, keyed by its stable WPILOG path. */
  public Map<String, List<Sample<Double>>> doubleSignals() {
    return doubleSignals;
  }

  /** Returns every string series, keyed by its stable WPILOG path. */
  public Map<String, List<Sample<String>>> stringSignals() {
    return stringSignals;
  }

  /** Alias for callers that use "series" terminology. */
  public Map<String, List<Sample<Boolean>>> booleanSeries() {
    return booleanSignals;
  }

  /** Alias for callers that use "series" terminology. */
  public Map<String, List<Sample<Double>>> doubleSeries() {
    return doubleSignals;
  }

  /** Alias for callers that use "series" terminology. */
  public Map<String, List<Sample<String>>> stringSeries() {
    return stringSignals;
  }

  public Set<String> signalPaths() {
    TreeSet<String> paths = new TreeSet<>();
    paths.addAll(booleanSignals.keySet());
    paths.addAll(doubleSignals.keySet());
    paths.addAll(stringSignals.keySet());
    return Collections.unmodifiableSet(paths);
  }

  public Optional<SignalType> signalType(String path) {
    Objects.requireNonNull(path, "path");
    if (booleanSignals.containsKey(path)) {
      return Optional.of(SignalType.BOOLEAN);
    }
    if (doubleSignals.containsKey(path)) {
      return Optional.of(SignalType.DOUBLE);
    }
    if (stringSignals.containsKey(path)) {
      return Optional.of(SignalType.STRING);
    }
    return Optional.empty();
  }

  public List<Sample<Boolean>> booleanSamples(String path) {
    return booleanSignals.getOrDefault(Objects.requireNonNull(path, "path"), List.of());
  }

  public List<Sample<Boolean>> booleanSeries(String path) {
    return booleanSamples(path);
  }

  public List<Sample<Double>> doubleSamples(String path) {
    return doubleSignals.getOrDefault(Objects.requireNonNull(path, "path"), List.of());
  }

  public List<Sample<Double>> doubleSeries(String path) {
    return doubleSamples(path);
  }

  public List<Sample<String>> stringSamples(String path) {
    return stringSignals.getOrDefault(Objects.requireNonNull(path, "path"), List.of());
  }

  public List<Sample<String>> stringSeries(String path) {
    return stringSamples(path);
  }

  public Optional<Sample<Boolean>> nearestBoolean(String path, long timestampMicros) {
    return nearest(booleanSamples(path), timestampMicros);
  }

  public Optional<Sample<Double>> nearestDouble(String path, long timestampMicros) {
    return nearest(doubleSamples(path), timestampMicros);
  }

  public Optional<Sample<String>> nearestString(String path, long timestampMicros) {
    return nearest(stringSamples(path), timestampMicros);
  }

  public Optional<Sample<Boolean>> latestBoolean(String path, long timestampMicros) {
    return latest(booleanSamples(path), timestampMicros);
  }

  public Optional<Sample<Double>> latestDouble(String path, long timestampMicros) {
    return latest(doubleSamples(path), timestampMicros);
  }

  public Optional<Sample<String>> latestString(String path, long timestampMicros) {
    return latest(stringSamples(path), timestampMicros);
  }

  public long startMicros() {
    return startMicros;
  }

  public long endMicros() {
    return endMicros;
  }

  public long durationMicros() {
    return endMicros - startMicros;
  }

  public List<HealthEvent> events() {
    return events;
  }

  public List<RobotModeInterval> modeIntervals() {
    return modeIntervals;
  }

  public List<TestSessionInterval> testSessionIntervals() {
    return testSessionIntervals;
  }

  /**
   * Returns a timestamp-clipped view. State signals carry their latest value into the start
   * boundary; numeric signals interpolate only while their real sample coverage exists.
   */
  public HealthLog slice(long rangeStartMicros, long rangeEndMicros) {
    if (rangeStartMicros < startMicros
        || rangeEndMicros > endMicros
        || rangeEndMicros < rangeStartMicros) {
      throw new IllegalArgumentException("slice must remain inside the log time range");
    }
    Map<String, List<Sample<Boolean>>> booleans = new LinkedHashMap<>();
    booleanSignals.forEach(
        (path, samples) ->
            booleans.put(path, sliceStep(samples, rangeStartMicros, rangeEndMicros)));
    Map<String, List<Sample<String>>> strings = new LinkedHashMap<>();
    stringSignals.forEach(
        (path, samples) ->
            strings.put(
                path,
                path.equals(EVENT_CATEGORY_PATH) || path.equals(EVENT_MESSAGE_PATH)
                    ? sliceDiscrete(samples, rangeStartMicros, rangeEndMicros)
                    : sliceStep(samples, rangeStartMicros, rangeEndMicros)));
    Map<String, List<Sample<Double>>> doubles = new LinkedHashMap<>();
    doubleSignals.forEach(
        (path, samples) ->
            doubles.put(
                path,
                isStepNumericPath(path)
                    ? sliceStep(samples, rangeStartMicros, rangeEndMicros)
                    : sliceDouble(samples, rangeStartMicros, rangeEndMicros)));
    return new HealthLog(booleans, doubles, strings, rangeStartMicros, rangeEndMicros);
  }

  private List<HealthEvent> deriveEvents() {
    List<Sample<String>> categories = stringSamples(EVENT_CATEGORY_PATH);
    List<Sample<String>> messages = stringSamples(EVENT_MESSAGE_PATH);
    if (messages.isEmpty()) {
      return List.of();
    }

    ArrayList<HealthEvent> result = new ArrayList<>(messages.size());
    for (int index = 0; index < messages.size(); index++) {
      Sample<String> message = messages.get(index);
      Sample<String> category =
          index < categories.size()
              ? categories.get(index)
              : latest(categories, message.timestampMicros()).orElse(null);
      String categoryValue = category == null ? "Uncategorized" : category.value();
      long timestamp =
          category == null
              ? message.timestampMicros()
              : Math.max(category.timestampMicros(), message.timestampMicros());
      result.add(new HealthEvent(timestamp, categoryValue, message.value()));
    }
    result.sort(Comparator.comparingLong(HealthEvent::timestampMicros));
    return List.copyOf(result);
  }

  private List<RobotModeInterval> deriveModeIntervals() {
    List<Sample<String>> samples = stringSamples(ROBOT_MODE_PATH);
    if (samples.isEmpty()) {
      return List.of();
    }

    ArrayList<Sample<String>> changes = new ArrayList<>(samples.size());
    for (Sample<String> sample : samples) {
      if (changes.isEmpty() || !changes.get(changes.size() - 1).value().equals(sample.value())) {
        changes.add(sample);
      }
    }

    ArrayList<RobotModeInterval> intervals = new ArrayList<>(changes.size());
    for (int index = 0; index < changes.size(); index++) {
      Sample<String> change = changes.get(index);
      long start = Math.max(startMicros, change.timestampMicros());
      long end =
          index + 1 < changes.size()
              ? Math.min(endMicros, changes.get(index + 1).timestampMicros())
              : endMicros;
      if (end >= start) {
        intervals.add(new RobotModeInterval(change.value(), start, end));
      }
    }
    return List.copyOf(intervals);
  }

  private List<TestSessionInterval> deriveTestSessionIntervals() {
    List<Sample<Boolean>> activeSamples = booleanSamples(TEST_SESSION_ACTIVE_PATH);
    if (activeSamples.isEmpty()) {
      return List.of();
    }
    ArrayList<TestSessionInterval> intervals = new ArrayList<>();
    boolean active = false;
    long activeStart = startMicros;
    int ordinal = 1;
    for (Sample<Boolean> sample : activeSamples) {
      long timestamp = Math.max(startMicros, Math.min(endMicros, sample.timestampMicros()));
      if (sample.value() && !active) {
        active = true;
        activeStart = timestamp;
      } else if (!sample.value() && active) {
        intervals.add(
            new TestSessionInterval(
                sessionNameAt(activeStart), activeStart, timestamp, ordinal++));
        active = false;
      }
    }
    if (active) {
      intervals.add(
          new TestSessionInterval(
              sessionNameAt(activeStart), activeStart, endMicros, ordinal));
    }
    return List.copyOf(intervals);
  }

  private String sessionNameAt(long timestampMicros) {
    return latestString(TEST_SESSION_NAME_PATH, timestampMicros)
        .map(Sample::value)
        .filter(value -> !value.isBlank())
        .orElse("Unnamed");
  }

  private static <T> List<Sample<T>> sliceStep(
      List<Sample<T>> samples, long startMicros, long endMicros) {
    if (samples.isEmpty()) {
      return List.of();
    }
    ArrayList<Sample<T>> result = new ArrayList<>();
    latest(samples, startMicros)
        .ifPresent(sample -> result.add(new Sample<>(startMicros, sample.value())));
    for (Sample<T> sample : samples) {
      if (sample.timestampMicros() > startMicros
          && sample.timestampMicros() <= endMicros) {
        result.add(sample);
      }
    }
    return List.copyOf(result);
  }

  private static <T> List<Sample<T>> sliceDiscrete(
      List<Sample<T>> samples, long startMicros, long endMicros) {
    return samples.stream()
        .filter(
            sample ->
                sample.timestampMicros() >= startMicros
                    && sample.timestampMicros() <= endMicros)
        .toList();
  }

  private static List<Sample<Double>> sliceDouble(
      List<Sample<Double>> samples, long startMicros, long endMicros) {
    if (samples.isEmpty()) {
      return List.of();
    }
    ArrayList<Sample<Double>> result = new ArrayList<>();
    double startValue = SeriesMath.valueAt(samples, startMicros);
    if (Double.isFinite(startValue)) {
      result.add(new Sample<>(startMicros, startValue));
    }
    for (Sample<Double> sample : samples) {
      if (sample.timestampMicros() > startMicros
          && sample.timestampMicros() < endMicros) {
        result.add(sample);
      }
    }
    double endValue = SeriesMath.valueAt(samples, endMicros);
    if (endMicros > startMicros && Double.isFinite(endValue)) {
      result.add(new Sample<>(endMicros, endValue));
    }
    return List.copyOf(result);
  }

  /** Returns whether a numeric path represents change-driven state rather than sampled telemetry. */
  public static boolean isStepNumericPath(String path) {
    Objects.requireNonNull(path, "path");
    return path.endsWith("/DeviceId")
        || path.endsWith("/Follower/LeaderDeviceId")
        || path.endsWith("/Faults")
        || path.endsWith("/StickyFaults")
        || path.equals("/Health/Power/DistributionModule")
        || path.equals("/Health/Logger/ErrorCount")
        || (path.startsWith("/Health/Robot/CAN/")
            && (path.endsWith("Count")
                || path.endsWith("Errors")));
  }

  private static <T> Map<String, List<Sample<T>>> immutableSignals(
      Map<String, List<Sample<T>>> source, String argumentName) {
    Objects.requireNonNull(source, argumentName);
    LinkedHashMap<String, List<Sample<T>>> copy = new LinkedHashMap<>();
    source.keySet().stream()
        .sorted()
        .forEach(
            path -> {
              Objects.requireNonNull(path, argumentName + " contains a null path");
              List<Sample<T>> samples =
                  new ArrayList<>(
                      Objects.requireNonNull(
                          source.get(path), argumentName + " contains a null series"));
              samples.forEach(sample -> Objects.requireNonNull(sample, "series contains null sample"));
              samples.sort(Comparator.comparingLong(Sample::timestampMicros));
              copy.put(path, List.copyOf(samples));
            });
    return Collections.unmodifiableMap(copy);
  }

  private static <T> Optional<Sample<T>> latest(
      List<Sample<T>> samples, long timestampMicros) {
    int low = 0;
    int high = samples.size() - 1;
    int result = -1;
    while (low <= high) {
      int middle = (low + high) >>> 1;
      if (samples.get(middle).timestampMicros() <= timestampMicros) {
        result = middle;
        low = middle + 1;
      } else {
        high = middle - 1;
      }
    }
    return result < 0 ? Optional.empty() : Optional.of(samples.get(result));
  }

  private static <T> Optional<Sample<T>> nearest(
      List<Sample<T>> samples, long timestampMicros) {
    if (samples.isEmpty()) {
      return Optional.empty();
    }

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

    if (low == 0) {
      return Optional.of(samples.get(0));
    }
    if (low == samples.size()) {
      return Optional.of(samples.get(samples.size() - 1));
    }

    Sample<T> before = samples.get(low - 1);
    Sample<T> after = samples.get(low);
    long distanceBefore = timestampMicros - before.timestampMicros();
    long distanceAfter = after.timestampMicros() - timestampMicros;
    return Optional.of(distanceBefore <= distanceAfter ? before : after);
  }
}
