package frc.robot.health.analyzer.io;

import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import edu.wpi.first.util.datalog.DataLogRecord.StartRecordData;
import frc.robot.health.analyzer.model.HealthLog;
import frc.robot.health.analyzer.model.Sample;
import frc.robot.health.analyzer.model.SignalType;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.BufferUnderflowException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.InputMismatchException;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/** Loads scalar {@code /Health/...} records from a standard WPILOG file. */
public final class WpiLogLoader {
  private static final String HEALTH_PREFIX = "/Health/";
  private static final long DEFAULT_MAX_LOG_BYTES = 512L * 1024L * 1024L;
  private final long maximumLogBytes;

  public WpiLogLoader() {
    this(Long.getLong("healthAnalyzer.maxLogBytes", DEFAULT_MAX_LOG_BYTES));
  }

  WpiLogLoader(long maximumLogBytes) {
    if (maximumLogBytes <= 0L || maximumLogBytes > Integer.MAX_VALUE) {
      throw new IllegalArgumentException("maximumLogBytes must be within 1..Integer.MAX_VALUE");
    }
    this.maximumLogBytes = maximumLogBytes;
  }

  /**
   * Reads a WPILOG into an immutable typed health data set.
   *
   * <p>Non-health entries and unsupported WPILOG types are intentionally ignored.
   *
   * @throws IOException if the file cannot be read or contains an invalid health record
   */
  public HealthLog load(Path path) throws IOException {
    Objects.requireNonNull(path, "path");
    if (!Files.isRegularFile(path)) {
      throw new IOException("WPILOG file does not exist or is not a regular file: " + path);
    }

    long fileSize = Files.size(path);
    if (fileSize > maximumLogBytes) {
      throw new IOException(
          "WPILOG is %,d bytes, above this analyzer's %,d-byte in-memory safety limit. "
                  .formatted(fileSize, maximumLogBytes)
              + "Trim the log or raise -DhealthAnalyzer.maxLogBytes after confirming heap space: "
              + path);
    }

    // DataLogReader(String) memory-maps the file and has no close method, which keeps the file
    // locked on Windows. A heap buffer releases the file handle as soon as readAllBytes returns.
    byte[] contents = Files.readAllBytes(path);
    if (contents.length > maximumLogBytes) {
      throw new IOException(
          "WPILOG grew to %,d bytes while being read, above the %,d-byte safety limit: %s"
              .formatted(contents.length, maximumLogBytes, path));
    }
    DataLogReader reader = new DataLogReader(ByteBuffer.wrap(contents));
    if (!reader.isValid()) {
      throw new IOException("Invalid WPILOG header: " + path);
    }
    validateExtraHeader(contents, path);

    LoadState state = new LoadState();
    try {
      // Use DataLogReader.forEach(), not its Iterator.hasNext(). The 2026 iterator conservatively
      // requires 16 bytes to remain and can therefore omit a legal short final scalar record.
      reader.forEach(
          record -> {
            try {
              state.accept(record);
            } catch (IOException ex) {
              throw new UncheckedIOException(ex);
            }
          });
    } catch (UncheckedIOException ex) {
      throw ex.getCause();
    } catch (BufferUnderflowException | InputMismatchException | IndexOutOfBoundsException ex) {
      throw new IOException("Malformed WPILOG record in " + path, ex);
    }
    return state.toHealthLog();
  }

  private static void validateExtraHeader(byte[] contents, Path path) throws IOException {
    int extraHeaderSize =
        ByteBuffer.wrap(contents).order(ByteOrder.LITTLE_ENDIAN).getInt(8);
    if (extraHeaderSize < 0 || 12L + extraHeaderSize > contents.length) {
      throw new IOException("Invalid WPILOG extra-header size: " + path);
    }
  }

  private static final class LoadState {
    Map<Integer, EntryDescriptor> entries = new HashMap<>();
    Map<String, SignalType> pathTypes = new HashMap<>();
    Map<String, List<Sample<Boolean>>> booleanSignals = new LinkedHashMap<>();
    Map<String, List<Sample<Double>>> doubleSignals = new LinkedHashMap<>();
    Map<String, List<Sample<String>>> stringSignals = new LinkedHashMap<>();
    long firstTimestamp = Long.MAX_VALUE;
    long lastTimestamp = Long.MIN_VALUE;

    void accept(DataLogRecord record) throws IOException {
      if (record.isStart()) {
        StartRecordData start = record.getStartData();
        entries.remove(start.entry);
        if (!start.name.startsWith(HEALTH_PREFIX)) {
          return;
        }

        SignalType type = SignalType.fromWpiType(start.type).orElse(null);
        if (type == null) {
          return;
        }
        SignalType previousType = pathTypes.putIfAbsent(start.name, type);
        if (previousType != null && previousType != type) {
          throw new IOException(
              "Health signal changed type from "
                  + previousType.wpiType()
                  + " to "
                  + type.wpiType()
                  + ": "
                  + start.name);
        }

        EntryDescriptor descriptor = new EntryDescriptor(start.name, type);
        entries.put(start.entry, descriptor);
        ensureSeries(descriptor, booleanSignals, doubleSignals, stringSignals);
        return;
      }

      if (record.isFinish()) {
        entries.remove(record.getFinishEntry());
        return;
      }

      if (record.isControl()) {
        return;
      }

      EntryDescriptor descriptor = entries.get(record.getEntry());
      if (descriptor == null) {
        return;
      }

      long timestamp = record.getTimestamp();
      if (timestamp < 0) {
        throw new IOException(
            "Health signal has a timestamp outside the supported signed range: "
                + descriptor.path());
      }

      switch (descriptor.type()) {
        case BOOLEAN -> {
          requirePayloadSize(record, 1, descriptor);
          booleanSignals
              .get(descriptor.path())
              .add(new Sample<>(timestamp, record.getBoolean()));
        }
        case DOUBLE -> {
          requirePayloadSize(record, Double.BYTES, descriptor);
          doubleSignals
              .get(descriptor.path())
              .add(new Sample<>(timestamp, record.getDouble()));
        }
        case INTEGER -> {
          requirePayloadSize(record, Long.BYTES, descriptor);
          doubleSignals
              .get(descriptor.path())
              .add(new Sample<>(timestamp, (double) record.getInteger()));
        }
        case STRING ->
            stringSignals
                .get(descriptor.path())
                .add(new Sample<>(timestamp, record.getString()));
      }
      firstTimestamp = Math.min(firstTimestamp, timestamp);
      lastTimestamp = Math.max(lastTimestamp, timestamp);
    }

    HealthLog toHealthLog() {
      long start = firstTimestamp;
      long end = lastTimestamp;
      if (start == Long.MAX_VALUE) {
        start = 0;
        end = 0;
      }
      return new HealthLog(booleanSignals, doubleSignals, stringSignals, start, end);
    }
  }

  private static void ensureSeries(
      EntryDescriptor descriptor,
      Map<String, List<Sample<Boolean>>> booleanSignals,
      Map<String, List<Sample<Double>>> doubleSignals,
      Map<String, List<Sample<String>>> stringSignals) {
    switch (descriptor.type()) {
      case BOOLEAN ->
          booleanSignals.computeIfAbsent(descriptor.path(), ignored -> new ArrayList<>());
      case DOUBLE, INTEGER ->
          doubleSignals.computeIfAbsent(descriptor.path(), ignored -> new ArrayList<>());
      case STRING -> stringSignals.computeIfAbsent(descriptor.path(), ignored -> new ArrayList<>());
    }
  }

  private static void requirePayloadSize(
      DataLogRecord record, int expectedSize, EntryDescriptor descriptor) throws IOException {
    if (record.getSize() != expectedSize) {
      throw new IOException(
          "Invalid "
              + descriptor.type().wpiType()
              + " payload size for "
              + descriptor.path()
              + " at "
              + record.getTimestamp()
              + " us: expected "
              + expectedSize
              + " bytes, got "
              + record.getSize());
    }
  }

  private record EntryDescriptor(String path, SignalType type) {}
}
