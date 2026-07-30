package frc.robot.health.analyzer.io.wpilog;

import frc.robot.health.analyzer.model.SignalType;
import java.io.BufferedOutputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

/**
 * Minimal, deterministic WPILOG 1.0 writer for tests and offline fixtures.
 *
 * <p>This implementation writes the file format directly and does not load WPILib JNI.
 */
public final class PureJavaWpiLogWriter implements AutoCloseable {
  private static final byte[] MAGIC = "WPILOG".getBytes(StandardCharsets.US_ASCII);
  private static final int VERSION_1_0 = 0x0100;
  private static final int CONTROL_START = 0;
  private static final int CONTROL_FINISH = 1;

  private final OutputStream output;
  private final Map<Integer, SignalType> entryTypes = new HashMap<>();
  private int nextEntry = 1;
  private boolean closed;

  public PureJavaWpiLogWriter(Path path) throws IOException {
    this(path, "");
  }

  public PureJavaWpiLogWriter(Path path, String extraHeader) throws IOException {
    Objects.requireNonNull(path, "path");
    output =
        new BufferedOutputStream(
            Files.newOutputStream(
                path,
                StandardOpenOption.CREATE,
                StandardOpenOption.TRUNCATE_EXISTING,
                StandardOpenOption.WRITE));
    try {
      writeFileHeader(Objects.requireNonNull(extraHeader, "extraHeader"));
    } catch (IOException | RuntimeException ex) {
      try {
        output.close();
      } catch (IOException closeException) {
        ex.addSuppressed(closeException);
      }
      throw ex;
    }
  }

  public int start(String name, SignalType type, long timestampMicros) throws IOException {
    return start(name, type, "", timestampMicros);
  }

  public int start(
      String name, SignalType type, String metadata, long timestampMicros) throws IOException {
    ensureOpen();
    if (nextEntry == Integer.MAX_VALUE) {
      throw new IllegalStateException("WPILOG fixture exhausted entry identifiers");
    }
    if (Objects.requireNonNull(name, "name").isEmpty()) {
      throw new IllegalArgumentException("name must not be empty");
    }
    Objects.requireNonNull(type, "type");
    Objects.requireNonNull(metadata, "metadata");
    requireTimestamp(timestampMicros);

    int entry = nextEntry++;
    ByteArrayOutputStream payload = new ByteArrayOutputStream();
    payload.write(CONTROL_START);
    writeLittleEndian(payload, entry, Integer.BYTES);
    writeInnerString(payload, name);
    writeInnerString(payload, type.wpiType());
    writeInnerString(payload, metadata);
    writeRecord(0, timestampMicros, payload.toByteArray());
    entryTypes.put(entry, type);
    return entry;
  }

  public int startBoolean(String name, long timestampMicros) throws IOException {
    return start(name, SignalType.BOOLEAN, timestampMicros);
  }

  public int startDouble(String name, long timestampMicros) throws IOException {
    return start(name, SignalType.DOUBLE, timestampMicros);
  }

  public int startInteger(String name, long timestampMicros) throws IOException {
    return start(name, SignalType.INTEGER, timestampMicros);
  }

  public int startString(String name, long timestampMicros) throws IOException {
    return start(name, SignalType.STRING, timestampMicros);
  }

  /** Appends a boolean using the same argument order as WPILib's DataLog append APIs. */
  public void appendBoolean(int entry, boolean value, long timestampMicros) throws IOException {
    requireType(entry, SignalType.BOOLEAN);
    writeRecord(entry, timestampMicros, new byte[] {(byte) (value ? 1 : 0)});
  }

  /** Appends a double using the same argument order as WPILib's DataLog append APIs. */
  public void appendDouble(int entry, double value, long timestampMicros) throws IOException {
    requireType(entry, SignalType.DOUBLE);
    byte[] payload =
        ByteBuffer.allocate(Double.BYTES)
            .order(ByteOrder.LITTLE_ENDIAN)
            .putDouble(value)
            .array();
    writeRecord(entry, timestampMicros, payload);
  }

  /** Appends a signed int64 using the same argument order as WPILib's DataLog append APIs. */
  public void appendInteger(int entry, long value, long timestampMicros) throws IOException {
    requireType(entry, SignalType.INTEGER);
    byte[] payload =
        ByteBuffer.allocate(Long.BYTES)
            .order(ByteOrder.LITTLE_ENDIAN)
            .putLong(value)
            .array();
    writeRecord(entry, timestampMicros, payload);
  }

  /** Appends a UTF-8 string using the same argument order as WPILib's DataLog append APIs. */
  public void appendString(int entry, String value, long timestampMicros) throws IOException {
    requireType(entry, SignalType.STRING);
    writeRecord(
        entry,
        timestampMicros,
        Objects.requireNonNull(value, "value").getBytes(StandardCharsets.UTF_8));
  }

  /** Writes an optional finish control record and prevents further appends to the entry. */
  public void finish(int entry, long timestampMicros) throws IOException {
    ensureOpen();
    if (!entryTypes.containsKey(entry)) {
      throw new IllegalArgumentException("Unknown or already finished WPILOG entry: " + entry);
    }
    requireTimestamp(timestampMicros);
    ByteArrayOutputStream payload = new ByteArrayOutputStream(5);
    payload.write(CONTROL_FINISH);
    writeLittleEndian(payload, entry, Integer.BYTES);
    writeRecord(0, timestampMicros, payload.toByteArray());
    entryTypes.remove(entry);
  }

  public void flush() throws IOException {
    ensureOpen();
    output.flush();
  }

  @Override
  public void close() throws IOException {
    if (!closed) {
      closed = true;
      output.close();
    }
  }

  private void writeFileHeader(String extraHeader) throws IOException {
    byte[] extra = extraHeader.getBytes(StandardCharsets.UTF_8);
    output.write(MAGIC);
    writeLittleEndian(output, VERSION_1_0, Short.BYTES);
    writeLittleEndian(output, extra.length, Integer.BYTES);
    output.write(extra);
  }

  private void writeRecord(int entry, long timestampMicros, byte[] payload) throws IOException {
    ensureOpen();
    requireTimestamp(timestampMicros);
    int entryLength = byteLength(entry, Integer.BYTES);
    int sizeLength = byteLength(payload.length, Integer.BYTES);
    int timestampLength = byteLength(timestampMicros, Long.BYTES);
    int lengthByte =
        (entryLength - 1) | ((sizeLength - 1) << 2) | ((timestampLength - 1) << 4);

    output.write(lengthByte);
    writeLittleEndian(output, entry, entryLength);
    writeLittleEndian(output, payload.length, sizeLength);
    writeLittleEndian(output, timestampMicros, timestampLength);
    output.write(payload);
  }

  private void requireType(int entry, SignalType expected) throws IOException {
    ensureOpen();
    SignalType actual = entryTypes.get(entry);
    if (actual == null) {
      throw new IllegalArgumentException("Unknown or finished WPILOG entry: " + entry);
    }
    if (actual != expected) {
      throw new IllegalArgumentException(
          "WPILOG entry " + entry + " stores " + actual.wpiType() + ", not " + expected.wpiType());
    }
  }

  private void ensureOpen() throws IOException {
    if (closed) {
      throw new IOException("WPILOG writer is closed");
    }
  }

  private static void requireTimestamp(long timestampMicros) {
    if (timestampMicros < 0) {
      throw new IllegalArgumentException("timestampMicros must be non-negative");
    }
  }

  private static int byteLength(long value, int maximumBytes) {
    int bytes = 1;
    while (bytes < maximumBytes && (value >>> (bytes * 8)) != 0) {
      bytes++;
    }
    return bytes;
  }

  private static void writeInnerString(OutputStream output, String value) throws IOException {
    byte[] bytes = value.getBytes(StandardCharsets.UTF_8);
    writeLittleEndian(output, bytes.length, Integer.BYTES);
    output.write(bytes);
  }

  private static void writeLittleEndian(OutputStream output, long value, int bytes)
      throws IOException {
    for (int index = 0; index < bytes; index++) {
      output.write((int) (value >>> (index * 8)) & 0xff);
    }
  }
}
