package frc.robot.health.analyzer.io.wpilog;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import frc.robot.health.analyzer.model.SignalType;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class PureJavaWpiLogWriterTest {
  @TempDir Path tempDirectory;

  @Test
  void writesAStandardLogReadableByWpilibWithoutJni() throws Exception {
    Path file = tempDirectory.resolve("pure-java-fixture.wpilog");
    int booleanEntry;
    int doubleEntry;
    int integerEntry;
    int stringEntry;
    String longValue = "机器人健康".repeat(40);

    try (PureJavaWpiLogWriter writer =
        new PureJavaWpiLogWriter(file, "health-analyzer-fixture")) {
      booleanEntry = writer.startBoolean("/Health/Robot/Enabled", 0);
      doubleEntry =
          writer.start(
              "/Health/Power/Voltage", SignalType.DOUBLE, "{\"unit\":\"V\"}", 1);
      integerEntry = writer.startInteger("/Health/Robot/CAN/BusOffCount", 2);
      stringEntry = writer.startString("/Health/Robot/Mode", 255);
      writer.appendBoolean(booleanEntry, true, 256);
      writer.appendDouble(doubleEntry, 12.375, 65_536);
      writer.appendInteger(integerEntry, 42, 65_537);
      writer.appendString(stringEntry, longValue, 0x01_02_03_04_05_06L);
      writer.flush();
    }

    DataLogReader reader = read(file);
    assertTrue(reader.isValid());
    assertEquals((short) 0x0100, reader.getVersion());
    assertEquals("health-analyzer-fixture", reader.getExtraHeader());

    List<DataLogRecord> records = new ArrayList<>();
    reader.forEach(records::add);
    assertEquals(8, records.size());

    assertTrue(records.get(0).isStart());
    assertEquals(booleanEntry, records.get(0).getStartData().entry);
    assertEquals("/Health/Robot/Enabled", records.get(0).getStartData().name);
    assertEquals("boolean", records.get(0).getStartData().type);

    assertTrue(records.get(1).isStart());
    assertEquals(doubleEntry, records.get(1).getStartData().entry);
    assertEquals("{\"unit\":\"V\"}", records.get(1).getStartData().metadata);

    assertTrue(records.get(2).isStart());
    assertEquals(integerEntry, records.get(2).getStartData().entry);
    assertTrue(records.get(3).isStart());
    assertEquals(stringEntry, records.get(3).getStartData().entry);
    assertTrue(records.get(4).getBoolean());
    assertEquals(12.375, records.get(5).getDouble());
    assertEquals(65_536, records.get(5).getTimestamp());
    assertEquals(42, records.get(6).getInteger());
    assertEquals(longValue, records.get(7).getString());
    assertEquals(0x01_02_03_04_05_06L, records.get(7).getTimestamp());
  }

  @Test
  void validatesEntryTypesAndLifecycle() throws Exception {
    Path file = tempDirectory.resolve("validation.wpilog");
    PureJavaWpiLogWriter writer = new PureJavaWpiLogWriter(file);
    int entry = writer.startDouble("/Health/Power/Voltage", 0);

    assertThrows(
        IllegalArgumentException.class, () -> writer.appendBoolean(entry, true, 1));
    assertThrows(
        IllegalArgumentException.class, () -> writer.appendDouble(entry, 12.0, -1));
    writer.finish(entry, 2);
    assertThrows(
        IllegalArgumentException.class, () -> writer.appendDouble(entry, 12.0, 3));
    writer.close();
    assertThrows(IOException.class, writer::flush);
  }

  @Test
  void encodesScalarPayloadsExactly() throws Exception {
    Path file = tempDirectory.resolve("payloads.wpilog");
    try (PureJavaWpiLogWriter writer = new PureJavaWpiLogWriter(file)) {
      int falseEntry = writer.startBoolean("/Health/Test/False", 0);
      int trueEntry = writer.startBoolean("/Health/Test/True", 0);
      writer.appendBoolean(falseEntry, false, 1);
      writer.appendBoolean(trueEntry, true, 1);
    }

    List<byte[]> dataPayloads = new ArrayList<>();
    read(file)
        .forEach(
            record -> {
              if (!record.isControl()) {
                dataPayloads.add(record.getRaw());
              }
            });
    assertEquals(2, dataPayloads.size());
    assertArrayEquals(new byte[] {0}, dataPayloads.get(0));
    assertArrayEquals(new byte[] {1}, dataPayloads.get(1));
  }

  private static DataLogReader read(Path path) throws IOException {
    return new DataLogReader(ByteBuffer.wrap(Files.readAllBytes(path)));
  }
}
