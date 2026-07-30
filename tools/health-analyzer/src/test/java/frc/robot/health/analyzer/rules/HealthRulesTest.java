package frc.robot.health.analyzer.rules;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.file.Files;
import java.nio.file.Path;
import java.io.StringReader;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class HealthRulesTest {
    @TempDir
    Path temporaryDirectory;

    @Test
    void mergesSubsystemOverridesWithoutDiscardingDefaultThresholds() {
        HealthRules rules = HealthRules.fromJson(new StringReader("""
                {
                  "defaults":{
                    "STALL":{"enabled":true,"durationSeconds":0.5,
                      "thresholds":{"minStatorCurrent":60,"maxAbsVelocity":1}}
                  },
                  "subsystems":{
                    "Drive":{"STALL":{"durationSeconds":0.25,
                      "thresholds":{"minStatorCurrent":80}}},
                    "Intake":{"STALL":{"enabled":false}}
                  }
                }
                """));

        RuleConfig drive = rules.resolve(RuleType.STALL, "Drive");
        assertTrue(drive.enabled());
        assertEquals(0.25, drive.durationSeconds());
        assertEquals(80.0, drive.threshold("minStatorCurrent", 0));
        assertEquals(1.0, drive.threshold("maxAbsVelocity", 0));
        assertFalse(rules.resolve(RuleType.STALL, "Intake").enabled());
    }

    @Test
    void partialFileDeepMergesWithBundledDefaults() throws Exception {
        Path custom = temporaryDirectory.resolve("partial-rules.json");
        Files.writeString(custom, """
                {
                  "defaults": {
                    "STALL": {
                      "thresholds": {"minStatorCurrent": 75.0}
                    }
                  },
                  "subsystems": {
                    "Drive": {
                      "STALL": {"durationSeconds": 0.2}
                    }
                  }
                }
                """);

        HealthRules rules = HealthRules.load(custom);

        RuleConfig globalStall = rules.resolve(RuleType.STALL, null);
        assertEquals(Severity.ERROR, globalStall.severity());
        assertEquals(0.35, globalStall.durationSeconds());
        assertEquals(2.0, globalStall.threshold("minOutputVolts", 0.0));
        assertEquals(75.0, globalStall.threshold("minStatorCurrent", 0.0));
        assertEquals(0.2, rules.resolve(RuleType.STALL, "Drive").durationSeconds());
        assertEquals(
                Severity.ERROR,
                rules.resolve(RuleType.LOW_VOLTAGE, null).severity());
        assertEquals(
                0.45,
                rules.resolve(RuleType.MOTOR_IMBALANCE, null)
                        .threshold("ratio", 0.0));
        assertEquals(
                1.5,
                rules.resolve(RuleType.TRACKING_ERROR, "Shooter")
                        .durationSeconds());
        assertEquals(4, rules.followers().size());
    }

    @Test
    void explicitFollowerArrayReplacesBundledPairs() throws Exception {
        Path custom = temporaryDirectory.resolve("followers.json");
        Files.writeString(custom, """
                {
                  "followers": [
                    {"subsystem": "Drive", "leader": "Left", "follower": "Right"}
                  ]
                }
                """);

        HealthRules rules = HealthRules.load(custom);

        assertEquals(1, rules.followers().size());
        assertEquals("Drive", rules.followers().get(0).subsystem());
        assertEquals(
                Severity.ERROR,
                rules.resolve(RuleType.STALL, null).severity());
    }

    @Test
    void nullFollowerValueInheritsBundledPairs() throws Exception {
        Path custom = temporaryDirectory.resolve("null-followers.json");
        Files.writeString(custom, "{\"followers\": null}");

        assertEquals(4, HealthRules.load(custom).followers().size());
    }

    @Test
    void rejectsMisspelledTopLevelField() throws Exception {
        Path custom = temporaryDirectory.resolve("misspelled.json");
        Files.writeString(custom, "{\"defualts\": {}}");

        assertThrows(IllegalArgumentException.class, () -> HealthRules.load(custom));
    }
}
