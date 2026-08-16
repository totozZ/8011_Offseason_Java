package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.junit.jupiter.api.Test;

class NetworkTablesConventionTest {
    private static final Path MAIN_JAVA = Path.of("src", "main", "java");
    private static final Pattern LITERAL_TABLE = Pattern.compile("getTable\\(\"([^\"]+)\"\\)");
    private static final Pattern SMART_DASHBOARD_SCALAR = Pattern.compile(
            "SmartDashboard\\.put(?:Boolean|Number|String|BooleanArray|NumberArray|StringArray)");

    @Test
    void everyTeamOwnedRootUsesFrc8011Namespace() {
        List<String> roots = List.of(
                Constants.TelemetryConstants.ROBOT,
                Constants.TelemetryConstants.DRIVE,
                Constants.TelemetryConstants.VISION,
                Constants.TelemetryConstants.LED,
                Constants.TelemetryConstants.EXAMPLE,
                Constants.TelemetryConstants.EXAMPLE_TUNING);

        assertEquals("/FRC8011", Constants.TelemetryConstants.ROOT);
        assertTrue(roots.stream().allMatch(root -> root.startsWith("/FRC8011/")));
        assertEquals(
                "/SmartDashboard/FRC8011/Auto/Chooser",
                "/SmartDashboard/" + Constants.TelemetryConstants.AUTO_CHOOSER_KEY);
    }

    @Test
    void sourceUsesTypedTeamTablesAndNoSmartDashboardScalars() throws IOException {
        int sendableWriteCount = 0;
        for (Path source : javaSources()) {
            String text = Files.readString(source);
            Matcher tableMatcher = LITERAL_TABLE.matcher(text);
            while (tableMatcher.find()) {
                assertEquals(
                        "FRC8011",
                        tableMatcher.group(1),
                        () -> "Unexpected literal NT table in " + source);
            }
            assertFalse(
                    SMART_DASHBOARD_SCALAR.matcher(text).find(),
                    () -> "SmartDashboard scalar write found in " + source);
            sendableWriteCount += countOccurrences(text, "SmartDashboard.putData(");
        }
        assertEquals(1, sendableWriteCount);

        String robotContainer = Files.readString(
                MAIN_JAVA.resolve(Path.of("frc", "robot", "RobotContainer.java")));
        assertTrue(robotContainer.contains(
                "SmartDashboard.putData(Constants.TelemetryConstants.AUTO_CHOOSER_KEY, "
                        + "autoChooser)"));
    }

    @Test
    void visionUsesOnlyMegaTag2PoseInput() throws IOException {
        String limelightIo = Files.readString(
                MAIN_JAVA.resolve(Path.of("frc", "robot", "vision", "LimelightIO.java")));
        assertTrue(limelightIo.contains("botpose_orb_wpiblue"));
        assertFalse(limelightIo.contains("botpose_wpiblue"));
    }

    private static List<Path> javaSources() throws IOException {
        try (var paths = Files.walk(MAIN_JAVA)) {
            return paths.filter(path -> path.toString().endsWith(".java")).toList();
        }
    }

    private static int countOccurrences(String text, String value) {
        int count = 0;
        int index = 0;
        while ((index = text.indexOf(value, index)) >= 0) {
            count++;
            index += value.length();
        }
        return count;
    }
}
