// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import org.junit.jupiter.api.Test;

class DeployAssetsTest {
    private static final Path PATHPLANNER =
            Path.of("src", "main", "deploy", "pathplanner");

    @Test
    void cppPathPlannerAssetsArePresent() {
        assertTrue(Files.isRegularFile(PATHPLANNER.resolve("settings.json")));
        assertTrue(Files.isRegularFile(PATHPLANNER.resolve("navgrid.json")));
        assertTrue(Files.isRegularFile(
                PATHPLANNER.resolve("autos").resolve("Do Nothing.auto")));
    }

    @Test
    void doNothingIsTheOnlyAutoAndContainsNoCommands()
            throws IOException, ParseException {
        List<Path> autos;
        try (var files = Files.list(PATHPLANNER.resolve("autos"))) {
            autos = files
                    .filter(path -> path.getFileName().toString().endsWith(".auto"))
                    .toList();
        }

        assertEquals(1, autos.size());
        assertEquals("Do Nothing.auto", autos.get(0).getFileName().toString());

        JSONObject root = parse(autos.get(0));
        JSONObject command = (JSONObject) root.get("command");
        JSONObject data = (JSONObject) command.get("data");
        assertEquals("sequential", command.get("type"));
        assertTrue(((JSONArray) data.get("commands")).isEmpty());
    }

    @Test
    void pathPlannerRobotSettingsMatchCppDeployBaseline()
            throws IOException, ParseException {
        JSONObject settings = parse(PATHPLANNER.resolve("settings.json"));

        assertEquals(0.86, number(settings, "robotWidth"), 1e-9);
        assertEquals(0.86, number(settings, "robotLength"), 1e-9);
        assertEquals(55.0, number(settings, "robotMass"), 1e-9);
        assertEquals(6.883, number(settings, "robotMOI"), 1e-9);
        assertEquals(0.546, number(settings, "robotTrackwidth"), 1e-9);
        assertEquals(0.048, number(settings, "driveWheelRadius"), 1e-9);
        assertEquals(6.7403, number(settings, "driveGearing"), 1e-9);
        assertEquals(60.0, number(settings, "driveCurrentLimit"), 1e-9);
    }

    private static JSONObject parse(Path path)
            throws IOException, ParseException {
        return (JSONObject) new JSONParser().parse(Files.readString(path));
    }

    private static double number(JSONObject object, String key) {
        return ((Number) object.get(key)).doubleValue();
    }
}
