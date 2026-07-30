package frc.robot.health.analyzer.rules;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import java.io.IOException;
import java.io.Reader;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

/** JSON-backed rules with optional per-subsystem overrides. */
public final class HealthRules {
    private static final Set<String> TOP_LEVEL_FIELDS =
            Set.of("defaults", "subsystems", "followers");

    private Map<RuleType, RuleConfig> defaults = new EnumMap<>(RuleType.class);
    private Map<String, Map<RuleType, RuleConfig>> subsystems = new LinkedHashMap<>();
    private List<FollowerPair> followers = new ArrayList<>();

    public HealthRules() {}

    public static HealthRules load(Path path) throws IOException {
        try (Reader reader = Files.newBufferedReader(path)) {
            JsonElement document = JsonParser.parseReader(reader);
            HealthRules overrides = new Gson().fromJson(document, HealthRules.class);
            JsonObject object =
                    document != null && document.isJsonObject()
                            ? document.getAsJsonObject()
                            : new JsonObject();
            for (String field : object.keySet()) {
                if (!TOP_LEVEL_FIELDS.contains(field)) {
                    throw new IllegalArgumentException(
                            "Unknown top-level health-rules field: " + field);
                }
            }
            return merge(loadDefaults(), overrides, object);
        }
    }

    static HealthRules fromJson(Reader reader) {
        HealthRules rules = new Gson().fromJson(reader, HealthRules.class);
        return rules == null ? new HealthRules() : rules;
    }

    public static HealthRules loadDefaults() throws IOException {
        try (Reader reader = new java.io.InputStreamReader(
                Objects.requireNonNull(
                        HealthRules.class.getResourceAsStream("/health-rules.json"),
                        "Missing /health-rules.json"),
                java.nio.charset.StandardCharsets.UTF_8)) {
            return fromJson(reader);
        }
    }

    public RuleConfig resolve(RuleType type, String subsystem) {
        RuleConfig base = defaults == null ? null : defaults.get(type);
        RuleConfig override = null;
        if (subsystems != null && subsystem != null) {
            Map<RuleType, RuleConfig> scoped = subsystems.get(subsystem);
            if (scoped != null) {
                override = scoped.get(type);
            }
        }
        return RuleConfig.merge(base, override);
    }

    public List<FollowerPair> followers() {
        return followers == null ? List.of() : List.copyOf(followers);
    }

    public Map<RuleType, RuleConfig> defaults() {
        return defaults == null ? Map.of() : Map.copyOf(defaults);
    }

    public String toJson() {
        return new GsonBuilder().setPrettyPrinting().create().toJson(this);
    }

    private static HealthRules merge(
            HealthRules base,
            HealthRules overrides,
            JsonObject overrideDocument) {
        HealthRules merged = new HealthRules();
        HealthRules safeOverrides = overrides == null ? new HealthRules() : overrides;

        merged.defaults = new EnumMap<>(RuleType.class);
        merged.defaults.putAll(base.defaults());
        if (overrideDocument.has("defaults") && safeOverrides.defaults != null) {
            for (Map.Entry<RuleType, RuleConfig> entry : safeOverrides.defaults.entrySet()) {
                merged.defaults.put(
                        entry.getKey(),
                        RuleConfig.merge(merged.defaults.get(entry.getKey()), entry.getValue()));
            }
        }

        merged.subsystems = new LinkedHashMap<>();
        if (base.subsystems != null) {
            for (Map.Entry<String, Map<RuleType, RuleConfig>> entry
                    : base.subsystems.entrySet()) {
                Map<RuleType, RuleConfig> copy = new EnumMap<>(RuleType.class);
                if (entry.getValue() != null) {
                    copy.putAll(entry.getValue());
                }
                merged.subsystems.put(entry.getKey(), copy);
            }
        }
        if (overrideDocument.has("subsystems") && safeOverrides.subsystems != null) {
            for (Map.Entry<String, Map<RuleType, RuleConfig>> subsystem
                    : safeOverrides.subsystems.entrySet()) {
                Map<RuleType, RuleConfig> target =
                        merged.subsystems.computeIfAbsent(
                                subsystem.getKey(),
                                ignored -> new EnumMap<>(RuleType.class));
                if (subsystem.getValue() == null) {
                    continue;
                }
                for (Map.Entry<RuleType, RuleConfig> entry
                        : subsystem.getValue().entrySet()) {
                    target.put(
                            entry.getKey(),
                            RuleConfig.merge(target.get(entry.getKey()), entry.getValue()));
                }
            }
        }

        JsonElement followerOverrides = overrideDocument.get("followers");
        merged.followers =
                followerOverrides != null && !followerOverrides.isJsonNull()
                        ? new ArrayList<>(safeOverrides.followers())
                        : new ArrayList<>(base.followers());
        return merged;
    }

    public record FollowerPair(String subsystem, String leader, String follower) {
        public FollowerPair {
            Objects.requireNonNull(subsystem);
            Objects.requireNonNull(leader);
            Objects.requireNonNull(follower);
        }
    }
}
