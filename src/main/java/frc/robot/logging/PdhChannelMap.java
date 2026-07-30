// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.logging;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Explicit PDH/PDP wiring map. Unmapped channels stay unknown and are never
 * assigned to a subsystem by inference.
 */
public final class PdhChannelMap {
    public record Assignment(int channel, String subsystem, String device) {}

    private final int channelCount;
    private final Map<Integer, Assignment> assignments;

    private PdhChannelMap(int channelCount, Map<Integer, Assignment> assignments) {
        if (channelCount <= 0) {
            throw new IllegalArgumentException("channelCount must be positive");
        }
        this.channelCount = channelCount;
        this.assignments = Collections.unmodifiableMap(new LinkedHashMap<>(assignments));
    }

    public static PdhChannelMap unknown(int channelCount) {
        return new PdhChannelMap(channelCount, Map.of());
    }

    public static Builder builder(int channelCount) {
        return new Builder(channelCount);
    }

    public int channelCount() {
        return channelCount;
    }

    public Assignment assignment(int channel) {
        checkChannel(channel, channelCount);
        return assignments.get(channel);
    }

    public String deviceLabel(int channel) {
        Assignment assignment = assignment(channel);
        return assignment == null
                ? String.format(Locale.ROOT, "Unknown/Channel%02d", channel)
                : assignment.device();
    }

    public String subsystemLabel(int channel) {
        Assignment assignment = assignment(channel);
        return assignment == null ? "Unknown" : assignment.subsystem();
    }

    public int[] channelsForSubsystem(String subsystem) {
        if (subsystem == null || subsystem.isBlank()) {
            return new int[0];
        }
        List<Integer> channels = new ArrayList<>();
        for (Assignment assignment : assignments.values()) {
            if (assignment.subsystem().equalsIgnoreCase(subsystem.trim())) {
                channels.add(assignment.channel());
            }
        }
        return channels.stream().mapToInt(Integer::intValue).toArray();
    }

    public List<Assignment> assignments() {
        return List.copyOf(assignments.values());
    }

    private static void checkChannel(int channel, int channelCount) {
        if (channel < 0 || channel >= channelCount) {
            throw new IllegalArgumentException(
                    "channel " + channel + " is outside 0.." + (channelCount - 1));
        }
    }

    public static final class Builder {
        private final int channelCount;
        private final Map<Integer, Assignment> assignments = new LinkedHashMap<>();

        private Builder(int channelCount) {
            if (channelCount <= 0) {
                throw new IllegalArgumentException("channelCount must be positive");
            }
            this.channelCount = channelCount;
        }

        public Builder map(int channel, String subsystem, String device) {
            checkChannel(channel, channelCount);
            String cleanSubsystem = requireLabel(subsystem, "subsystem");
            String cleanDevice = requireLabel(device, "device");
            assignments.put(
                    channel,
                    new Assignment(channel, cleanSubsystem, cleanDevice));
            return this;
        }

        public PdhChannelMap build() {
            return new PdhChannelMap(channelCount, assignments);
        }

        private static String requireLabel(String value, String name) {
            if (value == null || value.isBlank()) {
                throw new IllegalArgumentException(name + " must not be blank");
            }
            return value.trim();
        }
    }
}
