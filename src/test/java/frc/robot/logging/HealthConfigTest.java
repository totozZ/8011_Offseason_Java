package frc.robot.logging;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.PowerDistribution;
import org.junit.jupiter.api.Test;

class HealthConfigTest {
    @Test
    void safeDefaultsDoNotGuessPowerDistributionOrRaisePhoenixRates() {
        HealthConfig config = HealthConfig.defaults();

        assertFalse(config.pdhEnabled());
        assertFalse(config.logNetworkTables());
        assertTrue(config.logJoysticks());
        assertTrue(config.startSignalLogger());
        assertFalse(config.configurePhoenixSignalFrequencies());
        assertEquals("Unknown/Channel00", config.pdhChannelMap().deviceLabel(0));
        assertEquals("Unknown", config.pdhChannelMap().subsystemLabel(23));
    }

    @Test
    void powerDistributionRequiresExplicitValidHardwareIdentity() {
        PdhChannelMap mapping = PdhChannelMap.unknown(24);

        assertThrows(
                IllegalArgumentException.class,
                () ->
                        HealthConfig.builder()
                                .powerDistribution(
                                        -1,
                                        PowerDistribution.ModuleType.kRev,
                                        mapping)
                                .build());
        assertThrows(
                IllegalArgumentException.class,
                () -> HealthConfig.builder().powerDistribution(1, null, mapping).build());

        HealthConfig configured =
                HealthConfig.builder()
                        .powerDistribution(
                                1,
                                PowerDistribution.ModuleType.kRev,
                                mapping)
                        .build();
        assertTrue(configured.pdhEnabled());
        assertEquals(1, configured.pdhModule());
        assertEquals(PowerDistribution.ModuleType.kRev, configured.pdhType());
    }

    @Test
    void explicitChannelMapPreservesUnknownChannelsAndSubsystemLookup() {
        PdhChannelMap mapping =
                PdhChannelMap.builder(24)
                        .map(3, "Drive", "FrontLeftDrive")
                        .map(9, "drive", "BackRightDrive")
                        .map(17, "Shooter", "ShooterLeader")
                        .build();

        assertArrayEquals(new int[] {3, 9}, mapping.channelsForSubsystem("DRIVE"));
        assertEquals("FrontLeftDrive", mapping.deviceLabel(3));
        assertEquals("Drive", mapping.subsystemLabel(3));
        assertEquals("Unknown/Channel04", mapping.deviceLabel(4));
        assertThrows(IllegalArgumentException.class, () -> mapping.assignment(24));
        assertThrows(
                IllegalArgumentException.class,
                () -> PdhChannelMap.builder(24).map(24, "Drive", "Invalid"));
    }

    @Test
    void revPdhTemperatureIsNotTreatedAsARealMeasurement() {
        assertFalse(
                RobotHealthLogger.supportsPowerDistributionTemperature(
                        PowerDistribution.ModuleType.kRev));
        assertTrue(
                RobotHealthLogger.supportsPowerDistributionTemperature(
                        PowerDistribution.ModuleType.kCTRE));
    }
}
