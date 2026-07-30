package frc.robot.health.analyzer.analysis;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.junit.jupiter.api.Test;

class SignalStatisticsTest {
    @Test
    void representsAnUnavailableIntegralAsNaNInsteadOfZero() {
        SignalStatistics statistics = SignalStatistics.unavailable("/Health/Missing");

        assertFalse(statistics.available());
        assertTrue(Double.isNaN(statistics.integralValueSeconds()));
        assertTrue(
                Double.isNaN(
                        SignalStatistics.of("/Health/Missing", List.of())
                                .integralValueSeconds()));
    }
}
