package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * 双飞轮射球子系统 (Shooter Subsystem)
 *
 * 两个飞轮对向旋转，将足球夹持并发射：
 * - LShoot (CAN 14): 主电机，开环 duty cycle 控制
 * - RShoot (CAN 15): 跟随 LShoot，方向反转
 *
 * 例：LShoot duty = -0.1 → RShoot 自动以相反方向旋转。
 */
public class ShooterSubsystem extends SubsystemBase {

    public static final double SHOOT_VELOCITY_RPS = -17.0;

    private static final double L_SHOOT_KP = 0.02;
    private static final double L_SHOOT_KS = 0.01;
    private static final double L_SHOOT_KV = 0.012;

    private static final double R_SHOOT_KP = 0.02;
    private static final double R_SHOOT_KS = 0.0135;
    private static final double R_SHOOT_KV = 0.012;

    private final TalonFX lShoot;
    private final TalonFX rShoot;

    private final VelocityDutyCycle velocityRequest = new VelocityDutyCycle(0);

    public ShooterSubsystem() {
        lShoot = new TalonFX(14, "rio");
        rShoot = new TalonFX(15, "rio");

        // ---- 电流限制 (飞轮用，保守40A) ----
        var currentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(40)
                .withSupplyCurrentLimitEnable(true);

        var lShootConfig = new TalonFXConfiguration()
                .withCurrentLimits(currentLimits)
                .withSlot0(new Slot0Configs()
                        .withKP(L_SHOOT_KP)
                        .withKS(L_SHOOT_KS)
                        .withKV(L_SHOOT_KV));

        var rShootConfig = new TalonFXConfiguration()
                .withCurrentLimits(currentLimits)
                .withSlot0(new Slot0Configs()
                        .withKP(R_SHOOT_KP)
                        .withKS(R_SHOOT_KS)
                        .withKV(R_SHOOT_KV));

        lShoot.getConfigurator().apply(lShootConfig);
        rShoot.getConfigurator().apply(rShootConfig);
    }

    public void setVelocityRPS(double velocityRPS) {
        lShoot.setControl(velocityRequest.withVelocity(velocityRPS));
        rShoot.setControl(velocityRequest.withVelocity(-velocityRPS));
    }

    /** 停止两个飞轮 */
    public void stop() {
        lShoot.stopMotor();
        rShoot.stopMotor();
    }

    /** 获取 LShoot 当前转速，单位 rotations per second */
    public double getVelocityRPS() {
        return lShoot.getVelocity().getValueAsDouble();
    }
}
