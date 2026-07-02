/**
 * 机器人硬件配置 — 8011 Kinematics Swerve + Shooter
 *
 * 底盘：四轮独立驱动+转向（Swerve Drive），每轮 1个驱动电机 + 1个转向电机 + 1个编码器
 * 射球：双飞轮（LShoot / RShoot），一个俯仰电机（Pitch）
 *
 * 下一步补充：PID参数、电机控制参数、自动调参结果
 */
 //请注意 此文档可能是cpp或者.h文件，而本项目用的是java
public final class RobotHardware {

    // ==================== CAN ID 分配 ====================

    // ---- 底盘：前左模块 (Front Left) ----
    // ⚠️ FLD 方向反转问题：FLD电机实际运行方向与预期相反（莫名问题）。
    //    解决：setInverted(true) 仅对 FLD 驱动电机单独反转。
    //    ⚠️ 不要对整个 FrontLeft 模块做反转！否则 FLT 转向电机也会被反转，
    //    导致旋转方向也跟着反。
    //    对应代码位置：TunerConstants.h → kInvertLeftSide 保持 false，
    //    只需在 FLD 初始化时单独 .setInverted(true)。
    public static final int FL_DRIVE  = 1;   // FLD  — 驱动电机 ⚠️ 方向反了，需单独invert
    public static final int FL_TURN   = 2;   // FLT  — 转向电机
    public static final int FL_CODER  = 3;   // FLCoder — 转向编码器

    // ---- 底盘：前右模块 (Front Right) ----
    public static final int FR_DRIVE  = 4;   // FRD
    public static final int FR_TURN   = 5;   // FRT
    public static final int FR_CODER  = 6;   // FRCoder

    // ---- 底盘：后左模块 (Back Left) ----
    public static final int BL_DRIVE  = 7;   // BLD
    public static final int BL_TURN   = 8;   // BLT
    public static final int BL_CODER  = 9;   // BLCoder

    // ---- 底盘：后右模块 (Back Right) ----
    public static final int BR_DRIVE  = 10;  // BRD
    public static final int BR_TURN   = 11;  // BRT
    public static final int BR_CODER  = 12;  // BRCoder

    // ---- 陀螺仪 ----
    public static final int PIGEON    = 13;  // Pigeon IMU (陀螺仪)

    // ---- 射球机构 ----
    public static final int L_SHOOT   = 14;  // 左飞轮电机 (飞轮驱动足球)
    public static final int R_SHOOT   = 15;  // 右飞轮电机 (飞轮驱动足球)

    // ---- 角度/旋转（基本不用） ----
    public static final int SPIN      = 16;  // Spin 旋转电机 (角度太小，实际不用)

    // ---- 俯仰 ----
    public static final int PITCH     = 17;  // Pitch 俯仰电机


    // ==================== 底盘尺寸参数（待实测填充） ====================
    // 轮距 (Track Width)   — 左右轮中心距，单位 米
    // 轴距 (Wheel Base)    — 前后轮中心距，单位 米
    // 轮径 (Wheel Diameter) — 驱动轮直径，单位 米
    // 转向减速比、驱动减速比

    public static final double TRACK_WIDTH    = 0.0;  // TODO
    public static final double WHEEL_BASE     = 0.0;  // TODO
    public static final double WHEEL_DIAMETER = 0.0;  // TODO


    // ==================== 电机/传感器参数（待PID调参后填充） ====================
    // 驱动电机 PID
    // 转向电机 PID
    // 飞轮电机 PID
    // 俯仰电机 PID
    // 编码器偏移量 (每个模块的零点)


    // ==================== 射球参数（待实测填充） ====================
    // 飞轮目标转速：17 RPS（左右同时定速度）
    // L_Shoot: KP=0.02, KS=0.01,   KV=0.012
    // R_Shoot: KP=0.02, KS=0.0135, KV=0.012
    // 实测：发射到落点 4.83m；球静止点到运动最高点 1.03m；发射到落地 1.1s
    // 俯仰角度范围及对应距离


    // ==================== 已知问题 / 注意事项 ====================
    //
    // [1] FLD 电机方向反转 (2025/2026 赛季)
    //     现象：FLD（前左驱动电机）实际旋转方向与代码预期相反。
    //     原因：不明（可能是电机安装朝向或接线极性差异）。
    //     解决：在 FLD 初始化时单独调用 setInverted(true)。
    //     ⚠️ 严禁方案：不要将整个 FrontLeft 模块反转（如 TunerConstants.h 中
    //        kInvertLeftSide = true），因为这会同时反转 FLT 转向电机，
    //        导致前左模块的旋转方向也跟着反，产生更难排查的路径跟踪偏移。
    //     正确做法：保持 kInvertLeftSide = false，仅对 FLD 驱动电机单独 invert。
    //
    // [2] CAN ID 差异注意（本文档 vs TunerConstants.h）
    //     本文档：FL_DRIVE=1, FL_TURN=2
    //     TunerConstants.h：kFrontLeftDriveMotorId=2, kFrontLeftSteerMotorId=1
    //     → 两处 ID 互换了，确认哪个是实际接线后统一。

    // ==================== 工具方法 ====================

    /** 禁止实例化 */
    private RobotHardware() {}

    /** 打印全部硬件映射，方便检查 */
    public static void printMapping() {
        System.out.println("=== 8011 Robot Hardware Mapping ===");
        System.out.printf("FL: Drive=%d  Turn=%d  Coder=%d%n", FL_DRIVE, FL_TURN, FL_CODER);
        System.out.printf("FR: Drive=%d  Turn=%d  Coder=%d%n", FR_DRIVE, FR_TURN, FR_CODER);
        System.out.printf("BL: Drive=%d  Turn=%d  Coder=%d%n", BL_DRIVE, BL_TURN, BL_CODER);
        System.out.printf("BR: Drive=%d  Turn=%d  Coder=%d%n", BR_DRIVE, BR_TURN, BR_CODER);
        System.out.printf("Pigeon: %d%n", PIGEON);
        System.out.printf("LShoot: %d  RShoot: %d%n", L_SHOOT, R_SHOOT);
        System.out.printf("Spin: %d  Pitch: %d%n", SPIN, PITCH);
    }
}

