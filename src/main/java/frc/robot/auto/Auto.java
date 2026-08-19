// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import static frc.robot.babyauto.BabyAuto.parallel;
import static frc.robot.babyauto.BabyAuto.sequence;
import static frc.robot.babyauto.BabyAuto.waitSeconds;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.babyauto.BabyAuto;

/**
 * BabyAuto 活动中，学生只需要修改这个文件里的 {@link #build(BabyAuto)}。
 *
 * <p>真机安全：第一次测试必须架空车轮和机构、清空机器人周围，并安排一人随时按下 Driver Station 的
 * Disable（Enter）或 E-stop（空格）。系统会在 15 秒时强制结束整套自动并停止所有输出。
 *
 * <p>底盘使用机器人坐标系：Vx 正数向前，Vy 正数向左，rotation 正数逆时针。Vx/Vy 单位是 m/s，
 * rotation 单位是 °/s，时间单位是秒。系统会把平移合速度限制到 1.0 m/s，把旋转限制到 90°/s。
 *
 * <p>{@code sequence(a, b)} 表示先做 a 再做 b；{@code parallel(a, b)} 表示同时做 a 和 b；
 * {@code waitSeconds(1.0)} 表示等待 1 秒。不要在 parallel 中同时放两个底盘动作或两个机构动作。
 *
 * <p>常用动作：{@code robot.intakeFor(2.0)} 自动吸球 2 秒；{@code robot.shootFor(3.0)} 先预转 1 秒，
 * 再射球 3 秒。它们结束时都会停止机构。
 *
 * <p>自由实验：{@code robot.feeder().speed(0.5)}、{@code robot.launcher().speed(0.8)} 会设置并保持
 * -1 到 +1 的电机输出；用对应的 {@code .stop()} 停止。即使忘写 stop，自动结束、被取消或进入 Teleop
 * 时，底层安全保护仍会停止全部输出。
 */
public final class Auto {
    private Auto() {}

    /** 返回课堂中要运行的完整自动命令。 */
    public static Command build(BabyAuto robot) {
        return sequence(
                // 一边向前移动，一边吸球。
                parallel(
                        robot.intakeFor(2.0),
                        robot.drive()
                                .setVx(0.6)
                                .setVy(0.0)
                                .setRotation(0.0)
                                .forSeconds(2.0)),

                // 停稳后稍等，再预转并射球。
                waitSeconds(0.5),
                robot.shootFor(3.0));

        /*
         * 原始电机动作示例（需要时替换上面的 return 内容）：
         *
         * return sequence(
         *         robot.launcher().speed(0.8),
         *         waitSeconds(1.0),
         *         robot.feeder().speed(0.6),
         *         waitSeconds(2.0),
         *         robot.feeder().stop(),
         *         robot.launcher().stop());
         */
    }
}
