/**
 * @file Wayimotor.h
 * @author Bingzhang Wang (wbz04@foxmail.com)
 * @brief A header file for the Wayimotor class, which is a class for the motor
 * control of the robot, which includes normal motor parameters and control
 * methods.
 * @version 0.1
 * @date 2025-02-18
 *
 * @copyright Copyright (c) 2025
 ******************************************************************************
 * @attention
 *
 * if you had modified this file, please make sure your code does not have many
 * bugs, update the version Number, write dowm your name and the date, the most
 * important is make sure the users will have clear and definite understanding
 * through your new brief.
 *
 * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
 * All rights reserved.</center></h2>
 ******************************************************************************
 */
#pragma once
#include <frc/controller/BangBangController.h>

#include <ctre/phoenix6/CANBus.hpp>
#include <ctre/phoenix6/controls/MotionMagicVelocityTorqueCurrentFOC.hpp>
#include <ctre/phoenix6/controls/VelocityTorqueCurrentFOC.hpp>
#include <ctre/phoenix6/controls/VoltageOut.hpp>

#include "ctre/phoenix6/TalonFX.hpp"

using namespace ctre::phoenix6;

struct WayimotorConfig {
  /* 下发目标值,相对于实际机械机构 */
  double targetPosition;
  double targetVelocity;
  double targetCurrent;

  /* 归一化目标值 (0-1) */
  double normalizedPosition;  // 归一化位置 0-1
  double normalizedVelocity;  // 归一化速度 -1-1
  double normalizedCurrent;   // 归一化电流 -1-1

  /* 物理限制,相对于实际机械结构 */
  double maxPosition = 1.0;   // 最大位置(转数)
  double minPosition = 0.0;   // 最小位置(转数)
  double maxVelocity = 50.0;  // 最大速度(转/秒) x44:120； x60
  double maxCurrent = 40.0;   // 最大电流(安培)

  units::angular_velocity::turns_per_second_t Veloutput;
  units::angular_velocity::turns_per_second_t mmVeloutput;
  units::angle::turn_t Posoutput;
  units::current::ampere_t Curoutput;
  units::angle::turn_t motionoutput;
  units::dimensionless::scalar_t DutyOutput;

  /* 电机当前值 */
  double currentPosition;
  double currentVelocity;
  double currentCurrent;
  double currentnormalizedPosition;  // 当前归一化位置
  double currentnormalizedVelocity;  // 当前归一化速度
  double currentnormalizedCurrent;   // 当前归一化电流

  /* 电机参数 */
  int deviceId;
  CANBus canbus;
  double offset = 0;     // 电机零点偏移(适用于位置控制)，相对于电机本身
  double gearRatio = 1;  // 电机减速比
  uint8_t mode =
      0;  // 电机控制模式：
          // 0-制动模式，1-速度环模式，2-位置环模式，3-电流环模式，4-motionmagic模式
  double target;               // 位置环归一化值
  double invert = 1;           // 电机是否反转
  double Current_speed = 0.5;  // 电流控制速度限制(0-1范围, 1为最大占空比输出)
  int followerId = -1;         // 从属电机ID, -1表示不是从属电机
  bool follow_invert = false;  // 从属电机是否反转,默认同方向
  bool useTorqueCurrent =
      false;  // BangBang输出模式: false=Voltage,
              // true=TorqueCurrent，再增加控制模式可能需要建立枚举
  double bangBangBoostVoltage = 2.0;  // BangBang增压电压 (V)
  double bangBangBoostCurrent = 5.0;  // BangBang增压电流 (A)
};

class Wayimotor {
 public:
  Wayimotor(int deviceId, CANBus canbus) : motor(deviceId, canbus) {
    wayiconfig.deviceId = deviceId;
    wayiconfig.canbus = canbus;
  };

  ~Wayimotor() {};

 private:
  WayimotorConfig wayiconfig;  // 用到的电机参数
  hardware::TalonFX motor;     // 猎鹰电机

  ctre::phoenix6::controls::StaticBrake brake{};  // 静态制动

  frc::BangBangController bangBangController{
      5.0};  // BangBang Controller with 5 tolerance

  controls::VelocityVoltage velocity =
      controls::VelocityVoltage{0_tps}.WithSlot(0);  // 速度闭环控制

  controls::MotionMagicVelocityTorqueCurrentFOC motionmagicvelocity =
      controls::MotionMagicVelocityTorqueCurrentFOC{0_tps}
          .WithUpdateFreqHz(500_Hz)
          .WithSlot(0);  // motionmagic控制速度

  controls::VelocityTorqueCurrentFOC velocitytorquecurrent =
      controls::VelocityTorqueCurrentFOC{0_tps}
          .WithUpdateFreqHz(500_Hz)
          .WithSlot(0);  // 速度电流FOC控制

  controls::PositionVoltage position =
      controls::PositionVoltage{0_tr}.WithSlot(0);  // 位置闭环控制

  controls::MotionMagicExpoVoltage mm_position =
      controls::MotionMagicExpoVoltage{0_tr}
          .WithUpdateFreqHz(50_Hz)
          .WithSlot(0)
          .WithEnableFOC(1);  // motionmagic控制位置

  controls::TorqueCurrentFOC Torque =
      controls::TorqueCurrentFOC{0_A}.WithUpdateFreqHz(500_Hz);  // 电流控制

  controls::MotionMagicExpoTorqueCurrentFOC motionmagic =
      controls::MotionMagicExpoTorqueCurrentFOC{0_tr}
          .WithUpdateFreqHz(500_Hz)
          .WithSlot(0);  // motionmagic控制位置

  controls::DutyCycleOut DutyCircle =
      controls::DutyCycleOut{0.0}.WithUpdateFreqHz(500_Hz);  // 速度环占空比输出

  controls::PositionDutyCycle positionDutyCycle =
      controls::PositionDutyCycle{0_tr}.WithSlot(0);  // 位置占空比控制

  controls::VoltageOut voltageOut = controls::VoltageOut{0_V}.WithUpdateFreqHz(
      500_Hz);  // 电压输出（用于SysId）

 public:
  void Control();              // 下发控制信号
  void Receive();              // 接收电机数据
  void Reset(double _offset);  // 复位,将当前角度视为0

  ctre::phoenix::StatusCode Applyconfig(configs::TalonFXConfiguration config) {
    return motor.GetConfigurator().Apply(config);
  }  // 应用电机配置

  void setmode(uint8_t mode) { wayiconfig.mode = mode; }  // 设置电机控制模式

  void setgearRatio(double gearRatio) {
    wayiconfig.gearRatio = gearRatio;
  }  // 设置电机减速比

  void setoffset(double offset) {
    wayiconfig.offset = offset;
  }  // 设置电机零点偏移

  void setinvert(double invert) {
    wayiconfig.invert = invert;
  }  // 设置电机是否反转

  void setbrake() { setmode(0); }  // 制动

  // 归一化控制方法
  void setNormalizedPosition(double normalizedPos);  // 0-1位置控制
  void setNormalizedVelocity(double normalizedVel);  // 0-1速度控制
  void setNormalizedCurrent(double normalizedCur);   // 0-1电流控制
  void setNormalizedMotion(double normalizedPos);    // 0-1 MotionMagic控制
  void setNormalizedMotionPosition(
      double normalizedPos);  // 0-1 MotionMagic位置控制
  void setNormalizedMotionVelocity(
      double normalizedVel);  // 0-1 MotionMagic速度控制
  void setNormalizedDutyCircle(double normalizedDuty);  // 0-1 占空比控制
  void setVelocityTorqueCurrent(double velocity);

  // 设置物理限制
  void setPhysicalLimits(double minPos, double maxPos, double maxVel,
                         double maxCur) {
    wayiconfig.minPosition = minPos;
    wayiconfig.maxPosition = maxPos;
    wayiconfig.maxVelocity = maxVel;
    wayiconfig.maxCurrent = maxCur;
  }
  void setPositionRange(double minPos, double maxPos) {
    wayiconfig.minPosition = minPos;
    wayiconfig.maxPosition = maxPos;
  }
  void setMaxVelocity(double maxVel) { wayiconfig.maxVelocity = maxVel; }
  void setMaxCurrent(double maxCur) { wayiconfig.maxCurrent = maxCur; }

  void setvelocity(double targetVelocity) {
    setmode(1);
    wayiconfig.targetVelocity = targetVelocity;
  }  // 设置速度

  void setposition(double targetPosition) {
    setmode(5);
    wayiconfig.targetPosition = targetPosition;
  }  // 设置位置

  void setcurrent(double targetCurrent) {
    setmode(3);
    wayiconfig.targetCurrent = targetCurrent;
  }  // 设置电流

  void setmotion(double targetPosition) {
    setmode(4);
    wayiconfig.targetPosition = targetPosition;
  }  // 设置motionmagic位置

  void setmotionvelocity(double targetVelocity) {
    setmode(5);
    wayiconfig.targetVelocity = targetVelocity;
  }  // 设置motionmagic速度

  void setvelocitytorquecurrent(double velocity) {
    setVelocityTorqueCurrent(velocity);
  }  // 设置速度电流FOC控制

  void setVoltage(units::volt_t voltage) {
    setmode(10);
    motor.SetControl(voltageOut.WithOutput(voltage));
  }  // 设置电压输出（用于SysId）

  void setBangBangVelocity(double velocity, bool useTorqueCurrent = false) {
    setmode(11);
    wayiconfig.targetVelocity = velocity;
    wayiconfig.useTorqueCurrent = useTorqueCurrent;
  }

  void setfollowControl(int _followID, bool _follow_invert) {
    wayiconfig.followerId = _followID;
    wayiconfig.follow_invert = _follow_invert;
    setmode(8);
  }

  void setmotionposition(double targetposition) {
    setmode(7);
    wayiconfig.targetPosition = targetposition;
  }  // 设置motionmagic速度

  WayimotorConfig& Getdata() { return wayiconfig; }  // 获取电机数据

  hardware::TalonFX& Getmotor() { return motor; }  // 获取电机对象

  void setCurrent_Speed(double speed) { wayiconfig.Current_speed = speed; }
};
