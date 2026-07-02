// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include "ctre/phoenix6/TalonFX.hpp"
#include <frc/XboxController.h>
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include "units/math.h"
#include "Constants.h"
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/WaitUntilCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc8011/Wayimotor.h>
#include "frc/Servo.h"

using namespace ctre::phoenix6;
namespace subsystems{
class ElevatorSubsystem : public frc2::SubsystemBase {
 public:
  
  ElevatorSubsystem(    frc2::CommandXboxController& m_joystick
) : m_joystick(m_joystick) {

    FortMotorInit();
    m_elevator1.SetPosition(0.0_tr);
  }

  /**
   * Example command factory method.
   */
  frc2::CommandPtr UpperMethodCommand();

  /**
   * @brief 使用遥控器控制上层机构
   * 
   */
  frc2::CommandPtr TeleopControlCommand();

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  bool UpperCondition();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  /** 炮台电机相关初始化 */
    void FortMotorInit();


    ctre::phoenix6::controls::StaticBrake m_brake{}; // 静态制动


    ctre::phoenix6::CANBus kCANBus{"rio"}; // CAN总线

    ctre::phoenix6::controls::VelocityVoltage m_velocityVoltage =
      ctre::phoenix6::controls::VelocityVoltage{0_tps}.WithSlot(0); // 速度闭环控制

    ctre::phoenix6::controls::TorqueCurrentFOC m_motorFOCRequest = ctre::phoenix6::controls::TorqueCurrentFOC{0_A}.WithUpdateFreqHz(500_Hz); // 电流控制

  
  ctre::phoenix6::hardware::TalonFX m_elevator1{15, kCANBus}; // 电梯电机
  ctre::phoenix6::hardware::TalonFX m_elevator2{16, kCANBus}; // 跟随电机

  double m_elevatorposition = 0; // 归一化：0-1
  double m_elevatorpositionoffset = 0; // 电梯电机位置偏移
  double m_elevatorGetposition = 0; // 电梯电机当前位置
  ctre::phoenix6::controls::MotionMagicExpoTorqueCurrentFOC m_motorMMRequest = ctre::phoenix6::controls::MotionMagicExpoTorqueCurrentFOC{0_tr}.WithUpdateFreqHz(500_Hz).WithSlot(0);
  
  ctre::phoenix6::controls::Follower m_motorFollower = ctre::phoenix6::controls::Follower{m_elevator1.GetDeviceID(), true}.WithUpdateFreqHz(500_Hz);


    frc2::CommandXboxController& m_joystick;

  Wayimotor m_stretch{20, kCANBus}; // 抓取电机，使抬升机构抓住笼子
  Wayimotor m_climb{21, kCANBus}; // 抬升电机，弯曲以抬升自身
  double releaseflag = 0;
  double lockflag = 0;
      frc::Servo climbrelease {0}; // 释放爬升机构
    frc::Servo climblock {1}; // 锁住爬升机构

  public:

    
    frc2::CommandPtr DefaultCommand(); // 默认命令，使用手柄控制上层机构

    frc2::CommandPtr SetHeightCommandPtr(double height); // 设置高度命令(0-1范围)

    frc2::CommandPtr plusHeightCommandPtr(); // 设置爬升电流 (正方向为收)
    frc2::CommandPtr minusHeightCommandPtr(); // 设置爬升电流 (正方向为收)


    std::function< bool()> isAllowtoPitchToZero();  //电梯降到最低时才可以收回arm

    frc2::CommandPtr SetClimbcurrent(double current); // 设置爬升电流 (正方向为收)

    double GetPosition() {
      return   m_elevatorGetposition;
    }

    void ProtectEle() {
      m_elevatorposition = m_elevatorGetposition;
    }

    double GetClimbCurrent() {
      return   m_climb.Getdata().currentCurrent;
    }

    void SetStretchVelocity(double _velocity){
        m_stretch.setvelocity(_velocity);
    }


    void Setreleaseflag(double mode) {
      if (mode == 1) {
      releaseflag = 1;
      }

      if (mode == 0) {
      releaseflag = 0;
      }

    }

    void Setlockflag(double mode) {

      if (mode == 1) {
      lockflag = 1;
      }

      if (mode == 0) {
      lockflag = 0;
      }

      // lockflag = 1;
    }

    void SetpushServo(double _angle) {
      climbrelease.SetAngle(_angle);
    }

    void SetlockServo(double _angle) {
      climblock.SetAngle(_angle);
    }
};
}
