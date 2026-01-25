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
#include <frc/DigitalInput.h>
#include <ctre/phoenix6/CANdi.hpp>


class ExampleSubsystem : public frc2::SubsystemBase {
 public:

  ExampleSubsystem(frc2::CommandXboxController& m_joystick
) : m_joystick(m_joystick) {
      Init(); // 构造时自动初始化
  }
  virtual ~ExampleSubsystem() = default;

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;



protected: // 子类可以访问
  // 一些常用的Subsystem会用到的变量
  ctre::phoenix6::CANBus kCANBus{"rio"}; // CAN总线名称,除了底盘基本都是rio

  frc2::CommandXboxController& m_joystick; // 子系统遥控器

  // 猎鹰/海妖电机配置config
  configs::TalonFXConfiguration config1{};
  configs::TalonFXConfiguration config2{};
  configs::TalonFXConfiguration config3{};
  configs::TalonFXConfiguration config4{};
  configs::TalonFXConfiguration config5{};
  

  // 传感器、电机相关初始化，子类需要重写
  virtual void Init() {
    }

  // 电机复位函数，取零点
  virtual void Reset() {

  }
  
  
private:
  
  
  
    // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
