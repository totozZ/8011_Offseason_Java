// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ElevatorSubsystem.h"

using namespace subsystems;

frc2::CommandPtr ElevatorSubsystem::UpperMethodCommand() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return RunOnce([/* this */] { /* one-time action goes here */ });
}

bool ElevatorSubsystem::UpperCondition() {
  // Query some boolean state, such as a digital sensor.
  return false;
}

void ElevatorSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.

  frc::SmartDashboard::PutNumber("m_elevatorGetposition", m_elevatorGetposition);
  frc::SmartDashboard::PutNumber("Elevator1 Velocity", m_elevator1.GetVelocity().GetValue().value());
  frc::SmartDashboard::PutNumber("Elevator1 Current", m_elevator1.GetTorqueCurrent().GetValueAsDouble());

    if (m_elevatorposition > OperatorConstants::max_height) {
    m_elevatorposition = OperatorConstants::max_height;
  }
  else if (m_elevatorposition < OperatorConstants::min_height) {
    m_elevatorposition = OperatorConstants::min_height;
  }

  auto desireelevatorposition = m_elevatorposition * -50_tr - m_elevatorpositionoffset * 1_tr; // -48.7最高
  m_elevator1.SetControl(m_motorMMRequest.WithPosition(desireelevatorposition));
  m_elevatorGetposition = m_elevator1.GetPosition().GetValueAsDouble() / -50.0;

  m_stretch.Control();
  m_stretch.Receive();
  m_climb.Control();
  m_climb.Receive();

  frc::SmartDashboard::PutNumber("m_elevatorposition", m_elevatorposition);
// 
  // frc::SmartDashboard::PutNumber("Stretch V", m_stretch.Getdata().currentVelocity);
  frc::SmartDashboard::PutNumber("Climb I", m_climb.Getdata().currentCurrent);
  
  // frc::SmartDashboard::PutNumber("Stretch Position", m_stretch.Getdata().currentPosition);
  // frc::SmartDashboard::PutNumber("Climb Position", m_climb.Getdata().currentPosition);
  if (releaseflag == 0){
    SetpushServo(0); // 常态
  }
  if (releaseflag > 0.5){
    SetpushServo(30); // 释放
  }

  if (lockflag == 0){
    SetlockServo(0); // 常态
    frc::SmartDashboard::PutNumber("lockfla2", 1);

  }

  if (lockflag > 0.5){
    SetlockServo(30); // 锁住
    frc::SmartDashboard::PutNumber("lockflag", 1);
  }
  
  // frc::SmartDashboard::PutNumber("ID1", m_climb.Getmotor().GetDeviceID());
  // frc::SmartDashboard::PutNumber("invert", m_climb.Getdata().invert);
  // frc::SmartDashboard::PutString("bus", m_climb.Getdata().canbus.GetName());
//1.027
}

void ElevatorSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}

void ElevatorSubsystem::FortMotorInit() {
  // 配置电机参数


    // elevator电机config配置
  configs::TalonFXConfiguration config1{};

  /* Configure Motion Magic */
  configs::MotionMagicConfigs &mm = config1.MotionMagic;
  mm.MotionMagicAcceleration = 15 * 16.3636_tr_per_s_sq; // Take approximately 0.5 seconds to reach max vel
  // Take approximately 0.1 seconds to reach max accel 
  mm.MotionMagicJerk = 150 * 16.3636_tr_per_s_cu;
  // expo所需参数
  mm.MotionMagicCruiseVelocity = 0_tps; // 5 (mechanism) rotations per second cruise
  mm.MotionMagicExpo_kV=0.0005_V / 1_tps; // 0.12
  mm.MotionMagicExpo_kA=0.0005_V / 1_tr_per_s_sq; // 0.1


  configs::Slot0Configs &slot0 = config1.Slot0;
  slot0.kG = -28; // Gear ratio of 1:2, 0.5 rotations per rotor rotation
  slot0.kS = 2; // Add 0.25 V output to overcome static friction
  slot0.kV = 0.05; // A velocity target of 1 rps results in 0.12 V output
  slot0.kA = 0.08; // An acceleration of 1 rps/s requires 0.01 V output
  slot0.kP = 80; // A position error of 0.2 rotations results in 12 V output
  slot0.kI = 25; // No output for integrated error
  slot0.kD = 6; // A velocity error of 1 rps results in 0.5 V output

  ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i) {
    status = m_elevator1.GetConfigurator().Apply(config1);
    if (status.IsOK()) break;
  }
  if (!status.IsOK()) {
    std::cout << "Could not configure device. Error: " << status.GetName() << std::endl;
  }
  
  // Elevator2电机配置follow模式
  m_elevator2.SetControl(controls::Follower{m_elevator1.GetDeviceID(), false});

  /* 爬升和stretch电机设置 */


  // 抓取电机
  configs::TalonFXConfiguration configs2{};

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
  configs2.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
  configs2.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
  configs2.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
  configs2.Slot0.kI = 0; // No output for integrated error
  configs2.Slot0.kD = 0; // No output for error derivative
  // Peak output of 8 volts
  configs2.Voltage.PeakForwardVoltage = 8_V;
  configs2.Voltage.PeakReverseVoltage = -8_V;

    /* Retry config apply up to 5 times, report if failure */
  ctre::phoenix::StatusCode status2 = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i) {
    status2 = m_stretch.Applyconfig(configs2);
    if (status2.IsOK()) break;
  }
  if (!status2.IsOK()) {
    std::cout << "Could not apply configs, error code: " << status2.GetName() << std::endl;
  }

  m_stretch.setvelocity(0);


  // 爬升电机
  configs::TalonFXConfiguration config3{};
  
      /* Retry config apply up to 5 times, report if failure */
  ctre::phoenix::StatusCode status3 = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i) {
    status3 = m_climb.Applyconfig(config3);
    if (status3.IsOK()) break;
  }
  if (!status3.IsOK()) {
    std::cout << "Could not apply configs, error code: " << status3.GetName() << std::endl;
  }

  m_climb.setcurrent(0);

  m_climb.setinvert(-1);

}

frc2::CommandPtr ElevatorSubsystem::TeleopControlCommand() {
              static double tempcurrent;
              static double tempstretch;

  // Use the joystick to control the subsystem.
  return this->RunOnce(
      [this] {     


          if (m_joystick.LeftBumper().Get()) {

            if (fabs(m_joystick.GetLeftY()) <= 0.05) { // joystick deadzone
            }
            else {
              tempcurrent += m_joystick.GetLeftY() * -0.5;
            }

          if (fabs(m_joystick.GetRightY()) <= 0.05) { // joystick deadzone
            }
            else {
              tempstretch = m_joystick.GetRightY() * -60;
            }
        }
        else {
          tempcurrent = 0.0001;
          tempstretch = 0;
        }
        // m_climb.setcurrent(tempcurrent); 
        // m_stretch.setvelocity(tempstretch);
        frc::SmartDashboard::PutNumber("Climb Itar", tempcurrent);
        frc::SmartDashboard::PutNumber("Stretch Itar", tempstretch);

      }
  );
}
 


  frc2::CommandPtr ElevatorSubsystem::DefaultCommand() {

    return this->RunOnce(
      [this] {
      }
    );

  }

  frc2::CommandPtr ElevatorSubsystem::SetHeightCommandPtr(double height) {
        return this->RunOnce(
        [this, height] {
          m_elevatorposition = height;
        }
      );
  }

  frc2::CommandPtr ElevatorSubsystem::plusHeightCommandPtr() {
        return this->RunOnce(
        [this] {
          //  = 2;
          
          m_elevatorposition += 0.005;
        }
      );
  }

  frc2::CommandPtr ElevatorSubsystem::minusHeightCommandPtr() {
        return this->RunOnce(
        [this] {
          //  = 2;
          
          m_elevatorposition -= 0.005;
        }
      ); 
  }

  std::function< bool()> ElevatorSubsystem::isAllowtoPitchToZero(){
    
    bool isAllow=false;
      if(m_elevator1.GetPosition().GetValue().value() <=0.05){
          isAllow=true;
      }
      return [isAllow]() { return isAllow; };

  }


  frc2::CommandPtr ElevatorSubsystem::SetClimbcurrent(double current) {
  return this->RunOnce(
        [this, current] {
        frc::SmartDashboard::PutNumber("Climb targetcurrent", current);
          
                  m_climb.setcurrent(current); 
        }
      );
  }