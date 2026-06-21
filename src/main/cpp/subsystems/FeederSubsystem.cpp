#include "subsystems/FeederSubsystem.h"

using namespace subsystems;

void FeederSubsystem::Initialization() {
  configs::TalonFXConfiguration backward_config{};
  backward_config.MotorOutput.Inverted = 1;
  configs::Slot0Configs& backward_slot0 = backward_config.Slot0;
  backward_slot0.kS = 0.12;
  backward_slot0.kV = 0.12;
  backward_slot0.kP = 0.03;
  backward_config.CurrentLimits.SupplyCurrentLimit = 80_A;
  backward_config.CurrentLimits.SupplyCurrentLowerLimit = 80_A;
  backward_config.CurrentLimits.StatorCurrentLimit = 120_A;
  backward_config.CurrentLimits.SupplyCurrentLimitEnable = true;

  ctre::phoenix::StatusCode backward_status =
      ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i) {
    backward_status = backward_feeder_.Applyconfig(backward_config);
    if (backward_status.IsOK()) {
      break;
    }
  }
  backward_feeder_.setinvert(-1);
  backward_feeder_.setCurrent_Speed(0.2);

  configs::TalonFXConfiguration upward_config{};
  upward_config.MotorOutput.Inverted = 0;
  configs::Slot0Configs& upward_slot0 = upward_config.Slot0;
  upward_slot0.kS = 10;
  upward_slot0.kP = 9;
  upward_config.CurrentLimits.StatorCurrentLimit = 120_A;
  upward_config.CurrentLimits.StatorCurrentLimitEnable = true;
  upward_config.CurrentLimits.SupplyCurrentLimit = 40_A;
  upward_config.CurrentLimits.SupplyCurrentLowerLimit = 40_A;
  upward_config.CurrentLimits.SupplyCurrentLimitEnable = true;

  ctre::phoenix::StatusCode upward_status =
      ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i) {
    upward_status = upward_feeder_.Applyconfig(upward_config);
    if (upward_status.IsOK()) {
      break;
    }
  }
  upward_feeder_.setinvert(-1);
  upward_feeder_.SetStatusSignalUpdateFrequency(50_Hz);
}

void FeederSubsystem::Periodic() { upward_feeder_.ReceiveVelocity(); }

void FeederSubsystem::SetUpwardFeederVelocity(double velocity_rps) {
  upward_feeder_.setvelocitytorquecurrent(velocity_rps);
  upward_feeder_.Control();
}

void FeederSubsystem::SetBackwardFeederDuty(double duty) {
  backward_feeder_.setNormalizedDutyCircle(duty);
  backward_feeder_.Control();
}

frc2::CommandPtr FeederSubsystem::SetBackwardFeederDutyCommandPtr(double duty) {
  return this->RunOnce([this, duty] { SetBackwardFeederDuty(duty); });
}

void FeederSubsystem::SetUpwardDuty(double duty) {
  upward_feeder_.setNormalizedDutyCircle(duty);
  upward_feeder_.Control();
}

void FeederSubsystem::Stop() {
  backward_feeder_.setcoast();
  upward_feeder_.setcoast();
  backward_feeder_.Control();
  upward_feeder_.Control();
}

frc2::CommandPtr FeederSubsystem::StopCommandPtr() {
  return this->RunOnce([this] { Stop(); });
}

double FeederSubsystem::GetBackwardFeederVelocity() {
  return backward_feeder_.GetVelocity();
}

double FeederSubsystem::GetUpwardFeederVelocity() {
  return upward_feeder_.GetVelocity();
}

double FeederSubsystem::GetUpwardFeederCurrent() {
  return upward_feeder_.GetCurrent();
}
