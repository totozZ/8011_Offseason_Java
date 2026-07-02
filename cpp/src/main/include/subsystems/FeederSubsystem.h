#pragma once

#include <frc2/command/sysid/SysIdRoutine.h>

#include "Constants.h"
#include "frc8011/Wayimotor.h"
#include "subsystems/ExampleSubsystem.h"

namespace subsystems {

class FeederSubsystem : public ExampleSubsystem {
 public:
  FeederSubsystem() { Initialization(); }

  void Periodic() override;
  void Stop();
  frc2::CommandPtr StopCommandPtr();

  void SetUpwardFeederVelocity(double velocity_rps);
  void SetBackwardFeederDuty(double duty);
  frc2::CommandPtr SetBackwardFeederDutyCommandPtr(double duty);
  void SetUpwardDuty(double duty);

  double GetBackwardFeederVelocity();
  double GetUpwardFeederVelocity();
  double GetUpwardFeederCurrent();

  frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction) {
    return sysid_routine_.Quasistatic(direction);
  }
  frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction) {
    return sysid_routine_.Dynamic(direction);
  }

 private:
  void Initialization();

  Wayimotor backward_feeder_{FeederConstants::BackwardFeederMotorID, kCANBus};
  Wayimotor upward_feeder_{FeederConstants::UpwardFeederMotorID, kCANBus};

  frc2::sysid::SysIdRoutine sysid_routine_{
      frc2::sysid::Config{std::nullopt, 4_V, std::nullopt, nullptr},
      frc2::sysid::Mechanism{
          [this](units::volt_t output) {
            backward_feeder_.setVoltage(output);
          },
          [this](frc::sysid::SysIdRoutineLog* log) {
            log->Motor("feeder")
                .voltage(
                    backward_feeder_.Getmotor().GetMotorVoltage().GetValue())
                .position(backward_feeder_.Getmotor().GetPosition().GetValue())
                .velocity(backward_feeder_.Getmotor().GetVelocity().GetValue());
          },
          this}};
};

}  // namespace subsystems
