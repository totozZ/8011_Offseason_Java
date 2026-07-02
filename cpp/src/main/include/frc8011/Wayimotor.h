#pragma once

#include <frc/controller/BangBangController.h>

#include <ctre/phoenix6/CANBus.hpp>
#include <ctre/phoenix6/controls/MotionMagicVelocityTorqueCurrentFOC.hpp>
#include <ctre/phoenix6/controls/VelocityTorqueCurrentFOC.hpp>
#include <ctre/phoenix6/controls/MotionMagicTorqueCurrentFOC.hpp>
#include <ctre/phoenix6/controls/PositionTorqueCurrentFOC.hpp>
#include <ctre/phoenix6/controls/CoastOut.hpp>
#include <ctre/phoenix6/controls/VoltageOut.hpp>

#include "ctre/phoenix6/TalonFX.hpp"

using namespace ctre::phoenix6;

struct WayimotorConfig {
  double targetPosition;
  double targetVelocity;
  double targetCurrent;

  double normalizedPosition;
  double normalizedVelocity;
  double normalizedCurrent;

  double maxPosition = 1.0;
  double minPosition = 0.0;
  double maxVelocity = 50.0;
  double maxCurrent = 40.0;

  units::angular_velocity::turns_per_second_t Veloutput;
  units::angular_velocity::turns_per_second_t mmVeloutput;
  units::angle::turn_t Posoutput;
  units::current::ampere_t Curoutput;
  units::angle::turn_t motionoutput;
  units::dimensionless::scalar_t DutyOutput;

  double currentPosition;
  double currentVelocity;
  double currentCurrent;
  double currentnormalizedPosition;
  double currentnormalizedVelocity;
  double currentnormalizedCurrent;

  int deviceId;
  CANBus canbus;
  double offset = 0;
  double gearRatio = 1;
  uint8_t mode = 0;
  double target;
  double invert = 1;
  double Current_speed = 0.5;
  int followerId = -1;
  bool follow_invert = false;
  bool useTorqueCurrent = false;
  double bangBangBoostVoltage = 2.0;
  double bangBangBoostCurrent = 5.0;
};

class Wayimotor {
 public:
  Wayimotor(int deviceId, CANBus canbus)
      : motor(deviceId, canbus),
        position_signal_(motor.GetPosition(false)),
        velocity_signal_(motor.GetVelocity(false)),
        torque_current_signal_(motor.GetTorqueCurrent(false)) {
    wayiconfig.deviceId = deviceId;
    wayiconfig.canbus = canbus;
    ConfigureStatusSignals();
  }

  ~Wayimotor() = default;

  void Control();
  void Receive();
  void ReceivePosition();
  void ReceiveVelocity();
  void ReceiveCurrent();
  void Reset(double _offset);

  /// One-shot blocking refresh of all status signals (position, velocity,
  /// torque-current).  Use this when you need guaranteed-fresh data in a
  /// specific code path (e.g. current-based homing) instead of enabling
  /// the global per-Receive refresh which blocks every cycle.
  void RefreshNow() {
    BaseStatusSignal::RefreshAll(position_signal_, velocity_signal_,
                                 torque_current_signal_);
  }

  ctre::phoenix::StatusCode Applyconfig(configs::TalonFXConfiguration config) {
    return motor.GetConfigurator().Apply(config);
  }

  void setmode(uint8_t mode) { wayiconfig.mode = mode; }

  void setgearRatio(double gearRatio) { wayiconfig.gearRatio = gearRatio; }

  void setoffset(double offset) { wayiconfig.offset = offset; }

  void setinvert(double invert) { wayiconfig.invert = invert; }

  void setbrake() { setmode(0); }

  void setcoast() { setmode(13); }

  void setNormalizedPosition(double normalizedPos);
  void setNormalizedVelocity(double normalizedVel);
  void setNormalizedCurrent(double normalizedCur);
  void setNormalizedMotion(double normalizedPos);
  void setNormalizedMotionPosition(double normalizedPos);
  void setNormalizedMotionVelocity(double normalizedVel);
  void setNormalizedDutyCircle(double normalizedDuty);

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
  }

  void setposition(double targetPosition) {
    setmode(2);
    wayiconfig.targetPosition = targetPosition;
  }

  void setpositiontorquecurrent(double targetPosition) {
    setmode(14);
    wayiconfig.targetPosition = targetPosition;
  }

  void setnormpositiontorquecurrent(double targetnormpos) {
    wayiconfig.normalizedPosition = std::clamp(targetnormpos, -1.0, 1.0);
    setmode(14);
    wayiconfig.targetPosition = wayiconfig.normalizedPosition *
                                   (wayiconfig.maxPosition -
                                    wayiconfig.minPosition) +
                               wayiconfig.minPosition;
  }

  void setmotionmagictorquecurrent(double targetPosition) {
    setmode(15);
    wayiconfig.targetPosition = targetPosition;
  }

  void setcurrent(double targetCurrent) {
    setmode(3);
    wayiconfig.targetCurrent = targetCurrent;
  }

  void setmotion(double targetPosition) {
    setmode(4);
    wayiconfig.targetPosition = targetPosition;
  }

  void setmotionvelocity(double targetVelocity) {
    setmode(5);
    wayiconfig.targetVelocity = targetVelocity;
  }

  void setvelocitytorquecurrent(double velocity) {
    setmode(9);
    wayiconfig.targetVelocity = velocity;
  }

  void setVoltage(units::volt_t voltage) {
    setmode(10);
    motor.SetControl(voltageOut.WithOutput(voltage));
  }

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
  }

  WayimotorConfig& Getdata() { return wayiconfig; }

  hardware::TalonFX& Getmotor() { return motor; }

  void setCurrent_Speed(double speed) { wayiconfig.Current_speed = speed; }

  void SetActiveStatusRefreshOnReceive(bool enabled) {
    refresh_status_signals_on_receive_ = enabled;
  }

  void SetStatusSignalUpdateFrequency(
      units::frequency::hertz_t update_frequency) {
    BaseStatusSignal::SetUpdateFrequencyForAll(
        update_frequency, position_signal_, velocity_signal_,
        torque_current_signal_);
  }

  double GetPosition();
  double GetVelocity();
  double GetCurrent();
  double GetNormalizedPosition();
  double GetNormalizedVelocity();
  double GetNormalizedCurrent();
  double GetAbsPosition();

 private:
  void ConfigureStatusSignals() {
    BaseStatusSignal::SetUpdateFrequencyForAll(
        kDefaultStatusSignalUpdateFrequency, position_signal_, velocity_signal_,
        torque_current_signal_);
  }

  static constexpr units::frequency::hertz_t
      kDefaultStatusSignalUpdateFrequency = 20_Hz;

  WayimotorConfig wayiconfig;
  hardware::TalonFX motor;
  StatusSignal<units::angle::turn_t> position_signal_;
  StatusSignal<units::angular_velocity::turns_per_second_t> velocity_signal_;
  StatusSignal<units::current::ampere_t> torque_current_signal_;
  bool refresh_status_signals_on_receive_ = false;

  ctre::phoenix6::controls::StaticBrake brake{};
  ctre::phoenix6::controls::CoastOut coast{};
  frc::BangBangController bangBangController{5.0};

  controls::VelocityVoltage velocity =
      controls::VelocityVoltage{0_tps}.WithSlot(0);

  controls::MotionMagicVelocityTorqueCurrentFOC motionmagicvelocity =
      controls::MotionMagicVelocityTorqueCurrentFOC{0_tps}
          .WithUpdateFreqHz(500_Hz)
          .WithSlot(0);

  controls::VelocityTorqueCurrentFOC velocitytorquecurrent =
      controls::VelocityTorqueCurrentFOC{0_tps}
          .WithUpdateFreqHz(500_Hz)
          .WithSlot(0);

  controls::PositionVoltage position =
      controls::PositionVoltage{0_tr}.WithSlot(0);

  controls::PositionTorqueCurrentFOC positiontorquecurrent =
      controls::PositionTorqueCurrentFOC{0_tr}
          .WithUpdateFreqHz(500_Hz)
          .WithSlot(0);

  controls::MotionMagicExpoVoltage mm_position =
      controls::MotionMagicExpoVoltage{0_tr}
          .WithUpdateFreqHz(50_Hz)
          .WithSlot(0)
          .WithEnableFOC(1);

  controls::TorqueCurrentFOC Torque =
      controls::TorqueCurrentFOC{0_A}.WithUpdateFreqHz(500_Hz);

  controls::MotionMagicExpoTorqueCurrentFOC motionmagic =
      controls::MotionMagicExpoTorqueCurrentFOC{0_tr}
          .WithUpdateFreqHz(500_Hz)
          .WithSlot(0);

  controls::MotionMagicTorqueCurrentFOC motionmagictorquecurrent =
      controls::MotionMagicTorqueCurrentFOC{0_tr}
          .WithUpdateFreqHz(500_Hz)
          .WithSlot(0);

  controls::DutyCycleOut DutyCircle =
      controls::DutyCycleOut{0.0}.WithUpdateFreqHz(500_Hz);

  controls::PositionDutyCycle positionDutyCycle =
      controls::PositionDutyCycle{0_tr}.WithSlot(0);

  controls::VoltageOut voltageOut =
      controls::VoltageOut{0_V}.WithUpdateFreqHz(500_Hz);
};