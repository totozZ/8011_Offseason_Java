#include "frc8011/Wayimotor.h"

#include <algorithm>

#include <frc/RobotBase.h>

void Wayimotor::Control() {
  if (frc::RobotBase::IsSimulation()) {
    return;
  }

  switch (wayiconfig.mode) {
    case 0:
      motor.SetControl(brake);
      break;
    case 13:
      motor.SetControl(coast);
      break;
    case 1:
      wayiconfig.Veloutput =
          wayiconfig.targetVelocity * wayiconfig.gearRatio * 1_tps *
          wayiconfig.invert;
      motor.SetControl(velocity.WithVelocity(wayiconfig.Veloutput));
      break;
    case 2:
      wayiconfig.Posoutput =
          (wayiconfig.targetPosition * wayiconfig.gearRatio *
               wayiconfig.invert +
           wayiconfig.offset) *
          1_tr;
      motor.SetControl(position.WithPosition(wayiconfig.Posoutput));
      break;
    case 3:
      wayiconfig.Curoutput = wayiconfig.targetCurrent * 1_A * wayiconfig.invert;
      motor.SetControl(
          Torque.WithOutput(wayiconfig.Curoutput)
              .WithMaxAbsDutyCycle(wayiconfig.Current_speed));
      break;
    case 4:
      wayiconfig.motionoutput =
          (wayiconfig.targetPosition * wayiconfig.gearRatio *
               wayiconfig.invert +
           wayiconfig.offset) *
          1_tr;
      motor.SetControl(motionmagic.WithPosition(wayiconfig.motionoutput));
      break;
    case 5:
      wayiconfig.mmVeloutput =
          wayiconfig.targetVelocity * wayiconfig.gearRatio * 1_tps *
          wayiconfig.invert;
      motor.SetControl(
          motionmagicvelocity.WithVelocity(wayiconfig.mmVeloutput));
      break;
    case 6:
      wayiconfig.DutyOutput = wayiconfig.targetVelocity /
                              wayiconfig.maxVelocity * 1.0 *
                              wayiconfig.invert;
      motor.SetControl(DutyCircle.WithOutput(wayiconfig.DutyOutput));
      break;
    case 7:
      wayiconfig.motionoutput =
          (wayiconfig.targetPosition * wayiconfig.gearRatio *
               wayiconfig.invert +
           wayiconfig.offset) *
          1_tr;
      motor.SetControl(mm_position.WithPosition(wayiconfig.motionoutput));
      break;
    case 8:
      motor.SetControl(
          controls::Follower{wayiconfig.followerId, wayiconfig.follow_invert});
      break;
    case 9:
      wayiconfig.Veloutput =
          wayiconfig.targetVelocity * wayiconfig.gearRatio * 1_tps *
          wayiconfig.invert;
      motor.SetControl(
          velocitytorquecurrent.WithVelocity(wayiconfig.Veloutput));
      break;
    case 11: {
      const double boost = bangBangController.Calculate(
          wayiconfig.currentVelocity, wayiconfig.targetVelocity);
      wayiconfig.Veloutput =
          wayiconfig.targetVelocity * wayiconfig.gearRatio * 1_tps *
          wayiconfig.invert;

      if (wayiconfig.useTorqueCurrent) {
        const double extraCurrent = boost * wayiconfig.bangBangBoostCurrent;
        motor.SetControl(velocitytorquecurrent.WithVelocity(wayiconfig.Veloutput)
                             .WithFeedForward(extraCurrent * 1_A));
      } else {
        const double extraVoltage = boost * wayiconfig.bangBangBoostVoltage;
        motor.SetControl(velocity.WithVelocity(wayiconfig.Veloutput)
                             .WithFeedForward(extraVoltage * 1_V));
      }
      break;
    }
    case 12:
      wayiconfig.Posoutput =
          (wayiconfig.targetPosition * wayiconfig.gearRatio *
               wayiconfig.invert +
           wayiconfig.offset) *
          1_tr;
      motor.SetControl(positionDutyCycle.WithPosition(wayiconfig.Posoutput));
      break;
    case 14:
      wayiconfig.Posoutput =
          (wayiconfig.targetPosition * wayiconfig.gearRatio *
               wayiconfig.invert +
           wayiconfig.offset) *
          1_tr;
      motor.SetControl(
          positiontorquecurrent.WithPosition(wayiconfig.Posoutput));
      break;
    case 15:
      wayiconfig.motionoutput =
          (wayiconfig.targetPosition * wayiconfig.gearRatio *
               wayiconfig.invert +
           wayiconfig.offset) *
          1_tr;
      motor.SetControl(
          motionmagictorquecurrent.WithPosition(wayiconfig.motionoutput));
      break;
    default:
      break;
  }
}

void Wayimotor::Receive() {
  if (frc::RobotBase::IsSimulation()) {
    return;
  }

  ReceivePosition();
  ReceiveVelocity();
  ReceiveCurrent();

  const double position_range =
      wayiconfig.maxPosition - wayiconfig.minPosition;
  wayiconfig.currentnormalizedPosition =
      (std::abs(position_range) > 1e-9)
          ? ((wayiconfig.currentPosition - wayiconfig.minPosition) /
             position_range)
          : 0.0;

  wayiconfig.currentnormalizedVelocity =
      (std::abs(wayiconfig.maxVelocity) > 1e-9)
          ? (wayiconfig.currentVelocity / wayiconfig.maxVelocity)
          : 0.0;
  wayiconfig.currentnormalizedCurrent =
      (std::abs(wayiconfig.maxCurrent) > 1e-9)
          ? (wayiconfig.currentCurrent / wayiconfig.maxCurrent)
          : 0.0;
}

void Wayimotor::ReceivePosition() {
  if (frc::RobotBase::IsSimulation()) {
    return;
  }
  if (refresh_status_signals_on_receive_) {
    BaseStatusSignal::RefreshAll(position_signal_);
  }
  wayiconfig.currentPosition =
      (position_signal_.GetValueAsDouble() - wayiconfig.offset) /
      wayiconfig.gearRatio * wayiconfig.invert;
}

void Wayimotor::ReceiveVelocity() {
  if (frc::RobotBase::IsSimulation()) {
    return;
  }
  if (refresh_status_signals_on_receive_) {
    BaseStatusSignal::RefreshAll(velocity_signal_);
  }
  wayiconfig.currentVelocity =
      velocity_signal_.GetValueAsDouble() / wayiconfig.gearRatio *
      wayiconfig.invert;
}

void Wayimotor::ReceiveCurrent() {
  if (frc::RobotBase::IsSimulation()) {
    return;
  }
  if (refresh_status_signals_on_receive_) {
    BaseStatusSignal::RefreshAll(torque_current_signal_);
  }
  wayiconfig.currentCurrent =
      torque_current_signal_.GetValueAsDouble() * wayiconfig.invert;
}

void Wayimotor::Reset(double _offset) {
  wayiconfig.offset = _offset;
  setmode(0);
}

void Wayimotor::setNormalizedVelocity(double normalizedVel) {
  wayiconfig.normalizedVelocity = std::clamp(normalizedVel, -1.0, 1.0);
  setmode(1);
  wayiconfig.targetVelocity =
      wayiconfig.normalizedVelocity * wayiconfig.maxVelocity;
}

void Wayimotor::setNormalizedPosition(double normalizedPos) {
  wayiconfig.normalizedPosition = std::clamp(normalizedPos, -1.0, 1.0);
  setmode(2);
  wayiconfig.targetPosition = wayiconfig.normalizedPosition *
                                  (wayiconfig.maxPosition -
                                   wayiconfig.minPosition) +
                              wayiconfig.minPosition;
}

void Wayimotor::setNormalizedCurrent(double normalizedCur) {
  wayiconfig.normalizedCurrent = std::clamp(normalizedCur, -1.0, 1.0);
  setmode(3);
  wayiconfig.targetCurrent =
      wayiconfig.normalizedCurrent * wayiconfig.maxCurrent;
}

void Wayimotor::setNormalizedMotion(double normalizedPos) {
  wayiconfig.normalizedPosition = std::clamp(normalizedPos, -1.0, 1.0);
  setmode(4);
  wayiconfig.targetPosition = wayiconfig.normalizedPosition *
                                  (wayiconfig.maxPosition -
                                   wayiconfig.minPosition) +
                              wayiconfig.minPosition;
}

void Wayimotor::setNormalizedMotionVelocity(double normalizedVel) {
  wayiconfig.normalizedVelocity = std::clamp(normalizedVel, -1.0, 1.0);
  setmode(5);
  wayiconfig.targetVelocity =
      wayiconfig.normalizedVelocity * wayiconfig.maxVelocity;
}

void Wayimotor::setNormalizedDutyCircle(double normalizedDuty) {
  wayiconfig.normalizedVelocity = std::clamp(normalizedDuty, -1.0, 1.0);
  setmode(6);
  wayiconfig.targetVelocity =
      wayiconfig.normalizedVelocity * wayiconfig.maxVelocity;
}

void Wayimotor::setNormalizedMotionPosition(double normalizedPos) {
  wayiconfig.normalizedPosition = std::clamp(normalizedPos, -1.0, 1.0);
  setmode(7);
  wayiconfig.targetPosition = wayiconfig.normalizedPosition *
                                  (wayiconfig.maxPosition -
                                   wayiconfig.minPosition) +
                              wayiconfig.minPosition;
}

double Wayimotor::GetAbsPosition() {
  return motor.GetPosition().GetValueAsDouble();
}


double Wayimotor::GetPosition() {
  return (motor.GetPosition().GetValueAsDouble() - wayiconfig.offset) /
      wayiconfig.gearRatio * wayiconfig.invert;
}

double Wayimotor::GetVelocity() {
  return motor.GetVelocity().GetValueAsDouble() / wayiconfig.gearRatio *
         wayiconfig.invert;
}

double Wayimotor::GetCurrent() {
  return motor.GetTorqueCurrent().GetValueAsDouble() * wayiconfig.invert;
}

double Wayimotor::GetNormalizedPosition() {
  const double position_range = wayiconfig.maxPosition - wayiconfig.minPosition;
  if (std::abs(position_range) <= 1e-9) {
    return 0.0;
  }
  return (GetPosition() - wayiconfig.minPosition) / position_range;
}


double Wayimotor::GetNormalizedCurrent() {
  if (std::abs(wayiconfig.maxCurrent) <= 1e-9) {
    return 0.0;
  }
  return GetCurrent() / wayiconfig.maxCurrent;
}

double Wayimotor::GetNormalizedVelocity() {
  if (std::abs(wayiconfig.maxVelocity) <= 1e-9) {
    return 0.0;
  }
  return GetVelocity() / wayiconfig.maxVelocity;
}