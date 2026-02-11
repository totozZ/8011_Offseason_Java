#include "frc8011/Wayimotor.h"
#include <algorithm>
#include <frc/RobotBase.h>

void Wayimotor::Control()
{
  // 模拟器中跳过电机控制，避免崩溃
  if (frc::RobotBase::IsSimulation())
  {
    return;
  }

  // 电机控制
  switch (wayiconfig.mode)
  {
    case 0:
      motor.SetControl(brake);
      break;
    case 1:
      wayiconfig.Veloutput = wayiconfig.targetVelocity * wayiconfig.gearRatio * 1_tps * wayiconfig.invert;
      motor.SetControl(velocity.WithVelocity(wayiconfig.Veloutput));
      break;
    case 2:
      wayiconfig.Posoutput =
          (wayiconfig.targetPosition * wayiconfig.gearRatio + wayiconfig.offset) * 1_tr * wayiconfig.invert;
      motor.SetControl(position.WithPosition(wayiconfig.Posoutput));
      break;
    case 3:
      wayiconfig.Curoutput = wayiconfig.targetCurrent * 1_A * wayiconfig.invert;
      motor.SetControl(Torque.WithOutput(wayiconfig.Curoutput).WithMaxAbsDutyCycle(wayiconfig.Current_speed));
      // frc::SmartDashboard::PutNumber("Climb I", m_climb.Getdata().targetCurrent);
      break;
    case 4:
      wayiconfig.motionoutput =
          (wayiconfig.targetPosition * wayiconfig.gearRatio + wayiconfig.offset) * 1_tr * wayiconfig.invert;
      motor.SetControl(motionmagic.WithPosition(wayiconfig.motionoutput));
      break;
    case 5:
      wayiconfig.mmVeloutput = wayiconfig.targetVelocity * wayiconfig.gearRatio * 1_tps * wayiconfig.invert;
      motor.SetControl(motionmagicvelocity.WithVelocity(wayiconfig.mmVeloutput));
      break;
    case 6:
      wayiconfig.DutyOutput =
          wayiconfig.targetVelocity / wayiconfig.maxVelocity * 1.0 * wayiconfig.invert;  // 将目标速度转换为占空比
      motor.SetControl(DutyCircle.WithOutput(wayiconfig.DutyOutput));                    // 设置占空比
      break;
    case 7:
      wayiconfig.motionoutput =
          (wayiconfig.targetPosition * wayiconfig.gearRatio + wayiconfig.offset) * 1_tr * wayiconfig.invert;
      motor.SetControl(mm_position.WithPosition(wayiconfig.motionoutput));
      break;
    case 8:
      motor.SetControl(controls::Follower{ wayiconfig.followerId, wayiconfig.follow_invert });
      break;
    case 9:
      wayiconfig.Veloutput = wayiconfig.targetVelocity * wayiconfig.gearRatio * 1_tps * wayiconfig.invert;
      motor.SetControl(velocitytorquecurrent.WithVelocity(wayiconfig.Veloutput));
      break;
    case 11:
    {
      // BangBang 作为速度监测器：检测是否掉速
      double boost = bangBangController.Calculate(wayiconfig.currentVelocity, wayiconfig.targetVelocity);
      // boost=1 表示掉速，需要增压；boost=0 表示达速

      // 基础速度控制 + BangBang 增压
      wayiconfig.Veloutput = wayiconfig.targetVelocity * wayiconfig.gearRatio * 1_tps * wayiconfig.invert;

      if (wayiconfig.useTorqueCurrent)
      {
        // 使用 VelocityTorqueCurrentFOC 控制，掉速时叠加额外电流
        double extraCurrent = boost * wayiconfig.bangBangBoostCurrent;
        motor.SetControl(velocitytorquecurrent.WithVelocity(wayiconfig.Veloutput).WithFeedForward(extraCurrent * 1_A));
      }
      else
      {
        // 使用 VelocityVoltage 控制，掉速时叠加额外电压
        double extraVoltage = boost * wayiconfig.bangBangBoostVoltage;
        motor.SetControl(velocity.WithVelocity(wayiconfig.Veloutput).WithFeedForward(extraVoltage * 1_V));
      }
    }
    break;
    default:
      break;
  }
}

void Wayimotor::Receive()
{
  // 模拟器中跳过电机数据接收，避免崩溃
  if (frc::RobotBase::IsSimulation())
  {
    return;
  }

  // 电机数据接收
  wayiconfig.currentPosition =
      (motor.GetPosition().GetValueAsDouble() - wayiconfig.offset) / wayiconfig.gearRatio * wayiconfig.invert;
  wayiconfig.currentVelocity = motor.GetVelocity().GetValueAsDouble() / wayiconfig.gearRatio * wayiconfig.invert;
  wayiconfig.currentCurrent = motor.GetTorqueCurrent().GetValueAsDouble() * wayiconfig.invert;

  wayiconfig.currentnormalizedPosition =
      (wayiconfig.currentPosition - wayiconfig.minPosition) / (wayiconfig.maxPosition - wayiconfig.minPosition);
  wayiconfig.currentnormalizedVelocity = wayiconfig.currentVelocity / wayiconfig.maxVelocity;
  wayiconfig.currentnormalizedCurrent = wayiconfig.currentCurrent / wayiconfig.maxCurrent;
}

void Wayimotor::Reset(double _offset)
{
  wayiconfig.offset = _offset;
  setposition(0);
}

void Wayimotor::setNormalizedVelocity(double normalizedVel)
{
  // 速度：-1-1 范围
  wayiconfig.normalizedVelocity = std::clamp(normalizedVel, -1.0, 1.0);
  setmode(1);
  wayiconfig.targetVelocity = wayiconfig.normalizedVelocity * wayiconfig.maxVelocity;
}

void Wayimotor::setNormalizedPosition(double normalizedPos)
{
  // 位置：0-1 范围
  wayiconfig.normalizedPosition = std::clamp(normalizedPos, -1.0, 1.0);
  setmode(2);
  wayiconfig.targetPosition =
      wayiconfig.normalizedPosition * (wayiconfig.maxPosition - wayiconfig.minPosition) + wayiconfig.minPosition;
}

void Wayimotor::setNormalizedCurrent(double normalizedCur)
{
  // 电流：-1-1 范围
  wayiconfig.normalizedCurrent = std::clamp(normalizedCur, -1.0, 1.0);
  setmode(3);
  wayiconfig.targetCurrent = wayiconfig.normalizedCurrent * wayiconfig.maxCurrent;
}

void Wayimotor::setNormalizedMotion(double normalizedPos)
{
  // MotionMagic位置：0-1 范围
  wayiconfig.normalizedPosition = std::clamp(normalizedPos, -1.0, 1.0);
  setmode(4);
  wayiconfig.targetPosition =
      wayiconfig.normalizedPosition * (wayiconfig.maxPosition - wayiconfig.minPosition) + wayiconfig.minPosition;
}

void Wayimotor::setNormalizedMotionVelocity(double normalizedVel)
{
  // MotionMagic速度：-1-1 范围
  wayiconfig.normalizedVelocity = std::clamp(normalizedVel, -1.0, 1.0);
  setmode(5);
  wayiconfig.targetVelocity = wayiconfig.normalizedVelocity * wayiconfig.maxVelocity;
}

void Wayimotor::setNormalizedDutyCircle(double normalizedDuty)
{
  // 占空比：-1-1 范围
  wayiconfig.normalizedVelocity = std::clamp(normalizedDuty, -1.0, 1.0);
  setmode(6);
  wayiconfig.targetVelocity = wayiconfig.normalizedVelocity * wayiconfig.maxVelocity;
}

void Wayimotor::setNormalizedMotionPosition(double normalizedPos)
{
  // MotionMagic位置：-1-1 范围
  wayiconfig.normalizedPosition = std::clamp(normalizedPos, -1.0, 1.0);
  setmode(7);
  wayiconfig.targetPosition =
      wayiconfig.normalizedPosition * (wayiconfig.maxPosition - wayiconfig.minPosition) + wayiconfig.minPosition;
}

void Wayimotor::setVelocityTorqueCurrent(double normalizedVel)
{
  wayiconfig.normalizedVelocity = std::clamp(normalizedVel, -1.0, 1.0);
  setmode(9);
  wayiconfig.targetVelocity = wayiconfig.normalizedVelocity * wayiconfig.maxVelocity;
}
