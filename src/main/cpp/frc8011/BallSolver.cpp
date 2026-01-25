/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2025, BoCheng Su
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "frc8011/BallSolver.h"
#include "Constants.h"

void BallSolver::getConfig()
{
  resistance_coeff_ = SolverConstants::ResistanceCoefficient;
  g_ = SolverConstants::G;
  dt_ = SolverConstants::DT;
  timeout_ = SolverConstants::Timeout;
  delay_ = SolverConstants::Delay;
  max_iterations_ = SolverConstants::MaxIterations;
  error_tolerance_ = SolverConstants::ErrorTolerance;
  adjust_speed_ = SolverConstants::AdjustSpeed;
  max_flight_time_ = SolverConstants::MaxFlightTime;
  shooter_min_speed_ = SolverConstants::ShooterMinSpeed;
  shooter_max_speed_ = SolverConstants::ShooterMaxSpeed;
}

void BallSolver::setTarget(Eigen::Vector3d pos, Eigen::Vector3d vel)
{
  target_pos_ = pos;
  target_vel_ = vel;
}

double BallSolver::calculateFlightTime(double rho, double ball_speed, double pitch) const
{
  double vel_rho = ball_speed * std::cos(pitch);
  if (vel_rho <= 0)
    return -1.0;
  // 解析近似（忽略目标移动）
  double temp = resistance_coeff_ * rho / vel_rho;
  if (temp >= 1.0 || temp < 0.)
    return -1.0;
  return -std::log(1.0 - temp) / resistance_coeff_;
}

Eigen::Vector3d BallSolver::predictTargetPos(double time) const
{
  Eigen::Vector3d predicted = target_pos_;
  predicted[0] += target_vel_[0] * time;
  predicted[1] += target_vel_[1] * time;
  predicted[2] += target_vel_[2] * time;
  return predicted;
}

double BallSolver::calculateBallZ(double vel_z, double time) const
{
  double k = resistance_coeff_;
  double exp_term = std::exp(-k * time);
  return (vel_z / k) * (1.0 - exp_term) + (g_ / (k * k)) * (1.0 - exp_term) - (g_ / k) * time;
}

bool BallSolver::selectBestCandidate(std::vector<Solution>& candidates, bool prefer_lob_shoot, double& out_value,
                                     double& out_t, double& out_yaw) const
{
  if (candidates.empty())
    return false;

  // 去重
  std::sort(candidates.begin(), candidates.end(),
            [](const Solution& a, const Solution& b) { return a.value < b.value; });
  candidates.erase(std::unique(candidates.begin(), candidates.end(),
                               [](const Solution& a, const Solution& b) {
                                 // pitch 角度去重阈值大一点，速度去重小一点，这里统一给一个合理值
                                 // 或者根据应用场景，pitch 通常相差 2 度以内，speed 相差 0.5 左右
                                 return std::abs(a.value - b.value) < 0.05;
                               }),
                   candidates.end());

  // 排序逻辑：下降阶段绝对优先
  std::sort(candidates.begin(), candidates.end(), [&](const Solution& a, const Solution& b) {
    bool a_descending = (a.vz_hit < -0.1);
    bool b_descending = (b.vz_hit < -0.1);

    if (a_descending != b_descending)
      return a_descending > b_descending;  // 下降阶段排前面

    // 同为下降阶段
    if (a_descending && b_descending)
    {
      if (prefer_lob_shoot)
        return a.t > b.t;  // 高弧优先：飞行时间更长
      else
        return a.t < b.t;  // 低弧优先：飞行时间更短
    }

    // 同为上升阶段：飞行时间更长更安全
    return a.t > b.t;
  });

  const Solution& best = candidates.front();

  out_value = best.value;
  out_t = best.t;
  out_yaw = std::atan2(best.predicted_target[1], best.predicted_target[0]);

  return true;
}

bool BallSolver::solveForGimBalAngle(double ball_speed)
{
  if (ball_speed <= 0)
    return false;

  // 步骤1：初始猜测（当前直线视线，无阻力无移动）
  double rho = std::hypot(target_pos_[0], target_pos_[1]);
  if (rho <= 0)
    return false;

  double current_z = target_pos_[2];  // 用可变 z 迭代调整 pitch
  solved_yaw_ = std::atan2(target_pos_[1], target_pos_[0]);
  solved_pitch_ = std::atan2(current_z, rho);

  flight_time_ = calculateFlightTime(rho, ball_speed, solved_pitch_);

  Eigen::Vector3d predicted_target = target_pos_;

  for (int iter = 0; iter < max_iterations_; ++iter)
  {
    // 步骤2：预测目标未来位置（考虑 delay + flight_time）
    double total_time = flight_time_ + delay_;
    predicted_target = predictTargetPos(total_time);

    // 步骤3：更新 yaw 和 pitch 猜测
    double predicted_rho = std::hypot(predicted_target[0], predicted_target[1]);
    solved_yaw_ = std::atan2(predicted_target[1], predicted_target[0]);
    solved_pitch_ = std::atan2(current_z, predicted_rho);

    // 步骤4：计算新的飞行时间（水平方向解析近似）
    flight_time_ = calculateFlightTime(predicted_rho, ball_speed, solved_pitch_);
    if (flight_time_ < 0 || flight_time_ > timeout_)
      return false;

    // 步骤5：计算子弹实际到达高度
    double vel_z = ball_speed * std::sin(solved_pitch_);
    double z_ball = calculateBallZ(vel_z, flight_time_);

    // 步骤6：误差（垂直 + 横向换算）
    double delta_z = predicted_target[2] - z_ball;
    double delta_yaw = solved_yaw_ - std::atan2(predicted_target[1], predicted_target[0]);
    double lateral_error = std::abs(delta_yaw) * predicted_rho;  // yaw 误差转横向距离
    double total_error = std::hypot(delta_z, lateral_error);

    if (total_error < error_tolerance_)
      return true;

    // 步骤7：更新 current_z（相当于调整 pitch）
    current_z += delta_z;  // 简单固定步长，也可用阻尼如 current_z += 0.8 * delta_z
  }

  // 未收敛
  solved_pitch_ = solved_yaw_ = flight_time_ = std::numeric_limits<double>::quiet_NaN();
  return false;
}

bool BallSolver::solveForSpeed(double pitch_angle, bool prefer_lob_shoot)
{
  // 计算水平距离（初始目标位置）
  double horizontal_distance = std::hypot(target_pos_[0], target_pos_[1]);
  if (horizontal_distance <= 0.1)
    horizontal_distance = 0.1;  // 防除零

  double sin_2theta = std::sin(2.0 * pitch_angle);
  if (sin_2theta <= 0.1)
    sin_2theta = 0.1;  // 防 pitch 接近 0° 或 90°

  // 高速初始猜测（倾向于平射/低弧弹道）
  double v_guess_high = std::sqrt(horizontal_distance * g_ / sin_2theta);
  v_guess_high = std::max(shooter_min_speed_, std::min(shooter_max_speed_, v_guess_high));

  // 中速猜测（一般吊射）
  double v_guess_mid = v_guess_high * 0.6;

  // 极低速猜测（超高弧吊射，远距离时非常有用）
  double v_guess_very_low = v_guess_high * 0.4;
  v_guess_mid = std::max(SolverConstants::ShooterMinSpeed, v_guess_mid);
  v_guess_very_low = std::max(SolverConstants::ShooterMinSpeed, v_guess_very_low);

  std::vector<Solution> candidates;

  // 统一的迭代函数
  auto run_iteration = [&](double v_start) -> std::optional<Solution> {
    double v = v_start;

    for (int iter = 0; iter < max_iterations_; ++iter)
    {
      double v_rho = v * std::cos(pitch_angle);
      double v_z = v * std::sin(pitch_angle);

      if (v_rho <= 0.0)
        return std::nullopt;

      double approx_t = calculateFlightTime(horizontal_distance, v, pitch_angle);
      if (approx_t < 0.0)
        return std::nullopt;

      double total_time = approx_t + delay_;
      Eigen::Vector3d predicted_target = predictTargetPos(total_time);

      double pred_rho = std::hypot(predicted_target[0], predicted_target[1]);

      double t = calculateFlightTime(pred_rho, v, pitch_angle);
      if (t < 0.0 || t > max_flight_time_)
        return std::nullopt;

      double z_ball = calculateBallZ(v_z, t);

      total_time = t + delay_;
      predicted_target = predictTargetPos(total_time);

      double vertical_error = predicted_target[2] - z_ball;

      if (std::abs(vertical_error) < error_tolerance_)
      {
        double exp_term = std::exp(-resistance_coeff_ * total_time);
        double vz_hit = v_z * exp_term - (g_ / resistance_coeff_) * (1.0 - exp_term);
        return Solution{ v, t, vz_hit, vertical_error, predicted_target };
      }

      // 打低 → 增大速度，打高 → 减小速度
      v += vertical_error * adjust_speed_;

      v = std::clamp(v, shooter_min_speed_, shooter_max_speed_);
    }
    return std::nullopt;
  };

  // 三分支迭代，增加极低速分支
  if (auto sol = run_iteration(v_guess_high))
    candidates.push_back(*sol);
  if (auto sol = run_iteration(v_guess_mid))
    candidates.push_back(*sol);
  if (auto sol = run_iteration(v_guess_very_low))
    candidates.push_back(*sol);

  if (!selectBestCandidate(candidates, prefer_lob_shoot, solve_speed_, flight_time_, solved_yaw_))
    return false;

  return true;
}

double BallSolver::getSolvedPitch() const
{
  return solved_pitch_;
}
double BallSolver::getSolvedYaw() const
{
  return solved_yaw_;
}

double BallSolver::getBallSpeed() const
{
  return solve_speed_;
}

double BallSolver::getFlightTime() const
{
  return flight_time_;
}