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

#pragma once

#include <cmath>
#include <iostream>
#include <Eigen/Geometry>
#include <limits>
#include <optional>
#include <vector>
#include <algorithm>

class BallSolver
{
public:
  BallSolver()
  {
    getConfig();
  };

  virtual ~BallSolver() = default;

  void setTarget(Eigen::Vector3d pos, Eigen::Vector3d vel);

  bool solveForGimBalAngle(double ball_speed);
  bool solveForSpeed(double pitch_angle, bool prefer_lob_shoot);

  double getSolvedPitch() const;
  double getSolvedYaw() const;
  double getBallSpeed() const;
  double getFlightTime() const;

private:
  void getConfig();
  double resistance_coeff_, g_, dt_, timeout_, delay_, max_iterations_, error_tolerance_;
  double shooter_min_speed_, shooter_max_speed_, max_flight_time_;
  Eigen::Vector3d target_pos_{ 0.0, 0.0, 0.0 };
  Eigen::Vector3d target_vel_{ 0.0, 0.0, 0.0 };
  double solved_pitch_, solved_yaw_, solve_speed_, flight_time_, adjust_speed_ = 0.;

  double calculateFlightTime(double rho, double ball_speed, double pitch) const;
  Eigen::Vector3d predictTargetPos(double time) const;
  double calculateBallZ(double vel_z, double time) const;

  struct Solution
  {
    double value;  // pitch or speed
    double t;
    double vz_hit;
    double vertical_error;
    Eigen::Vector3d predicted_target;
  };

  bool selectBestCandidate(std::vector<Solution>& candidates, bool prefer_lob_shoot, double& out_value, double& out_t,
                           double& out_yaw) const;
};
