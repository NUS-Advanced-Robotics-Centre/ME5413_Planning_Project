/** pid.hpp
 *
 * Copyright (C) 2024 Shuo SUN & Advanced Robotics Center, National University of Singapore
 *
 * MIT License
 *
 * Implementation of PID controller
 */

#pragma once

#include <iostream>
#include <cmath>

namespace control
{
class PID
{
 public:
  PID() {};
  PID(double dt, double max, double min, double Kp, double Kd, double Ki);
  ~PID() {};

  void updateSettings(const double Kp, const double Kd, const double Ki);
  // Returns the manipulated variable given a setpoint and current process value
  double calculate(const double setpoint, const double pv);

 private:
  double dt_;
  double max_;
  double min_;
  double Kp_;
  double Kd_;
  double Ki_;
  double pre_error_;
  double integral_;
};

PID::PID(double dt, double max, double min, double Kp, double Kd, double Ki) :
  dt_(dt),
  max_(max),
  min_(min),
  Kp_(Kp),
  Kd_(Kd),
  Ki_(Ki),
  pre_error_(0),
  integral_(0)
{};

void PID::updateSettings(const double Kp, const double Kd, const double Ki)
{
  this->Kp_ = Kp;
  this->Kd_ = Kd;
  this->Ki_ = Ki;
};

double PID::calculate(const double setpoint, const double pv)
{
  // Calculate error
  double error = setpoint - pv;

  // Proportional term
  const double P_term = Kp_ * error;

  // Integral term
  integral_ += error * dt_;
  const double I_term = Ki_ * integral_;

  // Derivative term
  const double derivative = (error - pre_error_) / dt_;
  const double D_term = Kd_ * derivative;

  // Calculate total output
  double output = P_term + I_term + D_term;

  // Restrict to max/min
  output = std::min(output, max_);
  output = std::max(output, min_);

  // Save error to previous error
  pre_error_ = error;

  return output;
};

} // namespace control
