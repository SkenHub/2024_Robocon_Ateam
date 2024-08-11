#include "pid.hpp"

Pid::Pid(void)
    : integral_error_(0), before_error_(0), kp_(0), ki_(0), kd_(0), d_error_(0),
      p_control_(0), i_control_(0), d_control_(0) {}

void Pid::setGain(double p_gain, double i_gain, double d_gain,
                  double time_constant) {
  kp_ = p_gain;
  ki_ = i_gain;
  kd_ = d_gain;
  if (time_constant > 0) {
    time_constant_ = time_constant;
  } else {
    time_constant_ = 0;
  }
}

double Pid::control(double target, double now, int control_period) {
  return Pid::control(target - now, control_period);
}

double Pid::control(double error, int control_period) {
  p_control_ = error * kp_;
  integral_error_ += error * (control_period / 1000.0);
  i_control_ = integral_error_ * ki_;
  if (time_constant_ != 0) {
    double alpha = (time_constant_ / (time_constant_ + control_period));
    d_error_ = before_error_ * alpha + (1 - alpha) * error;
  } else {
    d_error_ = error;
  }
  if (control_period != 0) {
    d_control_ = ((d_error_ - before_error_) / (control_period / 1000.0)) * kd_;
  }
  before_error_ = error;
  return p_control_ + i_control_ + d_control_;
}

void Pid::reset(void) {
  integral_error_ = 0;
  before_error_ = 0;
  d_error_ = 0;
}

double Pid::getControlValue(ControlType control_type) {
  if (control_type == ControlType::P) {
    return p_control_;
  } else if (control_type == ControlType::I) {
    return i_control_;
  } else {
    return d_control_;
  }
}
