#include "PID.hpp"

PID::PID(float kp, float ki, float kd, float dt) : kp_(kp), ki_(ki), kd_(kd), dt_(dt), integral_(0.0f), prev_error_(0.0f)
{
}

float PID::compute_control_law(float reference, float current_value)
{
    float error = reference - current_value;
    integral_ += error * dt_;
    float derivative = (error - prev_error_) / dt_;
    prev_error_ = error;

    return kp_ * error + ki_ * integral_ + kd_ * derivative;
}