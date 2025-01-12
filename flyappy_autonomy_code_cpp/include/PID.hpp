#pragma once

#include <memory>


class PID
{
private:
    float kp_;
    float ki_;
    float kd_;
    float dt_;
    float integral_;
    float prev_error_;

public:

    PID(float kp, float ki, float kd, float dt);
    ~PID() = default;

    float compute_control_law(float reference, float current_value);

};