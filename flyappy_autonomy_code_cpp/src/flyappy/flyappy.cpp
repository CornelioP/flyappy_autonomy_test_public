#include "flyappy.hpp"
#include <iostream>

namespace flyappy
{


Flyappy::Flyappy()
{
    state_estimate_ = std::make_shared<StateEstimate>(0.033f);
    gate_detector_ = std::make_shared<GateDetector>();
    longitudinal_controller_ = std::make_shared<PID>(1.0f, 0.0f, 0.0f, 0.033f);
    y_controller_ = std::make_shared<PID>(50.0f, 0.0f, 20.0f, 0.033f);
}

void Flyappy::detectGate(std::vector<point> points)
{   
  
    gate_detector_->detectWallPoints(points);
    gate_detector_->detectGate();

    // General print with gate detected point and width, then print the bird position
    std::cout << "Gate detected at: (" << gate_detector_->get_wall().gate_center.x << ", "
              << gate_detector_->get_wall().gate_center.y << ")"<< std::endl;
    
    //Print bird position
    std::cout << "Bird position: (" << state_estimate_->get_position().x << ", "
              << state_estimate_->get_position().y << ")"<< std::endl;

    // If the bird has passed the gate, reset the wall
    if (gate_detector_->hasPassedGate(state_estimate_->get_position().x))
    {
        gate_detector_->resetWall();
    }

    
}

std::tuple<float,float> Flyappy::computeControlLaw()
{
    // Retrieve current y position of the gate 
    float ref_y = gate_detector_->get_wall().gate_center.y;

    // Retrieve current position of the bird
    float fly_y = state_estimate_->get_position().y;
    float fly_x = state_estimate_->get_position().x;

    // Compute control law for y
    auto [acc_y, t] = computeYControlLaw(ref_y, fly_y);
    
    // Compute control law for x
    float acc_x = computeXControlLaw(fly_x, t);

    return {acc_x, acc_y};

}  

std::tuple<float, float> Flyappy::computeYControlLaw(float reference, float current_value)
{
    // Compute error between reference and current y position
    float error = reference - current_value;

    // Current vertical velocity of the bird
    float v_y = state_estimate_->get_velocity().y;

    // Time and acceleration initialization
    float t = 0.0f;
    float acc_y = 0.0f;

    // Compute stopping distance S with a safety margin
    float S = v_y * v_y / (2 * flyappy_constants::max_ay) + 2 * (std::abs(v_y) * flyappy_constants::delta_t + 0.5f * flyappy_constants::max_ay * flyappy_constants::delta_t * flyappy_constants::delta_t);

    // **PID control when close to target**
    if (std::abs(error) < 0.1f)
    {
        acc_y = y_controller_->compute_control_law(reference, current_value);

        // Avoid division by zero
        if (acc_y != 0.0f)
        {
            t = std::abs(v_y) / std::abs(acc_y);
        }
        else
        {
            t = 0.0f;  // No acceleration means no need to compute time
        }
    }
    else
    {
        // **Bang-bang control logic**

        // Going up and need to keep going up
        if (error > 0 && v_y >= 0)
        {
            if (S > std::abs(error))
            {
                acc_y = -flyappy_constants::max_ay;  // Slow down
                t = v_y / std::abs(acc_y);
            }
            else
            {
                acc_y = flyappy_constants::max_ay;  // Speed up
                t = (-v_y + std::sqrt(std::max(0.0f, v_y * v_y + 4 * flyappy_constants::max_ay * std::abs(error)))) / flyappy_constants::max_ay;
            }
        }
        // Need to go up but currently descending
        else if (error > 0 && v_y < 0)
        {
            acc_y = flyappy_constants::max_ay;
            t = (-v_y + std::sqrt(std::max(0.0f, v_y * v_y + 4 * flyappy_constants::max_ay * std::abs(error)))) / flyappy_constants::max_ay;
        }
        // Going down and need to keep going down
        else if (error < 0 && v_y <= 0)
        {
            if (S > std::abs(error))
            {
                acc_y = flyappy_constants::max_ay;  // Slow down descent
                t = -v_y / flyappy_constants::max_ay;
            }
            else
            {
                acc_y = -flyappy_constants::max_ay;  // Speed up descent
                t = (v_y + std::sqrt(std::max(0.0f, v_y * v_y - 4 * flyappy_constants::max_ay * std::abs(error)))) / flyappy_constants::max_ay;
            }
        }
        // Need to go down but currently ascending
        else if (error < 0 && v_y > 0)
        {
            acc_y = -flyappy_constants::max_ay;
            t = v_y / flyappy_constants::max_ay + std::sqrt(std::max(0.0f, v_y * v_y - 4 * flyappy_constants::max_ay * std::abs(error))) / flyappy_constants::max_ay;
        }
    }

    return {acc_y, t};
}


float Flyappy::computeXControlLaw(float current_x, float t)
{
    // Retrieve the current x velocity of the bird
    float v_x = state_estimate_->get_velocity().x;

    // Retrieve gate parameters
    float safe_pos = gate_detector_->get_wall().gate_safe_x;
    float max_pos = gate_detector_->get_wall().gate_end_x;

    float front_distance = safe_pos - current_x;  // Distance to the start of the gate
    float back_distance = max_pos - current_x;    // Distance to the end of the gate

    float acc_x = 0.0f;  // Initialize x-axis acceleration

    // **1. Bird is inside the gate (between `safe_pos` and `max_pos`)**
    if (current_x > safe_pos && current_x < max_pos)
    {
        // Stopping distance `S` with safety margin
        float S = v_x * v_x / (2 * flyappy_constants::max_ax) + 2 * (std::abs(v_x) * flyappy_constants::delta_t + 0.5f * flyappy_constants::max_ax * flyappy_constants::delta_t * flyappy_constants::delta_t);

        if (S < back_distance)
        {
            acc_x = flyappy_constants::max_ax;  // Accelerate forward
        }
        else
        {
            acc_x = -flyappy_constants::max_ax;  // Slow down to avoid overshooting
        }
    }
    // **2. Bird is behind the gate (needs to approach the gate)**
    else
    {
        float S = v_x * t;  // Compute how far it will go in time `t`

        if (S <= front_distance)
        {
            acc_x = 2 * (front_distance - S) / (t * t);  // Adjust acceleration to arrive in time `t`
        }
        else 
        {
            acc_x = -flyappy_constants::max_ax;  // Brake if too far behind
            // Bad 
            std::cout << "Brake if too far behind" << std::endl;
        }
    }

    // Clamp acceleration to maximum bounds
    return std::clamp(acc_x, -flyappy_constants::max_ax, flyappy_constants::max_ax);
}



}  // namespace flyappy
