#pragma once
#include "state_estimate.hpp"
#include "gate_detector.hpp"
#include "PID.hpp"
#include <memory>
#include <tuple>
#include <cmath>
#include <algorithm>



namespace flyappy_constants
{
    
    const float gate_width = 0.5;
    const float obstacle_distance = 1.92;
    const float safe_distance = 0.2;
    const float max_ay = 35.0;
    const float max_ax = 3.0;
    const float delta_t = 0.033;
}  




namespace flyappy
{

class Flyappy
{
private:
    std::shared_ptr<StateEstimate> state_estimate_;  // Use shared_ptr instead of unique_ptr
    std::shared_ptr<GateDetector> gate_detector_;  // Add gate detector
    std::shared_ptr<PID> longitudinal_controller_;  // Longitudinal controller
    std::shared_ptr<PID> y_controller_;  // Y controller

public:
    Flyappy();

    // Default destructor
    ~Flyappy() = default;

    // State estimate getter
    std::shared_ptr<StateEstimate> get_state_estimate() const { return state_estimate_; }

    // Longitudinal controller getter
    std::shared_ptr<PID> get_longitudinal_controller() const { return longitudinal_controller_; }

    // Compute control law
    std::tuple<float,float> computeControlLaw();

    // Compute y control law
    std::tuple<float,float> computeYControlLaw(float reference, float current_value);

    // Compute x control law
    float computeXControlLaw(float current_x, float t);

    void detectGate(std::vector<point> points);  
};

}  // namespace flyappy
