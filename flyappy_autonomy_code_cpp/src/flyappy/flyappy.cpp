#include "flyappy.hpp"
#include <iostream>

namespace flyappy
{


Flyappy::Flyappy()
{
    state_estimate_ = std::make_shared<StateEstimate>(0.033f);
    gate_detector_ = std::make_shared<GateDetector>();
}

void Flyappy::detectGate(std::vector<point> points)
{   
  
    gate_detector_->detectWallPoints(points);
    gate_detector_->detectGate();

    // If the bird has passed the gate, reset the wall
    if (gate_detector_->hasPassedGate(state_estimate_->get_position().x))
    {
        gate_detector_->resetWall();
    }

    
}


}  // namespace flyappy
