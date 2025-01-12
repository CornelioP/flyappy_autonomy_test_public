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


}  // namespace flyappy
