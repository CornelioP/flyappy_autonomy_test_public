#pragma once
#include "state_estimate.hpp"
#include "gate_detector.hpp"
#include <memory>

namespace flyappy
{

class Flyappy
{
private:
    std::shared_ptr<StateEstimate> state_estimate_;  // Use shared_ptr instead of unique_ptr
    std::shared_ptr<GateDetector> gate_detector_;  // Add gate detector

public:
    Flyappy();

    // Default destructor
    ~Flyappy() = default;

    std::shared_ptr<StateEstimate> get_state_estimate() const { return state_estimate_; }

    void detectGate(std::vector<point> points);  
};

}  // namespace flyappy
