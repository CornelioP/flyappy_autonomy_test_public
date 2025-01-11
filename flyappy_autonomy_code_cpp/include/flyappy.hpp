#pragma once
#include "state_estimate.hpp"
#include <memory>

namespace flyappy
{

class Flyappy
{
private:
    std::shared_ptr<StateEstimate> state_estimate_;  // Use shared_ptr instead of unique_ptr

public:
    Flyappy();

    // Default destructor
    ~Flyappy() = default;

    std::shared_ptr<StateEstimate> get_state_estimate() const { return state_estimate_; }
};

}  // namespace flyappy
