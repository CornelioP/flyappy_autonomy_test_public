#include "flyappy.hpp"

namespace flyappy
{


Flyappy::Flyappy()
{
    state_estimate_ = std::make_shared<StateEstimate>(0.33f);
}

}  // namespace flyappy
