#include "state_estimate.hpp"

// Constructor
StateEstimate::StateEstimate(const float dt) : dt_(dt)
{
    state_.pos = {0.0f, 0.0f};
    state_.vel = {0.0f, 0.0f};
}

// Destructor 
StateEstimate::~StateEstimate() = default;

// Get current velocity
const velocity& StateEstimate::get_velocity() const
{
    return state_.vel;
}

// Get current position
const position& StateEstimate::get_position() const
{
    return state_.pos;
}

// Update state with fixed dt_
void StateEstimate::update_state(float vx, float vy)
{
    state_.pos.x += vx * dt_;
    state_.pos.y += vy * dt_;
    state_.vel.x = vx;
    state_.vel.y = vy;
}

// Update state with custom dt
void StateEstimate::update_state(float dt, float vx, float vy)
{
    state_.pos.x += vx * dt;
    state_.pos.y += vy * dt;
    state_.vel.x = vx;
    state_.vel.y = vy;
}
