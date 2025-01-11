#pragma once

struct position
{
    float x;  
    float y;
};

struct velocity
{
    float x;
    float y;
};

struct state
{
    position pos;
    velocity vel;
};

// Class for state estimation
class StateEstimate
{
private:
    state state_;  // Stores position and velocity
    const float dt_;  // Fixed time step for updates

public:
    // Constructor
    StateEstimate(const float dt);

    // Default destructor
    ~StateEstimate();

    // Getters
    const velocity& get_velocity() const;
    const position& get_position() const;

    // State update functions
    void update_state(float vx, float vy);  // Update with fixed dt_
    void update_state(float dt, float vx, float vy);  // Update with custom dt
};