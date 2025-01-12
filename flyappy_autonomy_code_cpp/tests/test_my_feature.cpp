#include <gtest/gtest.h>

#include "flyappy.hpp"
#include "state_estimate.hpp"


// TEST STATE ESTIMATE


TEST(StateEstimate,Constructor)
{
    StateEstimate state_estimate(0.033f);
    const auto pos = state_estimate.get_position();
    const auto vel = state_estimate.get_velocity();
    EXPECT_FLOAT_EQ(pos.x, 0.0f);
    EXPECT_FLOAT_EQ(pos.y, 0.0f);
    EXPECT_FLOAT_EQ(vel.x, 0.0f);
    EXPECT_FLOAT_EQ(vel.y, 0.0f);
}


TEST(StateEstimate,UpdateStateFixeddt)
{
    StateEstimate state_estimate(0.033f);
    state_estimate.update_state(1.0f, 2.0f);
    const auto pos = state_estimate.get_position();
    const auto vel = state_estimate.get_velocity();
    EXPECT_FLOAT_EQ(pos.x, 1.0f * 0.033f);
    EXPECT_FLOAT_EQ(pos.y, 2.0f * 0.033f); ;
    EXPECT_FLOAT_EQ(vel.x, 1.0f);
    EXPECT_FLOAT_EQ(vel.y, 2.0f);
}

TEST(StateEstimate,UpdateStateVariabledt)
{
    StateEstimate state_estimate(0.0f);
    const float dt = 1.0f;
    state_estimate.update_state(dt,1.0f, 2.0f);
    const auto pos = state_estimate.get_position();
    const auto vel = state_estimate.get_velocity();
    EXPECT_FLOAT_EQ(pos.x, 1.0f);
    EXPECT_FLOAT_EQ(pos.y, 2.0f); 
    EXPECT_FLOAT_EQ(vel.x, 1.0f);
    EXPECT_FLOAT_EQ(vel.y, 2.0f);
}
