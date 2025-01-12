#include <gtest/gtest.h>

#include "flyappy.hpp"
#include "state_estimate.hpp"

// TEST STATE ESTIMATE

TEST(StateEstimate, Constructor)
{
    StateEstimate state_estimate(0.033f);
    const auto pos = state_estimate.get_position();
    const auto vel = state_estimate.get_velocity();
    EXPECT_FLOAT_EQ(pos.x, 0.0f);
    EXPECT_FLOAT_EQ(pos.y, 0.0f);
    EXPECT_FLOAT_EQ(vel.x, 0.0f);
    EXPECT_FLOAT_EQ(vel.y, 0.0f);
}

TEST(StateEstimate, UpdateStateFixeddt)
{
    StateEstimate state_estimate(0.033f);
    state_estimate.update_state(1.0f, 2.0f);
    const auto pos = state_estimate.get_position();
    const auto vel = state_estimate.get_velocity();
    EXPECT_FLOAT_EQ(pos.x, 1.0f * 0.033f);
    EXPECT_FLOAT_EQ(pos.y, 2.0f * 0.033f);
    ;
    EXPECT_FLOAT_EQ(vel.x, 1.0f);
    EXPECT_FLOAT_EQ(vel.y, 2.0f);
}

TEST(StateEstimate, UpdateStateVariabledt)
{
    StateEstimate state_estimate(0.0f);
    const float dt = 1.0f;
    state_estimate.update_state(dt, 1.0f, 2.0f);
    const auto pos = state_estimate.get_position();
    const auto vel = state_estimate.get_velocity();
    EXPECT_FLOAT_EQ(pos.x, 1.0f);
    EXPECT_FLOAT_EQ(pos.y, 2.0f);
    EXPECT_FLOAT_EQ(vel.x, 1.0f);
    EXPECT_FLOAT_EQ(vel.y, 2.0f);
}

// TEST GATE DETECTOR

TEST(GateDetector, Constructor)
{
    GateDetector gate_detector;
    const auto wall = gate_detector.get_wall();
    EXPECT_EQ(wall.wall_points.size(), 0);
    EXPECT_FLOAT_EQ(wall.wall_x_position, 0.0f);
    EXPECT_FLOAT_EQ(wall.gate_center.x, 0.0f);
    EXPECT_FLOAT_EQ(wall.gate_center.y, 0.0f);
}

TEST(GateDecttor, DetectWallPoints)
{
    GateDetector gate_detector;
    std::vector<point> points = {{5.058608, -1.470841}, {5.656852, -1.124972},
                                 {5.770874, -0.449608}, {5.680297, 0.205542},
                                 {5.601804, 1.331166},  {5.634413, 1.945406},
                                 {5.519407, 2.585456},  {4.619607, 2.584162}};

    gate_detector.detectWallPoints(points);
    const auto wall = gate_detector.get_wall();

    EXPECT_NEAR(wall.wall_x_position, 5.4427f, 0.0001f);  // Allow a small difference
    EXPECT_EQ(wall.wall_points.size(), 5);  // Number of points within threshold
    EXPECT_NEAR(wall.wall_points[0].x, 5.656852f, 0.001f);  // First point in list
    EXPECT_NEAR(wall.wall_points[4].x, 5.519407f, 0.001f);  // Last point in list
}

TEST(GateDetector, AddPointsToWall)
{
    GateDetector gate_detector;
    std::vector<point> points = {{5.058608, -1.470841}, {5.656852, -1.124972},
                                 {5.770874, -0.449608}, {5.680297, 0.205542},
                                 {5.601804, 1.331166},  {5.634413, 1.945406},
                                 {5.519407, 2.585456},  {4.619607, 2.584162}};

    gate_detector.detectWallPoints(points);

    // Now add more points to the wall 2 outliers and 3 inliers  then check the size of
    // the wall
    std::vector<point> new_points = {
            {5.0, 2.0}, {5.1, 2.1}, {5.2, 2.2}, {5.3, 2.3}, {5.4, 2.4}};

    gate_detector.detectWallPoints(new_points);

    const auto wall = gate_detector.get_wall();

    EXPECT_EQ(wall.wall_points.size(), 8);  // Number of points within threshold

}

TEST(GateDetector, HasPassedWall)
{
    GateDetector gate_detector;
    std::vector<point> points = {{5.058608, -1.470841}, {5.656852, -1.124972},
                                 {5.770874, -0.449608}, {5.680297, 0.205542},
                                 {5.601804, 1.331166},  {5.634413, 1.945406},
                                 {5.519407, 2.585456},  {4.619607, 2.584162}};

    gate_detector.detectWallPoints(points);

    // Check if the bird has passed the wall
    EXPECT_FALSE(gate_detector.hasPassedGate(5.0f));
    EXPECT_TRUE(gate_detector.hasPassedGate(5.5f));
}

TEST(GateDetector, HasResetWall)
{
    GateDetector gate_detector;
    std::vector<point> points = {{5.058608, -1.470841}, {5.656852, -1.124972},
                                 {5.770874, -0.449608}, {5.680297, 0.205542},
                                 {5.601804, 1.331166},  {5.634413, 1.945406},
                                 {5.519407, 2.585456},  {4.619607, 2.584162}};

    gate_detector.detectWallPoints(points);

    // Check if the bird has passed the wall
    EXPECT_FALSE(gate_detector.hasPassedGate(5.0f));
    EXPECT_TRUE(gate_detector.hasPassedGate(5.5f));

    // Reset the wall
    gate_detector.resetWall();
    const auto wall = gate_detector.get_wall();
    EXPECT_EQ(wall.wall_points.size(), 0);
    EXPECT_FLOAT_EQ(wall.wall_x_position, 0.0f);
    EXPECT_FLOAT_EQ(wall.gate_center.x, 0.0f);
    EXPECT_FLOAT_EQ(wall.gate_center.y, 0.0f);
}


