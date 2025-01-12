#pragma once

#include <vector>

struct point
{
    float x;
    float y;
};

struct Wall
{
    std::vector<point> wall_points;  // Points corresponding to the current wall
    point gate_center;               // Center point of the detected gate
    float wall_x_position;           // X-coordinate of the wall
    float gate_safe_x;               // x coordinate which is safe to pass the gate
    float gate_end_x;                // x coordinate which is the end of the gate
};

class GateDetector
{
  public:
    GateDetector();
    ~GateDetector() = default;

    // Detect points belonging to walls
    void detectWallPoints(const std::vector<point>& points);

    // Detect the gate
    void detectGate();

    // Wall getter
    Wall get_wall() const { return wall_; }

    // Check if the bird has passed the gate
    bool hasPassedGate(float bird_x);

    // Reset the wall
    void resetWall();

    // Compute RANSAC
    float computeRANSAC(const std::vector<point>& points, int iterations = 100, float threshold = 0.1);

  private:
    Wall wall_;  // Wall object
    const float threshold_;
    bool initialized_;
};
