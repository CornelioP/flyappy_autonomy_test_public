#include "gate_detector.hpp"
#include <algorithm>
#include <iostream>
#include <random>

GateDetector::GateDetector() : threshold_(0.3f), initialized_(false)
{
    wall_.wall_x_position = 0.0f;
    wall_.gate_center.x = 0.0f;
    wall_.gate_center.y = 0.0f;
}

void GateDetector::detectWallPoints(const std::vector<point>& points)
{
    if (points.empty()) return;  // Avoid division by zero

    // Compute the average x of incoming points if wall is not initialized
    if (!initialized_)
    {
        // Use RANSAC for robustness over outliers 
        const float ransac_x = computeRANSAC(points);
        wall_.wall_x_position = ransac_x;
        initialized_ = true;  // Mark that wall x-position has been set
    }

    // Accumulate points near the x-position of the wall, avoiding duplicates
    // TODO: Make more efficient by using a hash map

    for (const auto& p : points)
    {
        if (std::abs(p.x - wall_.wall_x_position) < threshold_)
        {
            // Check if the point is already in the wall points
            auto it = std::find_if(wall_.wall_points.begin(), wall_.wall_points.end(),
                                   [&p](const point& existing)
                                   {
                                       return std::abs(existing.x - p.x) < 1e-3f &&
                                              std::abs(existing.y - p.y) < 1e-3f;
                                   });

            if (it == wall_.wall_points.end())
            {
                wall_.wall_points.push_back(p);  // Only add if not a duplicate
            }
        }
    }
}

void GateDetector::detectGate()
{
    if (wall_.wall_points.size() < 5)
    {
        std::cerr << "Not enough points to detect a gate!" << std::endl;
        return;
    }

    // Step 1: Sort points based on y-coordinate
    std::sort(wall_.wall_points.begin(), wall_.wall_points.end(),
              [](const point& a, const point& b) { return a.y < b.y; });

    // Step 2: Find the largest gap between consecutive points
    float max_gap = 0.0f;
    size_t gap_start_index = 0;
    for (size_t i = 0; i < wall_.wall_points.size() - 1; ++i)
    {
        float gap = wall_.wall_points[i + 1].y - wall_.wall_points[i].y;
        if (gap > max_gap && gap < 3.0f)
        {
            max_gap = gap;
            gap_start_index = i;
        }
    }

    // Step 3: Compute the new gate center
    const point& lower_point = wall_.wall_points[gap_start_index];
    const point& upper_point = wall_.wall_points[gap_start_index + 1];
    float new_gate_x = (lower_point.x + upper_point.x) / 2.0f;  // Near the wall x-position
    float new_gate_y = (lower_point.y + upper_point.y) / 2.0f;

    // Step 4: Check if the new gate center is significantly different from the previous center
    float tolerance = 0.1f;  // Tolerance for significant changes in gate position (adjust as needed)
    float diff_x = std::abs(new_gate_x - wall_.gate_center.x);
    float diff_y = std::abs(new_gate_y - wall_.gate_center.y);

    if (diff_x < tolerance && diff_y < tolerance)
    {
        // The new gate is almost the same as the previous gate; ignore the update
        std::cerr << "Gate detected, but coordinates are within tolerance. Keeping previous gate center." << std::endl;
        return;
    }

    // Update the gate center if the difference is significant
    wall_.gate_center.x = new_gate_x;
    wall_.gate_center.y = new_gate_y;

    // Step 5: Assign the safe x-coordinate to pass the gate and the end of the gate
    wall_.gate_safe_x = wall_.gate_center.x - 0.25f;
    wall_.gate_end_x = wall_.gate_center.x + 0.25f;
}

float GateDetector::computeRANSAC(const std::vector<point>& points, int iterations, float threshold)
{
    if (points.size() < 2)
        return 0.0f;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, points.size() - 1);

    float best_x = 0.0f;
    int max_inliers = 0;

    for (int i = 0; i < iterations; ++i)
    {
        // Randomly select two points to fit a line
        const auto& p1 = points[dis(gen)];
        const auto& p2 = points[dis(gen)];
        float avg_x = (p1.x + p2.x) / 2.0f;

        int inliers = 0;
        for (const auto& p : points)
        {
            if (std::abs(p.x - avg_x) < threshold)
            {
                ++inliers;
            }
        }

        if (inliers > max_inliers)
        {
            max_inliers = inliers;
            best_x = avg_x;
        }
    }

    return best_x;
}



bool GateDetector::hasPassedGate(float bird_x)
{
    // Check if the bird has passed the x-position of the wall
    return bird_x > wall_.wall_x_position;
}

void GateDetector::resetWall()
{
    // Reset all accumulated points after passing the gate
    wall_.wall_points.clear();
    wall_.wall_x_position = 0.0f;
    initialized_ = false;
}
