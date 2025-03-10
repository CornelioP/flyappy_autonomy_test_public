#include "flyappy_ros.hpp"

namespace flyappy
{

inline constexpr uint32_t QUEUE_SIZE = 5u;

FlyappyRos::FlyappyRos(rclcpp::Node::SharedPtr node) : node_(node), first_time_(true)
{
    pub_acceleration_command_ = node_->create_publisher<geometry_msgs::msg::Vector3>(
            "/flyappy_acc", QUEUE_SIZE);
    sub_velocity_ = node_->create_subscription<geometry_msgs::msg::Vector3>(
            "/flyappy_vel", QUEUE_SIZE,
            std::bind(&FlyappyRos::velocityCallback, this, std::placeholders::_1));
    sub_laser_scan_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
            "/flyappy_laser_scan", QUEUE_SIZE,
            std::bind(&FlyappyRos::laserScanCallback, this, std::placeholders::_1));
    sub_game_ended_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/flyappy_game_ended", QUEUE_SIZE,
            std::bind(&FlyappyRos::gameEndedCallback, this, std::placeholders::_1));

    flyappy_ = std::make_unique<Flyappy>();
}

void FlyappyRos::velocityCallback(const geometry_msgs::msg::Vector3& msg)
{
    // Update the state estimate with the received velocity
    flyappy_->get_state_estimate()->update_state(msg.x, msg.y);
}

void FlyappyRos::laserScanCallback(const sensor_msgs::msg::LaserScan& msg)
{
    const auto pos = flyappy_->get_state_estimate()->get_position();
    const auto points = convertLaserScanToPoints(msg, pos.x, pos.y);

    if (first_time_ && points.size() > 7)
    {
        first_time_ = false;
        flyappy_->detectGate(points);
    }
    else if (!first_time_ && points.size() > 5)
    {
        flyappy_->detectGate(points);
    }

    // Structured binding for control law
    float acc_x = 0.0f, acc_y = 0.0f;
    if (first_time_)
    {
        float ref_vel = 2.0f;
        float fly_vel = flyappy_->get_state_estimate()->get_velocity().x;
        acc_x = flyappy_->get_longitudinal_controller()->compute_control_law(ref_vel,
                                                                             fly_vel);
    }
    else
    {
        auto [computed_acc_x, computed_acc_y] = flyappy_->computeControlLaw();
        acc_x = computed_acc_x;
        acc_y = computed_acc_y;
    }

    geometry_msgs::msg::Vector3 acc_msg;
    acc_msg.x = acc_x;
    acc_msg.y = acc_y;
    acc_msg.z = 0.0f;  // Assuming no z-axis control

    pub_acceleration_command_->publish(acc_msg);
}

void FlyappyRos::gameEndedCallback(const std_msgs::msg::Bool& msg)
{
    if (msg.data)
    {
        RCLCPP_INFO(node_->get_logger(), "Crash detected.");
    }
    else
    {
        RCLCPP_INFO(node_->get_logger(), "End of countdown.");
    }

    flyappy_.reset(new Flyappy{});
}

std::vector<point> FlyappyRos::convertLaserScanToPoints(
        const sensor_msgs::msg::LaserScan& msg, float curr_x, float curr_y)
{
    std::vector<point> points;
    std::size_t size = msg.ranges.size();

    // Convert the laser scan data to points
    for (std::size_t i = 0; i < size; ++i)
    {
        // Avoid accessing intensities if the array is empty or not meaningful
        if (msg.intensities[i] == 1)
        {
            float range = msg.ranges[i];

            // Skip invalid ranges (e.g., NaN, out-of-range)
            if (std::isnan(range) || range < msg.range_min || range > msg.range_max)
            {
                continue;
            }

            point p;
            float angle = msg.angle_min + i * msg.angle_increment;
            p.x = curr_x + range * cos(angle);  // Global x-coordinate
            p.y = curr_y + range * sin(angle);  // Global y-coordinate
            points.push_back(p);
        }
    }

    return points;
}
}  // namespace flyappy
