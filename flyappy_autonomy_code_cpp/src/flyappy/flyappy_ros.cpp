#include "flyappy_ros.hpp"

namespace flyappy
{

inline constexpr uint32_t QUEUE_SIZE = 5u;

FlyappyRos::FlyappyRos(rclcpp::Node::SharedPtr node) : node_(node)
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

void FlyappyRos::velocityCallback([[maybe_unused]] const geometry_msgs::msg::Vector3& msg)
{
    // Update the state estimate with the received velocity
    flyappy_->get_state_estimate()->update_state(msg.x, msg.y);

    // Print the state estimate
    const auto pos = flyappy_->get_state_estimate()->get_position();
    const auto vel = flyappy_->get_state_estimate()->get_velocity();
    RCLCPP_INFO(node_->get_logger(), "Position: (%f, %f), Velocity: (%f, %f)",
                pos.x, pos.y, vel.x, vel.y);

}

void FlyappyRos::laserScanCallback(const sensor_msgs::msg::LaserScan& msg)
{
    // Example of printing laser angle and range
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "Laser range: %f, angle: %f", msg.ranges[0], msg.angle_min);
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

}  // namespace flyappy
