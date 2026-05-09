#include "basic_topic/publisher_component.hpp"
#include <random>

using namespace std::chrono_literals;

namespace basic_topic
{

    PublisherComponent::PublisherComponent(const rclcpp::NodeOptions& options) :
        Node("publisher_node", options)
    {
        publishers_ = this->create_publisher<geometry_msgs::msg::Quaternion>("quaternion_data", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&PublisherComponent::publishData, this));
        RCLCPP_INFO(this->get_logger(), "PublisherComponent has been initialized.");
    }

    PublisherComponent::~PublisherComponent()
    {
        RCLCPP_INFO(this->get_logger(), "PublisherComponent is being destroyed.");
    }

    double PublisherComponent::normalize_angle(double angle)
    {
        angle = std::fmod(angle + kPi, 2.0 * kPi);
        if (angle < 0.0)
            angle += 2.0 * kPi;
        return angle - kPi;
    }

    geometry_msgs::msg::Quaternion PublisherComponent::rpy_to_quaternion(double roll, double pitch, double yaw)
    {
        geometry_msgs::msg::Quaternion q;
        const double cy = std::cos(yaw * 0.5);
        const double sy = std::sin(yaw * 0.5);
        const double cp = std::cos(pitch * 0.5);
        const double sp = std::sin(pitch * 0.5);
        const double cr = std::cos(roll * 0.5);
        const double sr = std::sin(roll * 0.5);

        q.w = cr * cp * cy + sr * sp * sy;
        q.x = sr * cp * cy - cr * sp * sy;
        q.y = cr * sp * cy + sr * cp * sy;
        q.z = cr * cp * sy - sr * sp * cy;
        return q;
    }

    geometry_msgs::msg::Quaternion PublisherComponent::generateData()
    {
        double roll = normalize_angle((std::rand() / static_cast<double>(RAND_MAX)) * 2.0 * kPi - kPi);
        double pitch = normalize_angle((std::rand() / static_cast<double>(RAND_MAX)) * 2.0 * kPi - kPi);
        double yaw = normalize_angle((std::rand() / static_cast<double>(RAND_MAX)) * 2.0 * kPi - kPi);
        geometry_msgs::msg::Quaternion q = rpy_to_quaternion(roll, pitch, yaw);
        RCLCPP_INFO(this->get_logger(), "Generated Euler angles: [roll: %.3f, pitch: %.3f, yaw: %.3f]", roll, pitch, yaw);
        return q;
    }

    void PublisherComponent::publishData()
    {
        auto message = generateData();
        publishers_->publish(message);
    }

}  // namespace basic_topic

RCLCPP_COMPONENTS_REGISTER_NODE(basic_topic::PublisherComponent)
