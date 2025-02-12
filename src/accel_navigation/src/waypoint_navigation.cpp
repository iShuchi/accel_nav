#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

using namespace std::chrono_literals;

class WaypointNavigationNode : public rclcpp::Node
{
public:
    WaypointNavigationNode() : Node("waypoint_navigation_node")
    {
        // Publishers
        initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);

        // Subscriber for odometry
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&WaypointNavigationNode::odomCallback, this, std::placeholders::_1));

        // **New Subscriber for Goal Input**
        goal_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/goal_input", 10, std::bind(&WaypointNavigationNode::goalCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Waiting for goals on /goal_input topic...");
    }

private:
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goal_sub_;  // **New Goal Subscriber**

    // Variables for odometry and goal
    nav_msgs::msg::Odometry current_odom_;
    geometry_msgs::msg::PoseStamped current_goal_;

    // **Callback for receiving goal from topic**
    void goalCallback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received new goal: (%.2f, %.2f)", msg->x, msg->y);

        current_goal_.header.stamp = this->get_clock()->now();
        current_goal_.header.frame_id = "map";
        current_goal_.pose.position.x = msg->x;
        current_goal_.pose.position.y = msg->y;
        current_goal_.pose.position.z = 0.0; // Assume a flat surface

        // Publish the goal
        goal_pub_->publish(current_goal_);
    }

    // **Odometry Callback - Check if the goal is reached**
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_odom_ = *msg;

        // Check if the robot has reached the goal (within a threshold)
        double distance = std::sqrt(std::pow(current_odom_.pose.pose.position.x - current_goal_.pose.position.x, 2) +
                                    std::pow(current_odom_.pose.pose.position.y - current_goal_.pose.position.y, 2));

        if (distance < 0.2) // Threshold for reaching the goal
        {
            RCLCPP_INFO(this->get_logger(), "Goal Reached!");

            // Create and publish the marker in RViz
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = this->get_clock()->now();
            marker.ns = "goal_marker";
            marker.id = 0;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = current_goal_.pose.position.x;
            marker.pose.position.y = current_goal_.pose.position.y;
            marker.pose.position.z = current_goal_.pose.position.z + 0.5; // Slightly above ground
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            marker.lifetime = rclcpp::Duration(1s);

            marker_pub_->publish(marker);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointNavigationNode>());
    rclcpp::shutdown();
    return 0;
}
