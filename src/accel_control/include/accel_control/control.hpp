#ifndef CONTROL_HPP
#define CONTROL_HPP

#include "rclcpp/rclcpp.hpp"
#include "accel_msgs/msg/map.hpp"
#include "accel_msgs/srv/share_map.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

class Control : public rclcpp::Node
{
private:
    rclcpp::Subscription<accel_msgs::msg::Map>::SharedPtr sub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_to_rviz;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_merged_grid;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Service<accel_msgs::srv::ShareMap>::SharedPtr service;

public:
    rclcpp::Client<accel_msgs::srv::ShareMap>::SharedPtr client;
    void timer_callback();
    void service_callback(const accel_msgs::srv::ShareMap::Request::SharedPtr request, const accel_msgs::srv::ShareMap::Response::SharedPtr response);
    void sub_callback(accel_msgs::msg::Map::SharedPtr msg);

    Control() : Node("control_node")
    {
        this->declare_parameter("timer_frequancy", 1);

        this->occupancy_grid_pub_to_rviz = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

        this->sub = create_subscription<accel_msgs::msg::Map>("/laser_scan", 10, std::bind(&Control::sub_callback, this, _1)); // lidar topic ine subscribe olacak.

        this->service = create_service<accel_msgs::srv::ShareMap>("publish_map_service", std::bind(&Control::service_callback, this, _1, _2));
        this->client = create_client<accel_msgs::srv::ShareMap>("publish_map_service");

        auto timer_frequency = this->get_parameter("timer_frequancy").as_int();
        this->timer = create_wall_timer(std::chrono::seconds(timer_frequency), std::bind(&Control::timer_callback, this));
    }
};

void Control::timer_callback()
{
    auto my_map = nav_msgs::msg::OccupancyGrid();
    std_msgs::msg::Header header;
    int height = 20;
    int width = 10;
    header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    RCLCPP_INFO(get_logger(), "Timer has been working");
    header.frame_id = "map_frame";
    my_map.header = header;

    my_map.info.resolution = 0.3;

    my_map.info.origin.position.x = 0; 
    my_map.info.origin.position.y = 0;
    my_map.info.origin.position.z = 0;

    my_map.info.origin.orientation.x = 0; 
    my_map.info.origin.orientation.y = 0;
    my_map.info.origin.orientation.z = 0;
    my_map.info.origin.orientation.w = 1.0;
    my_map.info.width = width;
    my_map.info.height = height;
    std::vector<signed char> data(width * height);
    for (int i = 0; i < width * height; i++)
    {
        if (i < width)
        {
            data[i] = 100;
        }
        if (i % width == 0) // sutunu işaretler
        {
            data[i] = 100;
        }
        if (i % width == 9) // sutunu işaretler
        {
            data[i] = 100;
        }
        if (i > width * (height - 1))
        {
            data[i] = 100;
        }
    }
    my_map.data = data;

    this->occupancy_grid_pub_to_rviz->publish(my_map); 
}

void Control::sub_callback(accel_msgs::msg::Map::SharedPtr msg)
{
    RCLCPP_ERROR(get_logger(), "data: %d", msg->header.stamp.sec);
}
void Control::service_callback(const accel_msgs::srv::ShareMap::Request::SharedPtr request, const accel_msgs::srv::ShareMap::Response::SharedPtr response)
{
    auto merged_map = nav_msgs::msg::OccupancyGrid();
    response->custom_occupany_grid = merged_map;
    response->is_completed = true;
    response->status_message = "fenasal";
}
#endif
