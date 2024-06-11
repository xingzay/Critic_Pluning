#ifndef ROS2_H
#define ROS2_H
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/empty.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "widget.h"
using std::placeholders::_1;
class ROS2 : public rclcpp::Node,public Widget
{
public:
    ROS2();
    ~ROS2();
private:
    std::thread spin_thread; //多线程对象
    std::mutex mutex_; //互斥锁
    // 速度数据
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
    void vel_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel);

    // 里程计数据
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    double total_distance_;  // 累计距离
    nav_msgs::msg::Odometry last_odometry_;  // 上一次的里程计数据
    bool first_msg_;  // 标记是否是第一次接收
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom);

    // 判断turtlebot3_node节点是否启动
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr heartbeat_sub_;
    void heartbeat_callback(const std_msgs::msg::Empty&);
    QTimer* timer_;
    rclcpp::Time last_heartbeat_;
    void check_heartbeat();

    // 判断激光雷达是否启动
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    void check_scan_topic(const sensor_msgs::msg::LaserScan & scan);
    Point findMax(const sensor_msgs::msg::LaserScan& scan);
    void updateGUI();
};




#endif // ROS2_H
