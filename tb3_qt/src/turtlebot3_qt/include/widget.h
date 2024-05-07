#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QLabel>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <mutex>
#include <QTimer>
#include <QDateTime>

namespace Ui {
class Widget;
}

class Widget : public QWidget
{
    Q_OBJECT

public:
    explicit Widget(QWidget *parent = nullptr);
    ~Widget();

private:
    Ui::Widget *ui;
    rclcpp::Node::SharedPtr node;
    std::thread spin_thread; //多线程对象
    std::mutex mutex; //互斥锁
    // 速度数据
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
    void vel_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel);

    // 里程计数据
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    double total_distance_;  // 累计距离
    nav_msgs::msg::Odometry last_odometry_;  // 上一次的里程计数据
    bool first_msg_;  // 标记是否是第一次接收
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom);


    void update_time();
};

#endif // WIDGET_H
