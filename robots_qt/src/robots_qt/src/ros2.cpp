#include "ros2.h"
#include "ui_widget.h"
ROS2::ROS2()
    : rclcpp::Node("ros2_node"), Widget(nullptr),last_heartbeat_(rclcpp::Clock().now())
{
    vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        10,
        std::bind(&ROS2::vel_callback,this,_1));
    // 路程 -- 里程计信息
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        // "/odometry/filtered",
        "/odom",
        10,
        std::bind(&ROS2::odom_callback,this,_1)
        );

    // 机器人状态 -- 判断turtlebot3_node节点是否启动
    heartbeat_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "/heart_beat",
        10,
        std::bind(&ROS2::heartbeat_callback,this,_1)
        );
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &ROS2::check_heartbeat);
    // 激光雷达 -- 判断是否订阅到/scan话题
    laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        10,
        std::bind(&ROS2::check_scan_topic,this,_1)
        );
}

ROS2::~ROS2()
{
    // 确保正确关闭线程
    if (spin_thread.joinable()) {
        spin_thread.join();
    }
    if (timer_) {
        timer_->stop();  // 停止定时器
    }
}
void ROS2::vel_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel){

    std::lock_guard<std::mutex> lock(mutex_); // 确保互斥
    // 构建显示文本
    QString linear_text = QString::number(cmd_vel->linear.x,'f',3) + " m/s";
    QString angular_text = QString::number(cmd_vel->angular.z,'f',3) + " rad/s";
    // 确保在 Qt 主线程中更新 QLabel

    ui->linear_label->setText(linear_text);
    QMetaObject::invokeMethod(ui->linear_label, "setText", Qt::AutoConnection, Q_ARG(QString, linear_text));
    ui->angular_label->setText(angular_text);
    QMetaObject::invokeMethod(ui->angular_label, "setText", Qt::AutoConnection, Q_ARG(QString, angular_text));
}

// 显示总路程以及当前位置
void ROS2::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom){
    std::lock_guard<std::mutex> lock(mutex_); // 确保互斥
    if(first_msg_){
        last_odometry_ = *odom;  // 如果是第一次接收，只存储数据
        first_msg_ = false;
        return;
    }else{
        // 计算相邻两次里程计之间的距离
        double dx = odom->pose.pose.position.x - last_odometry_.pose.pose.position.x;
        double dy = odom->pose.pose.position.y - last_odometry_.pose.pose.position.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        total_distance_ += distance;  // 累加总距离
        last_odometry_ = *odom;  // 更新上一次的里程计数据
    }
    // 利用里程计数据，计算小车走过的总路程
    QString route_text = QString::number(total_distance_,'f',3) + "  m";
    ui->route_label->setText(route_text);
    QMetaObject::invokeMethod(ui->route_label, "setText", Qt::AutoConnection, Q_ARG(QString, route_text));

    QString position_text =
        QString("(%1, %2)").arg(QString::number(odom->pose.pose.position.x, 'f', 2)).
        arg(QString::number(odom->pose.pose.position.y, 'f', 2));
    ui->robot_position_label->setText(position_text);

}

// 检测turtlebot3_node
void ROS2::heartbeat_callback(const std_msgs::msg::Empty& ) {
    std::lock_guard<std::mutex> lock(mutex_); // 确保互斥
    last_heartbeat_ = rclcpp::Clock().now();  // 更新心跳时间
    // 收到心跳信号后，将颜色更改为绿色
    QMetaObject::invokeMethod(
        this,
        [this]() {
            ui->check_tb3_node_label->setStyleSheet("background-color: green;");
            ui->check_robot_label->setStyleSheet("background-color: green;");
        },
        Qt::AutoConnection
        );
}

void ROS2::check_heartbeat() {
    std::lock_guard<std::mutex> lock(mutex_);  // 确保线程安全
    auto time_diff = (rclcpp::Clock().now() - last_heartbeat_).seconds();

    if (time_diff > 3) {  // 如果超过3秒没有收到心跳信号
        // 将颜色设为红色
        QMetaObject::invokeMethod(
            this,
            [this]() {
                ui->check_tb3_node_label->setStyleSheet("background-color: red;");
                ui->check_robot_label->setStyleSheet("background-color: red;");
                ui->check_scan_node_label->setStyleSheet("background-color: red;");
            },
            Qt::AutoConnection
            );
    }
}

void ROS2::check_scan_topic(const sensor_msgs::msg::LaserScan & scan){
    std::lock_guard<std::mutex> lock(mutex_);  // 确保线程安全
    // 显示绿色 发布激光强度（频率要慢）
    ui->check_scan_node_label->setStyleSheet("background-color: green;");
    for (int i = 0; i < 15; i++){
        auto max = findMax(scan);
        emit dataReady(i, max.range, max.intensity, max.x, max.y);  // 发射信号
        // std::cout << "激光最大数据:"
        //           << " 距离:" << max.range << " 强度:" << max.intensity
        //           << "坐标: (" << max.x << ", " << max.y << ")" << std::endl;
    }
    return ;
}

void Widget::updateLaserTable(int index, float range, float intensity, float x, float y) {
    // setItem(行row，列column，内容)
    ui->laser_tableWidget->setItem(index, 0, new QTableWidgetItem(QString::number(index)));
    ui->laser_tableWidget->setItem(index, 1, new QTableWidgetItem(QString::number(range)));
    ui->laser_tableWidget->setItem(index, 2, new QTableWidgetItem(QString::number(intensity)));
    ui->laser_tableWidget->setItem(index, 3, new QTableWidgetItem(QString("(%1, %2)").arg(x).arg(y)));
}

Point ROS2::findMax(const sensor_msgs::msg::LaserScan& scan)
{
    Point max{0, 0.0, 0.0, 0.0, 0.0};
    for (std::vector<float>::size_type i = 0; i < scan.ranges.size(); i++)
    {
        double intensity = scan.intensities[i];
        if (intensity > max.intensity)
        {
            max.index = i;
            max.range = scan.ranges[i];
            max.intensity = intensity;
            // 角度转弧度
            double angle = scan.angle_min + scan.angle_increment * max.index;
            max.x = max.range * std::cos(angle);
            max.y = max.range * std::sin(angle);
        }
    }
    return max;
}
