#include "widget.h"
#include "./ui_widget.h"
#include <QMetaObject>
Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget),
    mutex_(),
    last_heartbeat_(rclcpp::Clock().now())
{
    ui->setupUi(this);

    // 将check_tb3_node_label设置为圆形,默认颜色为红色
    ui->check_tb3_node_label->setFixedSize(21, 21);
    ui->check_tb3_node_label->setStyleSheet("border-radius: 50px; background-color: red;");

    int argc = 0; char **argv = NULL;
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("ROS2_Node");

    vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        10,
        std::bind(&Widget::vel_callback,this,std::placeholders::_1));
    // 路程 -- 里程计信息
    odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
        // "/odometry/filtered",
        "/odom",
        10,
        std::bind(&Widget::odom_callback,this,std::placeholders::_1)
        );

    // 机器人状态 -- 判断turtlebot3_node节点是否启动
    heartbeat_sub_ = node->create_subscription<std_msgs::msg::Empty>(
        "/heart_beat",
        10,
        std::bind(&Widget::heartbeat_callback,this,std::placeholders::_1)
    );

    // 创建定时器，用于检查心跳信号是否超时
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &Widget::check_heartbeat);
    timer_->start(2000);  // 每2秒检查一次  时间短计算量大



    //多线程运行spin
    spin_thread = std::thread([this]() {
        rclcpp::spin(node);
    });

    // timer
    QTimer *timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &Widget::update_time);
    timer->start(1000);
}

Widget::~Widget()
{
    // 确保正确关闭线程
    if (spin_thread.joinable()) {
        spin_thread.join();
    }

    if (timer_) {
        timer_->stop();  // 停止定时器
    }

    delete ui;
}

void Widget::vel_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel){
    std::unique_lock<std::mutex> lock(mutex_); // 确保互斥
    // 构建显示文本
    QString linear_text = QString::number(cmd_vel->linear.x,'f',3) + " m/s";
    QString angular_text = QString::number(cmd_vel->angular.z,'f',3) + " rad/s";
    // 确保在 Qt 主线程中更新 QLabel
    ui->linear_label->setText(linear_text);
    QMetaObject::invokeMethod(ui->linear_label, "setText", Qt::AutoConnection, Q_ARG(QString, linear_text));
    ui->angular_label->setText(angular_text);
    QMetaObject::invokeMethod(ui->angular_label, "setText", Qt::AutoConnection, Q_ARG(QString, angular_text));
    return;
}

void Widget::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom){
    std::unique_lock<std::mutex> lock(mutex_); // 确保互斥
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
    return;
}

void Widget::heartbeat_callback(const std_msgs::msg::Empty &) {
    std::unique_lock<std::mutex> lock(mutex_); // 确保互斥
    last_heartbeat_ = rclcpp::Clock().now();  // 更新心跳时间
    // 收到心跳信号后，将颜色更改为绿色
    QMetaObject::invokeMethod(
        this,
        [this]() {
            ui->check_tb3_node_label->setStyleSheet("background-color: green;");
        },
        Qt::AutoConnection
        );
}

void Widget::check_heartbeat() {
    std::lock_guard<std::mutex> lock(mutex_);  // 确保线程安全
    auto time_diff = (rclcpp::Clock().now() - last_heartbeat_).seconds();

    if (time_diff > 3) {  // 如果超过3秒没有收到心跳信号
        // 将颜色设为红色
        QMetaObject::invokeMethod(
            this,
            [this]() {
                ui->check_tb3_node_label->setStyleSheet("background-color: red;");
            },
            Qt::AutoConnection
            );
    }
}





void Widget::update_time(){
    QDateTime currentTime = QDateTime::currentDateTime();
    QString timeString = currentTime.toString("yyyy-MM-dd hh:mm:ss");
    ui->update_time->setText(timeString);
}
