#include "widget.h"
#include "./ui_widget.h"
#include <QMetaObject>
Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
{
    ui->setupUi(this);
    int argc = 0; char **argv = NULL;
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("turtlebot3_node");
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
    delete ui;
}

void Widget::vel_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel){
    std::unique_lock<std::mutex> lock(mutex); // 确保互斥
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
    std::unique_lock<std::mutex> lock(mutex); // 确保互斥
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

void Widget::update_time(){
    QDateTime currentTime = QDateTime::currentDateTime();
    QString timeString = currentTime.toString("yyyy-MM-dd hh:mm:ss");
    ui->update_time->setText(timeString);
}
