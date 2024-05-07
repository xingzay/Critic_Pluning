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
        1,
        std::bind(&Widget::vel_callback,this,std::placeholders::_1));

    //多线程运行spin
    spin_thread = std::thread([this]() {
        rclcpp::spin(node);
    });
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
    QString angular_text = QString::number(cmd_vel->angular.z,'f',3) + " rad/s"; // 确保小数点后三位
    // 确保在 Qt 主线程中更新 QLabel
    ui->linear_label->setText(linear_text);
    QMetaObject::invokeMethod(ui->linear_label, "setText", Qt::AutoConnection, Q_ARG(QString, linear_text));
    ui->angular_label->setText(angular_text);
    QMetaObject::invokeMethod(ui->angular_label, "setText", Qt::AutoConnection, Q_ARG(QString, angular_text));
    return ;
}


