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

    // // 连接信号和私有槽
    LOG_IN *login = new LOG_IN(this); // Widget是LOG_IN的父组件。当Widget销毁时，Qt会自动销毁所有子组件，不用delete
    login->hide();  // 隐藏登录界面，直到需要显示为止

    // 连接信号到Widget中的槽函数
    connect(login, &LOG_IN::loginSuccess, this, &Widget::switchTointerfaces);
    connect(login, &LOG_IN::showLogin, this, &Widget::show);

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
    rclcpp::shutdown();
    delete ui;
}

void Widget::updateGUI(){
    return;
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

//主界面
void Widget::on_MainWindow_clicked()
{
    ui->stackedWidget->setCurrentIndex(0);
}
//定时任务
void Widget::on_Time_Work_clicked()
{
    ui->stackedWidget->setCurrentIndex(1);
}
//调试界面
void Widget::on_Test_Window_clicked()
{
    ui->stackedWidget->setCurrentIndex(2);
}
//报警信息界面
void Widget::on_Error_clicked()
{
    int initialRowCount = 15;
    ui->tableWidget->setRowCount(initialRowCount);

    for (int i = 0; i < initialRowCount; ++i) {
        for (int j = 0; j < ui->tableWidget->columnCount(); ++j) {
            QTableWidgetItem *item = new QTableWidgetItem("Data");
            ui->tableWidget->setItem(i, j, item);
        }
    }
    ui->stackedWidget->setCurrentIndex(3);
}

void Widget::on_Clear_clicked()
{
    int rowCount = ui->tableWidget->rowCount();
    // 清除表格中除第一行以外的所有数据
    for (int i = 0; i < rowCount; ++i) {
        for (int j = 0; j < ui->tableWidget->columnCount(); ++j) {
            QTableWidgetItem *item = ui->tableWidget->item(i, j);
            if (item) {
                delete item;
            }
        }
    }
}

void Widget::on_Clean_clicked()
{
    // 获取选中的行号
    int currentRow = ui->tableWidget->currentRow();

    // 如果没有选中行，直接返回
    if (currentRow < 0)
        return;
    //获取行数和列数
    int columnCount = ui->tableWidget->columnCount();
    int rowCount = ui->tableWidget->rowCount();
    // 删除选中行的数据
    for (int i = 0; i < columnCount; ++i) {
        QTableWidgetItem *item = ui->tableWidget->item(currentRow, i);
        if (item) {
            delete item;
        }
    }
    // 将后续行的数据向上移动填补空白行
    for (int i = currentRow; i < rowCount - 1; ++i) {
        for (int j = 0; j < columnCount; ++j) {
            //QTableWidgetItem *currentItem = ui->tableWidget->item(i, j);
            QTableWidgetItem *nextItem = ui->tableWidget->item(i + 1, j);
            if (nextItem) {
                // 将下一行的数据复制到当前行
                ui->tableWidget->setItem(i, j, new QTableWidgetItem(nextItem->text()));
                delete nextItem; // 删除下一行的数据
            } else {
                // 如果下一行为空，清空当前行
                ui->tableWidget->setItem(i, j, new QTableWidgetItem(""));
            }
        }
    }
}

//状态监测
void Widget::on_Condition_clicked()
{
    ui->stackedWidget->setCurrentIndex(4);
}
//用户参数登录界面
void Widget::on_user_param_clicked()
{
    this->close();
    log_in->switchToPage(1);
    log_in->show();
}
//系统参数登录界面
void Widget::on_system_param_clicked()
{
    //第二个登录界面
    this->close();
    log_in->switchToPage(2);
    log_in->show();
}
void Widget::switchTointerfaces(int num){
    if(num == 1){
        //进入用户参数界面
        ui->stackedWidget->setCurrentIndex(5);

    }else if (num ==2){
        //进入系统参数界面
        ui->stackedWidget->setCurrentIndex(6);
    }
}

void Widget::on_last_clicked()
{
    int currentIndex = ui->stackedWidget_2->currentIndex();
    int previousPageIndex = (currentIndex - 1 + ui->stackedWidget_2->count()) % ui->stackedWidget_2->count();// 循环切换到上一个页面
    ui->stackedWidget_2->setCurrentIndex(previousPageIndex);
}

void Widget::on_next_clicked()
{
    int currentIndex = ui->stackedWidget_2->currentIndex();
    int nextPageIndex = (currentIndex + 1) % ui->stackedWidget_2->count(); // 循环切换到下一个页面
    ui->stackedWidget_2->setCurrentIndex(nextPageIndex);
}
//机器人介绍界面
void Widget::on_robot_introduction_clicked()
{
    ui->stackedWidget->setCurrentIndex(7);
}


void Widget::update_time(){
    QDateTime currentTime = QDateTime::currentDateTime();
    QString timeString = currentTime.toString("yyyy-MM-dd hh:mm:ss");
    ui->update_time->setText(timeString);
}

