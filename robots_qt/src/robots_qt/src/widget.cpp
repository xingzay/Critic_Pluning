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

    // 将check_tb3_node_label以及check_robot_label默认背景颜色为红色

    ui->check_tb3_node_label->setStyleSheet("background-color: red;");
    ui->check_robot_label->setStyleSheet("background-color: red;");

    ui->check_scan_node_label->setStyleSheet("background-color: red;");

    // 改变表格大小
    connect(this, &Widget::dataReady, this, &Widget::updateLaserTable);


    int argc = 0; char **argv = NULL;
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("ROS2_Node_QT");

    vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        10,
        std::bind(&Widget::vel_callback,this,_1));
    // 路程 -- 里程计信息
    odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
        // "/odometry/filtered",
        "/odom",
        10,
        std::bind(&Widget::odom_callback,this,_1)
        );

    // 机器人状态 -- 判断turtlebot3_node节点是否启动
    heartbeat_sub_ = node->create_subscription<std_msgs::msg::Empty>(
        "/heart_beat",
        10,
        std::bind(&Widget::heartbeat_callback,this,_1)
        );
    // 激光雷达 -- 判断是否订阅到/scan话题
    laser_scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        10,
        std::bind(&Widget::check_scan_topic,this,_1)
    );

    // 创建定时器，用于检查心跳信号是否超时
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &Widget::check_heartbeat);
    connect(timer_, &QTimer::timeout, this, &Widget::update_time);
    timer_->start(1000);

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

    if (timer_) {
        timer_->stop();  // 停止定时器
    }
    rclcpp::shutdown();
    delete ui;
}
void Widget::updateGUI(){
    return;
}
// 显示速度
void Widget::vel_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel){
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
void Widget::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom){
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
void Widget::heartbeat_callback(const std_msgs::msg::Empty& ) {
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

void Widget::check_heartbeat() {
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

void Widget::check_scan_topic(const sensor_msgs::msg::LaserScan & scan){
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

Point Widget::findMax(const sensor_msgs::msg::LaserScan& scan)
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


//主界面
void Widget::on_MainWindow_clicked()
{
    ui->stackedWidget->setCurrentIndex(0);
}

//激光雷达信息界面
void Widget::on_Time_Work_clicked()
{
    ui->stackedWidget->setCurrentIndex(1);
}

//报警信息界面
void Widget::on_Error_clicked()
{
    int initialRowCount = 15;
    ui->tableWidget->setRowCount(initialRowCount);

    // for (int i = 0; i < initialRowCount; ++i) {
    //     for (int j = 0; j < ui->tableWidget->columnCount(); ++j) {
    //         QTableWidgetItem *item = new QTableWidgetItem("Data");
    //         ui->tableWidget->setItem(i, j, item);
    //     }
    // }
    ui->stackedWidget->setCurrentIndex(2);
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
    ui->stackedWidget->setCurrentIndex(3);
}
//用户参数登录界面
void Widget::on_user_param_clicked()
{
    //进入用户参数界面
    ui->stackedWidget->setCurrentIndex(4);
    // this->show();
}
//系统参数登录界面
void Widget::on_system_param_clicked()
{
    //进入系统参数界面
    ui->stackedWidget->setCurrentIndex(5);
    // this->show();
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
    ui->stackedWidget->setCurrentIndex(6);
}


void Widget::update_time(){
    QDateTime currentTime = QDateTime::currentDateTime();
    QString timeString = currentTime.toString("yyyy-MM-dd hh:mm:ss");
    ui->update_time->setText(timeString);
}


