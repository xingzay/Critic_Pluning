#include "widget.h"
#include "./ui_widget.h"
#include <QMetaObject>

Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget),
    mutex_()
{
    ui->setupUi(this);
    // connect(ui->pushButton_i,SIGNAL(clicked()),this,SLOT(slot_cmd_control()));
    // 将check_tb3_node_label以及check_robot_label默认背景颜色为红色

    ui->check_tb3_node_label->setStyleSheet("background-color: red;");
    ui->check_robot_label->setStyleSheet("background-color: red;");

    ui->check_scan_node_label->setStyleSheet("background-color: red;");

    // 改变表格大小
    connect(this, &Widget::dataReady, this, &Widget::updateLaserTable);
    // 创建定时器，用于检查心跳信号是否超时
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &Widget::update_time);
    timer_->start(1000);

    //多线程运行spin
    // spin_thread = std::thread([this]() {
    //     rclcpp::spin(node);
    // });

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
    // rclcpp::shutdown();
    delete ui;
}

void Widget::updateGUI(){
    return;
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



