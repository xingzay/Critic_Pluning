#ifndef WIDGET_H
#define WIDGET_H

#include <QTimer>
#include <QDateTime>
#include <QProcess>
#include <QWidget>
#include <QLabel>
#include <mutex>
#include <thread>


namespace Ui {
class Widget;
}
struct Point
{
    long index;
    float range;
    float intensity;
    float x;
    float y;
};
class Widget : public QWidget
{
    Q_OBJECT

public:
    explicit Widget(QWidget *parent = nullptr);
    // rclcpp::Node::SharedPtr node;
    ~Widget();
protected:
    Ui::Widget *ui;
private:
    // Ui::Widget *ui;
    std::thread spin_thread; //多线程对象
    std::mutex mutex_; //互斥锁
    QTimer* timer_;


signals:
    void dataReady(int index, float range, float intensity, float x, float y);


private slots:
    void updateGUI();
    void on_MainWindow_clicked();

    void on_Clear_clicked();

    void on_Clean_clicked();

    void on_Time_Work_clicked();

    void on_Error_clicked();

    void on_Condition_clicked();

    void on_user_param_clicked();

    void on_system_param_clicked();

    void on_last_clicked();

    void on_next_clicked();

    void update_time();

    void on_robot_introduction_clicked();

    void updateLaserTable(int index, float range, float intensity, float x, float y);
};

#endif // WIDGET_H
