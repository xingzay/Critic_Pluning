#include "ros2.h"
#include <QApplication>

// int main(int argc, char *argv[])
// {
//     // rclcpp::init(argc,argv);
//     QApplication a(argc, argv);
//     Widget * w =new Widget();
//     w->show();

//     return a.exec();
// }

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);  // 初始化ROS 2

    QApplication app(argc, argv);  // 初始化Qt

    auto node = std::make_shared<ROS2>();
    node->show();

    auto spin_thread = std::thread([&]() {
        rclcpp::spin(node);
        rclcpp::shutdown();
    });

    int result = app.exec();

    spin_thread.join();
    return result;
}
