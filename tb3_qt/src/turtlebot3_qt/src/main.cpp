#include "widget.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    // rclcpp::init(argc,argv);
    QApplication a(argc, argv);
    Widget * w =new Widget();
    w->show();
    return a.exec();
}
