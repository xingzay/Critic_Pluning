#ifndef LOG_IN_H
#define LOG_IN_H

#include <QWidget>
#include <QMessageBox>

namespace Ui {
class LOG_IN;
}

class LOG_IN : public QWidget
{
    Q_OBJECT

public:
    explicit LOG_IN(QWidget *parent = nullptr);
    ~LOG_IN();
    void switchToPage(int num);
signals:
    void loginSuccess(int interfaceID);  // 信号：登录成功并传递接口ID
    void showLogin();                   // 信号：打开登录界面

private slots:

    void on_ON_clicked();
    void on_ON_2_clicked();

    void on_OFF_2_clicked();

    void on_OFF_clicked();

private:
    Ui::LOG_IN *ui;
};

#endif // LOG_IN_H
