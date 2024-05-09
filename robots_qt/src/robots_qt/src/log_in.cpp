#include "log_in.h"
#include "./ui_log_in.h"

LOG_IN::LOG_IN(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::LOG_IN){
    ui->setupUi(this);
}

LOG_IN::~LOG_IN()
{
    delete ui;
}
void LOG_IN::switchToPage(int num){
    if(num == 1){
        ui->stackedWidget->setCurrentIndex(0);
    }else if(num ==2){
        ui->stackedWidget->setCurrentIndex(1);
    }
}

void LOG_IN::on_ON_clicked(){
    QString password_1;
    password_1 = ui->lineEdit_1->text();
    if(password_1 == "123"){
        QMessageBox::information(this,"提示","登陆成功！");
        this->close();
        emit loginSuccess(1);  // 发射信号
        emit showLogin();
    }else {
        QMessageBox::information(this,"提示","密码输出错误");
    }
}


void LOG_IN::on_ON_2_clicked(){
    QString password_2;
    password_2 = ui->lineEdit_2->text();
    if(password_2 == "321"){
        QMessageBox::information(this,"提示","登陆成功！");
        this->close();
        emit loginSuccess(2);  // 发射信号
        emit showLogin();
    }else {
        QMessageBox::information(this,"提示","密码输出错误");
    }
}


void LOG_IN::on_OFF_2_clicked(){
    this->close();
    emit showLogin();
}


void LOG_IN::on_OFF_clicked(){
    this->close();
    emit showLogin();
}

