#include "mainwindow.h"
#include "ui_mainwindow.h"
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}
void MainWindow::init(ros::NodeHandle&nh) {
    pub = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1);
    vel.linear.x=0;
}
void MainWindow::run() {
     pub.publish(vel) ;
}
void MainWindow::on_pushButton_2_clicked()
{
    vel.linear.x = 5;
}

void MainWindow::on_pushButton_3_clicked()
{
    std::cout<<"2";
}

void MainWindow::on_pushButton_clicked()
{
 std::cout<<"3";
}
