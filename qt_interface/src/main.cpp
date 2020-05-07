#include "mainwindow.h"
#include <QApplication>


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    ros::init(argc,argv,"qt_interface");

    ros::NodeHandle nh;
    // ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1);
    // geometry_msgs::Twist vel;

    ros::Rate loop_rate(10);

    while(ros::ok()) {
    w.run();
    loop_rate.sleep();
    // vel.linear.x = 5;
    // pub.publish(vel) ;
    }
    

    return a.exec();
}
