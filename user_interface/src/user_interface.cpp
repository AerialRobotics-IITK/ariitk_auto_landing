#include<ros/ros.h>

#include <ncurses.h>
#include<geometry_msgs/PoseStamped.h>
#include<gazebo_msgs/ModelStates.h>
#include<nav_msgs/Odometry.h>

    int X_KEY         = 1;
    int X_DESCRIPTION = 10;
    int X_VALUE       = 35;
    
    enum YCursorRows {HEADING=6, Linear=8,Circle,Eight,Random };
    enum RandomY {forward =8, right,left,ispeed,quit,current_speed};


    
    nav_msgs::Odometry husky_odom_,quad_odom_;
    geometry_msgs::Twist twist;
 

    
    int ch;
   






    std::string trajectory_name = "linear";
    int flag =0;
void printScreen() {
    clear();
    ros::spinOnce();
    printw(" ******************************************************************************\n");
    printw("                               User Interface ARIITK \n");
    printw("                                ARIITK AUTO_LANDING \n");
    printw("                        https://github.com/auto_landing\n");
    printw(" ******************************************************************************\n\n");


    wmove(stdscr, HEADING, X_KEY);
    printw("Key");
    wmove(stdscr, HEADING, X_DESCRIPTION);
    printw("Description");
    wmove(stdscr, HEADING, X_VALUE);
    printw("Info");

    wmove(stdscr, Linear, X_KEY);
    printw("l");
    wmove(stdscr, Linear, X_DESCRIPTION);
    printw("Linear");
    wmove(stdscr, Linear, X_VALUE);
    printw("set linear trajectory");

    wmove(stdscr, Circle, X_KEY);
    printw("c");
    wmove(stdscr, Circle, X_DESCRIPTION);
    printw("Circle");
    wmove(stdscr, Circle, X_VALUE);
    printw("set circle trajectory");

    wmove(stdscr, Eight, X_KEY);
    printw("e");
    wmove(stdscr, Eight, X_DESCRIPTION);
    printw("Eight");
    wmove(stdscr, Eight, X_VALUE);
    printw("set eight trajcetory");

    wmove(stdscr, HEADING, X_KEY);
    printw("Key");
    wmove(stdscr, HEADING, X_DESCRIPTION);
    printw("Description");
    wmove(stdscr, HEADING, X_VALUE);
    printw("Info");

    wmove(stdscr, Random, X_KEY);
    printw("r");
    wmove(stdscr, Random, X_DESCRIPTION);
    printw("Random");
    wmove(stdscr, Random, X_VALUE);
    printw("set your own trajectory");

    wmove(stdscr, 14, X_KEY);
    printw("            ------------------------------------------------                \n");

    wmove(stdscr, 16, X_KEY);
    printw("Pose");
    wmove(stdscr, 16, 20);
    printw("x");
    wmove(stdscr, 16, 30);
    printw("y");
    wmove(stdscr, 16, 40);
    printw("z");

    wmove(stdscr, 18, X_KEY);
    printw("Husky_Pose");
    wmove(stdscr, 18, 20);
    printw("%ld",husky_odom_.pose.pose.position.x);
    wmove(stdscr, 18, 30);
    printw("%ld",husky_odom_.pose.pose.position.y);
    wmove(stdscr, 18, 40);
    printw("%ld",husky_odom_.pose.pose.position.z);

    wmove(stdscr, 19, X_KEY);
    printw("Firefly_Pose");
    wmove(stdscr, 19, 20);
    printw("%ld",quad_odom_.pose.pose.position.x);
    wmove(stdscr, 19, 30);
    printw("%ld",quad_odom_.pose.pose.position.x);
    wmove(stdscr, 19, 40);
    printw("%ld",quad_odom_.pose.pose.position.z);

}
void printRandom(double speed) {
    clear(); 
    ros::spinOnce();
    printw(" ******************************************************************************\n");
    printw("                               User Interface ARIITK \n");
    printw("                                ARIITK AUTO_LANDING \n");
    printw("                        https://github.com/auto_landing\n");
    printw(" ******************************************************************************\n\n");

    wmove(stdscr, HEADING, X_KEY);
    printw("Key");
    wmove(stdscr, HEADING, X_DESCRIPTION);
    printw("Description");
    wmove(stdscr, HEADING, X_VALUE);
    printw("Info");

    wmove(stdscr, forward, X_KEY);
    printw("w");
    wmove(stdscr, forward, X_DESCRIPTION);
    printw("forward");
    wmove(stdscr, forward, X_VALUE);
    printw("move husky forward");

    wmove(stdscr, right, X_KEY);
    printw("d");
    wmove(stdscr, right, X_DESCRIPTION);
    printw("right");
    wmove(stdscr, right, X_VALUE);
    printw("move husky right");

    wmove(stdscr, left, X_KEY);
    printw("a");
    wmove(stdscr, left, X_DESCRIPTION);
    printw("left");
    wmove(stdscr, left, X_VALUE);
    printw("move husky left");

    wmove(stdscr, ispeed, X_KEY);
    printw("i");
    wmove(stdscr, ispeed, X_DESCRIPTION);
    printw("increase speed");
    wmove(stdscr, ispeed, X_VALUE);
    printw("increases husky speed");

    wmove(stdscr, quit, X_KEY);
    printw("q");
    wmove(stdscr, quit, X_DESCRIPTION);
    printw("quit");
    wmove(stdscr, quit, X_VALUE);
    printw("exit from random mode");


    wmove(stdscr, current_speed, X_KEY);
    printw("-");
    wmove(stdscr, current_speed, X_DESCRIPTION);
    printw("current_speed");
    wmove(stdscr, current_speed, X_VALUE);
    printw("%ld",speed);

    wmove(stdscr, 15, X_KEY);
    printw("            ------------------------------------------------                \n");

    wmove(stdscr, 17, X_KEY);
    printw("Pose");
    wmove(stdscr, 17, 20);
    printw("x");
    wmove(stdscr, 17, 30);
    printw("y");
    wmove(stdscr, 17, 40);
    printw("z");

    wmove(stdscr, 19, X_KEY);
    printw("Husky_Pose");
    wmove(stdscr, 19, 20);
    printw("%ld",husky_odom_.pose.pose.position.x);
    wmove(stdscr, 19, 30);
    printw("%ld",husky_odom_.pose.pose.position.y);
    wmove(stdscr, 19, 40);
    printw("%ld",husky_odom_.pose.pose.position.z);

    wmove(stdscr, 20, X_KEY);
    printw("Firefly_Pose");
    wmove(stdscr, 20, 20);
    printw("%ld",quad_odom_.pose.pose.position.x);
    wmove(stdscr, 20, 30);
    printw("%ld",quad_odom_.pose.pose.position.x);
    wmove(stdscr, 20, 40);
    printw("%ld",quad_odom_.pose.pose.position.z);

}
void callBackKey(int c) {
    switch(c) {
        case 'l':
            trajectory_name = "linear";
            break;
        case 'e':
            trajectory_name = "eight";
            break;
        case 'c':
            trajectory_name = "circle";
            break;
        case 'r':
            flag=1;
            break;
        case 'w':
            ch ='w';
            break;
        case 'a':
            ch = 'a';
            break;
        case 'd':
            ch = 'd';
            break;
        case 'q':
            flag = 0;
            break;

    }
}

void modelStateCallback(const gazebo_msgs::ModelStates& msg) {
    
    int index = 0;
    int idx =0;

    while(msg.name[index++] != "/") {}
    while(msg.name[idx++] != "firefly") {}

    husky_odom_.pose.pose.position.x = msg.pose[index-1].position.x;
    husky_odom_.pose.pose.position.y = msg.pose[index-1].position.y;
    husky_odom_.pose.pose.position.z = msg.pose[index-1].position.z;

    quad_odom_.pose.pose.position.x = msg.pose[idx-1].position.x;
    quad_odom_.pose.pose.position.y = msg.pose[idx-1].position.y;
    quad_odom_.pose.pose.position.z = msg.pose[idx-1].position.z;

    std::cout<<"1";
    std::cout << husky_odom_.pose.pose.position.x;
}
double randomTrajectory() {
    double speed = 1;
    double lin,ang;
    char key = ch;
 std::map<char, std::vector<float>> moveBindings
{
  {'w', {1, 0,  0}},
  {'d', {0, 0, -1}},
  {'a', {0, 0, 1}},
};

std::map<char, float> speedBindings
{
  {'i', 1.1}
};
 if (moveBindings.count(key) == 1){
     lin = moveBindings[key][0];
     ang = moveBindings[key][2];
 }

 else if (speedBindings.count(key) ==1) {
     speed = speed*speedBindings[key];

 }
 else {
     lin = 0;
     ang =0;
 }
 twist.linear.x = lin * speed;
 twist.angular.z = ang * speed;

 return speed;
}
int main(int argc ,char **argv) {
    ros::init(argc,argv,"user_interface");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1);
    ros::Subscriber gazebo_model_state_sub_ = nh.subscribe("/gazebo/model_states",1,modelStateCallback);
    ros::spinOnce();
    initscr();
    cbreak();              // disables buffering of types characters
    noecho();              // suppresses automatic output of typed characters
    keypad(stdscr, TRUE);  // to capture special keypad characters
   
    printScreen();
    ros::Rate loop_rate(100);

    while(ros::ok()) {
    
     ch =getch();
    callBackKey(ch);
    ros::spinOnce();
    if(!flag) printScreen();
    if(flag) {
        printRandom(randomTrajectory());
        pub.publish(twist);
    }
    refresh();

    loop_rate.sleep();
    }
    endwin();

    return 0;
}