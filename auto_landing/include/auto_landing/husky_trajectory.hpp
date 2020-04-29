#pragma once

#include <ros/ros.h>

// #include <ncurses.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include<gazebo_msgs/ModelStates.h>

namespace ariitk::auto_landing {

class HuskyTrajectory {
   public:
      void init(ros::NodeHandle& nh, ros::NodeHandle& nh_private, char **argv);
      void run();

   private:
      void modelStateCallback(const gazebo_msgs::ModelStates& msg);
      void linearTrajectory();
      void circularTrajectory();
      void eightTrajectory();

      ros::Publisher trajectory_pub_;
      ros::Subscriber gazebo_model_state_sub_;
      
      geometry_msgs::Twist husky_velocity_;
      nav_msgs::Odometry husky_odom_;

      double angular_velocity_,linear_velocity_;
      double time_[2];
      double husky_x_ ,husky_y_;
      int count_ ;
      double t_inverse;
      std::string trajectory_name_;
};

} //namespace ariitk::auto_landing

