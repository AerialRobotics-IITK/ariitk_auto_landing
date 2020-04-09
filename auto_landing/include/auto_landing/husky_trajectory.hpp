#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

namespace ariitk::auto_landing {

class HuskyTrajectory {
   public:
      void init(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
      void run();

   private:
      double angular_velocity_,linear_velocity_;
      ros::Publisher trajectory_pub_;
      geometry_msgs::Twist husky_velocity_;
};

} //namespace ariitk::auto_landing

