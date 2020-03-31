#include <auto_landing/trajectory_publisher_visualizer.hpp>

namespace ariitk::auto_landing{

ExampleTrajectoryGeneration::ExampleTrajectoryGeneration()
    : dimension_(3), nh_(), nh_private_("~") {

        trajectory_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command_trajectory", 1);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
        mav_odometry_sub_ = nh_.subscribe("mav_odometry", 10, &ExampleTrajectoryGeneration::mavOdometryCallback, this);
        husky_odometry_sub_ = nh_.subscribe("husky_odometry", 10, &ExampleTrajectoryGeneration::huskyOdometryCallback, this);

        
        mav_trajectory_generation::Vertex start_(dimension_), end_(dimension_);
        mav_trajectory_generation::Vertex::Vector vertices_;
        
        nh_private_.getParam("v_max", v_max_);
        nh_private_.getParam("a_max", a_max_);
        nh_private_.getParam("distance", distance_);

        // nh_private_.getParam("points", points_);

        // int number_of_points = points_.size();
        // if (number_of_points < 2) {
        //     ROS_ERROR ("The number of points given in the trajectory is less than 2. Please specify atleast 2 points.");
        //     return;
        // }
        // derivative_to_optimize_ = int(points_[0]["derivative_order"]);
        
        // start_.makeStartOrEnd(Eigen::Vector3d(getValueAsDouble(points_[0]["x"]), getValueAsDouble(points_[0]["y"]), getValueAsDouble(points_[0]["z"])), int(points_[0]["derivative_order"]));
        // end_.makeStartOrEnd(Eigen::Vector3d(getValueAsDouble(points_[number_of_points-1]["x"]), getValueAsDouble(points_[number_of_points-1]["y"]), getValueAsDouble(points_[0]["z"])), int(points_[0]["derivative_order"]));
        
        // vertices_.push_back(start_);
        // for (int i=1;i<number_of_points-1;i++) {
        //     mav_trajectory_generation::Vertex point(dimension_);
        //     point.addConstraint(int(points_[i]["derivative_order"]), Eigen::Vector3d(getValueAsDouble(points_[i]["x"]), getValueAsDouble(points_[i]["y"]), getValueAsDouble(points_[i]["z"])));
        //     vertices_.push_back(point);
        // }
        // vertices_.push_back(end_);


    }

void ExampleTrajectoryGeneration::run() {
    generateTrajectory(generateLinearTrajectory());
    // generateLinearTrajectory();
    marker_pub_.publish(markers_);
    trajectory_pub_.publish(generated_trajectory_);
}

mav_trajectory_generation::Vertex::Vector ExampleTrajectoryGeneration::generateLinearTrajectory() {
    mav_trajectory_generation::Vertex start(dimension_), end(dimension_);
    mav_trajectory_generation::Vertex::Vector vertices;
    // ROS_WARN("Odometry is %lf, %lf, %lf\n",mav_odom_.pose.pose.position.x, mav_odom_.pose.pose.position.y, mav_odom_.pose.pose.position.z);
    
    Eigen::Vector3d start_point(husky_odom_.pose.pose.position.x, husky_odom_.pose.pose.position.y, 4.0);
    Eigen::Vector3d start_point_velocity(husky_odom_.twist.twist.linear.x, husky_odom_.twist.twist.linear.y, husky_odom_.twist.twist.linear.z);
    // Eigen::Vector3d end_point(mav_odom_.pose.pose.position.x+(husky_odom_.pose.pose.position.x-mav_odom_.pose.pose.position.x)/2, mav_odom_.pose.pose.position.y+(husky_odom_.pose.pose.position.y-mav_odom_.pose.pose.position.y)/2, 4.0);
    Eigen::Vector3d husky_direction = start_point_velocity;
    husky_direction.normalize();
    Eigen::Vector3d end_point(husky_odom_.pose.pose.position.x, husky_odom_.pose.pose.position.y, 4.0);
// ROS_WARN("Start point is %lf %lf %lf\nEnd Point is %lf %lf %lf\nEnd Point Velocity is %lf %lf %lf\n",start_point[0],start_point[1],start_point[2],end_point[0],end_point[1],end_point[2], end_point_velocity[0],end_point_velocity[1],end_point_velocity[2]);
    start.makeStartOrEnd(start_point, 3);
    end.makeStartOrEnd(end_point+(husky_direction*2), 3);
    // end.makeStartOrEnd(Eigen::Vector3d(mav_odom_.pose.pose.position.x+10.5, mav_odom_.pose.pose.position.y, 5.0), 3);
    // end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d(2,0,0));
    start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, start_point_velocity);

    vertices.push_back(start);

    // for (int i=0.5;i<=4;i+=0.5) {
    //     mav_trajectory_generation::Vertex point(dimension_);
    //     point.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(0.5*0.5*i*i, 0.0, 5.0));
    //     point.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d(0.5*i, 0.0, 0.0));
    //     point.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, Eigen::Vector3d(0.5, 0.0, 0.0));
    //     vertices.push_back(point);
    // }

    // for (int i=4.5;i<=7;i+=0.5) {
    //     mav_trajectory_generation::Vertex point(dimension_);
    //     point.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(4+2*(i-4), 0.0, 5.0));
    //     point.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d(4.0,0.0,0.0));
    //     point.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, Eigen::Vector3d(0.0, 0.0, 0.0));
    //     vertices.push_back(point);
    // }

    vertices.push_back(end);

    return vertices;

}

void ExampleTrajectoryGeneration::generateTrajectory(std::vector<mav_trajectory_generation::Vertex> vertices) {    
    std::vector<double> segment_times;

    segment_times = mav_trajectory_generation::estimateSegmentTimes(vertices, v_max_, a_max_);

    mav_trajectory_generation::PolynomialOptimization<10> opt(dimension_);

    opt.setupFromVertices(vertices, segment_times, 1);
    opt.solveLinear();
    opt.getTrajectory(&result_);

    mav_msgs::EigenTrajectoryPoint::Vector trajectory_points;

    mav_trajectory_generation::sampleWholeTrajectory(result_, 0.1, &trajectory_points);
    mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_points, &generated_trajectory_);

    std::string frame_id = "world";

    mav_trajectory_generation::drawMavTrajectory(result_, distance_, frame_id, &markers_);
        
}

double ExampleTrajectoryGeneration::getValueAsDouble(XmlRpc::XmlRpcValue& value) {
    if (value.getType() == 2) {
        return (double(int(value)));
    } else if (value.getType() == 3){
        return (double(value));
    } else {
        ROS_ERROR("Parameter specified is not integer or double. Taking 0.0 as the value.");
        return 0.0;
    }
    
}

void ExampleTrajectoryGeneration::mavOdometryCallback(const nav_msgs::Odometry& msg) {
    mav_odom_ = msg;
    // ROS_INFO("Odom call back, x:%lf, y:%lf, z:%lf\n",mav_odom_.pose.pose.position.x, mav_odom_.pose.pose.position.y, mav_odom_.pose.pose.position.z);
}

void ExampleTrajectoryGeneration::huskyOdometryCallback(const gazebo_msgs::ModelStates& msg) {
    int index=0;
    std::string name = msg.name[index];
    while (name != "/") {
        index++;
        name = msg.name[index];
    }

    husky_odom_.pose.pose = msg.pose[index];
    husky_odom_.twist.twist = msg.twist[index];
}

} //namespace ariitk::auto_landing