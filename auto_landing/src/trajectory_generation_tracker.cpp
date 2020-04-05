#include <auto_landing/trajectory_generation_tracker.hpp>

namespace ariitk::auto_landing{

TrajectoryGenerationTracking::TrajectoryGenerationTracking(char** argv)
    : dimension_(3), nh_(), nh_private_("~"), husky_acceleration_(0,0,0) {

    trajectory_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command_trajectory", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
    mav_odometry_sub_ = nh_.subscribe("mav_odometry", 10, &TrajectoryGenerationTracking::mavOdometryCallback, this);
    husky_odometry_sub_ = nh_.subscribe("husky_odometry", 10, &TrajectoryGenerationTracking::huskyOdometryCallback, this);
    
    nh_private_.getParam("v_max", v_max_);
    nh_private_.getParam("a_max", a_max_);
    nh_private_.getParam("distance", distance_);

    if (std::string(argv[1]) == "false") publish_visualization_=false;
    else publish_visualization_=true;
}

void TrajectoryGenerationTracking::run() {
    generateTrajectory(computePoints());
    if (publish_visualization_) marker_pub_.publish(markers_);
    trajectory_pub_.publish(generated_trajectory_);
}

mav_trajectory_generation::Vertex::Vector TrajectoryGenerationTracking::computePoints() {
    mav_trajectory_generation::Vertex start(dimension_), end(dimension_);
    mav_trajectory_generation::Vertex::Vector vertices;
    
    Eigen::Vector3d start_point(husky_odom_.pose.pose.position.x, husky_odom_.pose.pose.position.y, 4.0);
    Eigen::Vector3d start_point_velocity(husky_odom_.twist.twist.linear.x, husky_odom_.twist.twist.linear.y, husky_odom_.twist.twist.linear.z);    
    Eigen::Vector3d husky_direction = start_point_velocity;
    husky_direction.normalize();
    
    start.makeStartOrEnd(start_point, 3);
    end.makeStartOrEnd(start_point+(husky_direction*2), 3);
    
    start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, start_point_velocity);
    start.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, husky_acceleration_);

    vertices.push_back(start);
    vertices.push_back(end);

    return vertices;
}

void TrajectoryGenerationTracking::generateTrajectory(std::vector<mav_trajectory_generation::Vertex> vertices) {    
    std::vector<double> segment_times;

    segment_times = mav_trajectory_generation::estimateSegmentTimes(vertices, v_max_, a_max_);

    mav_trajectory_generation::PolynomialOptimization<10> opt(dimension_);

    opt.setupFromVertices(vertices, segment_times, 1);
    opt.solveLinear();
    opt.getTrajectory(&result_);

    mav_msgs::EigenTrajectoryPoint::Vector trajectory_points;

    mav_trajectory_generation::sampleWholeTrajectory(result_, 0.1, &trajectory_points);
    mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_points, &generated_trajectory_);

    if (publish_visualization_) {
        std::string frame_id = "world";
        mav_trajectory_generation::drawMavTrajectory(result_, distance_, frame_id, &markers_);    
    }

}

void TrajectoryGenerationTracking::mavOdometryCallback(const nav_msgs::Odometry& msg) {
    mav_odom_ = msg;
}

void TrajectoryGenerationTracking::huskyOdometryCallback(const gazebo_msgs::ModelStates& msg) {
    int index=0;
    std::string name = msg.name[index];
    while (name != "/") {
        index++;
        name = msg.name[index];
    }

    husky_acceleration_(0) = (msg.twist[index].linear.x-husky_odom_.twist.twist.linear.x)/0.1;
    husky_acceleration_(1) = (msg.twist[index].linear.y-husky_odom_.twist.twist.linear.y)/0.1;
    husky_acceleration_(2) = (msg.twist[index].linear.z-husky_odom_.twist.twist.linear.z)/0.1;

    husky_odom_.pose.pose = msg.pose[index];
    husky_odom_.twist.twist = msg.twist[index];
} 

}//namespace ariitk::auto_landing