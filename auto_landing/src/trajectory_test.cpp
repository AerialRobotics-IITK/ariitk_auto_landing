#include <auto_landing/trajectory_test.hpp>

namespace ariitk::auto_landing{

ExampleTrajectoryGeneration::ExampleTrajectoryGeneration()
    : dimension_(3), nh_(), nh_private_("~") {

        mav_trajectory_generation::Vertex start_(dimension_), end_(dimension_);
        mav_trajectory_generation::Vertex::Vector vertices_;
        
        nh_.getParam("v_max", v_max_);
        nh_.getParam("a_max", a_max_);
        nh_.getParam("distance", distance_);
        nh_.getParam("points", points_);

        int number_of_points = points_.size();
        if (number_of_points < 2) {
            ROS_ERROR ("The number of points given in the trajectory is less than 2. Please specify atleast 2 points.");
            return;
        }
        derivative_to_optimize_ = int(points_[0]["derivative_order"]);
        
        start_.makeStartOrEnd(Eigen::Vector3d(getValueAsDouble(points_[0]["x"]), getValueAsDouble(points_[0]["y"]), getValueAsDouble(points_[0]["z"])), int(points_[0]["derivative_order"]));
        end_.makeStartOrEnd(Eigen::Vector3d(getValueAsDouble(points_[number_of_points-1]["x"]), getValueAsDouble(points_[number_of_points-1]["y"]), getValueAsDouble(points_[0]["z"])), int(points_[0]["derivative_order"]));
        
        vertices_.push_back(start_);
        for (int i=1;i<number_of_points-1;i++) {
            mav_trajectory_generation::Vertex point(dimension_);
            point.addConstraint(int(points_[i]["derivative_order"]), Eigen::Vector3d(getValueAsDouble(points_[i]["x"]), getValueAsDouble(points_[i]["y"]), getValueAsDouble(points_[i]["z"])));
            vertices_.push_back(point);
        }
        vertices_.push_back(end_);

        generateTrajectory(vertices_);

    }

void ExampleTrajectoryGeneration::run() {
    visualization_msgs::MarkerArray markers;
    std::string frame_id = "world";

    mav_trajectory_generation::drawMavTrajectory(result_, distance_, frame_id, &markers);

    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);
    marker_pub_.publish(markers);
}

void ExampleTrajectoryGeneration::generateTrajectory(std::vector<mav_trajectory_generation::Vertex> vertices) {    
    std::vector<double> segment_times;

    segment_times = mav_trajectory_generation::estimateSegmentTimes(vertices, v_max_, a_max_);

    mav_trajectory_generation::PolynomialOptimization<10> opt(dimension_);

    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize_);
    opt.solveLinear();
    opt.getTrajectory(&result_);

    mav_msgs::EigenTrajectoryPoint::Vector trajectory_points;

    mav_trajectory_generation::sampleWholeTrajectory(result_, 0.1, &trajectory_points);
    mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_points, &generated_trajectory_);
    
    trajectory_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("firefly/command/trajectory", 1);
    
    trajectory_pub_.publish(generated_trajectory_);
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

} //namespace ariitk::auto_landing