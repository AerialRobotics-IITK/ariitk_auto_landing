#include <auto_landing/trajectory_test.hpp>

namespace ariitk::auto_landing{

ExampleTrajectoryGeneration::ExampleTrajectoryGeneration()
    : dimension_(3), derivative_to_optimize_(mav_trajectory_generation::derivative_order::SNAP), start_(dimension_), middle_one_(dimension_), middle_two_(dimension_), end_(dimension_) {
        start_.makeStartOrEnd(Eigen::Vector3d(0,0,1), derivative_to_optimize_);
        middle_one_.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(1,0,1));
        middle_two_.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(2,0,1));
        end_.makeStartOrEnd(Eigen::Vector3d(3,0,1), derivative_to_optimize_);

        vertices_.push_back(start_);
        vertices_.push_back(middle_one_);
        vertices_.push_back(middle_two_);
        vertices_.push_back(end_);

        mav_trajectory_generation::Trajectory result = this->generateTrajectory(vertices_);
        mav_msgs::EigenTrajectoryPoint::Vector trajectory_points;
        trajectory_msgs::MultiDOFJointTrajectory generated_trajectory;

        mav_trajectory_generation::sampleWholeTrajectory(result, 0.1, &trajectory_points);
        mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_points, &generated_trajectory);
        
        std::cout << generated_trajectory; 
           
    }

mav_trajectory_generation::Trajectory ExampleTrajectoryGeneration::generateTrajectory(std::vector<mav_trajectory_generation::Vertex>& vertices) {
    std::vector<double> segment_times;
    const double v_max = 10.0;
    const double a_max = 10.0;

    segment_times = mav_trajectory_generation::estimateSegmentTimes(vertices_, v_max, a_max);

    const int N = 10;
    mav_trajectory_generation::PolynomialOptimization<N> opt(dimension_);

    opt.setupFromVertices(vertices_, segment_times, derivative_to_optimize_);
    opt.solveLinear();

    mav_trajectory_generation::Trajectory trajectory;
    opt.getTrajectory(&trajectory);    

    return trajectory;
}

}