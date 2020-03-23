#include <ros/ros.h>

#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_msgs/conversions.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory_sampling.h>

namespace ariitk::auto_landing{
class ExampleTrajectoryGeneration {
    public:
        ExampleTrajectoryGeneration();
        ~ExampleTrajectoryGeneration(){};
        mav_trajectory_generation::Trajectory generateTrajectory (std::vector<mav_trajectory_generation::Vertex>& vertices);

    private:
        const int dimension_;
        const int derivative_to_optimize_;

        mav_trajectory_generation::Vertex start_, middle_one_, middle_two_, end_;
        std::vector<mav_trajectory_generation::Vertex> vertices_;


};

}