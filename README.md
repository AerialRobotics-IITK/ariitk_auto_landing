# ariitk_auto_landing
This repository contains ROS packages for autonomous landing of an MAV on a moving platform.

Installation instructions
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
wstool init
git clone https://github.com/AerialRobotics-IITK/ariitk_auto_landing
wstool merge ariitk_auto_landing/install/install_https.rosinstall
wstool update
catkin build
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Run the simulation using
```
mon launch auto_landing autonomous_landing_sim.launch
```

The parameters that can be set in the autonomous_landing_sim.launch file
- **mav_name**: Name of the MAV to be used in the RotorS simulation
- **x**: x coordinate of spawn point of husky
- **y**: y coordinate of spawn point of husky
- **publish_visualization_marker**: When tracking using trajectory generation, whether to use rviz to visualize the generated trajectory
- **use_detection(experimental)**: Use detection to calculate the odometry of the platform 
- **use_odometry_for_platform**: Use odometry instead of gazebo/model_states
- **use_wind**: Add wind to the simulation

For more info, please refer the wiki.
