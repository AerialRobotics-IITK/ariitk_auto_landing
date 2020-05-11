# ariitk_auto_landing
ROS packages for autonomous landing of an MAV on a moving platform.

Tasks to be done  
- [x] Re-arrange nodes in different packages
- [x] Finish the rqt_gui interface
- [x] Rename and make proper launch files
- [x] Clean up parameter files
- [x] Strictly check the files to adhere to conventions
- [x] Use different modes for tracking, either ground truth or Detection+Pose estimation, also whether to use trajectory generation or not
- [ ] Finish README and Wiki
- [x] Check all files and parameterize any functional hard-coded values
- [x] Rigorous testing
- [x] Add functionality for moving husky via keyboard, and maybe also a function that gives the husky a random trajectory
- [x] Add wind support to launch files
- [ ] When platform is not detected, what to do
- [x] Add Rosmon
- [x] Replace husky everywhere with platform, also firefly with mav/quad

Optional (If time permits)
- [ ] Implement the sim in HEADLESS gazebo mode, using only rviz.
- [ ] Stop quad motors after landing
