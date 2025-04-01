#This package provides an autonomous mapping and navigation system for TurtleBot3 using ROS 2.
#.
#Clone the repository and build the package:
#.
# cd ~/ros2_ws/src
# git clone https://github.com/KonstantinKolodi/av_course
# cd ~/ros2_ws
# colcon build --packages-select my_robot_controller
# source install/setup.bash
# source ~/.bashrc
#.
#Launching the Simulation
#.
# ros2 launch my_robot_controller start_mapping.launch.py
#.
#Features
#.
#Obstacle avoidance (smooth turns, wall following)
#Corner detection (avoids getting stuck)
#Adaptive movement based on LiDAR data
#.
#Dependencies
#.
#ROS 2 (Humble)
#turtlebot3_gazebo
#turtlebot3_cartographer
#TurtleBot3 Simulation Package
#.
#Troubleshooting
#.
#If start_mapping.launch.py is not found:
# source install/setup.bash
#If issues persist, rebuild:
# colcon build --symlink-install
# source ~/.bashrc
# source install/setup.bash
