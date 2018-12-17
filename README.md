# Occupancy Grid Mapping using Kinect
This project is part of the Autonomous Systems course from Instituto Superior Técnico. 

The main goal of this project is to implement the Occupancy Grid Mapping algorithm and estimate, accurately, maps from different divisions using the Microsoft Kinect depth camera and the Pioneer-3DX.

# How to run the code
To run this implementation of the Occupancy Grid Mapping you need to:

- Install and run libfreenect package
  - roslaunch freenect_launch freenect.launch
  
- Install Depthimage_to_laserscan package
  - use the launch file depthtolaser.launch (or modify it accordingly)
  - use the cfg file Depth.cfg (or modify it accordingly)
  
 # How to improve the results
 This algorithm is using the Pioneer-3DX odometry to get the robot pose.
 Since the odometry accumulates error over time, you should use GMapping to create a map and then use AMCL package to get a better robot localization.
  - Install GMapping package
    - use the launch file slam_gmapping_pr2.launch
  - Install AMCL package
    - use the amcl_diff.launch
