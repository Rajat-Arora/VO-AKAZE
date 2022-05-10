# VO-AKAZE
The `STEREO_VO` package is a ROS based package for stereo visual odometry using AKAZE as feature detector and descriptor. The software takes in synchronized stereo images and generates 6DOF pose estimation of the left camera frame.

The software is tested on Ubuntu 20.04 with ROS Noetic.

## Dependencies

Most of the dependencies are standard including `Eigen`, `OpenCV`. The standard shipment from Ubuntu 16.04 and ROS Noetic works fine.


## Compling
The software is a standard catkin workspace. Make sure the package is on `ROS_PACKAGE_PATH` after cloning the package to your workspace. And the normal procedure for compiling a catkin package should work which is as follows:-

* `git clone https://github.com/Rajat-Arora/VO-AKAZE.git`

