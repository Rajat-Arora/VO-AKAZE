# VO-AKAZE
The `STEREO_VO` package is a ROS based package for stereo visual odometry using AKAZE as feature detector and descriptor. The software takes in synchronized stereo images and generates 6DOF pose estimation of the left camera frame.

The software is tested on Ubuntu 20.04 with ROS Noetic.

## Dependencies
Most of the dependencies are standard including `Eigen`, `OpenCV`. The standard shipment from Ubuntu 20.04 and ROS Noetic works fine.

## Compling
The software is a standard catkin workspace. Make sure the package is on `ROS_PACKAGE_PATH` after cloning the package to your workspace. And the normal procedure for compiling a catkin package should work which is as follows:-

* `git clone https://github.com/Rajat-Arora/VO-AKAZE.git`
* `cd VO-AKAZE && rosdep install --from-paths src --ignore-src -r -y`
* `catkin_make`

## EuRoC dataset example usage

First obtain the [EuRoC](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) dataset from the official website.


Recommended EuRoC ROS Bags:
- [Vicon Room 1 01](http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/V1_01_easy/V1_01_easy.bag)
- [Machine Hall 01](http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.bag)

Once the `stereo_vo` is built and sourced (via `source <path to catkin_ws>/devel/setup.bash`), there is a launch file prepared for the [EuRoC](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)  datset named `stereo_vo_euroc.launch` respectively. Launch files instantiates a ROS node:

* `svo` processes stereo images to detect, track features and computes the trajectory and path which could be visualized in rviz.

These launch file can be executed via

```
roslaunch stereo_vo stereo_vo_euroc.launch
```

Once the nodes are running you need to run the dataset rosbags (in a different terminal) 
```
rosbag play <Path to bag file>
```
For example:
```
rosbag play V1_01_easy.bag
```

To visualize the pose you can use the provided rviz configurations found in `VO-ORB/src/stereo_vo/rviz` folder (EuRoC: `rviz_euroc_config.rviz`).
