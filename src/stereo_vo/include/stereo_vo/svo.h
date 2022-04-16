#pragma once

#include <ros/ros.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/video.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>




class svo{

public:
	svo(ros::NodeHandle& nodeHandle);

private:

	bool load_parameters();

	bool initialize();

	void rectify();
	
	bool find_feature_matches();
	
		





};
