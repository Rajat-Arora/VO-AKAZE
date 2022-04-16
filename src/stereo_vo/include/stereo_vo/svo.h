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

// Methods

	bool load_parameters();

	bool initialize();

	void rectify();
	
    void initialize_first_frame();
		
	bool find_feature_matches();
	
	void pose_estimation_3d2d();	
	
	void vo_callback(const std_msgs::String::ConstPtr& msg);  //Change message type	

// Variables

	//  Indicate if this is the first image message. 
	bool  is_first_img; 
	
	//! ROS node handle.
	ros::NodeHandle& nodeHandle_;
  
	//! ROS topic subscriber.
	ros::Subscriber subscriber_;

	//! ROS topic name to subscribe to.
	std::string subscriber_topic_;
  
	//! ROS topic publisher.
	ros::Publisher publisher_;
	
	//! ROS topic name to publish to.
	std::string publisher_topic_;
	
	

};
