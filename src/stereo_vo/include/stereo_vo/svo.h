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

	bool create_ros_io();   //almost done

	void rectify();
	
    void initialize_first_frame();
		
	bool find_feature_matches();
	
	void pose_estimation_3d2d();	
	
	void vo_callback(const  sensor_msgs::ImageConstPtr& cam0_img, const  sensor_msgs::ImageConstPtr& cam1_img);	


//------------------- Variables -------------------------------

	//  Indicate if this is the first image message. 
	bool  is_first_img; 
	
	//! ROS node handle.
	ros::NodeHandle& node_handle_;
  
	message_filters::Subscriber<sensor_msgs::Image> cam0_img_sub_; 
	
	message_filters::Subscriber<sensor_msgs::Image> cam1_img_sub_; 
  
	message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> stereo_sub_;

    
	//! ROS topic subscriber.
//	ros::Subscriber subscriber_;

	//! ROS topic name to subscribe to.
	std::string cam0_topic_;

	std::string cam1_topic_;
  
	
	ros::Publisher vo_odom_pub_;
  	ros::Publisher path_pub_;
	
    //! ROS topic name to publish to.
	std::string vo_odom_topic_;

	std::string path_topic_;
	
	//VO Variables
	cv::Mat voR;
	cv::Mat voT;
  	cv::Mat pos_n;
  	tf::Quaternion quat_rvec;

  	//tf
  	tf::TransformBroadcaster tf_pub_vo;

  	//VO w.r.t world
  	tf::Transform T_vo_w;

  	//Display path
  	nav_msgs::Path path_msg;
  	geometry_msgs::PoseStamped poseStamped_msg;	



};
