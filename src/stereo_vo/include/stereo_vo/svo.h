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

	bool load_parameters(); // loaded all callibration parameters and topic names. Add topic names in config file

	bool create_ros_io();   //almost done check for tf, vo 

	void rectify(cv::Mat& imgL, cv::Mat& imgR); //complete
	
    void initialize_first_frame(); // Almost complete
		
	bool find_feature_matches(const  cv::Mat &img_1,  const  cv::Mat &img_2,        // Almost complete
                          std::vector<KeyPoint> &kpL_matched,
                          std::vector<KeyPoint> &kpR_matched,
                          std::vector<DMatch> &descriptorL);
	
	void pose_estimation_3d2d();	
	
	void vo_callback(const  sensor_msgs::ImageConstPtr& cam0_img, const  sensor_msgs::ImageConstPtr& cam1_img);	

	cv::Mat  getTransformCV ( const  ros::NodeHandle &nh, const  std::string &field)

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

	std::vector<KeyPoint> kpL_prev_;
    std::vector<KeyPoint> kpR_prev_;
    std::vector<DMatch> desL_prev_;


  	//tf
  	tf::TransformBroadcaster tf_pub_vo;

  	//VO w.r.t world
  	tf::Transform T_vo_w;

  	//Display path
  	nav_msgs::Path path_msg;
  	geometry_msgs::PoseStamped poseStamped_msg;	

	//VO Callibration variables to be loaded from Calibration file

	cv::Mat K0_, K1_, P0_, P1_, p0_, p1_, R0_, R1_, D0_, D1_; 

	
	//Image Pointers
	
	cv_bridge::CvImageConstPtr cam0_prev_img_ptr_;
  	cv_bridge::CvImageConstPtr cam0_curr_img_ptr_;
  	cv_bridge::CvImageConstPtr cam1_curr_img_ptr_;

	cv::Ptr<Feature2D> orb_ = ORB::create(1000);
	cv::Ptr<DescriptorMatcher> matcher_ = DescriptorMatcher::create("BruteForce-Hamming");
};
