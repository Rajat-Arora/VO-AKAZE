#include  <ros/ros.h> 
#include "stereo_vo/svo.h"



svo::svo(ros::NodeHandle& nodeHandle)
	: node_handle_(nodeHandle)
{
	if (!load_parameters()) {
    	ROS_ERROR("Could not read parameters.");
    	ros::requestShutdown();
  	}

	if (!create_ros_io()) {
		ROS_ERROR("Could not create ROS IO.");
        ros::requestShutdown();
	}
  
}


svo::



svo::create_ros_io(){

	cam0_img_sub_.subscribe(node_handle_, cam0_topic_ ,  10); 
  	cam1_img_sub_.subscribe(node_handle_,  cam1_topic_ ,  10); 
	
	stereo_sub_.connectInput(cam0_img_sub_, cam1_img_sub_); 
	stereo_sub_.registerCallback(&svo::vo_callback,  this);
	
	//  publisher for VO 
	vo_odom_pub = nh. advertise <nav_msgs::Odometry>( vo_odom_topic_ ,  10 ); 

	//  publisher for path 
	path_pub = nh. advertise <nav_msgs::Path>( path_topic_ ,  10 ); 
	
}
 
