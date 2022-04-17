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


bool svo::load_parameters(){

	K0_ =  svo::getTransformCV(node_handle_, "K0");	
	K1_ =  svo::getTransformCV(node_handle_, "K1");	
	P0_ =  svo::getTransformCV(node_handle_, "P0");	
	P1_ =  svo::getTransformCV(node_handle_, "P1");	
	p0_ =  svo::getTransformCV(node_handle_, "p0");	
	p1_ =  svo::getTransformCV(node_handle_, "p1");	
	R0_ =  svo::getTransformCV(node_handle_, "R0");	
	R1_ =  svo::getTransformCV(node_handle_, "R1");	
	D0_ =  svo::getTransformCV(node_handle_, "D0");	
	D1_ =  svo::getTransformCV(node_handle_, "D1");

	node_handle_.getParam("cam0_topic", cam0_topic_);
	node_handle_.getParam("cam1_topic", cam1_topic_);
	node_handle_.getParam("vo_odom_topic", vo_odom_topic_);
	node_handle_.getParam("path_topic", path_topic_);
		
return true;

}



void svo::create_ros_io(){

	cam0_img_sub_.subscribe(node_handle_, cam0_topic_ ,  10); 
  	cam1_img_sub_.subscribe(node_handle_,  cam1_topic_ ,  10); 
	
	stereo_sub_.connectInput(cam0_img_sub_, cam1_img_sub_); 
	stereo_sub_.registerCallback(&svo::vo_callback,  this);
	
	//  publisher for VO 
	vo_odom_pub_ = nh. advertise <nav_msgs::Odometry>( vo_odom_topic_ ,  10 ); 

	//  publisher for path 
	path_pub_ = nh. advertise <nav_msgs::Path>( path_topic_ ,  10 ); 

return;
	
}



void svo::vo_callback(const sensor_msgs::ImageConstPtr& cam0_img, const sensor_msgs::ImageConstPtr& cam1_img){

	cam0_curr_img_ptr = cv_bridge::toCvShare(cam0_img, sensor_msgs::image_encodings::MONO8);
  	cam1_curr_img_ptr = cv_bridge::toCvShare(cam1_img, sensor_msgs::image_encodings::MONO8);

	if (is_first_img) {
    	
		initializeFirstFrame();
    	is_first_img = false;
	}
	else{
	
		

	}


}



void svo::initialize_first_frame(){

	  const Mat& img_L = cam0_curr_img_ptr->image;
	  const Mat& img_R = cam1_curr_img_ptr->image;
	  

}





cv::Mat  svo::getTransformCV ( const  ros::NodeHandle &nh, 
                       const  std::string &field) { 
  
	cv::Mat T; 
  	try  { 
    //  first try reading kalibr format 
  	T =  getKalibrStyleTransform (nh, field); 
  	}  catch  (std::runtime_error &e) { 
   	//  maybe it's the old style format? 
    ROS_DEBUG_STREAM ( " cannot read transform  "  << field 
                     <<  "  in kalibr format, trying old one! " ); 
    try  { 
      T =  getVec16Transform (nh, field); 
    }  catch  (std::runtime_error &e) { 
      std::string msg =  " cannot read transform  "  + field +  "  error:  "  + e. what (); 
      ROS_ERROR_STREAM (msg); 
      throw  std::runtime_error (msg); 
    } 
  } 
  return  T; 
}



 
 
