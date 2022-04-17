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

void svo::rectify(cv::Mat& imgL, cv::Mat& imgR){

	cv::Mat M1L, M2L, M1R, M2R;
	cv::initUndistortRectifyMap(K0_, D0_, R0_, p0_, cv::Size2d(752,480), CV_32FC1, M1L, M2L);
	cv::initUndistortRectifyMap(K1_, D1_, R1_, p1_, cv::Size2d(752,480), CV_32FC1, M1R, M2R);

	cv::Mat rectified_imgL, rectified_imgR;

	cv::remap(imgL, rectified_imgL, M1L, M2L, cv::INTER_LINEAR);
	cv::remap(imgR, rectified_imgR, M1R, M2R, cv::INTER_LINEAR);

	imgL = rectified_imgL;
	imgR = rectified_imgR;

	return;
}


void find_feature_matches(const  cv::Mat &img_1,  const  cv::Mat &img_2, 
                          std::vector<KeyPoint> &kpL_matched, 
                          std::vector<KeyPoint> &kpR_matched, 
                          std::vector<DMatch> &descriptorL){

	std::vector<KeyPoint> keypoints_1, keypoints_2;
	cv::Mat descriptors_1, descriptors_2; 
	 
  	// -- Step 1: Detect Oriented FAST corner positions and Calculate the BRIEF descriptor based on the corner position 

	svo::orb_->detectAndCompute(img_1, noArray(), keypoints1, descriptors_1);
 	svo::orb_->detectAndCompute(img_2, noArray(), keypoints2, descriptors_2); 

	// -- Step 2: Match the Brief descriptors in the two images, using the Hamming distance 
  	vector<DMatch> match, matches; 
	svo::matcher_->match (descriptors_1, descriptors_2, match, Mat()); 

  	// -- Step 4: Match point pair filtering 
  	double  min_dist =  10000 , max_dist =  0 ; 

  	// Find the minimum and maximum distances between all matches, that is, the distances between the most similar and least similar two sets of points 
  	for  ( int  i =  0 ; i < descriptors_1.rows ; i++) { 
    	double  dist = match[i].distance ; 
    	if  (dist < min_dist) min_dist = dist; 
    	if  (dist > max_dist) max_dist = dist; 
  	} 


 	 // When the distance between descriptors is greater than twice the minimum distance, the matching is considered incorrect. But sometimes the minimum distance is very small, and an empirical value of 30 is set as the lower limit. 
  	for  ( int  i= 0 ; i< descriptors_1.rows ; i++) { 
    	if  (match[i].distance  <=  max ( 2  * min_dist,  30.0 )) { 
      	matches.push_back (match[i]); 
    	} 
  	}
	
	 for(int i=0; i<matches.size();i++)
    {
     kpL_matched.push_back(kpts1[matches[i].queryIdx].pt);
     kpR_matched.push_back(kpts2[matches[i].trainIdx].pt);
     descriptorL.push_back(desc1.row(matches[i].queryIdx));
    }

	return;
 
}




void svo::initialize_first_frame(){

	const Mat& img_L = cam0_curr_img_ptr->image;
	const Mat& img_R = cam1_curr_img_ptr->image;
	  
	svo::rectify(img_L, img_R);

	std::vector<KeyPoint> kpL_matched;
    std::vector<KeyPoint> kpR_matched;
    std::vector<DMatch> descriptorL;

	svo::find_feature_matches(img_L, img_R, kpL_matched, kpR_matched, descriptorL);

	kpL_prev_ = kpL_matched;
	kpR_prev_ = kpR_matched;
	desL_prev_ = descriptorL;
	
	return;
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



 
 
