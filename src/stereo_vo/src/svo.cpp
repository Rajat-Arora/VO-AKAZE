#include  <ros/ros.h> 
#include "stereo_vo/svo.h"



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

bool svo::create_ros_io(){

	cam0_img_sub_.subscribe(node_handle_, cam0_topic_ ,  10); 
  	cam1_img_sub_.subscribe(node_handle_,  cam1_topic_ ,  10); 
	
	stereo_sub_.connectInput(cam0_img_sub_, cam1_img_sub_); 
	stereo_sub_.registerCallback(&svo::vo_callback,  this);
	
	//  publisher for VO 
	vo_odom_pub_ = node_handle_. advertise <nav_msgs::Odometry>( vo_odom_topic_ ,  10 ); 


	//  publisher for path 
	path_pub_ = node_handle_. advertise <nav_msgs::Path>( path_topic_ ,  10 ); 

return true;
	
}


svo::svo(ros::NodeHandle& nodeHandle)
	: node_handle_(nodeHandle)
{
	svo::is_first_img = true;

	if (!svo::load_parameters()) {
    	ROS_ERROR("Could not read parameters.");
    	ros::requestShutdown();
  	}

	if (!svo::create_ros_io()) {
		ROS_ERROR("Could not create ROS IO.");
        ros::requestShutdown();
	}
  
}



void svo::vo_callback(const sensor_msgs::ImageConstPtr& cam0_img, const sensor_msgs::ImageConstPtr& cam1_img){

	svo::cam0_curr_img_ptr_ = cv_bridge::toCvShare(cam0_img, sensor_msgs::image_encodings::MONO8);
  	svo::cam1_curr_img_ptr_ = cv_bridge::toCvShare(cam1_img, sensor_msgs::image_encodings::MONO8);

	if (svo::is_first_img) {
    	
		svo::initialize_first_frame();
    	svo::is_first_img = false;
	}
	else{
		
		cv::Mat img_L = cam0_curr_img_ptr_->image;
		cv::Mat img_R = cam1_curr_img_ptr_->image;
		
		svo::rectify(img_L, img_R);

		std::vector<cv::KeyPoint> kpL_matched;
    	std::vector<cv::KeyPoint> kpR_matched;
    	cv::Mat descriptorL;
		std::vector<cv::KeyPoint> kpL_next;
 	    std::vector<cv::KeyPoint> kpR_next;
	    cv::Mat desL_next;

		svo::find_feature_matches(img_L, img_R, kpL_matched, kpR_matched, descriptorL); //Find matches between previous frame.

		kpL_next = kpL_matched;
		kpR_next = kpR_matched;
		desL_next = descriptorL;

        //Find matches between previous and current frame.
        std::vector<cv::DMatch> matches_prefinal, matches_final;
   		svo::matcher_->match (desL_prev_, desL_next, matches_prefinal, cv::Mat()); 
        
        
  		double  min_dist =  10000 , max_dist =  0 ; 

  		// Find the minimum and maximum distances between all matches, that is, the distances between the most similar and least similar two sets of points 
  		for  ( int  i =  0 ; i < matches_prefinal.size() ; i++) { 
    		double  dist = matches_prefinal[i].distance ; 
    		if  (dist < min_dist) min_dist = dist; 
    		if  (dist > max_dist) max_dist = dist; 
  		} 


 	 // When the distance between descriptors is greater than twice the minimum distance, the matching is considered incorrect. But sometimes the minimum distance is very small, and an empirical value of 30 is set as the lower limit. 
  	for  ( int  i= 0 ; i< matches_prefinal.size() ; i++) { 
    	if  (matches_prefinal[i].distance  <=  std::max(2*min_dist, 30.0)) { 
      		matches_final.push_back (matches_prefinal[i]); 
    	} 
  	}

    std::vector<cv::Point2f> kpL_prev_prefinal, kpR_prev_prefinal, kpL_next_prefinal;

	 for(int i=0; i<matches_final.size();i++)
    {
     kpL_prev_prefinal.push_back(kpL_prev_[matches_final[i].queryIdx].pt);
     kpR_prev_prefinal.push_back(kpR_prev_[matches_final[i].queryIdx].pt);
     kpL_next_prefinal.push_back(kpL_next[matches_final[i].queryIdx].pt);
    }

    // Set previous to current frame for next iteration
    kpL_prev_ =  kpL_next;
    kpR_prev_ =  kpR_next;
    desL_prev_ = desL_next;


    cv::Mat A,A1,rvec,tvec,_R,k1,k2,k3,pos;
    cv::triangulatePoints(P0_,P1_,kpL_prev_prefinal,kpR_prev_prefinal,A);
    cv::convertPointsFromHomogeneous(A.t(),A1);

    cv::decomposeProjectionMatrix(P0_,k1,k2,k3);
    cv::solvePnPRansac(A1,kpL_next_prefinal,k1,cv::noArray(),rvec,tvec,false,145,1.0,0.99,cv::noArray(),cv::SOLVEPNP_ITERATIVE);
 
    Rodrigues(rvec,_R);
    voR = _R * voR;
    voT = _R * voT + tvec;
    pos_n = -voR.t() * voT; //translation of current w.r.t initial


  //  cv::Mat rot; // rvec_initial_frame;
  //  rot = voR.t(); //rotation matrix w.r.t initial

 //   Matrix3f rot_eigen;
  //  cv2eigen(rot,rot_eigen);
  //  Vector3f rvec_initial_frame = rot_eigen.eulerAngles(2,1,0);
  //  pos_n = pos; //current w.r.t initial imu frame
 // T_vo_w.setOrigin(tf::Vector3(pos_n.at<double>(0,0),pos_n.at<double>(1,0),pos_n.at<double>(2,0)));
//  quat_rvec.setRPY(rvec_initial_frame[0],rvec_initial_frame[1],rvec_initial_frame[2]);
//  quat_rvec.normalize();
//  T_vo_w.setRotation(quat_rvec);



}

	return;
}


void svo::rectify(cv::Mat imgL, cv::Mat imgR){

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


void svo::find_feature_matches(const cv::Mat& img_1, const cv::Mat& img_2, 
                          std::vector<cv::KeyPoint>& kpL_matched, 
                          std::vector<cv::KeyPoint>& kpR_matched, 
                          cv::Mat& descriptorL){

	std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
	cv::Mat descriptors_1, descriptors_2; 
	 
  	// -- Step 1: Detect Oriented FAST corner positions and Calculate the BRIEF descriptor based on the corner position 

	svo::orb_->detectAndCompute(img_1, cv::noArray(), keypoints_1, descriptors_1);
 	svo::orb_->detectAndCompute(img_2, cv::noArray(), keypoints_2, descriptors_2); 

	// -- Step 2: Match the Brief descriptors in the two images, using the Hamming distance 
  	std::vector<cv::DMatch> match, matches; 
	svo::matcher_->match (descriptors_1, descriptors_2, match, cv::Mat()); 

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
    	if  (match[i].distance  <=  std::max(2*min_dist, 30.0)) { 
      	matches.push_back (match[i]); 
    	} 
  	}

	 for(int i=0; i<matches.size();i++)
    {
     kpL_matched.push_back(keypoints_1[matches[i].queryIdx].pt);
     kpR_matched.push_back(keypoints_2[matches[i].trainIdx].pt);
     descriptorL.push_back(descriptors_1.row(matches[i].queryIdx));
    }

	return;
 
}




void svo::initialize_first_frame(){

	 cv::Mat img_L = cam0_curr_img_ptr_->image;
	 cv::Mat img_R = cam1_curr_img_ptr_->image;
	  
	svo::rectify(img_L, img_R);

	std::vector<cv::KeyPoint> kpL_matched;
    std::vector<cv::KeyPoint> kpR_matched;
    cv::Mat descriptorL;

	svo::find_feature_matches(img_L, img_R, kpL_matched, kpR_matched, descriptorL);

	kpL_prev_ = kpL_matched;
	kpR_prev_ = kpR_matched;
	desL_prev_ = descriptorL;
	
	return;
}


cv::Mat  svo::getVec16Transform ( const  ros::NodeHandle &nh, 
                          const  std::string &field){
	
	std::vector<double> v;
  	nh.getParam(field, v);
  	if (v.size() != 16) {
    	throw std::runtime_error("invalid vec16!");
  	}
  	cv::Mat T = cv::Mat(v).clone().reshape(1, 4); // one channel 4 rows
  	return T;
}

cv::Mat svo::getKalibrStyleTransform(const ros::NodeHandle &nh,
             	                   const std::string &field) {
  cv::Mat T = cv::Mat::eye(4, 4, CV_64FC1);
  XmlRpc::XmlRpcValue lines;
  if (!nh.getParam(field, lines)) {
    throw (std::runtime_error("cannot find transform " + field));
  }
  if (lines.size() != 4 || lines.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    throw (std::runtime_error("invalid transform " + field));
  }
  for (int i = 0; i < lines.size(); i++) {
    if (lines.size() != 4 || lines.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      throw (std::runtime_error("bad line for transform " + field));
    }
    for (int j = 0; j < lines[i].size(); j++) {
      if (lines[i][j].getType() != XmlRpc::XmlRpcValue::TypeDouble) {
        throw (std::runtime_error("bad value for transform " + field));
      } else {
        T.at<double>(i,j) = static_cast<double>(lines[i][j]);
      }
    }
  }
  return T;
}



cv::Mat  svo::getTransformCV ( const  ros::NodeHandle &nh, 
                       const  std::string &field) { 
  
	cv::Mat T; 
  	try  { 
    //  first try reading kalibr format 
  	T =  getKalibrStyleTransform(nh, field); 
  	}  catch  (std::runtime_error &e) { 
   	//  maybe it's the old style format? 
    ROS_DEBUG_STREAM ( " cannot read transform  "  << field 
                     <<  "  in kalibr format, trying old one! " ); 
    try  { 
      T =  getVec16Transform (nh, field); 
    }  catch  (std::runtime_error &e) { 
      std::string msg =  " cannot read transform  "  + field +  "  error:  "  + e.what(); 
      ROS_ERROR_STREAM (msg); 
      throw  std::runtime_error (msg); 
    } 
  } 
  return  T; 
}



 
 
