#include  <ros/ros.h> 
#include "stereo_vo/svo.h"
#include <sensor_msgs/image_encodings.h>

bool svo::load_parameters(){

 
	std::vector<double> K0_temp(9);
    node_handle_.getParam("K0", K0_temp);
	K0_ = (cv::Mat_<double>(3, 3) << 
      K0_temp[0], K0_temp[1], K0_temp[2], 
      K0_temp[3], K0_temp[4], K0_temp[5], 
      K0_temp[6], K0_temp[7], K0_temp[8]);

	std::vector<double> K1_temp(9);
    node_handle_.getParam("K1", K1_temp);
	K1_ = (cv::Mat_<double>(3, 3) << 
      K1_temp[0], K1_temp[1], K1_temp[2], 
      K1_temp[3], K1_temp[4], K1_temp[5], 
      K1_temp[6], K1_temp[7], K1_temp[8]);

	std::vector<double> P0_temp(12);
    node_handle_.getParam("P0", P0_temp);
	P0_ = (cv::Mat_<double>(3, 4) << 
      P0_temp[0], P0_temp[1], P0_temp[2], P0_temp[3],
	  P0_temp[4], P0_temp[5], P0_temp[6], P0_temp[7], 
      P0_temp[8], P0_temp[9], P0_temp[10], P0_temp[11]);
    
	std::vector<double> P1_temp(12);
    node_handle_.getParam("P1", P1_temp);
	P1_ = (cv::Mat_<double>(3, 4) << 
      P1_temp[0], P1_temp[1], P1_temp[2], P1_temp[3],
	  P1_temp[4], P1_temp[5], P1_temp[6], P1_temp[7], 
      P1_temp[8], P1_temp[9], P1_temp[10], P1_temp[11]);

	std::vector<double> p0_temp(9);
    node_handle_.getParam("p0", p0_temp);
	p0_ = (cv::Mat_<double>(3, 3) << 
      p0_temp[0], p0_temp[1], p0_temp[2], 
      p0_temp[3], p0_temp[4], p0_temp[5], 
      p0_temp[6], p0_temp[7], p0_temp[8]);

   std::vector<double> p1_temp(9);
    node_handle_.getParam("p1", p1_temp);
	p1_ = (cv::Mat_<double>(3, 3) << 
      p1_temp[0], p1_temp[1], p1_temp[2], 
      p1_temp[3], p1_temp[4], p1_temp[5], 
      p1_temp[6], p1_temp[7], p1_temp[8]);

	std::vector<double> R0_temp(9);
    node_handle_.getParam("R0", R0_temp);
	R0_ = (cv::Mat_<double>(3, 3) << 
      R0_temp[0], R0_temp[1], R0_temp[2], 
      R0_temp[3], R0_temp[4], R0_temp[5], 
      R0_temp[6], R0_temp[7], R0_temp[8]);

	std::vector<double> R1_temp(9);
    node_handle_.getParam("R1", R1_temp);
	R1_ = (cv::Mat_<double>(3, 3) << 
      R1_temp[0], R1_temp[1], R1_temp[2], 
      R1_temp[3], R1_temp[4], R1_temp[5], 
      R1_temp[6], R1_temp[7], R1_temp[8]);

	std::vector<double> D0_temp(5);
    node_handle_.getParam("D0", D0_temp);
	D0_ = (cv::Mat_<double>(1, 5) << 
      D0_temp[0], D0_temp[1], D0_temp[2], D0_temp[3], D0_temp[4]);

	std::vector<double> D1_temp(5);
    node_handle_.getParam("D1", D1_temp);
	D1_ = (cv::Mat_<double>(1, 5) << 
      D1_temp[0], D1_temp[1], D1_temp[2], D1_temp[3], D1_temp[4]);

	node_handle_.getParam("cam0_topic", cam0_topic_);
	node_handle_.getParam("cam1_topic", cam1_topic_);
	node_handle_.getParam("vo_odom_topic", vo_odom_topic_);
	node_handle_.getParam("path_topic", path_topic_);
		
    ROS_INFO("cam0_topic: %s", cam0_topic_.c_str());
    ROS_INFO("cam1_topic: %s", cam1_topic_.c_str());
    ROS_INFO("vo_odom_topic: %s", vo_odom_topic_.c_str());
    ROS_INFO("path_topic: %s", path_topic_.c_str());
 
	myfile.open("/home/rajat/example_vo.txt");
   
return true;

}

bool svo::create_ros_io(){

	cam0_img_sub_.subscribe(node_handle_, cam0_topic_ ,  10); 
  	cam1_img_sub_.subscribe(node_handle_,  cam1_topic_ ,  10); 
	
	stereo_sub_.connectInput(cam0_img_sub_, cam1_img_sub_); 
	stereo_sub_.registerCallback(&svo::vo_callback,  this);
	
	//  publisher for VO 
	vo_odom_pub_ = node_handle_.advertise<nav_msgs::Odometry>(vo_odom_topic_ ,  10); 


	//  publisher for path 
	path_pub_ = node_handle_.advertise<nav_msgs::Path>(path_topic_ , 10); 

return true;
	
}


svo::svo(ros::NodeHandle& nd) : 
	node_handle_(nd),
    stereo_sub_(10),
	voR(cv::Mat::eye(3, 3, CV_64F)),
	rot(cv::Mat::eye(3, 3, CV_64F)),
    voT(cv::Mat::zeros(3, 1, CV_64F)),
    pos_n(cv::Mat::zeros(3, 1, CV_64F))
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
  return; 
}



void svo::vo_callback(const sensor_msgs::ImageConstPtr& cam0_img, const sensor_msgs::ImageConstPtr& cam1_img){

	svo::cam0_curr_img_ptr_ = cv_bridge::toCvShare(cam0_img, sensor_msgs::image_encodings::MONO8);
  	svo::cam1_curr_img_ptr_ = cv_bridge::toCvShare(cam1_img, sensor_msgs::image_encodings::MONO8);

	if (svo::is_first_img) {
      //  ROS_INFO("Check1");	
		svo::initialize_first_frame();
    	svo::is_first_img = false;
      //  ROS_INFO("%ld", kpL_prev_.size());
      //  ROS_INFO("%ld", kpR_prev_.size());
      //  ROS_INFO("%d", desL_prev_.size());
	}
	else{
	//	ROS_INFO("Check2");
		cv::Mat img_L = cam0_curr_img_ptr_->image;
		cv::Mat img_R = cam1_curr_img_ptr_->image;
		
		svo::rectify(img_L, img_R);

		std::vector<cv::Point2f> kpL_matched;
    	std::vector<cv::Point2f> kpR_matched;
    	cv::Mat descriptorL;
		std::vector<cv::Point2f> kpL_next;
 	    std::vector<cv::Point2f> kpR_next;
	    cv::Mat desL_next;

		svo::find_feature_matches(img_L, img_R, kpL_matched, kpR_matched, descriptorL); //Find matches between previous frame.

		kpL_next = kpL_matched;
		kpR_next = kpR_matched;
		desL_next = descriptorL;
        
      //  ROS_INFO("%ld", kpL_next.size());
      //  ROS_INFO("%ld", kpR_next.size());
        //Find matches between previous and current frame.
        std::vector<cv::DMatch> matches_prefinal, matches_final;
   		svo::matcher_->match(desL_prev_, desL_next, matches_prefinal, cv::Mat()); 
        
	//	ROS_INFO("Check2");
      //  ROS_INFO("%ld", kpL_next.size());
      //  ROS_INFO("%ld", kpR_next.size());
        
 


 		double  min_dist =  10000 , max_dist =  0 ; 

  		// Find the minimum and maximum distances between all matches, that is, the distances between the most similar and least similar two sets of points 
  		for  ( int  i =  0 ; i < matches_prefinal.size() ; i++) { 
    		double  dist = matches_prefinal[i].distance ; 
    		if  (dist < min_dist) min_dist = dist; 
    		if  (dist > max_dist) max_dist = dist; 
  		} 


 	 // When the distance between descriptors is greater than twice the minimum distance, the matching is considered incorrect. But sometimes the minimum distance is very small, and an empirical value of 30 is set as the lower limit. 
  	for  ( int  i= 0 ; i< matches_prefinal.size() ; i++) { 
    	if  (matches_prefinal[i].distance  <=  std::max(2.5*min_dist, 35.0)) { 
      		matches_final.push_back (matches_prefinal[i]); 
    	} 
  	}

    std::vector<cv::Point2f> kpL_prev_prefinal, kpR_prev_prefinal, kpL_next_prefinal;

	 for(int i=0; i<matches_final.size();i++)
    {
     kpL_prev_prefinal.push_back(kpL_prev_[matches_final[i].queryIdx]);
     kpR_prev_prefinal.push_back(kpR_prev_[matches_final[i].queryIdx]);
     kpL_next_prefinal.push_back(kpL_next[matches_final[i].trainIdx]);
    }

    // Set previous to current frame for next iteration
    kpL_prev_ =  kpL_next;
    kpR_prev_ =  kpR_next;
    desL_prev_ = desL_next;

    ROS_INFO("%ld",kpL_prev_prefinal.size());
    ROS_INFO("%ld",kpR_prev_prefinal.size());
    ROS_INFO("%ld",kpL_next_prefinal.size());
    
	//ROS_INFO("Check3");
    cv::Mat A,A1,rvec,tvec,_R,k1,k2,k3;
    cv::triangulatePoints(P0_,P1_,kpL_prev_prefinal,kpR_prev_prefinal,A);
    cv::convertPointsFromHomogeneous(A.t(),A1);
    cv::decomposeProjectionMatrix(P0_,k1,k2,k3);
  //  ROS_INFO("%d",A1.rows);
   // ROS_INFO("%d",A1.cols);

    cv::solvePnPRansac(A1,kpL_next_prefinal,k1,cv::noArray(),rvec,tvec,false,145,0.75,0.90,cv::noArray(),cv::SOLVEPNP_ITERATIVE);
    
//	ROS_INFO_STREAM(rvec);
    cv::Rodrigues(rvec,_R);
    voR = _R * voR;
    voT = _R * voT + tvec;
    rot = voR.t();
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
	svo::publish();
	return;
}


void svo::publish(){

  // Publishing VO Odometery
  nav_msgs::Odometry vo_odom_msg;
  vo_odom_msg.header.stamp = cam0_curr_img_ptr_->header.stamp; 
  vo_odom_msg.header.frame_id = "world";
  vo_odom_msg.child_frame_id = "vo";

  // Add data
  Eigen::Matrix3f rot_eigen; 
  cv2eigen(rot,rot_eigen);
  Eigen::Vector3f rvec_initial_frame = rot_eigen.eulerAngles(2,1,0);
  tf::Quaternion quat_rot;
  quat_rot.setRPY(rvec_initial_frame[0],rvec_initial_frame[1],rvec_initial_frame[2]);
  quat_rot.normalize();
  
  T_vo_w.setRotation(quat_rot);
  T_vo_w.setOrigin(tf::Vector3(pos_n.at<double>(1,0),pos_n.at<double>(1,0),pos_n.at<double>(2,0)));

  vo_odom_msg.pose.pose.position.x = pos_n.at<double>(0,0);
  vo_odom_msg.pose.pose.position.y = pos_n.at<double>(1,0);
  vo_odom_msg.pose.pose.position.z = pos_n.at<double>(2,0);
  vo_odom_msg.pose.pose.orientation.x = quat_rot.x();
  vo_odom_msg.pose.pose.orientation.y = quat_rot.y();
  vo_odom_msg.pose.pose.orientation.z = quat_rot.z();
  vo_odom_msg.pose.pose.orientation.w = quat_rot.w();
  vo_odom_pub_.publish(vo_odom_msg);


	cv::Mat pose;
	cv::hconcat(rot, pos_n, pose);
    pose = pose.reshape(0,1);
 
    for(int m=0;m<12;m++)
   		 myfile<<pose.at<double>(m)<<" ";
    myfile<<"\n";   




  tf_pub_vo.sendTransform(tf::StampedTransform(T_vo_w, cam0_curr_img_ptr_->header.stamp, "world", "vo"));

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


void svo::find_feature_matches(const cv::Mat& img_1, const cv::Mat& img_2, 
                          std::vector<cv::Point2f>& kpL_matched, 
                          std::vector<cv::Point2f>& kpR_matched, 
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
    	if  (match[i].distance  <=  std::max(2.5*min_dist, 35.0)) { 
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

	std::vector<cv::Point2f> kpL_matched;
    std::vector<cv::Point2f> kpR_matched; 
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



 
 
