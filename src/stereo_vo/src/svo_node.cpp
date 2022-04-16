#include  <ros/ros.h> 
#include "stereo_vo/svo.h"


int main(int argc, char** argv){
	
	ros::init(argc, argv, "stereo_vo");
	
	ros::NodeHandle nodeHandle("~");
	svo svo_obj(nodeHandle);

  	ros::spin();
  
	return 0;
} 
