#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <iomanip>


void poseCallBack(const turtlesim::Pose& msg) {
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed \
		<< "pose = (" << msg.x << ", " << msg.y <<")" \
		<< " dir = " << msg.theta); 
}

int main(int argc, char** argv) 
{
	ros::init(argc, argv, "subscribe_to_pose");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("turtle/pose1", 1000, &poseCallBack);
	ros::spin();
	return 0;
}

