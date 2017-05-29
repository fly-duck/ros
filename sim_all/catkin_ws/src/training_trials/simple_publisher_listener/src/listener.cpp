#include "ros/ros.h"
#include "std_msgs/String.h"


class Listener{
	public:
		Listener(ros::NodeHandle& n) {
			sub = n.subscribe("chatter", 1000, chatterCallback);
		}
		void static chatterCallback(const std_msgs::String::ConstPtr& msg) {
			ROS_INFO("I heard: [%s]", msg->data.c_str());
		}
		void spin() {
			ros::spin();
		}
	private:
		ros::Subscriber sub;
};

int main(int argc, char **argv)
{
	ros::NodeHandle n;
	Listener listener(n);
	listener.spin();
	return 0;
}
