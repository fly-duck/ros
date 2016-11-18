#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "kobuki_msgs/CliffEvent.h"
#include "kobuki_msgs/BumperEvent.h"
#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value
#include <iostream>
#include <unistd.h>
#include <tf/transform_listener.h>

class TurtleMotions {
	public:
		// Construst a new TurtleMotions object and hook up this ROS node
		// to the simulated robot's velocity control and laser topics
		TurtleMotions(ros::NodeHandle& nh, int cmd, int val) :
			m_cmd(static_cast<CMD>(cmd)), m_val(val),
			fsm(FSM_MOVE_FORWARD),
			rotateStartTime(ros::Time::now()),
			moveStartTime(ros::Time::now()),
			rotateDuration(0.f) {
				// Initialize random time generator
				srand(time(NULL));
				// Advertise a new publisher for the simulated robot's velocity command topic
				// (the second argument indicates that if multiple command messages are in
				// the queue to be sent, only the last command will be sent)
				// DEBUG: for sim				commandPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
				//commandPub = nh.advertise<geometry_msgs::Twist>("follower_velocity_smoother/raw_cmd_vel", 1);
				commandPub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 1);
				// Subscribe to the simulated robot's laser scan topic and tell ROS to call
				// this->commandCallback() whenever a new message is published on that topic
				laserSub = nh.subscribe("scan", 1, &TurtleMotions::commandCallback, this);
				cliffSub = nh.subscribe("mobile_base/events/cliff", 1, &TurtleMotions::cliffSensorCallback, this);
				bumperSub = nh.subscribe("mobile_base/events/bumper", 1, &TurtleMotions::bumperCallback, this);

				poseSub_1 = nh.subscribe("odom", 1, \
						&TurtleMotions::poseCallback_1, this);
				poseSub_2 = nh.subscribe("odom_combined", 1, \
						&TurtleMotions::poseCallback_2, this);
			}

		// Send a velocity command
		void move(double linearVelMPS, double angularVelRadPS) {
			geometry_msgs::Twist msg; // The default constructor will set all commands to 0
			msg.linear.x = linearVelMPS;
			msg.angular.z = angularVelRadPS;
			commandPub.publish(msg);
		}

		void poseCallback_1(const nav_msgs::Odometry::ConstPtr& msg) {
			double roll, pitch;
std::cout << "JFDHLKHDJKHSFHFDFHD\n";
			x_1 = msg->pose.pose.position.x;
			y_1 = msg->pose.pose.position.y;
std::cout << x_1 << " -- " << y_1 << std::endl;
			// x = -msg->pose.pose.position.y;
			//y = msg->pose.pose.position.x;
			heading_1=tf::getYaw(msg->pose.pose.orientation);
		};

		void poseCallback_2(const nav_msgs::Odometry::ConstPtr& msg) {
std::cout << "IS THIS EVEN CALLED\n";
			double roll, pitch;
std::cout << x_2 << " -- " << y_2 << std::endl;
			x_2 = msg->pose.pose.position.x;
			y_2 = msg->pose.pose.position.y;
			// x = -msg->pose.pose.position.y;
			//y = msg->pose.pose.position.x;
			heading_2=tf::getYaw(msg->pose.pose.orientation);
		};

		void translate(double distance) {
			ros::Duration time = ros::Duration(distance / FORWARD_SPEED_MPS) ;
			double prev_x_1 = x_1;
			double prev_y_1 = y_1;
			double prev_x_2 = x_2;
			double prev_y_2 = y_2;
			double odom_d = 0.0;
			double odom_comb_d = 0.0;
			while(ros::Time::now() - moveStartTime < time) {
				move(FORWARD_SPEED_MPS, 0);
				odom_d += sqrt(pow(prev_x_1 - x_1, 2) + pow(prev_y_1 - y_1, 2));
				odom_comb_d += sqrt(pow(prev_x_2 - x_2, 2) + pow(prev_y_2 - y_2, 2));
			/*	prev_x_1 = x_1;
				prev_y_1 = y_1;
				prev_x_2 = x_2;
				prev_y_2 = y_2;*/
			}
				odom_d = sqrt(pow(prev_x_1 - x_1, 2) + pow(prev_y_1 - y_1, 2));
				odom_comb_d = sqrt(pow(prev_x_2 - x_2, 2) + pow(prev_y_2 - y_2, 2));
			std::cout << "\nodom Estimate : " << odom_d << std::endl;
			std::cout << "\nodom_combined Estimate : " << odom_comb_d << std::endl;
			move(0, 0);
		}
		
		void rotate_rel(double angle) {
			ros::Duration time = ros::Duration(abs(angle / ROTATE_SPEED_RADPS));
			int i = (ROTATE_SPEED_RADPS > 0) ? 1 : -1;
			while(ros::Time::now() - rotateStartTime < time) {
				move(0, i*ROTATE_SPEED_RADPS);
			}
		}
		
		void rotate_abs(double angle) {
		}

		// Process the incoming laser scan message
		void commandCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
			if (fsm == FSM_MOVE_FORWARD) {
				// Compute the average range value between MIN_SCAN_ANGLE and	MAX_SCAN_ANGLE
				//
				// NOTE: ideally, the following loop should have additional checks to ensure
				// that indices are not out of bounds, by computing:
				//
				// - currAngle = msg->angle_min + msg->angle_increment*currIndex
				//
				// and then ensuring that currAngle <= msg->angle_max
				unsigned int minIndex = ceil((MIN_SCAN_ANGLE_RAD - msg->angle_min) / msg->angle_increment);
				unsigned int maxIndex = ceil((MAX_SCAN_ANGLE_RAD - msg->angle_min) / msg->angle_increment);
				float closestRange = msg->ranges[minIndex];
				if(closestRange > msg->range_min && closestRange < msg->range_max/2) {
					for (unsigned int currIndex = minIndex + 1; currIndex < maxIndex; currIndex++) {
						if (msg->ranges[currIndex] < closestRange) {
							closestRange = msg->ranges[currIndex];
						}
					}
					ROS_INFO_STREAM("Range: " << closestRange);
					// TODO: if range is smaller than PROXIMITY_RANGE_M, update fsm and rotateStartTime,
					// and also choose a reasonable rotateDuration (keeping in mind of the value
					// of ROTATE_SPEED_RADPS)
					//
					// HINT: you can obtain the current time by calling:
					//
					// - ros::Time::now()
					//
					// HINT: you can set a ros::Duration by calling:
					//
					// - ros::Duration(DURATION_IN_SECONDS_FLOATING_POINT)
					//
					// HINT: you can generate a random number between 0 and 99 by calling:
					//
					// - rand() % 100
					//
					// see http://www.cplusplus.com/reference/clibrary/cstdlib/rand/ for more details
					/////////////////////// ANSWER CODE BEGIN ///////////////////
					std::cout << closestRange << "--\n";
					std::cout << PROXIMITY_RANGE_M << "--\n";
					std::cout << "fsm not is spin  -> " << fsm << "--\n";

					if(closestRange < PROXIMITY_RANGE_M) {
						fsm = FSM_ROTATE;
					}

					/////////////////////// ANSWER CODE END ///////////////////
				}
			}
		}
		void  cliffSensorCallback(const kobuki_msgs::CliffEventConstPtr msg) {
			if (fsm == FSM_MOVE_FORWARD && msg->state == kobuki_msgs::CliffEvent::CLIFF)
			{
				fsm = FSM_ROTATE;
			}
		}

		void  bumperCallback(const kobuki_msgs::BumperEventConstPtr msg) {
			if (fsm == FSM_MOVE_FORWARD && msg->state == kobuki_msgs::BumperEvent::PRESSED)
			{
				fsm = FSM_ROTATE;
			}
		}

		// Main FSM loop for ensuring that ROS messages are
		// processed in a timely manner, and also for sending
		// velocity controls to the simulated robot based on the FSM state
		void spin() {
			double dir = (rand()%2 == 0) ? -1 : 1;
			std::cout << "direction  " << dir << "--\n"; 
			ros::Rate rate(10); // Specify the FSM loop rate in Hz
			while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C

				if(m_cmd == MOVE) {
					moveStartTime = ros::Time::now();
					translate(m_val);
				}
				break;


				/*******Random walk behaviour*******/
				// TODO: Either call:
				//
				// - move(0, ROTATE_SPEED_RADPS); // Rotate right
				//
				// or
				//
				// - move(FORWARD_SPEED_MPS, 0); // Move foward
				//
				// depending on the FSM state; also change the FSM state when appropriate
				/////////////////////// ANSWER CODE BEGIN ///////////////////
				std::cout << "fsm in spin -> " << fsm << "--\n";
				if(fsm == FSM_MOVE_FORWARD) {
					//move(FORWARD_SPEED_MPS, 0);
				} else {
					rotateStartTime = ros::Time::now();
					rotateDuration = ros::Duration( (1 + rand()%5 ) /2 ); //to rotate in [ pi/4 , 3*Pi/2] range
					std::cout << "Duration: " << rotateDuration << std::endl;
					while(ros::Time::now() - rotateStartTime < rotateDuration) {
						move(0, dir * ROTATE_SPEED_RADPS); // Rotate right (FIXME:or left)
						// move(0, ROTATE_SPEED_RADPS);
						fsm = FSM_MOVE_FORWARD;
					}
					/////////////////////// ANSWER CODE END ///////////////////
				}
				ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
				rate.sleep(); // Sleep for the rest of the cycle, to enforce the FSM loop rate
			}
		}

		enum FSM {FSM_MOVE_FORWARD, FSM_ROTATE};
		enum CMD {MOVE, ROTATE_REL, ROTATE_ABS};

		// Tunable parameters
		// TODO: tune parameters as you see fit
		const static double MIN_SCAN_ANGLE_RAD = -15.0/180*M_PI;
		const static double MAX_SCAN_ANGLE_RAD = +15.0/180*M_PI;
		const static float PROXIMITY_RANGE_M = 0.75; // Should be smaller than sensor_msgs::LaserScan::range_max
		const static double FORWARD_SPEED_MPS = 0.2;
		const static double ROTATE_SPEED_RADPS = M_PI/4;


	protected:

		ros::Publisher commandPub; // Publisher to the simulated robot's velocity command topic
		ros::Subscriber laserSub; // Subscriber to the simulated robot's laser scan topic
		ros::Subscriber cliffSub; // Subscriber to the simulated robot's cliff sensor topic
		ros::Subscriber bumperSub; // Subscriber to the simulated robot's cliff sensor topic

		ros::Subscriber poseSub_1; // Subscriber to the current robot's ground truth pose topic
		ros::Subscriber poseSub_2; // Subscriber to the current robot's ground truth pose topic

		double x_1; // in simulated Stage units, + = East/right
		double y_1; // in simulated Stage units, + = North/up
		double heading_1; // in radians, 0 = East (+x dir.), pi/2 = North (+y dir.)

		double x_2; // in simulated Stage units, + = East/right
		double y_2; // in simulated Stage units, + = North/up
		double heading_2; // in radians, 0 = East (+x dir.), pi/2 = North (+y dir.)

		enum CMD m_cmd;
		double m_val;

		enum FSM fsm; // Finite state machine for the random walk algorithm
		ros::Time rotateStartTime; // Start time of the rotation
		ros::Time moveStartTime; // Start time of the rotation
		ros::Duration rotateDuration; // Duration of the rotation

};

int main(int argc, char **argv) {

	bool printUsage = false;
	int cmd = -1;
	int val = -1;

	//--> Parse and validate input arguments
	if (argc <= 2) {
		printUsage = true;
	} else {
		try {
			cmd = boost::lexical_cast<int>(argv[1]);
			val = boost::lexical_cast<int>(argv[2]);

			if (cmd < 0) { printUsage = true; }
			//else if (val <= 0) { printUsage = true; }
		} catch (std::exception err) {
			printUsage = true;
		}
	}
	if (printUsage) {
		std::cout << "Usage: " << argv[0] << " [MOVE_COMMAND] [DISTANCE_OR_ANGLE]" << std::endl;
		std::cout << "the possible values of [MOVE_COMMAND] are:\n";
		std::cout << "0 - for moving forward\n";
		std::cout << "1 - calls rotate_rel function\n";
		std::cout << "2 - calls rotate_abs function\n";

		return EXIT_FAILURE;
	}
	//<---



	ros::init(argc, argv, "turtle_motions"); // Initiate new ROS node named "turtle_motions"
	ros::NodeHandle n;
	TurtleMotions walker(n, cmd, val);
	walker.spin(); // Execute FSM loop
	return 0;
}
