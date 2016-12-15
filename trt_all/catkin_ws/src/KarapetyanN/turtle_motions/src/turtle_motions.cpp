#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "kobuki_msgs/CliffEvent.h"
#include "kobuki_msgs/BumperEvent.h"
#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value
#include <iostream>
#include <unistd.h>
#include <tf/transform_listener.h>


#define DEBUG

class TurtleMotions {
	public:
		// Construst a new TurtleMotions object and hook up this ROS node
		// to the simulated robot's velocity control and laser topics
		TurtleMotions(ros::NodeHandle& nh, int cmd, double val) :
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
				// commandPub = nh.advertise<geometry_msgs::Twist>("follower_velocity_smoother/raw_cmd_vel", 1);
				commandPub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 1);
				//commandPub = nh.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1);

				// Subscribe to the simulated robot's laser scan topic and tell ROS to call
				// this->commandCallback() whenever a new message is published on that topic
				laserSub = nh.subscribe("scan", 1, &TurtleMotions::commandCallback, this);
				cliffSub = nh.subscribe("mobile_base/events/cliff", 1, &TurtleMotions::cliffSensorCallback, this);
				bumperSub = nh.subscribe("mobile_base/events/bumper", 1, &TurtleMotions::bumperCallback, this);

				poseSub_1 = nh.subscribe("odom", 1, &TurtleMotions::poseCallback, this);
				poseSub_2 = nh.subscribe("robot_pose_ekf/odom_combined", 1, &TurtleMotions::poseCallback_comb, this);
			}

		// Send a velocity command
		void move(double linearVelMPS, double angularVelRadPS) {
			geometry_msgs::Twist msg; // The default constructor will set all commands to 0
			msg.linear.x = linearVelMPS;
			msg.angular.z = angularVelRadPS;
			commandPub.publish(msg);
		}

		//FIXME: this could be replaced into separate srtuct
		void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
			double roll, pitch;
			x = msg->pose.pose.position.x;
			y = msg->pose.pose.position.y;
#ifdef DEBUG_1
			std::cout << "Odom x " << x << " -- y " << y << std::endl;
#endif
			// x = -msg->pose.pose.position.y;
			//y = msg->pose.pose.position.x;
			heading=tf::getYaw(msg->pose.pose.orientation);
called = true;
			std::cout << ros::Time::now() << " Odom x " << x << " -- y " << y  << " -- heading " << heading << std::endl;
		};

		//FIXME: this could be replaced into separate srtuct
		void poseCallback_comb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
			double roll, pitch;
#ifdef DEBUG_1
			std::cout << x_comb << " -- " << y_comb << std::endl;
#endif
			x_comb = msg->pose.pose.position.x;
			y_comb = msg->pose.pose.position.y;
			// x = -msg->pose.pose.position.y;
			//y = msg->pose.pose.position.x;
			heading_comb=tf::getYaw(msg->pose.pose.orientation);
		};

		void translate(double distance) {
			ros::Rate rate(10); // Specify the FSM loop rate in Hz

			double odom_d = 0.0;
			double odom_comb_d = 0.0;
			double prev_x = x;
			double prev_y = y;

			double prev_x_comb = x_comb;
			double prev_y_comb = y_comb;

			ros::Duration time = ros::Duration(distance / FORWARD_SPEED_MPS) ;

			while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C
				if(ros::Time::now() - moveStartTime >= time) {
					move(0, 0);
					break;
				}
				move(FORWARD_SPEED_MPS, 0);
				ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
				rate.sleep(); // Sleep for the rest of the cycle, to enforce the FSM loop rate
			}

			odom_d = sqrt(pow((prev_x - x), 2) + pow((prev_y - y), 2));
			odom_comb_d = sqrt(pow((prev_x_comb - x_comb), 2) + pow((prev_y_comb - y_comb), 2));
			std::cout << "Printing the odometry information for translate: \n";
			std::cout << "commanded distance: " << distance << "\n";
			std::cout << "odom estimate: " <<  fabs(odom_d) << "\n"; 
			std::cout << "odom_comb estimate: " <<  fabs(odom_comb_d) << "\n"; 
		}


		//the angle is in degree not in radian
		void rotate_rel(double angle) {
#ifdef DEBUG
			std::cout << "\n*********roation starts**********\n";
			std::cout << ros::Time::now() << "ROTATE_REL -> " << angle << '\n';
#endif
			ros::Rate rate(10); // Specify the FSM loop rate in Hz
while(!called) {
				ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
				rate.sleep(); // Sleep for the rest of the cycle, to enforce the FSM loop rate
} 
			double odom_angle =0.0;
			double odom_comb_angle =0.0;
			double prev_h = heading;
			double prev_h_comb = heading_comb;
			std::cout << ros::Time::now() << "ROTATE_REL -> " << angle << '\n';

			ros::Duration time = ros::Duration(fabs(((angle * M_PI)/180) / ROTATE_SPEED_RADPS));
			double dir = (angle > 0) ? 1.0 : -1.0;
			while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C
				if(ros::Time::now() - rotateStartTime >= time) {
					move(0, 0);
					break;
				}
				move(0, dir*ROTATE_SPEED_RADPS);


				ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
				rate.sleep(); // Sleep for the rest of the cycle, to enforce the FSM loop rate
			}
#ifdef DEBUG
			std::cout << (heading*180)/M_PI <<std::endl;
			std::cout << (prev_h*180)/M_PI <<std::endl;
			std::cout << "\n*********roation ends**********\n";
#endif

			odom_angle = prev_h - heading;
			odom_comb_angle = prev_h_comb - heading_comb;
			std::cout << "Printing the odometry information for relative rotation: \n";
			std::cout << "commanded distance: " << angle << "\n";
			std::cout << "odom estimate: " <<  fabs((odom_angle * 180)/M_PI) << "\n"; 
			std::cout << "odom_comb estimate: " <<  fabs((odom_comb_angle*180)/M_PI) << "\n"; 
		}

		void rotate_abs(double angle) {
#ifdef DEBUG
			std::cout << "ROTATE_ABS -> " << angle << '\n';
#endif
			angle = heading - angle;
			angle = atan2(sin(angle), cos(angle));// to get an angle from [-pi/2, pi/2] angles
			rotate_rel((angle*180)/M_PI);
#ifdef DEBUG
			std::cout << "ROTATE_ABS normailized angle rotated -> " << angle << '\n';
#endif
		}

		void make_square(double side) {
			for(int i = 1; i <=4; i++) {
				translate(side);
				rotate_rel(90);
			}
		}

		// Process the incoming laser scan message
		void commandCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
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
		void start() {
                if(m_cmd == MOVE) {
                    moveStartTime = ros::Time::now();
                    translate(m_val);
                }

                if(m_cmd == ROTATE_REL) {
                    rotateStartTime = ros::Time::now();
#ifdef DEBUG
                    std::cout << "vbbbbbbbbbbbbbbbbbbbb=> " << m_val << std::endl;
#endif
                    rotate_rel(m_val);
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

		double x; // in simulated Stage units, + = East/right
		double y; // in simulated Stage units, + = North/up
		double heading; // in radians, 0 = East (+x dir.), pi/2 = North (+y dir.)

		double x_comb; // in simulated Stage units, + = East/right
		double y_comb; // in simulated Stage units, + = North/up
		double heading_comb; // in radians, 0 = East (+x dir.), pi/2 = North (+y dir.)

		enum CMD m_cmd;
		double m_val;

		enum FSM fsm; // Finite state machine for the random walk algorithm
		ros::Time rotateStartTime; // Start time of the rotation
		ros::Time moveStartTime; // Start time of the rotation
		ros::Duration rotateDuration; // Duration of the rotation

bool called  = false;

};

int main(int argc, char **argv) {

	bool printUsage = false;
	int cmd = -1;
	double val = -1.0;

	//--> Parse and validate input arguments
	if (argc <= 2) {
		printUsage = true;
	} else {
		try {
			cmd = boost::lexical_cast<int>(argv[1]);
			val = boost::lexical_cast<double>(argv[2]);
#ifdef DEBUG
			std::cout << "val======= " << val << std::endl;
#endif

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
	walker.start();
	return 0;
}
