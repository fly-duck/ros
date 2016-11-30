#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>
#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value
#include <cmath>
#include <bullet/LinearMath/btQuaternion.h> // Needed to convert rotation ...
#include <bullet/LinearMath/btMatrix3x3.h>  // ... quaternion into Euler angles

#include <boost/thread/mutex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_listener.h>

//typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct Pose {
  double x; // in simulated Stage units
  double y; // in simulated Stage units
  double heading; // in radians
  ros::Time t; // last received time
  
  // Construct a default pose object with the time set to 1970-01-01
  Pose() : x(0), y(0), heading(0), t(0.0) {};
  
  // Process incoming pose message for current robot
  void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    double roll, pitch;
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    btQuaternion q = btQuaternion(msg->pose.pose.orientation.x, \
            msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, \
            msg->pose.pose.orientation.w);
    btScalar h = btScalar(heading);
    btScalar p = btScalar(pitch);
    btScalar r = btScalar(roll);
    btMatrix3x3(q).getEulerYPR(h, p, r);
    heading = double(h);
    t = msg->header.stamp;
  };
};


class PotFieldBot {
public:
  // Construst a new Potential Field controller object and hook up
  // this ROS node to the simulated robot's pose, velocity control,
  // and laser topics
  PotFieldBot(ros::NodeHandle& nh, int robotID, int n, \
      double gx, double gy) : ID(robotID), numRobots(n), \
      goalX(gx), goalY(gy), goalReached(false),
      canvas(16, 16, CV_8UC1) {
    // Initialize random time generator
    srand(time(NULL));
    repulsive_x = 0.0;
    repulsive_y = 0.0;

    // Advertise a new publisher for the current simulated robot's
    // velocity command topic (the second argument indicates that
    // if multiple command messages are in the queue to be sent,
    // only the last command will be sent)
    commandPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // Subscribe to the current simulated robot's laser scan topic and
    // tell ROS to call this->laserCallback() whenever a new message
    // is published on that topic
    laserSub = nh.subscribe("base_scan", 1, \
      &PotFieldBot::laserCallback, this);
    
    // Subscribe to each robot' ground truth pose topic
    // and tell ROS to call pose->poseCallback(...) whenever a new
    // message is published on that topic
    for (int i = 0; i < numRobots; i++) {
      pose.push_back(Pose());
    }
    for (int i = 0; i < numRobots; i++) {
 //FIXME:     poseSubs.push_back(nh.subscribe("/robot_" + \
        boost::lexical_cast<std::string>(i) + 
 poseSubs.push_back(nh.subscribe(\
        "/base_pose_ground_truth", 1, \
        &Pose::poseCallback, &pose[i]));
    }
  };

  // Update grayscale intensity on canvas pixel (x, y) (in robot coordinate frame)

  // Send a velocity command
  void move(double linearVelMPS, double angularVelRadPS) {
    geometry_msgs::Twist msg; // The default constructor will set all commands to 0
    msg.linear.x = linearVelMPS;
    msg.angular.z = angularVelRadPS;
    commandPub.publish(msg);
  };


  double getAngle(std::pair<double, double> v1, std::pair<double, double> v2) {
      double angle = (v1.first * v2.first + v1.second * v2.second);
      angle = angle / (sqrt(v1.first*v1.first+ v1.second*v1.second) * sqrt(v2.first*v2.first + v2.second*v2.second));
      return angle;
  }


  // Process incoming laser scan message
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
      // TODO: parse laser data
      // (see http://www.ros.org/doc/api/sensor_msgs/html/msg/LaserScan.html)
      if(goalReached)
          return;
      for (int i = 0; i < numRobots; i++) {
          repulsive_x = 0.0; // x coordinate of repulsive force in stage coordinate system; (cumulative repulsive for all rays)
          repulsive_y = 0.0; // y coordinate of repulsive force in stage coordinate system;
          float phi = pose[i].heading;
          for(unsigned int currIndex = 0; currIndex < msg->ranges.size(); currIndex++) {
              float currRange = msg->ranges[currIndex];
              float currAngle = msg->angle_min + currIndex * msg->angle_increment;
              if( currAngle <= msg->angle_max/2 && currAngle >= msg->angle_min/2) {
                  if(phi < 0) {
                      phi +=2*M_PI;
                  }
                  if(currRange > msg->range_min/2 && currRange < msg->range_max/2) {
                      double Fr_i = 0.0;
                      if(currRange < B && currRange > D_SAFE + EPS) { //computing the Repulsive Force magnitude
                          Fr_i = A /pow((currRange - D_SAFE),2) ;
                      } else if(currRange < D_SAFE + EPS){
                          Fr_i = A / pow(EPS,2);
                      } else {
                          Fr_i = 0;
                      }
                      double currRay_x_rc = Fr_i*std::cos(-currAngle); // in the robot's coordiante system
                      double currRay_y_rc = Fr_i*std::sin(-currAngle);

                      repulsive_x+=(currRay_x_rc);
                      repulsive_y+=(currRay_y_rc);
                  }
              }

          }
          double currRay_x_st = (repulsive_x * std::cos(phi)  - repulsive_y * std::sin(phi)); //relative to stage coordinates
          double currRay_y_st = (repulsive_x * std::sin(phi)  + repulsive_y  * std::cos(phi));
          if(currRay_y_st!=0 && currRay_x_st!=0) {
              repulsive_x = currRay_x_st;// + pose[i].x;
              repulsive_y = currRay_y_st;// + pose[i].y;
          }
          // DEBUG-------------------------------
          std::cout  << "-----> ONLY REPULSIVE from LASERCALLBACK ----------------------------------\n";
          std::cout  << "---------------------------------------\n";
          std::cout  << "-- Repulsive -- "<< repulsive_x << " " << repulsive_y <<" ---------------";
          std::cout  << "---------------------------------------\n";
          std::cout  << "-----ONLY REPULSIVE from LASERCALLBACK <----------------------------------\n";
          // DEBUG-------------------------------
      }
  };


  // Main FSM loop for ensuring that ROS messages are
  // processed in a timely manner, and also for sending
  // velocity controls to the simulated robot based on the FSM state
  void spin() {
      ros::Rate rate(30); // Specify the FSM loop rate in Hz

      canvasMutex.lock();
      canvas = cv::Scalar(CELL_UNKNOWN);
      canvasMutex.unlock();

      while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C
      //cv::imshow("Occupancy Grid Canvas", canvas);
      //cumulative_y =0;
      //cumulative_x =0;
      ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
          // TODO: remove demo code, compute potential function, actuate robot
          for (int i = 0; i < numRobots; i++) {
              double d = sqrt( pow(goalX - pose[i].x, 2) + pow(goalY - pose[i].y, 2));
              double angular_v = 0.0;
              double linear_v = 0.0;

              //double attractive_theta = atan2(goalY - pose[i].y, goalX-pose[i].x);
              double attractive_theta = atan2(goalY - pose[i].y, goalX - pose[i].x);
              double attractive_x = G * d * cos(attractive_theta);
              double attractive_y = G * d * sin(attractive_theta);
              
              // DEBUG------------------------------->
              //plotLine(-scale*pose[i].y, scale*pose[i].x, -scale*cumulative_y, scale*cumulative_x);
//    double attractive_theta = tf::getYaw(msg->pose.pose.orientation);
              //double rotation_angle = 
              std::cout  << "-- Current Pose -- "<< pose[i].x << " " << pose[i].y <<" ---------------\n";
              std::cout  << "---------------------------------------\n";
              std::cout  << "-- repulsive coordinates -- "<< repulsive_x << " " << repulsive_y <<" ---------------\n";
              std::cout  << " -- MAGNITUGE OF REPULSIVE FORCE " << sqrt(pow(repulsive_x-pose[i].x, 2) \
                      + pow(repulsive_y - pose[i].y, 2)) << "\n";
              std::cout  << " -- MAGNITUGE OF ATTRACTIVE FORCE " << G*d  << "\n";
              // DEBUG-------------------------------!


              double cumulative_x = repulsive_x + attractive_x;
              double cumulative_y = repulsive_y + attractive_y;


              // DEBUG: just for checking the attractive force 
              //cumulative_x = attractive_x;
              //cumulative_y = attractive_y;
              //--DEBUG-
              
              //double cumulative_theta =(atan2(cumulative_y - pose[i].y, cumulative_x - pose[i].x)); 
              double cumulative_theta =(atan2(cumulative_y, cumulative_x)); 
              //cumulative_theta = attractive_theta; //FIXME: remove this to make it work with cumulative
              angular_v = K*(cumulative_theta - pose[i].heading);
              
              // DEBUG-------------------------------
              //plotLine(-scale*pose[i].y, scale*pose[i].x, -scale*cumulative_y, scale*cumulative_x);
              std::cout  << "---------------------------------------\n";
              std::cout  << "-- Current Pose -- "<< pose[i].x << " " << pose[i].y <<" ---------------\n";
              std::cout  << "---------------------------------------\n";
              std::cout  << "-- THETA -->  "<< attractive_theta <<" ---------------\n";
              std::cout  << "-- Rotation Angle -->  "<< fabs(cumulative_theta - pose[i].heading) <<" ---------------\n";
              std::cout  << "-- HeadinG -->  "<< pose[i].heading <<" ---------------\n";
              std::cout  << "-- Distance --> " << d << "--------\n";
              std::cout  << "---------------------------------------\n";
              // DEBUG-------------------------------
              if(d > 0) {
                  linear_v = sqrt(pow(cumulative_x,2) + pow(cumulative_y, 2));
                  if(d < D_SAFE) {
                      angular_v = 0.0;
                      linear_v = 0.0;
                      goalReached = true;
                      std::cout << "---- > The goal is reached!!!\n";
                      move(angular_v, linear_v);
                  } else if(d !=0 ) {

                      if(fabs(cumulative_theta - pose[i].heading) > EPS_THETA) {
                          //fsm = FSM_ROTATE;
                          ros::Time rotateStartTime = ros::Time::now();
                          ros::Duration rotateDuration = ros::Duration(1/G); //to rotate in [ pi/4 , 3*Pi/2] range
                          move(0, angular_v);
                          std::cout << "TRYING TO ROTATE\n";
                          if(fabs(cumulative_theta - attractive_theta) > EPS_THETA) {
                              //move(linear_v, 0);
                          }
                          //}
                      } else {
                     //     move(0, angular_v);
                              move(linear_v, 0);
                      }
                  }
                  /*if(prevPose.x == pose[i].x && prevPose.y == pose[i].y){
					moveStartTime = ros::Time::now();
					rotateDuration = ros::Duration(double(1 + (rand()%7)) ); //to rotate in [ pi/4 , 3*Pi/2] range
                      while(moveStartTime - ros::Time::now() < moveDuration && d > D_SAFE + EPS) {
                      move(-linear_v,0);
                      }
                  }*/
              }
    
        // Demo code: print each robot's pose
        std::cout << std::endl;
        std::cout << i << "        ";
        std::cout << "Pose: " << pose[i].x << ", " << pose[i].y << ", " << pose[i].heading << std::endl;
      }

      ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
      rate.sleep(); // Sleep for the rest of the cycle, to enforce the FSM loop rate
    }
  };

  // Tunable motion controller parameters
  const static double FORWARD_SPEED_MPS = 2.0;
  const static double ROTATE_SPEED_RADPS = M_PI/2;

  // Short safety distance constants
  const static double EPS = 0.5;
  const static double EPS_THETA = 0.3; //radians
  const static double D_SAFE = 1.0; // in meters
  const static double G = 1.0; // gamma
  const static double K = 1.0; // scaling constant
  const static double A = 0.1; //alpha
  const static double B = 3.0; //betta in meters


  const static char CELL_OCCUPIED = 0;
  const static char CELL_UNKNOWN = 86;
  const static char CELL_FREE = 172;
  const static char CELL_ROBOT = 255;

  const static int scale = 10;

  bool goalReached;

protected:
  ros::Publisher commandPub; // Publisher to the current robot's velocity command topic
  ros::Subscriber laserSub; // Subscriber to the current robot's laser scan topic
  std::vector<ros::Subscriber> poseSubs; // List of subscribers to all robots' pose topics
  std::vector<Pose> pose; // List of pose objects for all robots
  int ID; // 0-indexed robot ID
  int numRobots; // Number of robots, positive value
  double goalX, goalY; // Coordinates of goal

  double repulsive_x ; // x coordinate of repulsive force in stage coordinate system; (cumulative repulsive for all rays)
  double repulsive_y; // y coordinate of repulsive force in stage coordinate system;

  cv::Mat canvas; // Occupancy grid canvas
  boost::mutex canvasMutex; // Mutex for occupancy grid canvas object
};


int main(int argc, char **argv) {
  int robotID = -1, numRobots = 0;
  double goalX, goalY;
  bool printUsage = false;
  
  // Parse and validate input arguments
  if (argc <= 4) {
    printUsage = true;
  } else {
    try {
      robotID = boost::lexical_cast<int>(argv[1]);
      numRobots = boost::lexical_cast<int>(argv[2]);
      goalX = boost::lexical_cast<double>(argv[3]);
      goalY = boost::lexical_cast<double>(argv[4]);

      if (robotID < 0) { printUsage = true; }
      if (numRobots <= 0) { printUsage = true; }
    } catch (std::exception err) {
      printUsage = true;
    }
  }
  if (printUsage) {
    std::cout << "Usage: " << argv[0] << " [ROBOT_NUM_ID] [NUM_ROBOTS] [GOAL_X] [GOAL_Y]" << std::endl;
    return EXIT_FAILURE;
  }
  
  ros::init(argc, argv, "potfieldbot_" + std::string(argv[1])); // Initiate ROS node
  //FIXME: ros::NodeHandle n("robot_" + std::string(argv[1])); // Create named handle "robot_#"
  ros::NodeHandle n; // Create named handle "robot_#"
  PotFieldBot robbie(n, robotID, numRobots, goalX, goalY); // Create new random walk object
  robbie.spin(); // Execute FSM loop

  return EXIT_SUCCESS;
};
