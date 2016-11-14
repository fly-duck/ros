#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value
#include <boost/thread/mutex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_listener.h>

#include <cmath> // for sin and cos

using namespace boost::posix_time;


class GridMapper {
    public:
        // Construst a new occupancy grid mapper  object and hook up
        // this ROS node to the simulated robot's pose, velocity control,
        // and laser topics
        GridMapper(ros::NodeHandle& nh, int width, int height) :
            canvas(height, width, CV_8UC1) {
                // Initialize random time generator
                srand(time(NULL));

                // Advertise a new publisher for the current simulated robot's
                // velocity command topic (the second argument indicates that
                // if multiple command messages are in the queue to be sent,
                // only the last command will be sent)
                commandPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

                // Subscribe to the current simulated robot's laser scan topic and
                // tell ROS to call this->laserCallback() whenever a new message
                // is published on that topic
                laserSub = nh.subscribe("base_scan", 1,\
                        &GridMapper::laserCallback, this);
                //laserSub = nh.subscribe("scan", 1, \
                        &GridMapper::laserCallback, this);

                // Subscribe to the current simulated robot' ground truth pose topic
                // and tell ROS to call this->poseCallback(...) whenever a new
                // message is published on that topic
                poseSub = nh.subscribe("base_pose_ground_truth", 1,\
                        &GridMapper::poseCallback, this);
                //poseSub = nh.subscribe("odom_combined", 1, \
                        &GridMapper::poseCallback, this);

                // Create resizeable named window
                cv::namedWindow("Occupancy Grid Canvas", \
                        CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);
            };


        // Save a snapshot of the occupancy grid canvas
        // NOTE: image is saved to same folder where code was executed
        void saveSnapshot() {
            std::string filename = "grid_" + to_iso_string(second_clock::local_time()) + ".jpg";
            std::cout <<  "-->" << filename << std::endl;
            canvasMutex.lock();
            cv::imwrite(filename, canvas);
            canvasMutex.unlock();
        };


        // Update grayscale intensity on canvas pixel (x, y) (in robot coordinate frame)
        void plot(int x, int y, char value) {
            canvasMutex.lock();
            x+=canvas.rows/2;
            y+=canvas.cols/2;
            if (x >= 0 && x < canvas.rows && y >= 0 && y < canvas.cols) {
                if(canvas.at<char>(x,y) != CELL_OCCUPIED){
                        canvas.at<char>(x, y) = value;
                        }
            }
            canvasMutex.unlock();
        };

        void plotLine(int x0, int y0, int x1, int y1) 
        {
            int dx = x1 - x0;
            int dy = y1 - y0;
            int D = 2*dy - dx;
            int y_l = y0;

            for(int x_l = x0; x_l < x1; ++x_l) {
                plot(x_l, y_l, CELL_FREE);
                if(D >=0) {
                    y_l++;
                    D -=dx;
                }
                D += dy;
            }
        }

        // Update grayscale intensity on canvas pixel (x, y) (in image coordinate frame)
        void plotImg(int x, int y, char value) {
            canvasMutex.lock();
            if (x >= 0 && x < canvas.cols && y >= 0 && y < canvas.rows) {
                canvas.at<char>(y, x) = value;
            }
            canvasMutex.unlock();
        };

        // Send a velocity command
        void move(double linearVelMPS, double angularVelRadPS) {
            geometry_msgs::Twist msg; // The default constructor will set all commands to 0
            msg.linear.x = linearVelMPS;
            msg.angular.z = angularVelRadPS;
            commandPub.publish(msg);
        };


        // Process incoming laser scan message
        void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
            // TODO: parse laser data and update occupancy grid canvas
            //       (use CELL_OCCUPIED, CELL_UNKNOWN, CELL_FREE, and CELL_ROBOT values)
            // (see http://www.ros.org/doc/api/sensor_msgs/html/msg/LaserScan.html)
						//
           /* points.clear();
            for(unsigned int currIndex = 0; currIndex < msg->ranges.size(); currIndex++) {
                float currRange = msg->ranges[currIndex];
                float currAngle = msg->angle_min + currIndex * msg->angle_increment;
                float phi = heading;
                if(phi < 0) {
                    phi +=2*M_PI;
                }
                if(currRange > msg->range_min && currRange < msg->range_max) {//&& currAngle == M_PI/2) {
                    if(currAngle < 0 ) {
                        currAngle = abs(currAngle);
                    } else {
                        currAngle = 2*M_PI - currAngle;
                    }
                    double currRay_x = x + currRange*std::cos(M_PI/2 + heading + currAngle); // the angle relative to the global 
                    std::cout  << "----> "<<heading  << std::endl;
                    double currRay_y = y + currRange*std::sin(M_PI/2 + heading + currAngle);
//                    currRay_x = currRay_x * std::cos(phi)  - currRay_y * std::sin(phi) + x;
  //                  currRay_y = currRay_x * std::sin(phi)  + currRay_y * std::cos(phi) + y;
                    plotLine(10*x, 10*y, 10*currRay_x, 10*currRay_y);
                    //sleep(0.5);
                    //plot(currRay_x, currRay_y,CELL_FREE);
                    //plot(10*currRay_x, 10*currRay_y,CELL_OCCUPIED);
                    points.push_back(std::make_pair(10*currRay_x, 10*currRay_y));
                }
            }*/
        };


        // Process incoming ground truth robot pose message
        void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
            double roll, pitch;
            x = - msg->pose.pose.position.y;
            y = msg->pose.pose.position.x;
            heading= tf::getYaw(msg->pose.pose.orientation);
        };


        // Main FSM loop for ensuring that ROS messages are
        // processed in a timely manner, and also for sending
        // velocity controls to the simulated robot based on the FSM state
        void spin() {
            int key = 0;

            // Initialize all pixel values in canvas to CELL_UNKNOWN
            canvasMutex.lock();
            canvas = cv::Scalar(CELL_UNKNOWN);
            canvasMutex.unlock();

            while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C
                // TODO: remove following demo code and make robot move around the environment
                plot(10*x, 10*y, CELL_OCCUPIED); // Demo code: plot robot's current position on canvas
                /*for(int i = 0; i < points.size(); ++i) {
                    plot(points.at(i).first, points.at(i).second, CELL_OCCUPIED);
                }*/

                // NOTE: DO NOT REMOVE CODE BELOW THIS LINE
                cv::imshow("Occupancy Grid Canvas", canvas);
                ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
                key = cv::waitKey(1000/SPIN_RATE_HZ); // Obtain keypress from user; wait at most N milliseconds
                //DEBUG
                std::cout << "---===>>> " << (key == 'x') << std::endl;
                std::cout << "---===>>> " << key << std::endl;
                //!DEBUG
                if (key == 'x' || key == 'X') {
                    break;
                } else if (key == ' ') {
                    saveSnapshot();
                }
            }

            ros::shutdown(); // Ensure that this ROS node shuts down properly
        };

        const static double MIN_SCAN_ANGLE_RAD = -15.0/180*M_PI;
        const static double MAX_SCAN_ANGLE_RAD = +15.0/180*M_PI;
        const static float PROXIMITY_RANGE_M = 2.0; // Should be smaller than sensor_msgs::LaserScan::range_max

        // Tunable motion controller parameters
        const static double FORWARD_SPEED_MPS = 2.0;
        const static double ROTATE_SPEED_RADPS = M_PI/2;

        const static int SPIN_RATE_HZ = 30;

        const static char CELL_OCCUPIED = 0;
        const static char CELL_UNKNOWN = 86;
        const static char CELL_FREE = 172;
        const static char CELL_ROBOT = 255;


    protected:
        ros::Publisher commandPub; // Publisher to the current robot's velocity command topic
        ros::Subscriber laserSub; // Subscriber to the current robot's laser scan topic
        ros::Subscriber poseSub; // Subscriber to the current robot's ground truth pose topic

        double x; // in simulated Stage units, + = East/right
        double y; // in simulated Stage units, + = North/up
        double heading; // in radians, 0 = East (+x dir.), pi/2 = North (+y dir.)

        cv::Mat canvas; // Occupancy grid canvas
        boost::mutex canvasMutex; // Mutex for occupancy grid canvas object
        std::vector<std::pair<int, int> > points;
};


int main(int argc, char **argv) {
    int width, height;
    bool printUsage = false;

    // Parse and validate input arguments
    if (argc <= 2) {
        printUsage = true;
    } else {
        try {
            width = boost::lexical_cast<int>(argv[1]);
            height = boost::lexical_cast<int>(argv[2]);

            if (width <= 0) { printUsage = true; }
            else if (height <= 0) { printUsage = true; }
        } catch (std::exception err) {
            printUsage = true;
        }
    }
    if (printUsage) {
        std::cout << "Usage: " << argv[0] << " [CANVAS_WIDTH] [CANVAS_HEIGHT]" << std::endl;
        return EXIT_FAILURE;
    }

    ros::init(argc, argv, "grid_mapper"); // Initiate ROS node
    ros::NodeHandle n; // Create default handle
    GridMapper robbie(n, width, height); // Create new grid mapper object
    robbie.spin(); // Execute FSM loop

    return EXIT_SUCCESS;
};
