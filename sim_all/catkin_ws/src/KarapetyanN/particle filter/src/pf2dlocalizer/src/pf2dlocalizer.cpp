#include "pf2dlocalizer.h"

#include <fstream>
#include <iostream>

#include <boost/thread/mutex.hpp> // boost::mutex::scoped_lock
#include <angles/angles.h> // normalize_angle, normalize_angle_positive

#include <unordered_map>
#include <list>
#include <tf/tf.h>

#define EPS 0.01
#define DEBUG_1

pf2dlocalizer::pf2dlocalizer(ros::NodeHandle &nh)
{
  ROS_INFO("pf2dlocalizer::pf2dlocalizer ");
  ros::NodeHandle ph = ros::NodeHandle("~");

  std::string odom_topic, scan_topic, environment;
  double vSigma, wSigma;
  double sensor_accuracy, processed_scans_number;
  bool consider_inf_scans, use_scans;
  int pf_size;

  // Topics to listen
  ph.param<std::string>("odom_topic", odom_topic, "odom");
  ph.param<std::string>("scan_topic", scan_topic, "scan");
  ph.param<bool>("use_scans", use_scans, true);

  // Environment and context
  ph.param<std::string>("environment", environment, "amoco_hall.yaml");
  
  // Motion sigmas
  ph.param<double>("vSigma", vSigma, 0.5);
  ph.param<double>("wSigma", wSigma, 0.01);

  // Laser sensor parameters
  ph.param<double>("sensor_accuracy", sensor_accuracy, 0.3);
  ph.param<double>("processed_scans_number", processed_scans_number, 1);
  ph.param<bool>("consider_inf_scans", consider_inf_scans, true);

  // Particle filter parameters
  ph.param<int>("pf_size", pf_size, 10000);
  ph.param<double>("ESS_threshold", ESS_threshold_, 0.8);

  // Propagation and update thresholds
  ph.param<double>("update_min_d", d_thresh_, 0.4);
  ph.param<double>("update_min_a", a_thresh_, 0.2);

  
  ROS_INFO_STREAM("SETTING: environment " << environment);
  ROS_INFO_STREAM("Motion: vSigma " << vSigma << " wSigma " << wSigma);
  ROS_INFO_STREAM("Sensor: sensor_accuracy " << sensor_accuracy << " processed_scans_number " << processed_scans_number << " consider_inf_scans " << consider_inf_scans);
  ROS_INFO_STREAM("PF: size " << pf_size << " ESS_threshold " << ESS_threshold_ << " update_min_d " << d_thresh_ << " update_min_a " << a_thresh_);
  

  // Initialization of the particle filter 
  ROS_INFO_STREAM("Uniform initialization");
  particle_filter_ = new pf(pf_size, environment, sensor_accuracy, processed_scans_number, consider_inf_scans, vSigma, wSigma);
  
  // Subscribing to sensor topics
  odomSub = nh.subscribe(odom_topic, 1, &pf2dlocalizer::handleOdom, this);
  init_odom_ = false;
  if (use_scans)
  {
    ROS_INFO_STREAM("using scan");
    laserSub = nh.subscribe(scan_topic, 1, &pf2dlocalizer::handleScan, this);
    init_laser_ = false;
  }
}

void pf2dlocalizer::handleOdom(const nav_msgs::Odometry::ConstPtr& odom) 
{
    // TODO: Handle odom data for the propagation step
    // Hint : call particle_filter_->move(...)
    // Hint : Also update the flag for laser update, which serves for decreasing
    // the computational burden. The flag is true, if since the last update
    // the robot has traveled more than d_thresh_ or rotated more than a_thresh_
    //////////////////// ANSWER CODE BEGIN //////////////////////
    double x = odom->pose.pose.position.x;
	double y = odom->pose.pose.position.y;
	double yaw = tf::getYaw(odom->pose.pose.orientation);
	ros::Time stamp = odom->header.stamp; 
#ifdef DEBUG_1
	std::cout << "Odom x " << x << " -- y " << y << std::endl;
#endif
	if(!init_odom_) {
		old_yaw_ = x;
		old_x_ = y;
		old_y_ = yaw;
		old_stamp_= stamp;
		init_odom_ = true;
	} else {

		ros::Duration dur = stamp - old_stamp_;
		double dt = dur.toSec();
		double rot = (old_yaw_ - yaw);
		double dist = sqrt(pow((old_x_ - x), 2) + pow((old_y_ - y), 2));
#ifdef DEBUG_1
		std::cout << "---------- " << dist << std::endl;
#endif
		particle_filter_->move(dist/dt, rot/dt, dt);//dt);//fabs(old_stamp_ - stamp));
		old_yaw_ = yaw;
		old_x_ = x;
		old_y_ = y;
		old_stamp_ = stamp;
		init_odom_ = true;

		if(dist > d_thresh_ || rot> a_thresh_) {
			update_ = true;
		}
	}
  //////////////////// ANSWER CODE END //////////////////////


  // Update the particle filter just checking the map
  particle_filter_->update();

  // Resampling
  double cv2=0.0;
  int ESS=particle_filter_->ESS(cv2);
  std::cout<<"cv2="<<cv2<<" ESS="<<ESS<<std::endl;
  if(ESS < ESS_threshold_ * particle_filter_->size()) 
  {
    particle_filter_->Resample();
  }
  
  // Output
  particle_filter_->draw();
  particle_filter_->StatWeight("huh");
}


void pf2dlocalizer::handleScan(const sensor_msgs::LaserScan::ConstPtr& msg) 
{
  // Update that happens according to d_thresh and a_thresh
  if (!init_laser_ || update_)
  {
    init_laser_ = true;
    
    // Update step
    particle_filter_->update_uniform(msg->ranges, msg->angle_min, msg->angle_increment, msg->range_max);

    // Resampling
    double cv2=0.0;
    int ESS=particle_filter_->ESS(cv2);
    std::cout<<"cv2="<<cv2<<" ESS="<<ESS<<std::endl;
    if(ESS < ESS_threshold_ * particle_filter_->size()) 
    {
      particle_filter_->Resample();
    }
    
    // Output
    particle_filter_->draw();
    particle_filter_->StatWeight("huh");
    
    // Setting back to false the laser update flag
    update_ = false;
  }
}

