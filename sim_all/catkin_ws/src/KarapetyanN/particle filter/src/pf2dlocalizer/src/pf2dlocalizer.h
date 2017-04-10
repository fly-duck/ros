#ifndef PF2DLOCALIZER_H
#define PF2DLOCALIZER_H

#include "pf.h"

#include <ros/ros.h> // NodeHandle, Subscriber
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;

class pf2dlocalizer {

 public:
  // Class constructor
  pf2dlocalizer(ros::NodeHandle &n);

 private:
  // sensors handling functions
  void handleOdom(const nav_msgs::Odometry::ConstPtr& msg);
  void handleScan(const sensor_msgs::LaserScan::ConstPtr& msg);
  
  ros::Subscriber     odomSub, laserSub;

  // Filter specifications
  pf *particle_filter_;
  double ESS_threshold_;
  
  // Last robot position according to odometry
  bool init_odom_;
  double old_x_;
  double old_y_;
  double old_yaw_;
  ros::Time old_stamp_;
  
  // Last scan update
  double last_update_x_;
  double last_update_y_;
  double last_update_yaw_;
  bool init_laser_;
  bool update_;
  
  // Update thresholds
  double d_thresh_, a_thresh_;

};

#endif
